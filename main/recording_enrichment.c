#include "recording_enrichment.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "avi_mjpeg_writer.h"
#include "coco_espdl_bridge.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "recording_enrichment";

typedef struct {
    char raw_name[96];
    char output_name[96];
    char raw_path[512];
    char output_path[512];
    char raw_meta_path[512];
    char meta_path[512];
    char method[16];
    avi_mjpeg_info_t raw_info;
    uint32_t completed_stride;
    uint32_t next_stride;
    int64_t start_ms;
    uint64_t start_epoch_ms;
    int64_t sort_time;
} enrichment_candidate_t;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static recording_enrichment_status_t s_status;
static recording_enrichment_infer_cb_t s_infer_cb;
static void *s_infer_arg;
static char s_requested_raw_name[96];

static bool has_suffix(const char *text, const char *suffix);

static void status_set_error(const char *message)
{
    taskENTER_CRITICAL(&s_status_mux);
    strlcpy(s_status.last_error, message ? message : "", sizeof(s_status.last_error));
    taskEXIT_CRITICAL(&s_status_mux);
}

static void status_begin(const enrichment_candidate_t *candidate)
{
    taskENTER_CRITICAL(&s_status_mux);
    s_status.running = true;
    s_status.cancelled = false;
    s_status.pass_stride = candidate->next_stride;
    s_status.completed_stride = candidate->completed_stride;
    s_status.frame_index = 0;
    s_status.frame_count = candidate->raw_info.frame_count;
    s_status.inferred_frames = 0;
    s_status.output_frames = 0;
    s_status.inference_coverage_x1000 = 0;
    strlcpy(s_status.raw_name, candidate->raw_name, sizeof(s_status.raw_name));
    strlcpy(s_status.output_name, candidate->output_name, sizeof(s_status.output_name));
    strlcpy(s_status.method, candidate->method, sizeof(s_status.method));
    strlcpy(s_status.last_error, "running", sizeof(s_status.last_error));
    taskEXIT_CRITICAL(&s_status_mux);
}

static void status_progress(uint32_t frame_index, uint32_t inferred_frames,
                            uint32_t output_frames, uint32_t frame_count)
{
    taskENTER_CRITICAL(&s_status_mux);
    s_status.frame_index = frame_index;
    s_status.inferred_frames = inferred_frames;
    s_status.output_frames = output_frames;
    s_status.inference_coverage_x1000 = frame_count ?
        (uint32_t)(((uint64_t)inferred_frames * 1000U) / frame_count) : 0;
    taskEXIT_CRITICAL(&s_status_mux);
}

static void status_stage_error(const char *stage, uint32_t frame_index, esp_err_t err)
{
    char message[128];
    if (frame_index > 0) {
        snprintf(message, sizeof(message), "%s frame %" PRIu32 " failed: %s",
                 stage ? stage : "pass", frame_index, esp_err_to_name(err));
    } else {
        snprintf(message, sizeof(message), "%s failed: %s",
                 stage ? stage : "pass", esp_err_to_name(err));
    }
    status_set_error(message);
}

void recording_enrichment_set_infer_callback(recording_enrichment_infer_cb_t cb, void *arg)
{
    taskENTER_CRITICAL(&s_status_mux);
    s_infer_cb = cb;
    s_infer_arg = arg;
    taskEXIT_CRITICAL(&s_status_mux);
}

esp_err_t recording_enrichment_request(const char *raw_name)
{
    if (!raw_name || strncmp(raw_name, "raw_", 4) != 0 || !has_suffix(raw_name, ".avi") ||
        strchr(raw_name, '/') || strchr(raw_name, '\\')) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_status_mux);
    strlcpy(s_requested_raw_name, raw_name, sizeof(s_requested_raw_name));
    strlcpy(s_status.last_error, "manual fill queued", sizeof(s_status.last_error));
    taskEXIT_CRITICAL(&s_status_mux);
    return ESP_OK;
}

bool recording_enrichment_has_request(void)
{
    bool has_request;
    taskENTER_CRITICAL(&s_status_mux);
    has_request = s_requested_raw_name[0] != '\0';
    taskEXIT_CRITICAL(&s_status_mux);
    return has_request;
}

static void status_finish(bool cancelled, uint32_t completed_stride, const char *message)
{
    taskENTER_CRITICAL(&s_status_mux);
    s_status.running = false;
    s_status.cancelled = cancelled;
    if (!cancelled && completed_stride > 0) {
        s_status.completed_stride = completed_stride;
        s_status.passes_completed++;
    }
    strlcpy(s_status.last_error, message ? message : "ok", sizeof(s_status.last_error));
    taskEXIT_CRITICAL(&s_status_mux);
}

void recording_enrichment_init(bool enabled)
{
    taskENTER_CRITICAL(&s_status_mux);
    memset(&s_status, 0, sizeof(s_status));
    s_status.enabled = enabled;
    strlcpy(s_status.last_error, enabled ? "idle" : "disabled", sizeof(s_status.last_error));
    taskEXIT_CRITICAL(&s_status_mux);
}

void recording_enrichment_get_status(recording_enrichment_status_t *status)
{
    if (!status) {
        return;
    }
    taskENTER_CRITICAL(&s_status_mux);
    *status = s_status;
    taskEXIT_CRITICAL(&s_status_mux);
}

static bool has_suffix(const char *text, const char *suffix)
{
    size_t text_len = strlen(text);
    size_t suffix_len = strlen(suffix);
    return text_len >= suffix_len && strcmp(text + text_len - suffix_len, suffix) == 0;
}

static int64_t time_from_name(const char *name)
{
    const char *mark = strstr(name, "_t");
    return mark ? atoll(mark + 2) : 0;
}

static int64_t sort_time_from_name(const char *name)
{
    if (!name) {
        return 0;
    }
    const char *p = name;
    if (strncmp(p, "raw_", 4) == 0) {
        p += 4;
    } else if (strncmp(p, "annotated_", 10) == 0) {
        p += 10;
    }
    if (strlen(p) >= 15 &&
        p[0] >= '0' && p[0] <= '9' &&
        p[1] >= '0' && p[1] <= '9' &&
        p[2] >= '0' && p[2] <= '9' &&
        p[3] >= '0' && p[3] <= '9' &&
        p[4] >= '0' && p[4] <= '9' &&
        p[5] >= '0' && p[5] <= '9' &&
        p[6] >= '0' && p[6] <= '9' &&
        p[7] >= '0' && p[7] <= '9' &&
        p[8] == '_' &&
        p[9] >= '0' && p[9] <= '9' &&
        p[10] >= '0' && p[10] <= '9' &&
        p[11] >= '0' && p[11] <= '9' &&
        p[12] >= '0' && p[12] <= '9' &&
        p[13] >= '0' && p[13] <= '9' &&
        p[14] >= '0' && p[14] <= '9') {
        char compact[15];
        memcpy(compact, p, 8);
        memcpy(compact + 8, p + 9, 6);
        compact[14] = '\0';
        return atoll(compact);
    }
    return time_from_name(name);
}

static bool json_get_string_field(const char *line, const char *key, char *out, size_t out_size)
{
    if (!line || !key || !out || out_size == 0) {
        return false;
    }
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(line, pattern);
    if (!p) {
        return false;
    }
    p += strlen(pattern);
    const char *end = strchr(p, '"');
    if (!end) {
        return false;
    }
    size_t n = (size_t)(end - p);
    if (n >= out_size) {
        n = out_size - 1;
    }
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static int64_t json_i64_from_line(const char *line, const char *key)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *value = strstr(line, needle);
    return value ? atoll(value + strlen(needle)) : -1;
}

static uint32_t json_u32_from_line(const char *line, const char *key)
{
    int64_t value = json_i64_from_line(line, key);
    return value > 0 ? (uint32_t)value : 0;
}

static void meta_name_for_avi(const char *avi_name, char *meta_name, size_t meta_name_size)
{
    if (!avi_name || !meta_name || meta_name_size == 0) {
        return;
    }
    strlcpy(meta_name, avi_name, meta_name_size);
    char *ext = strrchr(meta_name, '.');
    if (ext) {
        strlcpy(ext, ".jsonl", (size_t)(meta_name + meta_name_size - ext));
    } else {
        strlcat(meta_name, ".jsonl", meta_name_size);
    }
}

static bool text_contains_ci(const char *text, const char *needle)
{
    if (!text || !needle || !needle[0]) {
        return false;
    }
    size_t needle_len = strlen(needle);
    for (const char *p = text; *p; p++) {
        size_t i = 0;
        while (i < needle_len && p[i]) {
            char a = p[i];
            char b = needle[i];
            if (a >= 'A' && a <= 'Z') {
                a = (char)(a - 'A' + 'a');
            }
            if (b >= 'A' && b <= 'Z') {
                b = (char)(b - 'A' + 'a');
            }
            if (a != b) {
                break;
            }
            i++;
        }
        if (i == needle_len) {
            return true;
        }
    }
    return false;
}

static void method_from_text(const char *text, char *method, size_t method_size)
{
    if (!method || method_size == 0) {
        return;
    }
    if (text_contains_ci(text, "tiny")) {
        strlcpy(method, "tinycls", method_size);
    } else if (text_contains_ci(text, "coco")) {
        strlcpy(method, "coco", method_size);
    } else if (text_contains_ci(text, "fish")) {
        strlcpy(method, "fish31", method_size);
    }
}

static void read_candidate_sidecar(enrichment_candidate_t *candidate)
{
    strlcpy(candidate->method, "fish31", sizeof(candidate->method));
    method_from_text(candidate->raw_name, candidate->method, sizeof(candidate->method));
    candidate->start_ms = time_from_name(candidate->raw_name);
    candidate->sort_time = sort_time_from_name(candidate->raw_name);

    FILE *file = fopen(candidate->raw_meta_path, "r");
    if (!file) {
        return;
    }
    char line[2048] = {0};
    if (fgets(line, sizeof(line), file)) {
        char text[64] = {0};
        if (json_get_string_field(line, "method", text, sizeof(text))) {
            method_from_text(text, candidate->method, sizeof(candidate->method));
        }
        if (json_get_string_field(line, "model", text, sizeof(text))) {
            method_from_text(text, candidate->method, sizeof(candidate->method));
        }
        int64_t time_ms = json_i64_from_line(line, "time_ms");
        if (time_ms >= 0) {
            candidate->start_ms = time_ms;
            candidate->sort_time = time_ms;
        }
        int64_t epoch_ms = json_i64_from_line(line, "epoch_ms");
        if (epoch_ms > 0) {
            candidate->start_epoch_ms = (uint64_t)epoch_ms;
            candidate->sort_time = epoch_ms;
        }
    }
    fclose(file);
}

static uint32_t read_completed_stride(const enrichment_candidate_t *candidate)
{
    avi_mjpeg_info_t output_info = {0};
    if (avi_mjpeg_probe(candidate->output_path, &output_info) != ESP_OK ||
        output_info.frame_count != candidate->raw_info.frame_count) {
        return 0;
    }
    uint64_t delta = output_info.duration_ms > candidate->raw_info.duration_ms ?
        output_info.duration_ms - candidate->raw_info.duration_ms :
        candidate->raw_info.duration_ms - output_info.duration_ms;
    uint64_t frame_ms = candidate->raw_info.frame_count ?
        (candidate->raw_info.duration_ms + candidate->raw_info.frame_count - 1U) /
            candidate->raw_info.frame_count : 1U;
    if (delta > frame_ms) {
        return 0;
    }

    FILE *file = fopen(candidate->meta_path, "r");
    if (!file) {
        return 0;
    }
    char line[2048] = {0};
    bool have_line = fgets(line, sizeof(line), file) != NULL;
    fclose(file);
    if (!have_line) {
        return 0;
    }
    char annotated_method[16] = {0};
    char text[64] = {0};
    if (json_get_string_field(line, "method", text, sizeof(text))) {
        method_from_text(text, annotated_method, sizeof(annotated_method));
    }
    if (!annotated_method[0] && json_get_string_field(line, "model", text, sizeof(text))) {
        method_from_text(text, annotated_method, sizeof(annotated_method));
    }
    if (!annotated_method[0]) {
        method_from_text(candidate->output_name, annotated_method, sizeof(annotated_method));
    }
    if (annotated_method[0] && strcmp(annotated_method, candidate->method) != 0) {
        return 0;
    }
    return json_u32_from_line(line, "pass_stride");
}

static uint32_t next_stride(uint32_t completed, uint32_t initial)
{
    if (completed == 1U) {
        return 0;
    }
    if (completed == 0U) {
        return initial ? initial : 8U;
    }
    return completed > 1U ? completed / 2U : 1U;
}

static bool build_candidate(const char *dir, const char *raw_name, uint32_t initial_stride,
                            bool force_stride_one, enrichment_candidate_t *candidate)
{
    if (strncmp(raw_name, "raw_", 4) != 0 || !has_suffix(raw_name, ".avi")) {
        return false;
    }
    memset(candidate, 0, sizeof(*candidate));
    strlcpy(candidate->raw_name, raw_name, sizeof(candidate->raw_name));
    snprintf(candidate->output_name, sizeof(candidate->output_name),
             "annotated_%s", raw_name + 4);
    snprintf(candidate->raw_path, sizeof(candidate->raw_path), "%s/%s", dir, raw_name);
    snprintf(candidate->output_path, sizeof(candidate->output_path), "%s/%s",
             dir, candidate->output_name);
    char raw_meta_name[96];
    meta_name_for_avi(raw_name, raw_meta_name, sizeof(raw_meta_name));
    snprintf(candidate->raw_meta_path, sizeof(candidate->raw_meta_path), "%s/%s",
             dir, raw_meta_name);
    strlcpy(candidate->meta_path, candidate->output_path, sizeof(candidate->meta_path));
    char *ext = strrchr(candidate->meta_path, '.');
    if (!ext) {
        return false;
    }
    strlcpy(ext, ".jsonl", (size_t)(candidate->meta_path + sizeof(candidate->meta_path) - ext));
    if (avi_mjpeg_probe(candidate->raw_path, &candidate->raw_info) != ESP_OK) {
        return false;
    }
    read_candidate_sidecar(candidate);
    candidate->completed_stride = force_stride_one ? 0U : read_completed_stride(candidate);
    candidate->next_stride = force_stride_one ? 1U :
        next_stride(candidate->completed_stride, initial_stride);
    return candidate->next_stride > 0;
}

static esp_err_t find_candidate(const char *dir, uint32_t initial_stride,
                                enrichment_candidate_t *selected)
{
    char requested[96] = {0};
    taskENTER_CRITICAL(&s_status_mux);
    strlcpy(requested, s_requested_raw_name, sizeof(requested));
    taskEXIT_CRITICAL(&s_status_mux);
    if (requested[0]) {
        if (build_candidate(dir, requested, 1U, true, selected)) {
            selected->next_stride = 1U;
            return ESP_OK;
        }
        taskENTER_CRITICAL(&s_status_mux);
        if (strcmp(s_requested_raw_name, requested) == 0) {
            s_requested_raw_name[0] = '\0';
        }
        taskEXIT_CRITICAL(&s_status_mux);
        return ESP_ERR_NOT_FOUND;
    }

    DIR *handle = opendir(dir);
    if (!handle) {
        return ESP_ERR_NOT_FOUND;
    }
    bool found = false;
    struct dirent *entry;
    while ((entry = readdir(handle)) != NULL) {
        enrichment_candidate_t candidate;
        if (!build_candidate(dir, entry->d_name, initial_stride, false, &candidate)) {
            continue;
        }
        if (!found || candidate.next_stride > selected->next_stride ||
            (candidate.next_stride == selected->next_stride &&
             candidate.sort_time > selected->sort_time)) {
            *selected = candidate;
            found = true;
        }
    }
    closedir(handle);
    return found ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static bool path_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

static void restore_previous(const char *final_path, const char *prev_path)
{
    if (!path_exists(final_path) && path_exists(prev_path)) {
        rename(prev_path, final_path);
    } else if (path_exists(final_path)) {
        unlink(prev_path);
    }
}

static esp_err_t replace_pair(const char *video_new, const char *video_final,
                              const char *meta_new, const char *meta_final)
{
    char video_prev[544];
    char meta_prev[544];
    snprintf(video_prev, sizeof(video_prev), "%s.prev", video_final);
    snprintf(meta_prev, sizeof(meta_prev), "%s.prev", meta_final);
    restore_previous(video_final, video_prev);
    restore_previous(meta_final, meta_prev);

    bool had_video = path_exists(video_final);
    bool had_meta = path_exists(meta_final);
    unlink(video_prev);
    unlink(meta_prev);
    if ((had_video && rename(video_final, video_prev) != 0) ||
        (had_meta && rename(meta_final, meta_prev) != 0)) {
        restore_previous(video_final, video_prev);
        restore_previous(meta_final, meta_prev);
        return ESP_FAIL;
    }
    if (rename(video_new, video_final) != 0 || rename(meta_new, meta_final) != 0) {
        unlink(video_final);
        unlink(meta_final);
        restore_previous(video_final, video_prev);
        restore_previous(meta_final, meta_prev);
        return ESP_FAIL;
    }
    unlink(video_prev);
    unlink(meta_prev);
    return ESP_OK;
}

static esp_err_t run_pass(const enrichment_candidate_t *candidate,
                          uint32_t min_score, uint8_t jpeg_quality,
                          recording_enrichment_cancel_cb_t should_cancel,
                          void *cancel_arg)
{
    char video_part[544];
    char video_new[544];
    char meta_part[544];
    char meta_new[544];
    snprintf(video_part, sizeof(video_part), "%s.part", candidate->output_path);
    snprintf(video_new, sizeof(video_new), "%s.new", candidate->output_path);
    snprintf(meta_part, sizeof(meta_part), "%s.part", candidate->meta_path);
    snprintf(meta_new, sizeof(meta_new), "%s.new", candidate->meta_path);
    unlink(video_part);
    unlink(video_new);
    unlink(meta_part);
    unlink(meta_new);

    avi_mjpeg_reader_t *reader = NULL;
    avi_mjpeg_info_t info = {0};
    esp_err_t ret = avi_mjpeg_reader_open(&reader, candidate->raw_path, &info);
    if (ret != ESP_OK) {
        status_stage_error("open raw AVI", 0, ret);
        return ret;
    }
    uint32_t fps = info.scale ? (info.rate + info.scale / 2U) / info.scale : 0;
    if (fps == 0) {
        fps = 1;
    }
    avi_mjpeg_writer_t *writer = NULL;
    ret = avi_mjpeg_writer_open(&writer, video_part, video_new,
                                info.width, info.height, fps);
    if (ret != ESP_OK) {
        status_stage_error("open annotated AVI", 0, ret);
        avi_mjpeg_reader_close(reader);
        return ret;
    }
    FILE *meta = fopen(meta_part, "w");
    if (!meta) {
        status_stage_error("open annotated metadata", 0, ESP_FAIL);
        avi_mjpeg_writer_abort(writer);
        avi_mjpeg_reader_close(reader);
        unlink(video_part);
        return ESP_FAIL;
    }

    char last_meta_json[2048] = {0};
    uint32_t source_frame_index = 0;
    uint32_t inferred_frames = 0;
    uint32_t output_frames = 0;
    bool cancelled = false;
    while (output_frames < info.frame_count) {
        if (should_cancel && should_cancel(cancel_arg)) {
            cancelled = true;
            ret = ESP_ERR_TIMEOUT;
            break;
        }

        const uint8_t *raw_jpeg = NULL;
        size_t raw_size = 0;
        uint32_t frame_index = 0;
        ret = avi_mjpeg_reader_next(reader, &raw_jpeg, &raw_size, &frame_index);
        if (ret != ESP_OK) {
            status_stage_error("read raw AVI", output_frames + 1U, ret);
            break;
        }

        uint8_t *annotated = NULL;
        size_t annotated_size = 0;
        const uint8_t *output_jpeg = raw_jpeg;
        size_t output_size = raw_size;
        char meta_json[2048] = {0};
        bool key_frame = ((frame_index - 1U) % candidate->next_stride) == 0U;
        if (key_frame) {
            recording_enrichment_infer_cb_t infer_cb = NULL;
            void *infer_arg = NULL;
            taskENTER_CRITICAL(&s_status_mux);
            infer_cb = s_infer_cb;
            infer_arg = s_infer_arg;
            taskEXIT_CRITICAL(&s_status_mux);
            if (!infer_cb) {
                ret = ESP_ERR_NOT_SUPPORTED;
                break;
            }
            ret = infer_cb(raw_jpeg, raw_size, candidate->method, frame_index,
                           min_score, jpeg_quality,
                           meta_json, sizeof(meta_json),
                           &annotated, &annotated_size, infer_arg);
            if (ret != ESP_OK) {
                status_stage_error("infer annotated frame", frame_index, ret);
                break;
            }
            source_frame_index = frame_index;
            inferred_frames++;
            strlcpy(last_meta_json, meta_json, sizeof(last_meta_json));
            if (annotated && annotated_size > 0) {
                output_jpeg = annotated;
                output_size = annotated_size;
            }
            if (should_cancel && should_cancel(cancel_arg)) {
                cancelled = true;
                coco_espdl_free_jpeg(annotated);
                ret = ESP_ERR_TIMEOUT;
                break;
            }
        } else {
            strlcpy(meta_json, last_meta_json, sizeof(meta_json));
        }
        ret = avi_mjpeg_writer_add_frame(writer, output_jpeg, output_size);
        if (ret != ESP_OK) {
            status_stage_error("write annotated AVI", frame_index, ret);
            coco_espdl_free_jpeg(annotated);
            break;
        }

        const char *source = key_frame ?
            (meta_json[0] ? "inference" : "inference-empty") :
            (meta_json[0] ? "propagated" : "raw-no-inference");
        int64_t start_ms = candidate->start_ms;
        uint64_t offset_ms = info.frame_count > 1 ?
            ((uint64_t)(frame_index - 1U) * info.duration_ms) / (info.frame_count - 1U) : 0;
        if (fprintf(meta,
                    "{\"index_version\":4,\"segment\":\"%s\",\"kind\":\"annotated\","
                    "\"method\":\"%s\","
                    "\"frame_index\":%" PRIu32 ",\"time_ms\":%" PRId64
                    ",\"epoch_ms\":%" PRIu64
                    ",\"jpeg_bytes\":%u,\"width\":%" PRIu32 ",\"height\":%" PRIu32
                    "%s%s"
                    ",\"result_source\":\"%s\",\"source_frame_index\":%" PRIu32
                    ",\"inference_age_frames\":%" PRIu32 ",\"pass_stride\":%" PRIu32 "}\n",
                    candidate->output_name, candidate->method,
                    frame_index, start_ms + (int64_t)offset_ms,
                    candidate->start_epoch_ms ? candidate->start_epoch_ms + offset_ms : 0,
                    (unsigned)output_size, info.width, info.height,
                    meta_json[0] ? "," : "", meta_json,
                    source, source_frame_index,
                    source_frame_index ? frame_index - source_frame_index : 0,
                    candidate->next_stride) < 0) {
            status_stage_error("write annotated metadata", frame_index, ESP_FAIL);
            coco_espdl_free_jpeg(annotated);
            ret = ESP_FAIL;
            break;
        }
        coco_espdl_free_jpeg(annotated);
        output_frames++;
        status_progress(frame_index, inferred_frames, output_frames, info.frame_count);
        if ((output_frames % 32U) == 0U && fflush(meta) != 0) {
            status_stage_error("flush annotated metadata", frame_index, ESP_FAIL);
            ret = ESP_FAIL;
            break;
        }
    }

    avi_mjpeg_reader_close(reader);
    if (ret == ESP_OK && output_frames == info.frame_count) {
        avi_mjpeg_writer_set_duration_ms(writer, info.duration_ms);
        ret = avi_mjpeg_writer_close(writer);
        if (ret != ESP_OK) {
            status_stage_error("close annotated AVI", output_frames, ret);
        }
        writer = NULL;
        if (ret == ESP_OK && (fflush(meta) != 0 || fsync(fileno(meta)) != 0)) {
            status_stage_error("sync annotated metadata", output_frames, ESP_FAIL);
            ret = ESP_FAIL;
        }
    } else if (ret == ESP_OK) {
        status_stage_error("complete annotated AVI", output_frames, ESP_ERR_INVALID_SIZE);
        ret = ESP_ERR_INVALID_SIZE;
    }
    if (writer) {
        avi_mjpeg_writer_abort(writer);
    }
    if (fclose(meta) != 0 && ret == ESP_OK) {
        status_stage_error("close annotated metadata", output_frames, ESP_FAIL);
        ret = ESP_FAIL;
    }

    if (ret == ESP_OK && rename(meta_part, meta_new) != 0) {
        status_stage_error("stage annotated metadata", output_frames, ESP_FAIL);
        ret = ESP_FAIL;
    }
    avi_mjpeg_info_t output_info = {0};
    if (ret == ESP_OK &&
        (avi_mjpeg_probe(video_new, &output_info) != ESP_OK ||
         output_info.frame_count != info.frame_count ||
         output_info.duration_ms != info.duration_ms)) {
        status_stage_error("verify annotated AVI", output_frames, ESP_ERR_INVALID_RESPONSE);
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (ret == ESP_OK) {
        ret = replace_pair(video_new, candidate->output_path, meta_new, candidate->meta_path);
        if (ret != ESP_OK) {
            status_stage_error("publish annotated AVI", output_frames, ret);
        }
    }
    if (ret != ESP_OK) {
        unlink(video_part);
        unlink(video_new);
        unlink(meta_part);
        unlink(meta_new);
    }
    if (cancelled) {
        return ESP_ERR_TIMEOUT;
    }
    return ret;
}

esp_err_t recording_enrichment_process_next(
    const char *recording_dir,
    uint32_t initial_stride,
    uint32_t min_score,
    uint8_t jpeg_quality,
    recording_enrichment_cancel_cb_t should_cancel,
    void *cancel_arg)
{
    if (!recording_dir || initial_stride == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    recording_enrichment_status_t status;
    recording_enrichment_get_status(&status);
    if (!status.enabled) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (should_cancel && should_cancel(cancel_arg)) {
        return ESP_ERR_INVALID_STATE;
    }

    enrichment_candidate_t candidate;
    esp_err_t ret = find_candidate(recording_dir, initial_stride, &candidate);
    if (ret != ESP_OK) {
        status_set_error(ret == ESP_ERR_NOT_FOUND ? "idle: no incomplete raw AVI" :
                         esp_err_to_name(ret));
        return ret;
    }

    status_begin(&candidate);
    ESP_LOGI(TAG, "starting %s method=%s stride=%" PRIu32 " frames=%" PRIu32,
             candidate.raw_name, candidate.method, candidate.next_stride, candidate.raw_info.frame_count);
    ret = run_pass(&candidate, min_score, jpeg_quality, should_cancel, cancel_arg);
    if (ret != ESP_ERR_TIMEOUT) {
        taskENTER_CRITICAL(&s_status_mux);
        if (strcmp(s_requested_raw_name, candidate.raw_name) == 0) {
            s_requested_raw_name[0] = '\0';
        }
        taskEXIT_CRITICAL(&s_status_mux);
    }
    if (ret == ESP_OK) {
        status_finish(false, candidate.next_stride, "ok");
        ESP_LOGI(TAG, "completed %s method=%s stride=%" PRIu32,
                 candidate.output_name, candidate.method, candidate.next_stride);
    } else if (ret == ESP_ERR_TIMEOUT) {
        status_finish(true, candidate.completed_stride, "cancelled by foreground activity");
        ESP_LOGI(TAG, "cancelled %s stride=%" PRIu32,
                 candidate.raw_name, candidate.next_stride);
    } else {
        char error[128];
        recording_enrichment_status_t detail = {0};
        recording_enrichment_get_status(&detail);
        if (detail.last_error[0] && strcmp(detail.last_error, "running") != 0) {
            snprintf(error, sizeof(error), "%s", detail.last_error);
        } else {
            snprintf(error, sizeof(error), "pass failed: %s", esp_err_to_name(ret));
        }
        status_finish(false, candidate.completed_stride, error);
        ESP_LOGE(TAG, "%s for %s stride=%" PRIu32,
                 error, candidate.raw_name, candidate.next_stride);
    }
    return ret;
}
