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
    char meta_path[512];
    avi_mjpeg_info_t raw_info;
    uint32_t completed_stride;
    uint32_t next_stride;
    int64_t sort_time;
} enrichment_candidate_t;

static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static recording_enrichment_status_t s_status;

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

static uint32_t json_u32_from_line(const char *line, const char *key)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *value = strstr(line, needle);
    return value ? (uint32_t)strtoul(value + strlen(needle), NULL, 10) : 0;
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
    return have_line ? json_u32_from_line(line, "pass_stride") : 0;
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
                            enrichment_candidate_t *candidate)
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
    strlcpy(candidate->meta_path, candidate->output_path, sizeof(candidate->meta_path));
    char *ext = strrchr(candidate->meta_path, '.');
    if (!ext) {
        return false;
    }
    strlcpy(ext, ".jsonl", (size_t)(candidate->meta_path + sizeof(candidate->meta_path) - ext));
    if (avi_mjpeg_probe(candidate->raw_path, &candidate->raw_info) != ESP_OK) {
        return false;
    }
    candidate->completed_stride = read_completed_stride(candidate);
    candidate->next_stride = next_stride(candidate->completed_stride, initial_stride);
    candidate->sort_time = time_from_name(raw_name);
    return candidate->next_stride > 0;
}

static esp_err_t find_candidate(const char *dir, uint32_t initial_stride,
                                enrichment_candidate_t *selected)
{
    DIR *handle = opendir(dir);
    if (!handle) {
        return ESP_ERR_NOT_FOUND;
    }
    bool found = false;
    struct dirent *entry;
    while ((entry = readdir(handle)) != NULL) {
        enrichment_candidate_t candidate;
        if (!build_candidate(dir, entry->d_name, initial_stride, &candidate)) {
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

static void filter_result(coco_espdl_result_t *result, uint32_t min_score)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < result->detection_count; i++) {
        if (result->detections[i].score >= min_score) {
            result->detections[count++] = result->detections[i];
        }
    }
    result->detection_count = count;
    result->has_candidate = count > 0;
    if (count > 0) {
        const coco_espdl_detection_t *best = &result->detections[0];
        result->class_id = best->class_id;
        result->score = best->score;
        strlcpy(result->label, best->label, sizeof(result->label));
        result->x = best->x;
        result->y = best->y;
        result->w = best->w;
        result->h = best->h;
    } else {
        result->score = 0;
        strlcpy(result->label, "no-object", sizeof(result->label));
    }
}

static void detections_to_json(char *json, size_t size, const coco_espdl_result_t *result)
{
    size_t off = 0;
    off += snprintf(json + off, size - off, "[");
    for (uint32_t i = 0; i < result->detection_count && off < size; i++) {
        const coco_espdl_detection_t *det = &result->detections[i];
        off += snprintf(json + off, size - off,
                        "%s{\"class_id\":%" PRIu32 ",\"label\":\"%s\","
                        "\"score\":%" PRIu32 ",\"x\":%" PRId32
                        ",\"y\":%" PRId32 ",\"w\":%" PRId32 ",\"h\":%" PRId32 "}",
                        i ? "," : "", det->class_id, det->label, det->score,
                        det->x, det->y, det->w, det->h);
    }
    if (off < size) {
        snprintf(json + off, size - off, "]");
    } else {
        json[size - 1] = '\0';
    }
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
    ESP_RETURN_ON_ERROR(avi_mjpeg_reader_open(&reader, candidate->raw_path, &info),
                        TAG, "open raw AVI");
    uint32_t fps = info.scale ? (info.rate + info.scale / 2U) / info.scale : 0;
    if (fps == 0) {
        fps = 1;
    }
    avi_mjpeg_writer_t *writer = NULL;
    esp_err_t ret = avi_mjpeg_writer_open(&writer, video_part, video_new,
                                          info.width, info.height, fps);
    if (ret != ESP_OK) {
        avi_mjpeg_reader_close(reader);
        return ret;
    }
    FILE *meta = fopen(meta_part, "w");
    if (!meta) {
        avi_mjpeg_writer_abort(writer);
        avi_mjpeg_reader_close(reader);
        unlink(video_part);
        return ESP_FAIL;
    }

    coco_espdl_result_t last_result = {0};
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
            break;
        }

        bool key_frame = ((frame_index - 1U) % candidate->next_stride) == 0U;
        if (key_frame) {
            memset(&last_result, 0, sizeof(last_result));
            ret = coco_espdl_detect_jpeg_background(raw_jpeg, raw_size, &last_result);
            if (ret != ESP_OK) {
                break;
            }
            filter_result(&last_result, min_score);
            source_frame_index = frame_index;
            inferred_frames++;
            if (should_cancel && should_cancel(cancel_arg)) {
                cancelled = true;
                ret = ESP_ERR_TIMEOUT;
                break;
            }
        }

        uint8_t *annotated = NULL;
        size_t annotated_size = 0;
        const uint8_t *output_jpeg = raw_jpeg;
        size_t output_size = raw_size;
        int64_t annotate_start = esp_timer_get_time();
        if (last_result.detection_count > 0) {
            ret = coco_espdl_annotate_jpeg(raw_jpeg, raw_size, &last_result,
                                           min_score, jpeg_quality,
                                           &annotated, &annotated_size);
            if (ret != ESP_OK) {
                break;
            }
            output_jpeg = annotated;
            output_size = annotated_size;
        }
        int64_t annotate_ms = (esp_timer_get_time() - annotate_start) / 1000;
        ret = avi_mjpeg_writer_add_frame(writer, output_jpeg, output_size);
        if (ret != ESP_OK) {
            coco_espdl_free_jpeg(annotated);
            break;
        }

        char detections[2048];
        detections_to_json(detections, sizeof(detections), &last_result);
        const char *source = key_frame ?
            (last_result.detection_count > 0 ? "inference" : "inference-no-object") :
            (last_result.detection_count > 0 ? "propagated" : "raw-no-object");
        int64_t start_ms = candidate->sort_time;
        uint64_t offset_ms = info.frame_count > 1 ?
            ((uint64_t)(frame_index - 1U) * info.duration_ms) / (info.frame_count - 1U) : 0;
        if (fprintf(meta,
                    "{\"index_version\":4,\"segment\":\"%s\",\"kind\":\"annotated\","
                    "\"frame_index\":%" PRIu32 ",\"time_ms\":%" PRId64
                    ",\"jpeg_bytes\":%u,\"width\":%" PRIu32 ",\"height\":%" PRIu32
                    ",\"model\":\"coco-yolo11n-320-s8-v3-p4\",\"box_min_score\":%" PRIu32
                    ",\"best_score\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
                    ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64
                    ",\"detection_count\":%" PRIu32 ",\"detections\":%s"
                    ",\"result_source\":\"%s\",\"source_frame_index\":%" PRIu32
                    ",\"inference_age_frames\":%" PRIu32 ",\"pass_stride\":%" PRIu32 "}\n",
                    candidate->output_name, frame_index, start_ms + (int64_t)offset_ms,
                    (unsigned)output_size, info.width, info.height, min_score,
                    last_result.score, last_result.raw_candidate_count,
                    key_frame ? last_result.inference_ms : 0,
                    key_frame ? last_result.total_ms : annotate_ms,
                    last_result.detection_count, detections, source, source_frame_index,
                    source_frame_index ? frame_index - source_frame_index : 0,
                    candidate->next_stride) < 0) {
            coco_espdl_free_jpeg(annotated);
            ret = ESP_FAIL;
            break;
        }
        coco_espdl_free_jpeg(annotated);
        output_frames++;
        status_progress(frame_index, inferred_frames, output_frames, info.frame_count);
        if ((output_frames % 32U) == 0U && fflush(meta) != 0) {
            ret = ESP_FAIL;
            break;
        }
    }

    avi_mjpeg_reader_close(reader);
    if (ret == ESP_OK && output_frames == info.frame_count) {
        avi_mjpeg_writer_set_duration_ms(writer, info.duration_ms);
        ret = avi_mjpeg_writer_close(writer);
        writer = NULL;
        if (ret == ESP_OK && (fflush(meta) != 0 || fsync(fileno(meta)) != 0)) {
            ret = ESP_FAIL;
        }
    } else if (ret == ESP_OK) {
        ret = ESP_ERR_INVALID_SIZE;
    }
    if (writer) {
        avi_mjpeg_writer_abort(writer);
    }
    if (fclose(meta) != 0 && ret == ESP_OK) {
        ret = ESP_FAIL;
    }

    if (ret == ESP_OK && rename(meta_part, meta_new) != 0) {
        ret = ESP_FAIL;
    }
    avi_mjpeg_info_t output_info = {0};
    if (ret == ESP_OK &&
        (avi_mjpeg_probe(video_new, &output_info) != ESP_OK ||
         output_info.frame_count != info.frame_count ||
         output_info.duration_ms != info.duration_ms)) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (ret == ESP_OK) {
        ret = replace_pair(video_new, candidate->output_path, meta_new, candidate->meta_path);
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
    ESP_LOGI(TAG, "starting %s stride=%" PRIu32 " frames=%" PRIu32,
             candidate.raw_name, candidate.next_stride, candidate.raw_info.frame_count);
    ret = run_pass(&candidate, min_score, jpeg_quality, should_cancel, cancel_arg);
    if (ret == ESP_OK) {
        status_finish(false, candidate.next_stride, "ok");
        ESP_LOGI(TAG, "completed %s stride=%" PRIu32,
                 candidate.output_name, candidate.next_stride);
    } else if (ret == ESP_ERR_TIMEOUT) {
        status_finish(true, candidate.completed_stride, "cancelled by foreground activity");
        ESP_LOGI(TAG, "cancelled %s stride=%" PRIu32,
                 candidate.raw_name, candidate.next_stride);
    } else {
        char error[128];
        snprintf(error, sizeof(error), "pass failed: %s", esp_err_to_name(ret));
        status_finish(false, candidate.completed_stride, error);
        ESP_LOGE(TAG, "%s for %s stride=%" PRIu32,
                 error, candidate.raw_name, candidate.next_stride);
    }
    return ret;
}
