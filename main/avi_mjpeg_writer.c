#include "avi_mjpeg_writer.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "avi_mjpeg";

#define AVI_HEADER_BYTES 224U
#define AVI_MOVI_FOURCC_OFFSET 220U
#define AVI_FRAME_LIMIT 4096U

typedef struct {
    uint32_t offset;
    uint32_t size;
} avi_index_entry_t;

struct avi_mjpeg_writer {
    FILE *file;
    char *part_path;
    char *final_path;
    avi_index_entry_t *index;
    uint32_t index_count;
    uint32_t index_capacity;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t largest_frame;
    uint64_t jpeg_bytes;
    uint64_t duration_ms;
};

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static uint32_t get_u32(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static uint32_t gcd_u32(uint32_t a, uint32_t b)
{
    while (b != 0) {
        uint32_t next = a % b;
        a = b;
        b = next;
    }
    return a ? a : 1U;
}

static void duration_timebase(uint32_t frame_count, uint64_t duration_ms,
                              uint32_t fallback_fps, uint32_t *scale,
                              uint32_t *rate, uint32_t *usec_per_frame)
{
    *scale = 1U;
    *rate = fallback_fps ? fallback_fps : 1U;
    *usec_per_frame = 1000000U / *rate;
    if (frame_count == 0 || duration_ms == 0 || duration_ms > UINT32_MAX) {
        return;
    }

    uint64_t raw_rate = (uint64_t)frame_count * 1000U;
    if (raw_rate == 0 || raw_rate > UINT32_MAX) {
        return;
    }
    uint32_t duration32 = (uint32_t)duration_ms;
    uint32_t rate32 = (uint32_t)raw_rate;
    uint32_t divisor = gcd_u32(duration32, rate32);
    *scale = duration32 / divisor;
    *rate = rate32 / divisor;
    uint64_t usec = (duration_ms * 1000U + frame_count / 2U) / frame_count;
    *usec_per_frame = usec > UINT32_MAX ? UINT32_MAX : (uint32_t)usec;
}

static esp_err_t write_exact(FILE *file, const void *data, size_t size)
{
    errno = 0;
    size_t written = fwrite(data, 1, size, file);
    if (written == size) {
        return ESP_OK;
    }
    ESP_LOGE(TAG,
             "short write: requested=%u written=%u errno=%d ferror=%d position=%ld",
             (unsigned)size, (unsigned)written, errno, ferror(file), ftell(file));
    return ESP_FAIL;
}

static esp_err_t sync_file(avi_mjpeg_writer_t *writer, const char *reason)
{
    if (!writer || !writer->file) {
        return ESP_ERR_INVALID_ARG;
    }
    errno = 0;
    if (fflush(writer->file) != 0) {
        ESP_LOGE(TAG, "%s fflush failed for %s at frame %" PRIu32 ": errno=%d",
                 reason, writer->part_path, writer->index_count, errno);
        return ESP_FAIL;
    }
    int fd = fileno(writer->file);
    errno = 0;
    if (fd < 0 || fsync(fd) != 0) {
        ESP_LOGE(TAG, "%s fsync failed for %s at frame %" PRIu32 ": fd=%d errno=%d",
                 reason, writer->part_path, writer->index_count, fd, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t patch_u32(FILE *file, long offset, uint32_t value)
{
    uint8_t bytes[4];
    put_u32(bytes, value);
    if (fseek(file, offset, SEEK_SET) != 0) {
        return ESP_FAIL;
    }
    return write_exact(file, bytes, sizeof(bytes));
}

static esp_err_t ensure_index_capacity(avi_mjpeg_writer_t *writer)
{
    if (writer->index_count < writer->index_capacity) {
        return ESP_OK;
    }
    uint32_t next = writer->index_capacity ? writer->index_capacity * 2U : 256U;
    if (next > AVI_FRAME_LIMIT) {
        next = AVI_FRAME_LIMIT;
    }
    if (next <= writer->index_capacity) {
        return ESP_ERR_NO_MEM;
    }
    avi_index_entry_t *grown = realloc(writer->index, next * sizeof(*grown));
    if (!grown) {
        return ESP_ERR_NO_MEM;
    }
    writer->index = grown;
    writer->index_capacity = next;
    return ESP_OK;
}

static esp_err_t write_header(FILE *file, uint32_t width, uint32_t height, uint32_t fps)
{
    uint8_t h[AVI_HEADER_BYTES] = {0};
    memcpy(h + 0, "RIFF", 4);
    memcpy(h + 8, "AVI ", 4);
    memcpy(h + 12, "LIST", 4);
    put_u32(h + 16, 192);
    memcpy(h + 20, "hdrl", 4);
    memcpy(h + 24, "avih", 4);
    put_u32(h + 28, 56);
    put_u32(h + 32, 1000000U / fps);
    put_u32(h + 44, 0x10);
    put_u32(h + 56, 1);
    put_u32(h + 64, width);
    put_u32(h + 68, height);

    memcpy(h + 88, "LIST", 4);
    put_u32(h + 92, 116);
    memcpy(h + 96, "strl", 4);
    memcpy(h + 100, "strh", 4);
    put_u32(h + 104, 56);
    memcpy(h + 108, "vids", 4);
    memcpy(h + 112, "MJPG", 4);
    put_u32(h + 128, 1);
    put_u32(h + 132, fps);
    put_u32(h + 148, 0xffffffffU);
    put_u16(h + 160, (uint16_t)(width > UINT16_MAX ? UINT16_MAX : width));
    put_u16(h + 162, (uint16_t)(height > UINT16_MAX ? UINT16_MAX : height));

    memcpy(h + 164, "strf", 4);
    put_u32(h + 168, 40);
    put_u32(h + 172, 40);
    put_u32(h + 176, width);
    put_u32(h + 180, height);
    put_u16(h + 184, 1);
    put_u16(h + 186, 24);
    memcpy(h + 188, "MJPG", 4);
    uint64_t image_size = (uint64_t)width * height * 3U;
    put_u32(h + 192, image_size > UINT32_MAX ? UINT32_MAX : (uint32_t)image_size);

    memcpy(h + 212, "LIST", 4);
    memcpy(h + 220, "movi", 4);
    return write_exact(file, h, sizeof(h));
}

static esp_err_t finish_file(avi_mjpeg_writer_t *writer)
{
    if (!writer || !writer->file || writer->index_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    long idx_offset = ftell(writer->file);
    if (idx_offset < 0 || (uint64_t)idx_offset > (uint64_t)UINT32_MAX - 8U) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t word[8];
    memcpy(word, "idx1", 4);
    put_u32(word + 4, writer->index_count * 16U);
    ESP_RETURN_ON_ERROR(write_exact(writer->file, word, sizeof(word)), TAG, "write idx header");

    for (uint32_t i = 0; i < writer->index_count; i++) {
        uint8_t entry[16];
        memcpy(entry, "00dc", 4);
        put_u32(entry + 4, 0x10);
        put_u32(entry + 8, writer->index[i].offset);
        put_u32(entry + 12, writer->index[i].size);
        ESP_RETURN_ON_ERROR(write_exact(writer->file, entry, sizeof(entry)), TAG, "write idx entry");
    }

    long end = ftell(writer->file);
    if (end < 0 || (uint64_t)end > (uint64_t)UINT32_MAX - 8U) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t scale = 1U;
    uint32_t rate = writer->fps;
    uint32_t usec_per_frame = 1000000U / writer->fps;
    duration_timebase(writer->index_count, writer->duration_ms, writer->fps,
                      &scale, &rate, &usec_per_frame);
    uint32_t duration_sec = writer->duration_ms ?
        (uint32_t)((writer->duration_ms + 999U) / 1000U) :
        (writer->index_count + writer->fps - 1U) / writer->fps;
    uint32_t max_bytes_per_sec = duration_sec ?
        (uint32_t)((writer->jpeg_bytes + duration_sec - 1U) / duration_sec) : writer->largest_frame;

    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 4, (uint32_t)end - 8U), TAG, "patch riff");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 32, usec_per_frame), TAG, "patch frame period");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 36, max_bytes_per_sec), TAG, "patch rate");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 48, writer->index_count), TAG, "patch frames");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 60, writer->largest_frame), TAG, "patch avih buffer");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 128, scale), TAG, "patch stream scale");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 132, rate), TAG, "patch stream rate");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 140, writer->index_count), TAG, "patch stream length");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 144, writer->largest_frame), TAG, "patch stream buffer");
    ESP_RETURN_ON_ERROR(patch_u32(writer->file, 216,
                                  (uint32_t)idx_offset - AVI_MOVI_FOURCC_OFFSET),
                        TAG, "patch movi");

    return sync_file(writer, "final");
}

static void free_writer(avi_mjpeg_writer_t *writer)
{
    if (!writer) {
        return;
    }
    free(writer->index);
    free(writer->part_path);
    free(writer->final_path);
    free(writer);
}

esp_err_t avi_mjpeg_writer_open(avi_mjpeg_writer_t **out,
                                const char *part_path,
                                const char *final_path,
                                uint32_t width,
                                uint32_t height,
                                uint32_t fps)
{
    if (!out || !part_path || !final_path || !width || !height || !fps) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    avi_mjpeg_writer_t *writer = calloc(1, sizeof(*writer));
    if (!writer) {
        return ESP_ERR_NO_MEM;
    }
    writer->part_path = strdup(part_path);
    writer->final_path = strdup(final_path);
    writer->width = width;
    writer->height = height;
    writer->fps = fps;
    if (!writer->part_path || !writer->final_path) {
        free_writer(writer);
        return ESP_ERR_NO_MEM;
    }

    writer->file = fopen(part_path, "wb+");
    if (!writer->file) {
        free_writer(writer);
        return ESP_FAIL;
    }
    esp_err_t ret = write_header(writer->file, width, height, fps);
    if (ret != ESP_OK) {
        fclose(writer->file);
        writer->file = NULL;
        unlink(part_path);
        free_writer(writer);
        return ret;
    }
    ret = sync_file(writer, "header");
    if (ret != ESP_OK) {
        fclose(writer->file);
        writer->file = NULL;
        unlink(part_path);
        free_writer(writer);
        return ret;
    }
    *out = writer;
    return ESP_OK;
}

esp_err_t avi_mjpeg_writer_add_frame(avi_mjpeg_writer_t *writer,
                                     const uint8_t *jpeg,
                                     size_t jpeg_size)
{
    if (!writer || !writer->file || !jpeg || jpeg_size == 0 || jpeg_size > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(ensure_index_capacity(writer), TAG, "grow index");

    long chunk_offset = ftell(writer->file);
    if (chunk_offset < (long)AVI_HEADER_BYTES ||
        (uint64_t)chunk_offset + (uint64_t)jpeg_size + 9U > UINT32_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t chunk[8];
    memcpy(chunk, "00dc", 4);
    put_u32(chunk + 4, (uint32_t)jpeg_size);
    ESP_RETURN_ON_ERROR(write_exact(writer->file, chunk, sizeof(chunk)), TAG, "write frame header");
    ESP_RETURN_ON_ERROR(write_exact(writer->file, jpeg, jpeg_size), TAG, "write frame");
    if (jpeg_size & 1U) {
        const uint8_t pad = 0;
        ESP_RETURN_ON_ERROR(write_exact(writer->file, &pad, 1), TAG, "write frame pad");
    }

    writer->index[writer->index_count].offset = (uint32_t)chunk_offset - AVI_MOVI_FOURCC_OFFSET;
    writer->index[writer->index_count].size = (uint32_t)jpeg_size;
    writer->index_count++;
    writer->jpeg_bytes += jpeg_size;
    if (jpeg_size > writer->largest_frame) {
        writer->largest_frame = (uint32_t)jpeg_size;
    }
    if ((writer->index_count % 16U) == 0U) {
        ESP_RETURN_ON_ERROR(sync_file(writer, "periodic"), TAG, "sync AVI data");
    }
    return ESP_OK;
}

void avi_mjpeg_writer_set_duration_ms(avi_mjpeg_writer_t *writer, uint64_t duration_ms)
{
    if (writer) {
        writer->duration_ms = duration_ms;
    }
}

esp_err_t avi_mjpeg_writer_close(avi_mjpeg_writer_t *writer)
{
    if (!writer) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = finish_file(writer);
    if (writer->file) {
        if (fclose(writer->file) != 0 && ret == ESP_OK) {
            ESP_LOGE(TAG, "fclose failed for %s: errno=%d", writer->part_path, errno);
            ret = ESP_FAIL;
        }
        writer->file = NULL;
    }
    if (ret == ESP_OK) {
        unlink(writer->final_path);
        if (rename(writer->part_path, writer->final_path) != 0) {
            ESP_LOGE(TAG, "rename %s -> %s failed: errno=%d",
                     writer->part_path, writer->final_path, errno);
            ret = ESP_FAIL;
        }
    }
    free_writer(writer);
    return ret;
}

void avi_mjpeg_writer_abort(avi_mjpeg_writer_t *writer)
{
    if (!writer) {
        return;
    }
    if (writer->file) {
        fflush(writer->file);
        fclose(writer->file);
        writer->file = NULL;
    }
    free_writer(writer);
}

esp_err_t avi_mjpeg_recover_part(const char *part_path, const char *final_path)
{
    if (!part_path || !final_path) {
        return ESP_ERR_INVALID_ARG;
    }
    FILE *file = fopen(part_path, "rb+");
    if (!file) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t header[AVI_HEADER_BYTES] = {0};
    size_t header_bytes = fread(header, 1, sizeof(header), file);
    if (header_bytes != sizeof(header) ||
        memcmp(header, "RIFF", 4) != 0 ||
        memcmp(header + 8, "AVI ", 4) != 0 ||
        memcmp(header + AVI_MOVI_FOURCC_OFFSET, "movi", 4) != 0) {
        ESP_LOGW(TAG,
                 "invalid AVI part %s: header_bytes=%u head=%02x%02x%02x%02x",
                 part_path, (unsigned)header_bytes,
                 header[0], header[1], header[2], header[3]);
        fclose(file);
        return ESP_ERR_INVALID_RESPONSE;
    }

    avi_mjpeg_writer_t *writer = calloc(1, sizeof(*writer));
    if (!writer) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    writer->file = file;
    writer->part_path = strdup(part_path);
    writer->final_path = strdup(final_path);
    writer->width = get_u32(header + 64);
    writer->height = get_u32(header + 68);
    writer->fps = get_u32(header + 132);
    if (!writer->part_path || !writer->final_path || !writer->fps) {
        avi_mjpeg_writer_abort(writer);
        return ESP_ERR_INVALID_RESPONSE;
    }

    long valid_end = AVI_HEADER_BYTES;
    while (true) {
        uint8_t chunk[8];
        if (fseek(file, valid_end, SEEK_SET) != 0 || fread(chunk, 1, sizeof(chunk), file) != sizeof(chunk)) {
            break;
        }
        if (memcmp(chunk, "00dc", 4) != 0) {
            break;
        }
        uint32_t size = get_u32(chunk + 4);
        if (!size || size > 2U * 1024U * 1024U || ensure_index_capacity(writer) != ESP_OK) {
            break;
        }
        long next = valid_end + 8L + (long)size + (long)(size & 1U);
        if (fseek(file, next - 1L, SEEK_SET) != 0 || fgetc(file) == EOF) {
            break;
        }
        writer->index[writer->index_count].offset = (uint32_t)valid_end - AVI_MOVI_FOURCC_OFFSET;
        writer->index[writer->index_count].size = size;
        writer->index_count++;
        writer->jpeg_bytes += size;
        if (size > writer->largest_frame) {
            writer->largest_frame = size;
        }
        valid_end = next;
    }

    if (writer->index_count == 0 || ftruncate(fileno(file), valid_end) != 0 ||
        fseek(file, valid_end, SEEK_SET) != 0) {
        avi_mjpeg_writer_abort(writer);
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "recovering %s with %" PRIu32 " complete frames",
             part_path, writer->index_count);
    return avi_mjpeg_writer_close(writer);
}

esp_err_t avi_mjpeg_probe(const char *path, avi_mjpeg_info_t *info)
{
    if (!path || !info) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = fopen(path, "rb");
    if (!file) {
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t header[AVI_HEADER_BYTES] = {0};
    size_t header_bytes = fread(header, 1, sizeof(header), file);
    fclose(file);
    if (header_bytes != sizeof(header) ||
        memcmp(header, "RIFF", 4) != 0 ||
        memcmp(header + 8, "AVI ", 4) != 0 ||
        memcmp(header + AVI_MOVI_FOURCC_OFFSET, "movi", 4) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    struct stat st = {0};
    if (stat(path, &st) != 0) {
        return ESP_FAIL;
    }
    memset(info, 0, sizeof(*info));
    info->width = get_u32(header + 64);
    info->height = get_u32(header + 68);
    info->frame_count = get_u32(header + 48);
    info->scale = get_u32(header + 128);
    info->rate = get_u32(header + 132);
    info->file_bytes = (uint64_t)st.st_size;
    if (!info->width || !info->height || !info->frame_count ||
        !info->scale || !info->rate) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    info->duration_ms =
        ((uint64_t)info->frame_count * info->scale * 1000U + info->rate / 2U) / info->rate;
    return ESP_OK;
}

esp_err_t avi_mjpeg_retime_file(const char *path, uint64_t duration_ms)
{
    avi_mjpeg_info_t info = {0};
    ESP_RETURN_ON_ERROR(avi_mjpeg_probe(path, &info), TAG, "probe AVI before retime");
    if (duration_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t scale = 1U;
    uint32_t rate = 1U;
    uint32_t usec_per_frame = 1000000U;
    duration_timebase(info.frame_count, duration_ms, 1U, &scale, &rate, &usec_per_frame);

    FILE *file = fopen(path, "rb+");
    if (!file) {
        return ESP_FAIL;
    }
    esp_err_t ret = patch_u32(file, 32, usec_per_frame);
    if (ret == ESP_OK) {
        ret = patch_u32(file, 128, scale);
    }
    if (ret == ESP_OK) {
        ret = patch_u32(file, 132, rate);
    }
    if (ret == ESP_OK && (fflush(file) != 0 || fsync(fileno(file)) != 0)) {
        ret = ESP_FAIL;
    }
    fclose(file);
    return ret;
}
