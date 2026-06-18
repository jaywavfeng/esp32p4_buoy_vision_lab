#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct avi_mjpeg_writer avi_mjpeg_writer_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t frame_count;
    uint32_t rate;
    uint32_t scale;
    uint64_t duration_ms;
    uint64_t file_bytes;
} avi_mjpeg_info_t;

esp_err_t avi_mjpeg_writer_open(avi_mjpeg_writer_t **out,
                                const char *part_path,
                                const char *final_path,
                                uint32_t width,
                                uint32_t height,
                                uint32_t fps);

esp_err_t avi_mjpeg_writer_add_frame(avi_mjpeg_writer_t *writer,
                                     const uint8_t *jpeg,
                                     size_t jpeg_size);

void avi_mjpeg_writer_set_duration_ms(avi_mjpeg_writer_t *writer, uint64_t duration_ms);

esp_err_t avi_mjpeg_writer_close(avi_mjpeg_writer_t *writer);

void avi_mjpeg_writer_abort(avi_mjpeg_writer_t *writer);

esp_err_t avi_mjpeg_recover_part(const char *part_path, const char *final_path);

esp_err_t avi_mjpeg_probe(const char *path, avi_mjpeg_info_t *info);

esp_err_t avi_mjpeg_retime_file(const char *path, uint64_t duration_ms);

#ifdef __cplusplus
}
#endif
