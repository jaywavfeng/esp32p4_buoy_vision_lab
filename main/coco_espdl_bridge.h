#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COCO_ESPDL_MAX_DETECTIONS 8
#define COCO_ESPDL_CLASS_COUNT 80
#define COCO_ESPDL_NMS_THRESHOLD_X100 70

typedef struct {
    uint32_t class_id;
    uint32_t score;
    char label[24];
    int32_t x;
    int32_t y;
    int32_t w;
    int32_t h;
} coco_espdl_detection_t;

typedef struct {
    bool has_candidate;
    uint32_t raw_candidate_count;
    uint32_t detection_count;
    coco_espdl_detection_t detections[COCO_ESPDL_MAX_DETECTIONS];
    uint32_t class_id;
    uint32_t score;
    char label[24];
    int32_t x;
    int32_t y;
    int32_t w;
    int32_t h;
    uint32_t source_w;
    uint32_t source_h;
    uint32_t model_input_w;
    uint32_t model_input_h;
    int64_t preprocess_ms;
    int64_t inference_ms;
    int64_t postprocess_ms;
    int64_t total_ms;
} coco_espdl_result_t;

bool coco_espdl_available(void);
uint32_t coco_espdl_model_bytes(void);
esp_err_t coco_espdl_detect_jpeg(const uint8_t *jpg_data,
                                 size_t jpg_len,
                                 coco_espdl_result_t *out);
esp_err_t coco_espdl_detect_and_annotate_jpeg(const uint8_t *jpg_data,
                                              size_t jpg_len,
                                              uint32_t min_score,
                                              uint8_t jpeg_quality,
                                              coco_espdl_result_t *out,
                                              uint8_t **annotated_jpeg,
                                              size_t *annotated_len);
void coco_espdl_free_jpeg(uint8_t *jpeg);

#ifdef __cplusplus
}
#endif
