#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "tiny_cls_task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t source_w;
    uint32_t source_h;
    uint32_t model_input_w;
    uint32_t model_input_h;
    uint32_t class_id;
    uint32_t score;
    char label[TINY_CLS_LABEL_MAX_LEN];
    uint32_t top_k_count;
    tiny_cls_topk_t top_k[TINY_CLS_TOP_K];
    int64_t preprocess_ms;
    int64_t inference_ms;
    int64_t postprocess_ms;
    int64_t total_ms;
} tiny_cls_espdl_result_t;

bool tiny_cls_espdl_available(void);
uint32_t tiny_cls_espdl_model_bytes(void);
esp_err_t tiny_cls_espdl_classify_frame(const uint8_t *data,
                                        size_t data_len,
                                        uint32_t width,
                                        uint32_t height,
                                        uint32_t pixel_format,
                                        tiny_cls_espdl_result_t *out);
esp_err_t tiny_cls_espdl_classify_validation_jpeg(const uint8_t *jpg_data,
                                                  size_t jpg_len,
                                                  tiny_cls_espdl_result_t *out);

#ifdef __cplusplus
}
#endif
