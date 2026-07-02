#pragma once

#include <stdint.h>

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_APP_TINY_CLS_INPUT_SIZE
#define CONFIG_APP_TINY_CLS_INPUT_SIZE 192
#endif

#define TINY_CLS_MODEL_NAME "tiny-cnn-cls-192-6cls-p4"
#define TINY_CLS_INPUT_W CONFIG_APP_TINY_CLS_INPUT_SIZE
#define TINY_CLS_INPUT_H CONFIG_APP_TINY_CLS_INPUT_SIZE
#define TINY_CLS_INPUT_C 3
#define TINY_CLS_TOP_K 3
#define TINY_CLS_LABEL_MAX_LEN 24

typedef enum {
    TINY_CLS_UNKNOWN = 0,
    TINY_CLS_PLASTIC_BOTTLE = 1,
    TINY_CLS_FOAM = 2,
    TINY_CLS_BUOY = 3,
    TINY_CLS_NET = 4,
    TINY_CLS_SHIP_PART = 5,
    TINY_CLS_CLASS_COUNT = 6,
} tiny_cls_class_id_t;

static const char *const TINY_CLS_LABELS[TINY_CLS_CLASS_COUNT] = {
    "unknown",
    "plastic_bottle",
    "foam",
    "buoy",
    "net",
    "ship_part",
};

typedef struct {
    uint32_t class_id;
    uint32_t score;
    char label[TINY_CLS_LABEL_MAX_LEN];
} tiny_cls_topk_t;

typedef struct {
    uint32_t class_id;
    uint32_t score;
    char label[TINY_CLS_LABEL_MAX_LEN];
    uint32_t top_k_count;
    tiny_cls_topk_t top_k[TINY_CLS_TOP_K];
    int64_t inference_ms;
    int64_t analysis_ms;
} tiny_cls_task_result_t;

#ifdef __cplusplus
}
#endif
