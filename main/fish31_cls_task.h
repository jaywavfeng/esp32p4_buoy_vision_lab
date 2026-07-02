#pragma once

#include <stdint.h>

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_APP_FISH31_INPUT_SIZE
#define CONFIG_APP_FISH31_INPUT_SIZE 224
#endif

#define FISH31_CLS_MODEL_NAME "fish31-mbv3s075-224-p4"
#define FISH31_INPUT_W CONFIG_APP_FISH31_INPUT_SIZE
#define FISH31_INPUT_H CONFIG_APP_FISH31_INPUT_SIZE
#define FISH31_INPUT_C 3
#define FISH31_TOP_K 3
#define FISH31_LABEL_MAX_LEN 32

typedef enum {
    FISH31_FISH_01 = 0,
    FISH31_FISH_02,
    FISH31_FISH_03,
    FISH31_FISH_04,
    FISH31_FISH_05,
    FISH31_FISH_06,
    FISH31_FISH_07,
    FISH31_FISH_08,
    FISH31_FISH_09,
    FISH31_FISH_10,
    FISH31_FISH_11,
    FISH31_FISH_12,
    FISH31_FISH_13,
    FISH31_FISH_14,
    FISH31_FISH_15,
    FISH31_FISH_16,
    FISH31_FISH_17,
    FISH31_FISH_18,
    FISH31_FISH_19,
    FISH31_FISH_20,
    FISH31_FISH_21,
    FISH31_FISH_22,
    FISH31_FISH_23,
    FISH31_SAND_SEABED,
    FISH31_RUBBLE_ROCK_SEABED,
    FISH31_LIVE_CORAL,
    FISH31_DEAD_BLEACHED_CORAL,
    FISH31_BENTHIC_INVERTEBRATE,
    FISH31_SEAGRASS,
    FISH31_ALGAE_SUBSTRATE,
    FISH31_COMPLEX_UNDERWATER_BG,
    FISH31_CLASS_COUNT,
} fish31_class_id_t;

static const char *const FISH31_LABELS[FISH31_CLASS_COUNT] = {
    "fish_01",
    "fish_02",
    "fish_03",
    "fish_04",
    "fish_05",
    "fish_06",
    "fish_07",
    "fish_08",
    "fish_09",
    "fish_10",
    "fish_11",
    "fish_12",
    "fish_13",
    "fish_14",
    "fish_15",
    "fish_16",
    "fish_17",
    "fish_18",
    "fish_19",
    "fish_20",
    "fish_21",
    "fish_22",
    "fish_23",
    "sand_seabed",
    "rubble_rock_seabed",
    "live_coral",
    "dead_bleached_coral",
    "benthic_invertebrate",
    "seagrass",
    "algae_substrate",
    "complex_underwater_bg",
};

typedef struct {
    uint32_t class_id;
    uint32_t score;
    char label[FISH31_LABEL_MAX_LEN];
} fish31_topk_t;

typedef struct {
    uint32_t class_id;
    uint32_t score;
    char label[FISH31_LABEL_MAX_LEN];
    uint32_t top_k_count;
    fish31_topk_t top_k[FISH31_TOP_K];
    int64_t inference_ms;
    int64_t analysis_ms;
} fish31_task_result_t;

#ifdef __cplusplus
}
#endif
