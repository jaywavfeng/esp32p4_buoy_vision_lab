#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*recording_enrichment_cancel_cb_t)(void *arg);

typedef struct {
    bool enabled;
    bool running;
    bool cancelled;
    uint32_t pass_stride;
    uint32_t completed_stride;
    uint32_t frame_index;
    uint32_t frame_count;
    uint32_t inferred_frames;
    uint32_t output_frames;
    uint32_t inference_coverage_x1000;
    uint32_t passes_completed;
    char raw_name[96];
    char output_name[96];
    char last_error[128];
} recording_enrichment_status_t;

void recording_enrichment_init(bool enabled);

esp_err_t recording_enrichment_process_next(
    const char *recording_dir,
    uint32_t initial_stride,
    uint32_t min_score,
    uint8_t jpeg_quality,
    recording_enrichment_cancel_cb_t should_cancel,
    void *cancel_arg);

void recording_enrichment_get_status(recording_enrichment_status_t *status);

#ifdef __cplusplus
}
#endif
