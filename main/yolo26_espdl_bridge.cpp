#include "yolo26_espdl_bridge.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <inttypes.h>
#include <new>
#include <vector>

#include "dl_model_base.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "yolo26.hpp"

static const char *TAG = "yolo26_bridge";

/*
 * Legacy Coke/Sprite YOLO26n raw-head bridge.
 *
 * This bridge is disabled in the customer firmware unless
 * CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE is enabled. The current customer path
 * uses Fish31, TinyCNN, and COCO only.
 */
extern const uint8_t yolo26_model_start[] asm("_binary_yolo26_coke_sprite_raw_heads_416_allint8_p4_espdl_start");
extern const uint8_t yolo26_model_end[] asm("_binary_yolo26_coke_sprite_raw_heads_416_allint8_p4_espdl_end");

static const char *coke_sprite_classes[] = {
    "coke",
    "sprite",
};
static constexpr int kCokeSpriteClassCount = sizeof(coke_sprite_classes) / sizeof(coke_sprite_classes[0]);
static constexpr float kFallbackNmsThreshold = 0.70f;

static dl::Model *s_model;
static YOLO26 *s_processor;
static SemaphoreHandle_t s_lock;
static uint32_t s_model_input_w = 416;
static uint32_t s_model_input_h = 416;

static bool ensure_lock(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }
    return s_lock != nullptr;
}

static void read_model_input_shape(void)
{
    auto inputs = s_model->get_inputs();
    if (inputs.empty()) {
        return;
    }

    /*
     * ESP-DL image inputs are normally NHWC: [1, H, W, C]. Keep this conservative
     * read so UI box mapping does not become wildly wrong if an old model is used.
     */
    dl::TensorBase *input = inputs.begin()->second;
    if (input->shape.size() >= 4) {
        s_model_input_h = (uint32_t)input->shape[1];
        s_model_input_w = (uint32_t)input->shape[2];
    }
}

static esp_err_t ensure_model(void)
{
    if (s_model && s_processor) {
        return ESP_OK;
    }
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_model) {
        /*
         * Keep model parameters in flash rodata instead of PSRAM. This saves
         * memory on 32 MB PSRAM boards and matches the ESP-DL YOLO examples.
         */
        s_model = new (std::nothrow) dl::Model((const char *)yolo26_model_start,
                                               fbs::MODEL_LOCATION_IN_FLASH_RODATA,
                                               0,
                                               dl::MEMORY_MANAGER_GREEDY,
                                               nullptr,
                                               false);
        if (!s_model) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate dl::Model");
            return ESP_ERR_NO_MEM;
        }
        read_model_input_shape();
        ESP_LOGI(TAG, "YOLO26 model loaded, input=%" PRIu32 "x%" PRIu32 ", bytes=%" PRIu32,
                 s_model_input_w, s_model_input_h, yolo26_espdl_model_bytes());
    }

    if (!s_processor) {
        /*
         * The component keeps its internal confidence threshold at 0.25. The Web
         * box_min_score setting can still apply an extra display filter.
         */
        s_processor = new (std::nothrow) YOLO26(s_model, YOLO_TARGET_K, YOLO_CONF_THRESH, coke_sprite_classes);
        if (!s_processor) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate YOLO26 processor");
            return ESP_ERR_NO_MEM;
        }
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool yolo26_espdl_available(void)
{
#if CONFIG_APP_YOLO26_BOARD_ENABLE
    return (uintptr_t)yolo26_model_end > (uintptr_t)yolo26_model_start;
#else
    return false;
#endif
}

uint32_t yolo26_espdl_model_bytes(void)
{
    return (uint32_t)(yolo26_model_end - yolo26_model_start);
}

static void map_letterbox_box_to_detection(const dl::image::img_t &img,
                                           const std::vector<int> &box,
                                           yolo26_espdl_detection_t *out)
{
    float scale_x = (float)s_model_input_w / (float)img.width;
    float scale_y = (float)s_model_input_h / (float)img.height;
    float scale = std::min(scale_x, scale_y);
    float resized_w = (float)img.width * scale;
    float resized_h = (float)img.height * scale;
    float pad_x = ((float)s_model_input_w - resized_w) * 0.5f;
    float pad_y = ((float)s_model_input_h - resized_h) * 0.5f;

    float x1 = ((float)box[0] - pad_x) / scale;
    float y1 = ((float)box[1] - pad_y) / scale;
    float x2 = ((float)box[2] - pad_x) / scale;
    float y2 = ((float)box[3] - pad_y) / scale;

    x1 = std::max(0.0f, std::min(x1, (float)img.width - 1.0f));
    y1 = std::max(0.0f, std::min(y1, (float)img.height - 1.0f));
    x2 = std::max(0.0f, std::min(x2, (float)img.width - 1.0f));
    y2 = std::max(0.0f, std::min(y2, (float)img.height - 1.0f));

    out->x = (int32_t)std::lround(x1);
    out->y = (int32_t)std::lround(y1);
    out->w = (int32_t)std::lround(std::max(0.0f, x2 - x1));
    out->h = (int32_t)std::lround(std::max(0.0f, y2 - y1));
}

static float box_iou(const yolo26_espdl_detection_t &a, const yolo26_espdl_detection_t &b)
{
    int32_t ax2 = a.x + a.w;
    int32_t ay2 = a.y + a.h;
    int32_t bx2 = b.x + b.w;
    int32_t by2 = b.y + b.h;
    int32_t ix1 = std::max(a.x, b.x);
    int32_t iy1 = std::max(a.y, b.y);
    int32_t ix2 = std::min(ax2, bx2);
    int32_t iy2 = std::min(ay2, by2);
    int32_t iw = std::max<int32_t>(0, ix2 - ix1);
    int32_t ih = std::max<int32_t>(0, iy2 - iy1);
    float inter = (float)iw * (float)ih;
    float area_a = (float)std::max<int32_t>(0, a.w) * (float)std::max<int32_t>(0, a.h);
    float area_b = (float)std::max<int32_t>(0, b.w) * (float)std::max<int32_t>(0, b.h);
    float denom = area_a + area_b - inter;
    return denom > 0.0f ? inter / denom : 0.0f;
}

static void select_top_detections(std::vector<yolo26_espdl_detection_t> &candidates,
                                  yolo26_espdl_result_t *out)
{
    std::sort(candidates.begin(), candidates.end(),
              [](const yolo26_espdl_detection_t &a, const yolo26_espdl_detection_t &b) {
                  return a.score > b.score;
              });

    for (const auto &cand : candidates) {
        if (cand.w <= 0 || cand.h <= 0) {
            continue;
        }
        bool suppressed = false;
        for (uint32_t i = 0; i < out->detection_count; i++) {
            if (cand.class_id == out->detections[i].class_id &&
                box_iou(cand, out->detections[i]) > kFallbackNmsThreshold) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        out->detections[out->detection_count++] = cand;
        if (out->detection_count >= YOLO26_ESPDL_MAX_DETECTIONS) {
            break;
        }
    }

    if (out->detection_count > 0) {
        const yolo26_espdl_detection_t &best = out->detections[0];
        out->has_candidate = true;
        out->class_id = best.class_id;
        out->score = best.score;
        strlcpy(out->label, best.label, sizeof(out->label));
        out->x = best.x;
        out->y = best.y;
        out->w = best.w;
        out->h = best.h;
    }
}

esp_err_t yolo26_espdl_detect_jpeg(const uint8_t *jpg_data,
                                   size_t jpg_len,
                                   yolo26_espdl_result_t *out)
{
    if (!jpg_data || jpg_len == 0 || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    if (!yolo26_espdl_available()) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret = ensure_model();
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t total_start = esp_timer_get_time();

    /*
     * Inference path: JPEG decode, letterbox preprocess, model run, raw-head
     * postprocess, then map 416x416 letterbox boxes back to source coordinates.
     */
    auto img = s_processor->decode_jpeg(jpg_data, jpg_len);
    int64_t pre_start = esp_timer_get_time();
    s_processor->preprocess(img);
    int64_t pre_end = esp_timer_get_time();

    int64_t inf_start = esp_timer_get_time();
    s_model->run();
    int64_t inf_end = esp_timer_get_time();

    int64_t post_start = esp_timer_get_time();
    auto results = s_processor->postprocess(s_model->get_outputs());
    int64_t post_end = esp_timer_get_time();

    yolo26_espdl_result_t best = {};
    std::vector<yolo26_espdl_detection_t> candidates;
    for (const auto &res : results) {
        if (res.category < 0 || res.category >= kCokeSpriteClassCount || res.box.size() < 4) {
            continue;
        }
        yolo26_espdl_detection_t det = {};
        det.class_id = (uint32_t)res.category;
        det.score = (uint32_t)(res.score * 100.0f + 0.5f);
        strlcpy(det.label, coke_sprite_classes[res.category], sizeof(det.label));
        map_letterbox_box_to_detection(img, res.box, &det);
        if (det.w > 0 && det.h > 0) {
            candidates.push_back(det);
        }
    }

    best.raw_candidate_count = (uint32_t)candidates.size();
    select_top_detections(candidates, &best);
    best.source_w = (uint32_t)img.width;
    best.source_h = (uint32_t)img.height;
    best.model_input_w = s_model_input_w;
    best.model_input_h = s_model_input_h;
    best.preprocess_ms = (pre_end - pre_start) / 1000;
    best.inference_ms = (inf_end - inf_start) / 1000;
    best.postprocess_ms = (post_end - post_start) / 1000;
    best.total_ms = (post_end - total_start) / 1000;
    if (!best.has_candidate) {
        best.source_w = (uint32_t)img.width;
        best.source_h = (uint32_t)img.height;
        best.model_input_w = s_model_input_w;
        best.model_input_h = s_model_input_h;
        strlcpy(best.label, "no-object", sizeof(best.label));
    }
    *out = best;

    heap_caps_free(img.data);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
