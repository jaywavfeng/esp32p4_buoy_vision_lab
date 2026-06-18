#include "coco_espdl_bridge.h"

#include <algorithm>
#include <cstring>
#include <inttypes.h>
#include <list>
#include <new>
#include <vector>

#include "coco_detect.hpp"
#include "dl_detect_base.hpp"
#include "dl_image_draw.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "coco_bridge";

#if CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA
extern const uint8_t coco_detect_model_start[] asm("_binary_coco_detect_espdl_start");
extern const uint8_t coco_detect_model_end[] asm("_binary_coco_detect_espdl_end");
#endif

static const char *coco_classes[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
};

static_assert(sizeof(coco_classes) / sizeof(coco_classes[0]) == COCO_ESPDL_CLASS_COUNT,
              "COCO class table must contain 80 entries");

static constexpr uint32_t kModelInputSize = 320;
static constexpr uint32_t kFallbackModelBytes = 2860704;
static constexpr float kFallbackNmsThreshold = 0.70f;

static COCODetect *s_detector;
static SemaphoreHandle_t s_lock;

static bool ensure_lock(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }
    return s_lock != nullptr;
}

static float box_iou(const coco_espdl_detection_t &a, const coco_espdl_detection_t &b)
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

static void fill_detection_from_result(const dl::detect::result_t &res, coco_espdl_detection_t *det)
{
    memset(det, 0, sizeof(*det));
    det->class_id = (uint32_t)res.category;
    det->score = (uint32_t)(res.score * 100.0f + 0.5f);
    strlcpy(det->label, coco_classes[res.category], sizeof(det->label));
    int32_t x1 = std::max(0, res.box[0]);
    int32_t y1 = std::max(0, res.box[1]);
    int32_t x2 = std::max(0, res.box[2]);
    int32_t y2 = std::max(0, res.box[3]);
    det->x = x1;
    det->y = y1;
    det->w = std::max<int32_t>(0, x2 - x1);
    det->h = std::max<int32_t>(0, y2 - y1);
}

static void select_top_detections(std::vector<coco_espdl_detection_t> &candidates,
                                  coco_espdl_result_t *out)
{
    std::sort(candidates.begin(), candidates.end(),
              [](const coco_espdl_detection_t &a, const coco_espdl_detection_t &b) {
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
        if (out->detection_count >= COCO_ESPDL_MAX_DETECTIONS) {
            break;
        }
    }

    if (out->detection_count > 0) {
        const coco_espdl_detection_t &best = out->detections[0];
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

static esp_err_t ensure_detector(void)
{
    if (s_detector) {
        return ESP_OK;
    }
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_detector) {
        s_detector = new (std::nothrow) COCODetect(COCODetect::YOLO11N_320_S8_V3, false);
        if (!s_detector) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate COCO YOLO11n detector");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "COCO YOLO11n 320 model ready, input=%" PRIu32 "x%" PRIu32 ", bytes=%" PRIu32,
                 kModelInputSize, kModelInputSize, coco_espdl_model_bytes());
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool coco_espdl_available(void)
{
#if CONFIG_FLASH_COCO_DETECT_YOLO11N_320_S8_V3 || CONFIG_COCO_DETECT_MODEL_IN_SDCARD
#if CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA
    return (uintptr_t)coco_detect_model_end > (uintptr_t)coco_detect_model_start;
#else
    return true;
#endif
#else
    return false;
#endif
}

uint32_t coco_espdl_model_bytes(void)
{
#if CONFIG_COCO_DETECT_MODEL_IN_FLASH_RODATA
    return (uint32_t)(coco_detect_model_end - coco_detect_model_start);
#else
    return kFallbackModelBytes;
#endif
}

static esp_err_t detect_jpeg_internal(const uint8_t *jpg_data,
                                      size_t jpg_len,
                                      uint32_t min_score,
                                      uint8_t jpeg_quality,
                                      coco_espdl_result_t *out,
                                      uint8_t **annotated_jpeg,
                                      size_t *annotated_len)
{
    if (!jpg_data || jpg_len == 0 || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((annotated_jpeg == nullptr) != (annotated_len == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (annotated_jpeg) {
        *annotated_jpeg = nullptr;
        *annotated_len = 0;
    }

    if (!coco_espdl_available()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t ret = ensure_detector();
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t total_start = esp_timer_get_time();

    dl::image::jpeg_img_t jpeg_img = {.data = (void *)jpg_data, .data_len = jpg_len};
    dl::image::img_t img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data || img.width == 0 || img.height == 0) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int64_t run_start = esp_timer_get_time();
    std::list<dl::detect::result_t> &results = s_detector->run(img);
    int64_t run_end = esp_timer_get_time();

    coco_espdl_result_t best = {};
    std::vector<coco_espdl_detection_t> candidates;
    for (const auto &res : results) {
        if (res.category < 0 || res.category >= COCO_ESPDL_CLASS_COUNT || res.box.size() < 4) {
            continue;
        }
        coco_espdl_detection_t det = {};
        fill_detection_from_result(res, &det);
        if (det.w <= 0 || det.h <= 0) {
            continue;
        }
        candidates.push_back(det);
    }

    best.raw_candidate_count = (uint32_t)candidates.size();
    select_top_detections(candidates, &best);
    best.source_w = (uint32_t)img.width;
    best.source_h = (uint32_t)img.height;
    best.model_input_w = kModelInputSize;
    best.model_input_h = kModelInputSize;
    best.inference_ms = (run_end - run_start) / 1000;
    best.total_ms = (run_end - total_start) / 1000;
    if (!best.has_candidate) {
        strlcpy(best.label, "no-object", sizeof(best.label));
    }

    if (annotated_jpeg) {
        static const std::vector<std::vector<uint8_t>> colors = {
            {255, 48, 48}, {35, 210, 120}, {45, 145, 255}, {255, 205, 40},
            {220, 80, 255}, {30, 220, 220}, {255, 135, 30}, {245, 245, 245},
        };
        for (uint32_t i = 0; i < best.detection_count; i++) {
            const coco_espdl_detection_t &det = best.detections[i];
            if (det.score < min_score || det.w <= 1 || det.h <= 1) {
                continue;
            }
            int x1 = std::clamp<int>(det.x, 0, img.width - 2);
            int y1 = std::clamp<int>(det.y, 0, img.height - 2);
            int x2 = std::clamp<int>(det.x + det.w, x1 + 1, img.width - 1);
            int y2 = std::clamp<int>(det.y + det.h, y1 + 1, img.height - 1);
            dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2,
                                             colors[det.class_id % colors.size()], 3);
        }
#if CONFIG_SOC_JPEG_CODEC_SUPPORTED
        dl::image::jpeg_img_t encoded = dl::image::hw_encode_jpeg(img, jpeg_quality);
#else
        dl::image::jpeg_img_t encoded = dl::image::sw_encode_jpeg(img, jpeg_quality);
#endif
        if (!encoded.data || encoded.data_len == 0) {
            heap_caps_free(img.data);
            xSemaphoreGive(s_lock);
            return ESP_ERR_INVALID_RESPONSE;
        }
        *annotated_jpeg = static_cast<uint8_t *>(encoded.data);
        *annotated_len = encoded.data_len;
    }
    *out = best;

    heap_caps_free(img.data);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t coco_espdl_detect_jpeg(const uint8_t *jpg_data,
                                 size_t jpg_len,
                                 coco_espdl_result_t *out)
{
    return detect_jpeg_internal(jpg_data, jpg_len, 0, 80, out, nullptr, nullptr);
}

esp_err_t coco_espdl_detect_and_annotate_jpeg(const uint8_t *jpg_data,
                                              size_t jpg_len,
                                              uint32_t min_score,
                                              uint8_t jpeg_quality,
                                              coco_espdl_result_t *out,
                                              uint8_t **annotated_jpeg,
                                              size_t *annotated_len)
{
    return detect_jpeg_internal(jpg_data, jpg_len, min_score, jpeg_quality, out,
                                annotated_jpeg, annotated_len);
}

void coco_espdl_free_jpeg(uint8_t *jpeg)
{
    heap_caps_free(jpeg);
}
