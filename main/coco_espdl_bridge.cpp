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
static COCODetect *s_background_detector;
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

static esp_err_t ensure_detector(bool background, COCODetect **out)
{
    COCODetect **slot = background ? &s_background_detector : &s_detector;
    if (*slot) {
        *out = *slot;
        return ESP_OK;
    }
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!*slot) {
        *slot = new (std::nothrow) COCODetect(COCODetect::YOLO11N_320_S8_V3, false);
        if (!*slot) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate COCO YOLO11n detector");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "COCO YOLO11n 320 %s model ready, input=%" PRIu32 "x%" PRIu32
                 ", bytes=%" PRIu32,
                 background ? "background" : "foreground",
                 kModelInputSize, kModelInputSize, coco_espdl_model_bytes());
    }
    *out = *slot;
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

static const uint8_t *glyph5x7(char c)
{
    static const uint8_t blank[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t glyphs[][7] = {
        {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e}, // 0
        {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e}, // 1
        {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f}, // 2
        {0x1f, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0e}, // 3
        {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02}, // 4
        {0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e}, // 5
        {0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e}, // 6
        {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, // 7
        {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e}, // 8
        {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c}, // 9
    };
    static const uint8_t letters[][7] = {
        {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}, // A
        {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e}, // B
        {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e}, // C
        {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e}, // D
        {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f}, // E
        {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10}, // F
        {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f}, // G
        {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}, // H
        {0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e}, // I
        {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0c}, // J
        {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, // K
        {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f}, // L
        {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11}, // M
        {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}, // N
        {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}, // O
        {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10}, // P
        {0x0e, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0d}, // Q
        {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11}, // R
        {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e}, // S
        {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, // T
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}, // U
        {0x11, 0x11, 0x11, 0x11, 0x11, 0x0a, 0x04}, // V
        {0x11, 0x11, 0x11, 0x15, 0x15, 0x1b, 0x11}, // W
        {0x11, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x11}, // X
        {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04}, // Y
        {0x1f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1f}, // Z
    };
    static const uint8_t dash[7] = {0, 0, 0, 0x1f, 0, 0, 0};
    static const uint8_t underscore[7] = {0, 0, 0, 0, 0, 0, 0x1f};
    static const uint8_t colon[7] = {0, 0x04, 0x04, 0, 0x04, 0x04, 0};
    static const uint8_t slash[7] = {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10};
    static const uint8_t percent[7] = {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03};
    static const uint8_t dot[7] = {0, 0, 0, 0, 0, 0x0c, 0x0c};
    if (c >= '0' && c <= '9') {
        return glyphs[c - '0'];
    }
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    if (c >= 'A' && c <= 'Z') {
        return letters[c - 'A'];
    }
    if (c == '-') {
        return dash;
    }
    if (c == '_') {
        return underscore;
    }
    if (c == ':') {
        return colon;
    }
    if (c == '/') {
        return slash;
    }
    if (c == '%') {
        return percent;
    }
    if (c == '.') {
        return dot;
    }
    return blank;
}

static void set_rgb_pixel(dl::image::img_t &img, int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (!img.data || img.pix_type != dl::image::DL_IMAGE_PIX_TYPE_RGB888 ||
        x < 0 || y < 0 || x >= img.width || y >= img.height) {
        return;
    }
    uint8_t *p = static_cast<uint8_t *>(img.data) + y * img.row_step() + x * 3;
    p[0] = r;
    p[1] = g;
    p[2] = b;
}

static void fill_rect(dl::image::img_t &img, int x, int y, int w, int h,
                      uint8_t r, uint8_t g, uint8_t b)
{
    int x2 = std::clamp(x + w, 0, (int)img.width);
    int y2 = std::clamp(y + h, 0, (int)img.height);
    x = std::clamp(x, 0, (int)img.width);
    y = std::clamp(y, 0, (int)img.height);
    for (int yy = y; yy < y2; yy++) {
        for (int xx = x; xx < x2; xx++) {
            set_rgb_pixel(img, xx, yy, r, g, b);
        }
    }
}

static void draw_char(dl::image::img_t &img, int x, int y, char c, int scale,
                      uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t *rows = glyph5x7(c);
    for (int yy = 0; yy < 7; yy++) {
        for (int xx = 0; xx < 5; xx++) {
            if (rows[yy] & (1U << (4 - xx))) {
                fill_rect(img, x + xx * scale, y + yy * scale, scale, scale, r, g, b);
            }
        }
    }
}

static void draw_text(dl::image::img_t &img, int x, int y, const char *text, int scale,
                      uint8_t r, uint8_t g, uint8_t b)
{
    if (!text) {
        return;
    }
    int cursor = x;
    int advance = 6 * scale;
    for (const char *p = text; *p && cursor + 5 * scale < img.width; p++) {
        draw_char(img, cursor, y, *p, scale, r, g, b);
        cursor += advance;
    }
}

static esp_err_t encode_rgb_jpeg(dl::image::img_t &img,
                                 uint8_t jpeg_quality,
                                 uint8_t **annotated_jpeg,
                                 size_t *annotated_len)
{
#if CONFIG_SOC_JPEG_CODEC_SUPPORTED
    dl::image::jpeg_img_t encoded = dl::image::hw_encode_jpeg(img, jpeg_quality);
    if (!encoded.data || encoded.data_len == 0) {
        static bool warned_hw_encode = false;
        if (!warned_hw_encode) {
            ESP_LOGW(TAG, "hardware JPEG encode failed for annotated frame, falling back to software encoder");
            warned_hw_encode = true;
        }
        if (encoded.data) {
            heap_caps_free(encoded.data);
        }
        encoded = dl::image::sw_encode_jpeg(img, jpeg_quality);
    }
#else
    dl::image::jpeg_img_t encoded = dl::image::sw_encode_jpeg(img, jpeg_quality);
#endif
    if (!encoded.data || encoded.data_len == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *annotated_jpeg = static_cast<uint8_t *>(encoded.data);
    *annotated_len = encoded.data_len;
    return ESP_OK;
}

static esp_err_t annotate_decoded_image(dl::image::img_t &img,
                                        const coco_espdl_result_t &result,
                                        uint32_t min_score,
                                        uint8_t jpeg_quality,
                                        uint8_t **annotated_jpeg,
                                        size_t *annotated_len)
{
    static const std::vector<std::vector<uint8_t>> colors = {
        {255, 48, 48}, {35, 210, 120}, {45, 145, 255}, {255, 205, 40},
        {220, 80, 255}, {30, 220, 220}, {255, 135, 30}, {245, 245, 245},
    };
    uint32_t drawn = 0;
    for (uint32_t i = 0; i < result.detection_count; i++) {
        const coco_espdl_detection_t &det = result.detections[i];
        if (det.score < min_score || det.w <= 1 || det.h <= 1) {
            continue;
        }
        int x1 = std::clamp<int>(det.x, 0, img.width - 2);
        int y1 = std::clamp<int>(det.y, 0, img.height - 2);
        int x2 = std::clamp<int>(det.x + det.w, x1 + 1, img.width - 1);
        int y2 = std::clamp<int>(det.y + det.h, y1 + 1, img.height - 1);
        dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2,
                                         colors[det.class_id % colors.size()], 3);
        drawn++;
    }
    if (drawn == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    return encode_rgb_jpeg(img, jpeg_quality, annotated_jpeg, annotated_len);
}

static esp_err_t detect_jpeg_internal(const uint8_t *jpg_data,
                                      size_t jpg_len,
                                      uint32_t min_score,
                                      uint8_t jpeg_quality,
                                      coco_espdl_result_t *out,
                                      uint8_t **annotated_jpeg,
                                      size_t *annotated_len,
                                      bool background)
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
    COCODetect *detector = nullptr;
    esp_err_t ret = ensure_detector(background, &detector);
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
    std::list<dl::detect::result_t> &results = detector->run(img);
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
        ret = annotate_decoded_image(img, best, min_score, jpeg_quality,
                                     annotated_jpeg, annotated_len);
        if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
            heap_caps_free(img.data);
            xSemaphoreGive(s_lock);
            return ret;
        }
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
    return detect_jpeg_internal(jpg_data, jpg_len, 0, 80, out, nullptr, nullptr, false);
}

esp_err_t coco_espdl_detect_jpeg_background(const uint8_t *jpg_data,
                                            size_t jpg_len,
                                            coco_espdl_result_t *out)
{
    return detect_jpeg_internal(jpg_data, jpg_len, 0, 80, out, nullptr, nullptr, true);
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
                                annotated_jpeg, annotated_len, false);
}

esp_err_t coco_espdl_annotate_jpeg(const uint8_t *jpg_data,
                                   size_t jpg_len,
                                   const coco_espdl_result_t *result,
                                   uint32_t min_score,
                                   uint8_t jpeg_quality,
                                   uint8_t **annotated_jpeg,
                                   size_t *annotated_len)
{
    if (!jpg_data || jpg_len == 0 || !result || !annotated_jpeg || !annotated_len) {
        return ESP_ERR_INVALID_ARG;
    }
    *annotated_jpeg = nullptr;
    *annotated_len = 0;
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    dl::image::jpeg_img_t jpeg_img = {.data = (void *)jpg_data, .data_len = jpg_len};
    dl::image::img_t img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data || img.width == 0 || img.height == 0) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_RESPONSE;
    }
    esp_err_t ret = annotate_decoded_image(img, *result, min_score, jpeg_quality,
                                           annotated_jpeg, annotated_len);
    heap_caps_free(img.data);
    xSemaphoreGive(s_lock);
    return ret;
}

esp_err_t coco_espdl_annotate_label_jpeg(const uint8_t *jpg_data,
                                         size_t jpg_len,
                                         const char *title,
                                         const char *subtitle,
                                         uint8_t jpeg_quality,
                                         uint8_t **annotated_jpeg,
                                         size_t *annotated_len)
{
    if (!jpg_data || jpg_len == 0 || !annotated_jpeg || !annotated_len) {
        return ESP_ERR_INVALID_ARG;
    }
    *annotated_jpeg = nullptr;
    *annotated_len = 0;
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    dl::image::jpeg_img_t jpeg_img = {.data = (void *)jpg_data, .data_len = jpg_len};
    dl::image::img_t img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data || img.width == 0 || img.height == 0) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int title_len = title ? (int)strlen(title) : 0;
    int subtitle_len = subtitle ? (int)strlen(subtitle) : 0;
    int scale = img.width >= 640 ? 3 : 2;
    int title_w = title_len * 6 * scale;
    int subtitle_w = subtitle_len * 6 * 2;
    int banner_w = std::max(title_w, subtitle_w) + 24;
    banner_w = std::clamp(banner_w, 120, (int)img.width);
    int banner_h = subtitle_len > 0 ? 72 : 48;
    if (banner_h > img.height) {
        banner_h = img.height;
    }
    fill_rect(img, 0, 0, banner_w, banner_h, 8, 16, 24);
    fill_rect(img, 0, banner_h - 3, banner_w, 3, 255, 205, 40);
    draw_text(img, 12, 12, title ? title : "", scale, 255, 230, 90);
    if (subtitle_len > 0) {
        draw_text(img, 12, 46, subtitle, 2, 220, 242, 248);
    }
    esp_err_t ret = encode_rgb_jpeg(img, jpeg_quality, annotated_jpeg, annotated_len);
    heap_caps_free(img.data);
    xSemaphoreGive(s_lock);
    return ret;
}

void coco_espdl_free_jpeg(uint8_t *jpeg)
{
    heap_caps_free(jpeg);
}

void coco_espdl_release_background(void)
{
    if (!ensure_lock()) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    delete s_background_detector;
    s_background_detector = nullptr;
    xSemaphoreGive(s_lock);
}
