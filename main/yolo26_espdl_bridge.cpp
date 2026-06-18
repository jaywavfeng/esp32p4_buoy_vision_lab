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
 * 这个符号由 main/CMakeLists.txt 的 target_add_aligned_binary_data() 生成。
 * 这里嵌入的是自训练 Coke/Sprite YOLO26n raw-head 模型：
 *   models/yolo26_coke_sprite_o2o_416_s8_p4.espdl
 *
 * 早期崩溃版本的问题是 ONNX 只有一个最终 output0，里面包含了解码后的
 * [x1,y1,x2,y2,score,class] 拼接结果；yolo26 组件需要的是六个 raw one2one head
 * 输出。当前模型按官方 quantize_yolo26 教程重新导出为
 * one2one_p3_box/cls、one2one_p4_box/cls、one2one_p5_box/cls 后再量化。
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
     * ESP-DL 的图像输入通常是 NHWC: [1, H, W, C]。若后续模型改成其他布局，
     * 这里仍保守读取最后两个空间维度，避免网页框坐标完全失真。
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
         * param_copy=false 是 yolo26 官方示例推荐方式：模型参数留在 flash rodata，
         * 不复制到 PSRAM，既省内存，也符合 32MB PSRAM 板卡上的部署建议。
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
         * 组件内部的置信度阈值保持 0.25，用来减少候选输出数量；网页上的
         * box_min_score 仍会再做一次过滤，用于实验时动态调节画框阈值。
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
     * 推理流程：
     * 1. JPEG 解码成 RGB888；
     * 2. YOLO26 组件执行 letterbox resize + INT8 LUT 量化，直接写入模型输入；
     * 3. ESP-DL 调用 P4 上的定点算子跑模型；
     * 4. one-to-one head 后处理并取最高置信度候选；
     * 5. 把 416x416 letterbox 坐标映射回摄像头原始图像坐标，方便网页画框。
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
