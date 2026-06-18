#include "yolo11_espdl_bridge.h"

#include <algorithm>
#include <cstring>
#include <inttypes.h>
#include <list>
#include <new>
#include <vector>

#include "dl_detect_base.hpp"
#include "dl_detect_yolo11_postprocessor.hpp"
#include "dl_image_jpeg.hpp"
#include "dl_model_base.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "yolo11_bridge";

/*
 * 这里嵌入的是本工程自训练的 two-class Coke/Sprite YOLO11n 模型：
 *   models/yolo11_coke_sprite_416_s8_p4.espdl
 *
 * 训练脚本 `tools/train_yolo11_coke_sprite.py` 按 ESP-DL 官方 YOLO11 部署路线，
 * 把 Ultralytics Detect.forward 改为输出 box0/score0、box1/score1、box2/score2
 * 六个原始检测头；量化脚本 `tools/quantize_yolo11_espdl.py` 再使用 ESP-PPQ
 * 生成 ESP32-P4 可加载的 INT8 `.espdl`。板端后处理复用 ESP-DL 的
 * `dl::detect::yolo11PostProcessor`，它负责 DFL 解码、sigmoid、NMS 和
 * letterbox 坐标映射。
 */
extern const uint8_t yolo11_model_start[] asm("_binary_yolo11_coke_sprite_416_s8_p4_espdl_start");
extern const uint8_t yolo11_model_end[] asm("_binary_yolo11_coke_sprite_416_s8_p4_espdl_end");

static const char *coke_sprite_classes[] = {
    "coke",
    "sprite",
};
static constexpr int kCokeSpriteClassCount = sizeof(coke_sprite_classes) / sizeof(coke_sprite_classes[0]);
static constexpr float kDefaultScoreThreshold = 0.25f;
static constexpr float kDefaultNmsThreshold = 0.70f;
static constexpr float kFallbackNmsThreshold = 0.70f;

class CokeSpriteYolo11 : public dl::detect::DetectImpl {
public:
    CokeSpriteYolo11(const char *model_data, float score_thr, float nms_thr)
    {
        /*
         * param_copy=false：模型参数留在 flash rodata，不复制到 PSRAM。
         * 对 P4 这类视觉板很关键，因为摄像头帧、JPEG 编码、Web 推流和模型推理
         * 会同时占用大量 PSRAM。ESP-DL 官方示例也建议 rodata + param_copy=false。
         */
        m_model = new dl::Model(model_data,
                                fbs::MODEL_LOCATION_IN_FLASH_RODATA,
                                0,
                                dl::MEMORY_MANAGER_GREEDY,
                                nullptr,
                                false);
        m_model->minimize();

        /*
         * 训练/量化时输入范围是 0..1；这里用 {0,0,0}/{255,255,255} 完成 RGB888
         * 到模型输入的缩放。letterbox 灰边值 114 与 Ultralytics/量化校准保持一致。
         */
        m_image_preprocessor = new dl::image::ImagePreprocessor(m_model, {0, 0, 0}, {255, 255, 255});
        m_image_preprocessor->enable_letterbox({114, 114, 114});

        /*
         * 三个 stage 分别对应 416 输入下的 52x52、26x26、13x13 特征层。
         * stride/offset 的写法沿用 ESP-DL 官方 YOLO11n 示例；类别数由 score 输出
         * 通道自动决定，这里是 two-class: coke/sprite。
         */
        m_postprocessor = new dl::detect::yolo11PostProcessor(
            m_model, m_image_preprocessor, score_thr, nms_thr, 10, {{8, 8, 4, 4}, {16, 16, 8, 8}, {32, 32, 16, 16}});
    }
};

static CokeSpriteYolo11 *s_detector;
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
    if (!s_detector) {
        return;
    }
    dl::Model *model = s_detector->get_raw_model();
    if (!model) {
        return;
    }
    auto inputs = model->get_inputs();
    if (inputs.empty()) {
        return;
    }

    dl::TensorBase *input = inputs.begin()->second;
    if (input->shape.size() >= 4) {
        if (input->shape[1] == 3) {
            s_model_input_h = (uint32_t)input->shape[2];
            s_model_input_w = (uint32_t)input->shape[3];
        } else {
            s_model_input_h = (uint32_t)input->shape[1];
            s_model_input_w = (uint32_t)input->shape[2];
        }
    }
}

static float box_iou(const yolo11_espdl_detection_t &a, const yolo11_espdl_detection_t &b)
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

static void fill_detection_from_result(const dl::detect::result_t &res, yolo11_espdl_detection_t *det)
{
    memset(det, 0, sizeof(*det));
    det->class_id = (uint32_t)res.category;
    det->score = (uint32_t)(res.score * 100.0f + 0.5f);
    strlcpy(det->label, coke_sprite_classes[res.category], sizeof(det->label));
    int32_t x1 = std::max(0, res.box[0]);
    int32_t y1 = std::max(0, res.box[1]);
    int32_t x2 = std::max(0, res.box[2]);
    int32_t y2 = std::max(0, res.box[3]);
    det->x = x1;
    det->y = y1;
    det->w = std::max<int32_t>(0, x2 - x1);
    det->h = std::max<int32_t>(0, y2 - y1);
}

static void select_top_detections(std::vector<yolo11_espdl_detection_t> &candidates,
                                  yolo11_espdl_result_t *out)
{
    std::sort(candidates.begin(), candidates.end(),
              [](const yolo11_espdl_detection_t &a, const yolo11_espdl_detection_t &b) {
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
        if (out->detection_count >= YOLO11_ESPDL_MAX_DETECTIONS) {
            break;
        }
    }

    if (out->detection_count > 0) {
        const yolo11_espdl_detection_t &best = out->detections[0];
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
        s_detector = new (std::nothrow) CokeSpriteYolo11((const char *)yolo11_model_start,
                                                         kDefaultScoreThreshold,
                                                         kDefaultNmsThreshold);
        if (!s_detector) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate CokeSpriteYolo11");
            return ESP_ERR_NO_MEM;
        }
        read_model_input_shape();
        ESP_LOGI(TAG, "YOLO11 Coke/Sprite model loaded, input=%" PRIu32 "x%" PRIu32 ", bytes=%" PRIu32,
                 s_model_input_w, s_model_input_h, yolo11_espdl_model_bytes());
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool yolo11_espdl_available(void)
{
    return (uintptr_t)yolo11_model_end > (uintptr_t)yolo11_model_start;
}

uint32_t yolo11_espdl_model_bytes(void)
{
    return (uint32_t)(yolo11_model_end - yolo11_model_start);
}

esp_err_t yolo11_espdl_detect_jpeg(const uint8_t *jpg_data,
                                   size_t jpg_len,
                                   yolo11_espdl_result_t *out)
{
    if (!jpg_data || jpg_len == 0 || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    if (!yolo11_espdl_available()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t ret = ensure_detector();
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t total_start = esp_timer_get_time();

    /*
     * 单帧流程：
     * 1. 把摄像头任务保存的 JPEG 解码成 RGB888；
     * 2. CokeSpriteYolo11::run() 内部做 letterbox + INT8 输入量化 + model->run()；
     * 3. yolo11PostProcessor 解析 box/score 输出并做 NMS；
     * 4. 取最高置信度候选，转换成统一结构供网页画框、历史记录和 /api/status 使用。
     */
    dl::image::jpeg_img_t jpeg_img = {.data = (void *)jpg_data, .data_len = jpg_len};
    dl::image::img_t img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data || img.width == 0 || img.height == 0) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_RESPONSE;
    }

    int64_t run_start = esp_timer_get_time();
    std::list<dl::detect::result_t> &results = s_detector->run(img);
    int64_t run_end = esp_timer_get_time();

    yolo11_espdl_result_t best = {};
    std::vector<yolo11_espdl_detection_t> candidates;
    for (const auto &res : results) {
        if (res.category < 0 || res.category >= kCokeSpriteClassCount || res.box.size() < 4) {
            continue;
        }
        yolo11_espdl_detection_t det = {};
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
    best.model_input_w = s_model_input_w;
    best.model_input_h = s_model_input_h;
    best.inference_ms = (run_end - run_start) / 1000;
    best.total_ms = (run_end - total_start) / 1000;
    if (!best.has_candidate) {
        strlcpy(best.label, "no-object", sizeof(best.label));
        best.source_w = (uint32_t)img.width;
        best.source_h = (uint32_t)img.height;
        best.model_input_w = s_model_input_w;
        best.model_input_h = s_model_input_h;
    }
    *out = best;

    heap_caps_free(img.data);
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
