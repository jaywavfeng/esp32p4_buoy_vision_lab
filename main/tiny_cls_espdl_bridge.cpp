#include "tiny_cls_espdl_bridge.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <inttypes.h>
#include <new>
#include <vector>

#include "dl_image_jpeg.hpp"
#include "dl_image_preprocessor.hpp"
#include "dl_model_base.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "sdkconfig.h"

static const char *TAG = "tinycls_bridge";

extern const uint8_t tiny_cls_model_start[] asm("_binary_tiny_cls_model_espdl_start");
extern const uint8_t tiny_cls_model_end[] asm("_binary_tiny_cls_model_espdl_end");

static dl::Model *s_model;
static dl::image::ImagePreprocessor *s_preprocessor;
static SemaphoreHandle_t s_lock;
static uint32_t s_model_input_w = TINY_CLS_INPUT_W;
static uint32_t s_model_input_h = TINY_CLS_INPUT_H;

static bool ensure_lock(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
    }
    return s_lock != nullptr;
}

static void read_model_input_shape(void)
{
    if (!s_model) {
        return;
    }
    auto inputs = s_model->get_inputs();
    if (inputs.empty()) {
        return;
    }

    dl::TensorBase *input = inputs.begin()->second;
    if (input && input->shape.size() >= 4) {
        if (input->shape[3] == TINY_CLS_INPUT_C) {
            s_model_input_h = (uint32_t)input->shape[1];
            s_model_input_w = (uint32_t)input->shape[2];
        } else if (input->shape[1] == TINY_CLS_INPUT_C) {
            s_model_input_h = (uint32_t)input->shape[2];
            s_model_input_w = (uint32_t)input->shape[3];
        }
    }
}

static esp_err_t ensure_model(void)
{
    if (s_model && s_preprocessor) {
        return ESP_OK;
    }
    if (!ensure_lock()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_model) {
        s_model = new (std::nothrow) dl::Model((const char *)tiny_cls_model_start,
                                               fbs::MODEL_LOCATION_IN_FLASH_RODATA,
                                               0,
                                               dl::MEMORY_MANAGER_GREEDY,
                                               nullptr,
                                               false);
        if (!s_model) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate Tiny CNN dl::Model");
            return ESP_ERR_NO_MEM;
        }
        read_model_input_shape();
        ESP_LOGI(TAG, "Tiny CNN model loaded, input=%" PRIu32 "x%" PRIu32 ", classes=%u, bytes=%" PRIu32,
                 s_model_input_w, s_model_input_h, (unsigned)TINY_CLS_CLASS_COUNT,
                 tiny_cls_espdl_model_bytes());
    }

    if (!s_preprocessor) {
        /*
         * The PC training transform is (rgb / 255 - 0.5) / 0.5, equivalent to
         * (rgb - 127.5) / 127.5. The quantized ESP-DL model exposes NHWC input,
         * so ImagePreprocessor can resize and quantize directly.
         */
        s_preprocessor = new (std::nothrow) dl::image::ImagePreprocessor(
            s_model,
            {127.5f, 127.5f, 127.5f},
            {127.5f, 127.5f, 127.5f});
        if (!s_preprocessor) {
            xSemaphoreGive(s_lock);
            ESP_LOGE(TAG, "failed to allocate Tiny CNN preprocessor");
            return ESP_ERR_NO_MEM;
        }
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool tiny_cls_espdl_available(void)
{
    return (uintptr_t)tiny_cls_model_end > (uintptr_t)tiny_cls_model_start;
}

uint32_t tiny_cls_espdl_model_bytes(void)
{
    return (uint32_t)(tiny_cls_model_end - tiny_cls_model_start);
}

static float tensor_value(dl::TensorBase *tensor, int index)
{
    if (!tensor || index < 0 || index >= tensor->get_size()) {
        return 0.0f;
    }
    switch (tensor->get_dtype()) {
    case dl::DATA_TYPE_FLOAT:
        return tensor->get_element<float>(index);
    case dl::DATA_TYPE_INT8:
        return (float)tensor->get_element<int8_t>(index) * std::ldexp(1.0f, tensor->get_exponent());
    case dl::DATA_TYPE_INT16:
        return (float)tensor->get_element<int16_t>(index) * std::ldexp(1.0f, tensor->get_exponent());
    case dl::DATA_TYPE_INT32:
        return (float)tensor->get_element<int32_t>(index) * std::ldexp(1.0f, tensor->get_exponent());
    default:
        return 0.0f;
    }
}

static esp_err_t fill_topk(dl::TensorBase *output, tiny_cls_espdl_result_t *out)
{
    if (!output || !out || output->get_size() < (int)TINY_CLS_CLASS_COUNT) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    float logits[TINY_CLS_CLASS_COUNT] = {0};
    float max_logit = -INFINITY;
    for (uint32_t i = 0; i < TINY_CLS_CLASS_COUNT; i++) {
        logits[i] = tensor_value(output, (int)i);
        if (logits[i] > max_logit) {
            max_logit = logits[i];
        }
    }

    float probs[TINY_CLS_CLASS_COUNT] = {0};
    float sum = 0.0f;
    for (uint32_t i = 0; i < TINY_CLS_CLASS_COUNT; i++) {
        probs[i] = expf(logits[i] - max_logit);
        sum += probs[i];
    }
    if (sum <= 0.0f || !std::isfinite(sum)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (uint32_t i = 0; i < TINY_CLS_CLASS_COUNT; i++) {
        probs[i] /= sum;
    }

    uint32_t order[TINY_CLS_CLASS_COUNT] = {0};
    for (uint32_t i = 0; i < TINY_CLS_CLASS_COUNT; i++) {
        order[i] = i;
    }
    std::sort(order, order + TINY_CLS_CLASS_COUNT, [&](uint32_t a, uint32_t b) {
        return probs[a] > probs[b];
    });

    out->top_k_count = TINY_CLS_TOP_K < TINY_CLS_CLASS_COUNT ? TINY_CLS_TOP_K : TINY_CLS_CLASS_COUNT;
    for (uint32_t i = 0; i < out->top_k_count; i++) {
        uint32_t cls = order[i];
        out->top_k[i].class_id = cls;
        out->top_k[i].score = (uint32_t)(probs[cls] * 100.0f + 0.5f);
        strlcpy(out->top_k[i].label, TINY_CLS_LABELS[cls], sizeof(out->top_k[i].label));
    }

    out->class_id = out->top_k[0].class_id;
    out->score = out->top_k[0].score;
    strlcpy(out->label, out->top_k[0].label, sizeof(out->label));
    return ESP_OK;
}

static esp_err_t frame_to_img(const uint8_t *data,
                              size_t data_len,
                              uint32_t width,
                              uint32_t height,
                              uint32_t pixel_format,
                              dl::image::img_t *img)
{
    if (!data || !data_len || !width || !height || !img) {
        return ESP_ERR_INVALID_ARG;
    }

    dl::image::pix_type_t pix_type;
    size_t min_len = 0;
    switch (pixel_format) {
    case V4L2_PIX_FMT_RGB24:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
        min_len = (size_t)width * height * 3;
        break;
    case V4L2_PIX_FMT_RGB565:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;
        min_len = (size_t)width * height * 2;
        break;
    case V4L2_PIX_FMT_RGB565X:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;
        min_len = (size_t)width * height * 2;
        break;
    case V4L2_PIX_FMT_YUYV:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_YUYV;
        min_len = (size_t)width * height * 2;
        break;
    case V4L2_PIX_FMT_UYVY:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_UYVY;
        min_len = (size_t)width * height * 2;
        break;
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_SBGGR8:
        pix_type = dl::image::DL_IMAGE_PIX_TYPE_GRAY;
        min_len = (size_t)width * height;
        break;
    default:
        ESP_LOGW(TAG, "unsupported Tiny CNN frame pixel format 0x%08" PRIx32, pixel_format);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (data_len < min_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    *img = {
        .data = (void *)data,
        .width = (uint16_t)width,
        .height = (uint16_t)height,
        .pix_type = pix_type,
    };
    return ESP_OK;
}

static esp_err_t classify_img_locked(const dl::image::img_t &img, tiny_cls_espdl_result_t *out)
{
    int64_t total_start = esp_timer_get_time();
    int64_t pre_start = total_start;
    s_preprocessor->preprocess(img);
    int64_t pre_end = esp_timer_get_time();

    int64_t inf_start = esp_timer_get_time();
    s_model->run();
    int64_t inf_end = esp_timer_get_time();

    int64_t post_start = esp_timer_get_time();
    auto outputs = s_model->get_outputs();
    dl::TensorBase *logits = outputs.empty() ? nullptr : outputs.begin()->second;
    esp_err_t ret = fill_topk(logits, out);
    int64_t post_end = esp_timer_get_time();

    out->source_w = (uint32_t)img.width;
    out->source_h = (uint32_t)img.height;
    out->model_input_w = s_model_input_w;
    out->model_input_h = s_model_input_h;
    out->preprocess_ms = (pre_end - pre_start) / 1000;
    out->inference_ms = (inf_end - inf_start) / 1000;
    out->postprocess_ms = (post_end - post_start) / 1000;
    out->total_ms = (post_end - total_start) / 1000;
    return ret;
}

esp_err_t tiny_cls_espdl_classify_frame(const uint8_t *data,
                                        size_t data_len,
                                        uint32_t width,
                                        uint32_t height,
                                        uint32_t pixel_format,
                                        tiny_cls_espdl_result_t *out)
{
    if (!data || data_len == 0 || !width || !height || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    if (!tiny_cls_espdl_available()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t ret = ensure_model();
    if (ret != ESP_OK) {
        return ret;
    }

    dl::image::img_t img = {};
    ret = frame_to_img(data, data_len, width, height, pixel_format, &img);
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    ret = classify_img_locked(img, out);
    xSemaphoreGive(s_lock);
    return ret;
}

esp_err_t tiny_cls_espdl_classify_validation_jpeg(const uint8_t *jpg_data,
                                                  size_t jpg_len,
                                                  tiny_cls_espdl_result_t *out)
{
    if (!jpg_data || jpg_len == 0 || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));

    if (!tiny_cls_espdl_available()) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    dl::image::jpeg_img_t jpeg_img = {.data = (void *)jpg_data, .data_len = jpg_len};
    dl::image::img_t img = dl::image::sw_decode_jpeg(jpeg_img, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data || img.width == 0 || img.height == 0) {
        if (img.data) {
            heap_caps_free(img.data);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    esp_err_t ret = tiny_cls_espdl_classify_frame((const uint8_t *)img.data,
                                                  (size_t)img.width * img.height * 3U,
                                                  img.width,
                                                  img.height,
                                                  V4L2_PIX_FMT_RGB24,
                                                  out);
    heap_caps_free(img.data);
    return ret;
}
