#include "yolo11_espdl_bridge.h"
#include "yolo26_espdl_bridge.h"

#include <string.h>

bool yolo11_espdl_available(void)
{
    return false;
}

uint32_t yolo11_espdl_model_bytes(void)
{
    return 0;
}

esp_err_t yolo11_espdl_detect_jpeg(const uint8_t *jpg_data,
                                   size_t jpg_len,
                                   yolo11_espdl_result_t *out)
{
    (void)jpg_data;
    (void)jpg_len;
    if (out) {
        memset(out, 0, sizeof(*out));
    }
    return ESP_ERR_NOT_SUPPORTED;
}

bool yolo26_espdl_available(void)
{
    return false;
}

uint32_t yolo26_espdl_model_bytes(void)
{
    return 0;
}

esp_err_t yolo26_espdl_detect_jpeg(const uint8_t *jpg_data,
                                   size_t jpg_len,
                                   yolo26_espdl_result_t *out)
{
    (void)jpg_data;
    (void)jpg_len;
    if (out) {
        memset(out, 0, sizeof(*out));
    }
    return ESP_ERR_NOT_SUPPORTED;
}
