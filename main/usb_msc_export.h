#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/sdmmc_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*usb_msc_host_event_cb_t)(bool connected, void *arg);

typedef struct {
    bool initialized;
    bool host_connected;
    bool bus_active;
    bool storage_ready;
    bool writable;
    uint32_t last_sof_age_ms;
    char last_error[96];
} usb_msc_export_status_t;

esp_err_t usb_msc_export_init(usb_msc_host_event_cb_t callback, void *callback_arg);
esp_err_t usb_msc_export_attach_sdmmc(sdmmc_card_t *card);
esp_err_t usb_msc_export_detach_storage(void);
void usb_msc_export_get_status(usb_msc_export_status_t *status);

#ifdef __cplusplus
}
#endif
