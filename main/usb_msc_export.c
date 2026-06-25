#include "usb_msc_export.h"

#include <string.h>

#include "esp_log.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"

static const char *TAG = "usb_msc_export";

static usb_msc_host_event_cb_t s_host_callback;
static void *s_host_callback_arg;
static tinyusb_msc_storage_handle_t s_storage;
static usb_msc_export_status_t s_status = {
    .writable = true,
    .last_error = "not initialized",
};

static void set_error(const char *message, esp_err_t err)
{
    if (err == ESP_OK) {
        strlcpy(s_status.last_error, message ? message : "ok", sizeof(s_status.last_error));
    } else {
        snprintf(s_status.last_error, sizeof(s_status.last_error), "%s: %s",
                 message ? message : "error", esp_err_to_name(err));
    }
}

static void tinyusb_event_callback(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    if (!event) {
        return;
    }

    bool notify = false;
    bool connected = s_status.host_connected;
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        connected = true;
        notify = true;
        ESP_LOGI(TAG, "USB1 host configured");
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        connected = false;
        notify = true;
        ESP_LOGI(TAG, "USB1 host detached; TF remains owned by USB mode until reboot");
    }
    s_status.host_connected = connected;
    if (notify && s_host_callback) {
        s_host_callback(connected, s_host_callback_arg);
    }
}

esp_err_t usb_msc_export_init(usb_msc_host_event_cb_t callback, void *callback_arg)
{
    if (s_status.initialized) {
        return ESP_OK;
    }

    s_host_callback = callback;
    s_host_callback_arg = callback_arg;

    tinyusb_msc_driver_config_t msc_config = {
        .user_flags = {
            .auto_mount_off = 1,
        },
    };
    esp_err_t ret = tinyusb_msc_install_driver(&msc_config);
    if (ret != ESP_OK) {
        set_error("MSC driver install failed", ret);
        return ret;
    }

    tinyusb_config_t tinyusb_config = TINYUSB_DEFAULT_CONFIG(tinyusb_event_callback, NULL);
    tinyusb_config.task.priority = 8;
    tinyusb_config.task.size = 6144;
    ret = tinyusb_driver_install(&tinyusb_config);
    if (ret != ESP_OK) {
        tinyusb_msc_uninstall_driver();
        set_error("TinyUSB install failed", ret);
        return ret;
    }

    s_status.initialized = true;
    set_error("waiting for USB1 host", ESP_OK);
    ESP_LOGI(TAG, "USB HS MSC ready with no media; product=%s", CONFIG_APP_USB_MSC_PRODUCT_NAME);
    return ESP_OK;
}

esp_err_t usb_msc_export_attach_sdmmc(sdmmc_card_t *card)
{
    if (!s_status.initialized || !card) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_storage) {
        return ESP_OK;
    }

    tinyusb_msc_storage_config_t storage_config = {
        .medium.card = card,
        .fat_fs = {
            .base_path = NULL,
            .config = {
                .format_if_mount_failed = false,
                .max_files = 4,
                .allocation_unit_size = 0,
            },
            .do_not_format = true,
            .format_flags = 0,
        },
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB,
    };
    esp_err_t ret = tinyusb_msc_new_storage_sdmmc(&storage_config, &s_storage);
    if (ret != ESP_OK) {
        set_error("attach TF failed", ret);
        return ret;
    }

    s_status.storage_ready = true;
    set_error("USB host owns writable TF", ESP_OK);
    ESP_LOGI(TAG, "Writable TF card exposed on USB1; safe eject and reboot are required");
    return ESP_OK;
}

void usb_msc_export_get_status(usb_msc_export_status_t *status)
{
    if (status) {
        *status = s_status;
    }
}
