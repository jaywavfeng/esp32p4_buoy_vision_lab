#include "usb_msc_export.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "tusb.h"

static const char *TAG = "usb_msc_export";

static usb_msc_host_event_cb_t s_host_callback;
static void *s_host_callback_arg;
static tinyusb_msc_storage_handle_t s_storage;
static void *s_dma_reserve;
static bool s_forced_reconnect;
static char s_usb_serial[32] = "P4BUOY";
static const char *s_usb_string_desc[6];
static const char s_usb_langid[] = {0x09, 0x04};
static usb_msc_export_status_t s_status = {
    .writable = true,
    .last_error = "not initialized",
};

#define USB_MSC_DMA_RESERVE_BYTES (CONFIG_TINYUSB_MSC_BUFSIZE + 1024)

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
        s_status.bus_active = true;
        s_status.last_sof_age_ms = UINT32_MAX;
        notify = true;
        ESP_LOGI(TAG, "USB1 host configured");
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        connected = false;
        s_status.bus_active = false;
        s_status.last_sof_age_ms = UINT32_MAX;
        notify = true;
        ESP_LOGI(TAG, "USB1 host detached; queueing TF restore to application");
    }
    s_status.host_connected = connected;

    if (notify && s_forced_reconnect) {
        ESP_LOGI(TAG, "USB1 %s during MSC media reconnect",
                 connected ? "attached" : "detached");
        return;
    }

    if (notify && s_host_callback) {
        s_host_callback(connected, s_host_callback_arg);
    }
}

void app_usb_msc_eject_cb(void)
{
    if (s_forced_reconnect) {
        ESP_LOGI(TAG, "USB MSC eject ignored during forced media reconnect");
        return;
    }
    s_status.host_connected = false;
    s_status.bus_active = false;
    s_status.last_sof_age_ms = UINT32_MAX;
    set_error("USB host ejected media; restoring TF to application", ESP_OK);
    ESP_LOGI(TAG, "USB MSC media ejected by host; queueing TF restore to application");
    if (s_host_callback) {
        s_host_callback(false, s_host_callback_arg);
    }
}

static void reserve_dma_block(void)
{
    if (s_dma_reserve) {
        return;
    }
    s_dma_reserve = heap_caps_malloc(USB_MSC_DMA_RESERVE_BYTES, MALLOC_CAP_DMA);
    if (!s_dma_reserve) {
        ESP_LOGW(TAG, "USB MSC DMA reserve allocation failed (%u bytes)",
                 (unsigned)USB_MSC_DMA_RESERVE_BYTES);
    }
}

static void release_dma_reserve(void)
{
    if (s_dma_reserve) {
        heap_caps_free(s_dma_reserve);
        s_dma_reserve = NULL;
    }
}

static void prepare_usb_descriptors(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        snprintf(s_usb_serial, sizeof(s_usb_serial), "P4%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    s_usb_string_desc[0] = s_usb_langid;
    s_usb_string_desc[1] = "ZJU P4 Buoy";
    s_usb_string_desc[2] = "P4_BUOY TF Storage";
    s_usb_string_desc[3] = s_usb_serial;
    s_usb_string_desc[4] = "";
    s_usb_string_desc[5] = "P4_BUOY";
}

static void force_host_reenumeration_if_needed(void)
{
    if (!tud_inited() || (!tud_connected() && !tud_mounted())) {
        return;
    }

    s_forced_reconnect = true;
    ESP_LOGI(TAG, "Forcing USB MSC re-enumeration after TF media attach");
    (void)tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(180));
    (void)tud_connect();
    vTaskDelay(pdMS_TO_TICKS(220));
    s_forced_reconnect = false;
}

static void connect_host_after_media_change(void)
{
    if (!tud_inited()) {
        return;
    }
    if (!tud_connected() && !tud_mounted()) {
        s_forced_reconnect = true;
        ESP_LOGI(TAG, "Connecting USB MSC device after TF media attach");
        (void)tud_connect();
        vTaskDelay(pdMS_TO_TICKS(220));
        s_forced_reconnect = false;
        s_status.host_connected = tud_connected() || tud_mounted();
        s_status.bus_active = s_status.host_connected;
        return;
    }
    force_host_reenumeration_if_needed();
    s_status.host_connected = tud_connected() || tud_mounted();
    s_status.bus_active = s_status.host_connected;
}

static void soft_disconnect_host_for_restore(void)
{
    if (!tud_inited() ||
        (!s_status.host_connected && !tud_connected() && !tud_mounted())) {
        return;
    }
    s_forced_reconnect = true;
    ESP_LOGI(TAG, "Soft-disconnecting USB MSC before returning TF to application");
    (void)tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(180));
    s_status.host_connected = false;
    s_status.bus_active = false;
    s_status.last_sof_age_ms = UINT32_MAX;
    s_forced_reconnect = false;
}

static void refresh_stale_host_state(void)
{
    if (!s_status.initialized || !tud_inited()) {
        return;
    }
    bool tinyusb_connected = tud_connected() || tud_mounted();
    s_status.bus_active = tinyusb_connected;
    s_status.last_sof_age_ms = UINT32_MAX;
    if (s_status.host_connected && !tinyusb_connected) {
        s_status.host_connected = false;
        s_status.bus_active = false;
        set_error("USB host disconnected", ESP_OK);
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
    prepare_usb_descriptors();
    tinyusb_config.descriptor.string = s_usb_string_desc;
    tinyusb_config.descriptor.string_count = sizeof(s_usb_string_desc) / sizeof(s_usb_string_desc[0]);
    ret = tinyusb_driver_install(&tinyusb_config);
    if (ret != ESP_OK) {
        tinyusb_msc_uninstall_driver();
        set_error("TinyUSB install failed", ret);
        return ret;
    }
    s_status.initialized = true;
    reserve_dma_block();
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
    release_dma_reserve();

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
        reserve_dma_block();
        set_error("attach TF failed", ret);
        return ret;
    }

    s_status.storage_ready = true;
    set_error("USB host owns writable TF", ESP_OK);
    connect_host_after_media_change();
    ESP_LOGI(TAG, "Writable TF card exposed on USB1; Web restore or unplug can return TF to application");
    return ESP_OK;
}

esp_err_t usb_msc_export_detach_storage(void)
{
    if (!s_status.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_storage) {
        s_status.storage_ready = false;
        set_error("USB storage already detached", ESP_OK);
        return ESP_OK;
    }

    bool host_was_active = s_status.host_connected ||
                           (tud_inited() && (tud_connected() || tud_mounted()));
    if (host_was_active) {
        soft_disconnect_host_for_restore();
    }

    esp_err_t ret = ESP_OK;
    for (int attempt = 0; attempt < 12; attempt++) {
        ret = tinyusb_msc_delete_storage(s_storage);
        if (ret == ESP_OK || ret != ESP_ERR_INVALID_STATE) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (ret != ESP_OK) {
        set_error("detach TF failed", ret);
        if (host_was_active && tud_inited()) {
            (void)tud_connect();
        }
        return ret;
    }

    s_storage = NULL;
    s_status.storage_ready = false;
    reserve_dma_block();
    set_error("USB storage detached", ESP_OK);
    /*
     * Keep the USB device soft-disconnected after returning TF to the app.
     * Reconnecting an MSC interface without media leaves Windows with a
     * "No Media" P4_BUOY disk and makes the next Web restore/export cycle feel
     * stuck. The next explicit export attaches real TF media first, then calls
     * tud_connect() so Windows only sees a usable drive.
     */
    ESP_LOGI(TAG, "Writable TF card detached from USB MSC storage");
    return ESP_OK;
}

void usb_msc_export_get_status(usb_msc_export_status_t *status)
{
    refresh_stale_host_state();
    if (status) {
        *status = s_status;
    }
}
