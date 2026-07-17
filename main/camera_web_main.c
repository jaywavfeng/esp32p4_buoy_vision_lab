/*
 * ESP32-P4-WIFI6-DEV-KIT-A LAN camera server.
 *
 * Architecture:
 * - one camera task captures and encodes frames continuously
 * - the latest JPEG frame is copied into a shared PSRAM cache
 * - each /stream client runs on an async HTTP worker task
 * - power commands are queued to the camera task for fast HTTP response
 * - recognition_method keeps legacy backends in source, but the main UI/API path is off/tinycls/coco.
 * - network_mode controls sta/softap/apsta at runtime and is stored in NVS
 *
 * 中文结构说明：
 * - 摄像头任务负责 Wake/Standby、采集、编码和发布最新帧，网页命令只入队，不阻塞 HTTP。
 * - 独立 inference_task 负责板端模型推理，摄像头和 /stream 不等待模型跑完。
 * - HTTP worker 并发服务多个 /stream 客户端，所有客户端读取同一份最新帧缓存。
 * - 历史任务把抽帧识别结果写入 RAM 环形队列和 TF 卡，并按数量/索引大小自动淘汰旧数据。
 * - 当前主验证路径聚焦 Fish31/TinyCNN 分类和 COCO YOLO11n 检测；历史实验后端源码保留，不再作为页面默认入口。
 * - 无线模式运行时可切换：STA 连路由器，SoftAP 让板子自己开热点，APSTA 同时保留两条链路做稳定性对比。
 */

#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "diskio_sdmmc.h"
#include "wear_levelling.h"

#ifndef CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM
#define CONFIG_WIFI_RMT_STATIC_RX_BUFFER_NUM 10
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM
#define CONFIG_WIFI_RMT_DYNAMIC_RX_BUFFER_NUM 32
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_TX_BUFFER
#define CONFIG_WIFI_RMT_DYNAMIC_TX_BUFFER 1
#endif
#ifndef CONFIG_WIFI_RMT_TX_BUFFER_TYPE
#define CONFIG_WIFI_RMT_TX_BUFFER_TYPE 1
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_TX_BUFFER_NUM
#define CONFIG_WIFI_RMT_DYNAMIC_TX_BUFFER_NUM 32
#endif
#ifndef CONFIG_WIFI_RMT_DYNAMIC_RX_MGMT_BUF
#define CONFIG_WIFI_RMT_DYNAMIC_RX_MGMT_BUF 0
#endif
#ifndef CONFIG_WIFI_RMT_ESPNOW_MAX_ENCRYPT_NUM
#define CONFIG_WIFI_RMT_ESPNOW_MAX_ENCRYPT_NUM 7
#endif

#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"
#include "mdns.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"

#include "esp_video_device.h"
#include "example_video_common.h"
#include "avi_mjpeg_writer.h"
#include "coke_sprite_mlp_model.h"
#include "yolo11_espdl_bridge.h"
#include "yolo26_espdl_bridge.h"
#include "coco_espdl_bridge.h"
#include "tiny_cls_espdl_bridge.h"
#include "fish31_espdl_bridge.h"
#include "validation_assets_config.h"
#include "recording_enrichment.h"
#include "usb_msc_export.h"
#include "json_validator.h"
#include "json_writer.h"

#if CONFIG_ESP_HOSTED_ENABLED
#if !defined(CONFIG_ESP_HOSTED_CP_TARGET_ESP32C6) && !defined(CONFIG_SLAVE_IDF_TARGET_ESP32C6)
#error "This product build requires ESP32-C6 as ESP-Hosted coprocessor. Set ESP_IDF_VERSION=6.0 and rebuild."
#endif
#if defined(CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD) && !CONFIG_ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD
#error "ESP-Hosted must use the ESP32-P4 Function EV Board wiring."
#endif
#if !defined(CONFIG_ESP_HOSTED_SPI_HOST_INTERFACE) && !defined(CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE)
#error "This product build requires ESP-Hosted SPI or SDIO transport."
#endif
#if defined(CONFIG_ESP_HOSTED_SPI_HOST_INTERFACE)
#if CONFIG_ESP_HOSTED_SPI_GPIO_MOSI != 14 || CONFIG_ESP_HOSTED_SPI_GPIO_MISO != 15 || \
    CONFIG_ESP_HOSTED_SPI_GPIO_CLK != 18 || CONFIG_ESP_HOSTED_SPI_GPIO_CS != 19 || \
    CONFIG_ESP_HOSTED_SPI_GPIO_HANDSHAKE != 16 || CONFIG_ESP_HOSTED_SPI_GPIO_DATA_READY != 17 || \
    CONFIG_ESP_HOSTED_SPI_GPIO_RESET_SLAVE != 54
#error "ESP-Hosted SPI pins must match the ESP32-P4 Function EV Board C6 wiring."
#endif
#endif
#endif

/* ESP-Hosted is initialized explicitly in wifi_init_runtime().
 * Keep the minimal API declarations here so the storage maintenance window can
 * temporarily shut Hosted down before a TF remount/format attempt.
 */
#if CONFIG_ESP_HOSTED_ENABLED
extern int esp_hosted_init(void);
extern int esp_hosted_deinit(void);
extern int esp_hosted_connect_to_slave(void);
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#ifndef CONFIG_APP_ETH_ENABLE
#define CONFIG_APP_ETH_ENABLE 0
#endif
#ifndef CONFIG_APP_ETH_DHCP_TIMEOUT_MS
#define CONFIG_APP_ETH_DHCP_TIMEOUT_MS 8000
#endif
#ifndef CONFIG_APP_ETH_STATIC_FALLBACK_IP
#define CONFIG_APP_ETH_STATIC_FALLBACK_IP "169.254.100.2"
#endif
#ifndef CONFIG_APP_ETH_STATIC_FALLBACK_NETMASK
#define CONFIG_APP_ETH_STATIC_FALLBACK_NETMASK "255.255.0.0"
#endif
#ifndef CONFIG_APP_MDNS_ENABLE
#define CONFIG_APP_MDNS_ENABLE 0
#endif
#ifndef CONFIG_APP_HOSTNAME
#define CONFIG_APP_HOSTNAME "p4-buoy"
#endif
#ifndef CONFIG_APP_FILE_DOWNLOAD_CHUNK_BYTES
#define CONFIG_APP_FILE_DOWNLOAD_CHUNK_BYTES 32768
#endif

#define APP_ETH_PHY_ADDR 1
#define APP_ETH_PHY_RST_GPIO 51
#define APP_ETH_MDC_GPIO 31
#define APP_ETH_MDIO_GPIO 52
#define APP_ETH_RMII_CLK_GPIO 50
#define APP_ETH_RMII_TX_EN_GPIO 49
#define APP_ETH_RMII_TXD0_GPIO 34
#define APP_ETH_RMII_TXD1_GPIO 35
#define APP_ETH_RMII_CRS_DV_GPIO 28
#define APP_ETH_RMII_RXD0_GPIO 29
#define APP_ETH_RMII_RXD1_GPIO 30

#ifndef CONFIG_APP_BOOT_STANDBY
#define CONFIG_APP_BOOT_STANDBY 0
#endif
#ifndef CONFIG_APP_SD_ENABLE
#define CONFIG_APP_SD_ENABLE 0
#endif
#ifndef CONFIG_APP_SD_MOUNT_POINT
#define CONFIG_APP_SD_MOUNT_POINT "/sdcard"
#endif
#ifndef CONFIG_APP_FLASH_STORAGE_LABEL
#define CONFIG_APP_FLASH_STORAGE_LABEL "storage"
#endif
#ifndef CONFIG_APP_HISTORY_ENABLE
#define CONFIG_APP_HISTORY_ENABLE 0
#endif
#ifndef CONFIG_APP_HISTORY_SAVE_JPEG
#define CONFIG_APP_HISTORY_SAVE_JPEG 0
#endif
#ifndef CONFIG_APP_RECORDING_ENABLE
#define CONFIG_APP_RECORDING_ENABLE 0
#endif
#ifndef CONFIG_APP_RECORDING_SEGMENT_MS
#define CONFIG_APP_RECORDING_SEGMENT_MS 60000
#endif
#ifndef CONFIG_APP_RECORDING_MAX_FPS
#define CONFIG_APP_RECORDING_MAX_FPS 4
#endif
#ifndef CONFIG_APP_FIELD_RECORDING_MAX_FPS
#define CONFIG_APP_FIELD_RECORDING_MAX_FPS CONFIG_APP_RECORDING_MAX_FPS
#endif
#ifndef CONFIG_APP_RECORDING_MAX_SEGMENTS
#define CONFIG_APP_RECORDING_MAX_SEGMENTS 240
#endif
#ifndef CONFIG_APP_RECORDING_QUEUE_DEPTH
#define CONFIG_APP_RECORDING_QUEUE_DEPTH 4
#endif
#ifndef CONFIG_APP_SUMMARY_INTERVAL_MS
#define CONFIG_APP_SUMMARY_INTERVAL_MS 60000
#endif
#ifndef CONFIG_APP_DATASET_RUN_MAX_FRAMES
#define CONFIG_APP_DATASET_RUN_MAX_FRAMES 16
#endif
#ifndef CONFIG_APP_CAN_BOX_MIN_SCORE
#define CONFIG_APP_CAN_BOX_MIN_SCORE 96
#endif
#ifndef CONFIG_APP_INFERENCE_INTERVAL_MS
#define CONFIG_APP_INFERENCE_INTERVAL_MS 2000
#endif
#ifndef CONFIG_APP_DEFAULT_NETWORK_MODE
#define CONFIG_APP_DEFAULT_NETWORK_MODE 0
#endif
#ifndef CONFIG_APP_AP_STATIC_IP
#define CONFIG_APP_AP_STATIC_IP "192.168.4.1"
#endif
#ifndef CONFIG_APP_AP_NETMASK
#define CONFIG_APP_AP_NETMASK "255.255.255.0"
#endif
#ifndef CONFIG_APP_AP_GATEWAY
#define CONFIG_APP_AP_GATEWAY CONFIG_APP_AP_STATIC_IP
#endif
#ifndef CONFIG_APP_NETWORK_BOOT_WINDOW_MS
#define CONFIG_APP_NETWORK_BOOT_WINDOW_MS 300000
#endif
#ifndef CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS
#define CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS 300000
#endif
#ifndef CONFIG_APP_NETWORK_WATCHDOG_PERIOD_MS
#define CONFIG_APP_NETWORK_WATCHDOG_PERIOD_MS 5000
#endif
#ifndef CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS
#define CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS 0
#endif
#ifndef CONFIG_APP_DEFAULT_RECOGNITION_METHOD
#define CONFIG_APP_DEFAULT_RECOGNITION_METHOD 6
#endif
#ifndef CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE
#define CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE 0
#endif
#ifndef CONFIG_APP_SD_BUS_WIDTH
#define CONFIG_APP_SD_BUS_WIDTH 1
#endif
#ifndef CONFIG_APP_SD_MAX_FREQ_KHZ
#define CONFIG_APP_SD_MAX_FREQ_KHZ 10000
#endif
#ifndef CONFIG_APP_SD_USE_SDMMC
#define CONFIG_APP_SD_USE_SDMMC 1
#endif
#ifndef CONFIG_APP_SD_PIN_CMD
#define CONFIG_APP_SD_PIN_CMD 44
#endif
#ifndef CONFIG_APP_SD_PIN_CLK
#define CONFIG_APP_SD_PIN_CLK 43
#endif
#ifndef CONFIG_APP_SD_PIN_D0
#define CONFIG_APP_SD_PIN_D0 39
#endif
#ifndef CONFIG_APP_SD_PIN_D1
#define CONFIG_APP_SD_PIN_D1 40
#endif
#ifndef CONFIG_APP_SD_PIN_D2
#define CONFIG_APP_SD_PIN_D2 41
#endif
#ifndef CONFIG_APP_SD_PIN_D3
#define CONFIG_APP_SD_PIN_D3 42
#endif
#define APP_SD_POWER_RESET_PIN GPIO_NUM_45

#define WIFI_SSID CONFIG_APP_WIFI_SSID
#define WIFI_PASSWORD CONFIG_APP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY CONFIG_APP_WIFI_MAX_RETRY
#define WIFI_AP_SSID CONFIG_APP_AP_SSID
#define WIFI_AP_PASSWORD CONFIG_APP_AP_PASSWORD
#define SETTINGS_NAMESPACE "buoy_lab"
#define SETTINGS_VERSION 16
#define RUNTIME_CONFIG_BLOB_MAGIC 0x42434647U
#define RUNTIME_CONFIG_BLOB_VERSION 1U
#define RUNTIME_CONFIG_ACTIVE_KEY "cfg_active"
#define RUNTIME_CONFIG_SLOT0_KEY "cfg_slot0"
#define RUNTIME_CONFIG_SLOT1_KEY "cfg_slot1"
#define RUNTIME_CONFIG_FLAG_VISION BIT0
#define RUNTIME_CONFIG_FLAG_HISTORY BIT1
#define RUNTIME_CONFIG_FLAG_RECORDING BIT2
#define RUNTIME_CONFIG_FLAG_FIELD_AUTO BIT3
#define RUNTIME_CONFIG_FLAGS_MASK (RUNTIME_CONFIG_FLAG_VISION | \
                                   RUNTIME_CONFIG_FLAG_HISTORY | \
                                   RUNTIME_CONFIG_FLAG_RECORDING | \
                                   RUNTIME_CONFIG_FLAG_FIELD_AUTO)
#define YOLO26_MODEL_NAME "coke-sprite-yolo26n-416-p4"
#define YOLO11_MODEL_NAME "coke-sprite-yolo11n-416-p4"
#define COCO_MODEL_NAME "coco-yolo11n-320-s8-v3-p4"
#define TINYCLS_MODEL_NAME TINY_CLS_MODEL_NAME
#define FISH31_MODEL_NAME FISH31_CLS_MODEL_NAME
#define YOLO26_INPUT_SIZE 416U
#define YOLO11_INPUT_SIZE 416U
#define COCO_INPUT_SIZE 320U
#define TINYCLS_INPUT_SIZE TINY_CLS_INPUT_W
#define FISH31_INPUT_SIZE FISH31_INPUT_W
#define MLP_MODEL_BYTES (sizeof(CAN_CLASSIFIER_W1) + sizeof(CAN_CLASSIFIER_B1) + \
                         sizeof(CAN_CLASSIFIER_W2) + sizeof(CAN_CLASSIFIER_B2))
#define APP_MAX_DETECTIONS 8U
#define APP_YOLO_NMS_THRESHOLD_X100 70U

#define STREAM_BOUNDARY_TEXT CONFIG_APP_HTTP_PART_BOUNDARY
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY_TEXT
#define STREAM_BOUNDARY "\r\n--" STREAM_BOUNDARY_TEXT "\r\n"
#define STREAM_PART "Content-Type: image/jpeg\r\nContent-Length: %" PRIu32 "\r\nX-Frame-Seq: %" PRIu32 "\r\nX-Capture-Latency-Ms: %" PRId64 "\r\nX-Encode-Ms: %" PRId64 "\r\nX-Motion-Score: %" PRIu32 "\r\nX-Object-Score: %" PRIu32 "\r\nX-Scene: %s\r\n\r\n"
#define HTTP_STOP_QUIESCE_TIMEOUT_MS 65000U
#define HTTP_STOP_QUIESCE_POLL_MS 20U
#define HTTP_FILE_SEND_WAIT_TIMEOUT_SEC 5U
#define RECORDING_CLEANUP_BATCH_MAX 128U
#define RECORDING_CLEANUP_BATCH_FALLBACK 24U
#define RECORDING_CLEANUP_NAME_SIZE 256U
#define RECORDING_CLEANUP_LOCK_TIMEOUT_MS 1000U

#define HISTORY_ROOT_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4"
#define HISTORY_SNAPSHOT_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/snapshots"
#define HISTORY_JSONL_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.jsonl"
#define HISTORY_JSONL_OLD_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.old.jsonl"
#define RECORDING_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings"
#define RECORDING_RECOVERY_DIR RECORDING_DIR "/lost_found"
#define RECORDING_INDEX_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings.jsonl"
#define RECORDING_SUMMARY_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/summaries.jsonl"
#define RECORDING_INDEX_OLD_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings.old.jsonl"
#define RECORDING_SUMMARY_OLD_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/summaries.old.jsonl"
#define RECORDING_INDEX_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings.tmp.jsonl"
#define RECORDING_SUMMARY_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/summaries.tmp.jsonl"
#define RECORDING_INDEX_OLD_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings.old.tmp.jsonl"
#define RECORDING_SUMMARY_OLD_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/summaries.old.tmp.jsonl"
#define SESSION_INDEX_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/sessions.jsonl"
#define EVENT_INDEX_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/events.jsonl"
#define EVENT_INDEX_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/events.tmp.jsonl"
#define HISTORY_JSONL_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.tmp.jsonl"
#define HISTORY_JSONL_OLD_TMP_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.old.tmp.jsonl"
#define DATASET_ROOT_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/datasets"
#define DATASET_RUN_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/dataset_runs"
#define SNAPSHOT_URI_PREFIX "/snapshot/"
#define RECORDING_URI_PREFIX "/recording/"
#define RECORDING_META_URI_PREFIX "/recordingmeta/"
#define RECORDING_ANNOTATED_URI_PREFIX "/annotated/"
#define RECORDING_MANIFEST_URI "/api/recording/manifest"
#define RECORDING_FRAME_SVG_URI "/api/recording/frame.svg"
#define DATASET_FRAME_SVG_URI "/api/dataset/frame.svg"
#define RECORDING_AVI_HEADER_BYTES 224U
#define HISTORY_QUEUE_DEPTH 8
#define RECORDING_LABEL_BUCKETS 8
#define JSONL_TAIL_LINE_BYTES 2048
#define DATASET_NAME_MAX 32
#define DATASET_PATH_MAX 160
#define DATASET_IMAGE_UPLOAD_MAX_BYTES (8U * 1024U * 1024U)
#define DATASET_METADATA_UPLOAD_MAX_BYTES (1U * 1024U * 1024U)
#define DATASET_UPLOAD_FREE_RESERVE_BYTES (1U * 1024U * 1024U)
#define DATASET_UPLOAD_BACKUP_SUFFIX ".upload.bak"
#define DATASET_RECOVERY_MAX_DIR_DEPTH 4U
#define DATASET_RECOVERY_MAX_SCAN_ENTRIES 4096U
#define DATASET_RECOVERY_MAX_SCAN_MS 5000U
#define DATASET_RUN_LATENCY_CAP 512
#define BUILTIN_COCO_VIDEO_DATASET "coco_video_demo"
#define BUILTIN_COCO_VIDEO_FRAMES 16U
#define BUILTIN_TINYCLS_VIDEO_DATASET "tinycls_marine_demo"
#define BUILTIN_TINYCLS_VIDEO_FRAMES 16U
#define BUILTIN_FISH31_VIDEO_DATASET "fish31_video_demo"
#define BUILTIN_FISH31_VIDEO_FRAMES 16U
#define BOARD_IMAGE_VALIDATION_MAX_ANALYSIS_MS 2500
#define APP_JSONL_INDEX_VERSION 4U
#define APP_RECORDING_MIN_FREE_BYTES (512ULL * 1024ULL * 1024ULL)
#define APP_RECORDING_MIN_FREE_PERCENT 5U
#define APP_MIN_VALID_EPOCH_MS 1577836800000ULL
#define APP_MAX_VALID_EPOCH_MS 4102444800000ULL
#define APP_RECORDING_SEGMENT_MIN_MS 5000U
#define APP_RECORDING_SEGMENT_MAX_MS 14400000U
#define APP_FIELD_IDLE_TIMEOUT_MIN_MS 10000U
#define APP_FIELD_IDLE_TIMEOUT_MAX_MS 86400000U
#define APP_NETWORK_ACCESS_GRACE_MIN_MS 30000U
#define APP_WEB_CLIENT_TIMEOUT_MS 10000U
#define APP_WEB_CLIENT_SLOTS 12U

#define VISION_GRID_W 16
#define VISION_GRID_H 12
#define VISION_GRID_N (VISION_GRID_W * VISION_GRID_H)
#define CAN_CLASS_UNKNOWN 0
#define CAN_CLASS_COKE 1
#define CAN_CLASS_SPRITE 2

typedef enum {
    POWER_STATE_STARTING = 0,
    POWER_STATE_RUNNING,
    POWER_STATE_STOPPING,
    POWER_STATE_STANDBY,
    POWER_STATE_ERROR,
} power_state_t;

typedef enum {
    CAMERA_CMD_WAKE = 0,
    CAMERA_CMD_STANDBY,
} camera_cmd_t;

typedef enum {
    RECOGNITION_METHOD_OFF = 0,
    RECOGNITION_METHOD_MLP,
    RECOGNITION_METHOD_YOLO26,
    RECOGNITION_METHOD_YOLO11,
    RECOGNITION_METHOD_COCO,
    RECOGNITION_METHOD_TINYCLS,
    RECOGNITION_METHOD_FISH31,
} recognition_method_t;

typedef enum {
    VALIDATION_SAMPLE_NONE = 0,
    VALIDATION_SAMPLE_COKE,
    VALIDATION_SAMPLE_SPRITE,
    VALIDATION_SAMPLE_DEMO_01,
    VALIDATION_SAMPLE_DEMO_02,
    VALIDATION_SAMPLE_DEMO_03,
    VALIDATION_SAMPLE_DEMO_04,
    VALIDATION_SAMPLE_TINY_01,
    VALIDATION_SAMPLE_TINY_02,
    VALIDATION_SAMPLE_TINY_03,
    VALIDATION_SAMPLE_TINY_04,
    VALIDATION_SAMPLE_FISH31_01,
    VALIDATION_SAMPLE_FISH31_02,
    VALIDATION_SAMPLE_FISH31_03,
    VALIDATION_SAMPLE_FISH31_04,
} validation_sample_t;

typedef enum {
    NETWORK_MODE_STA = 0,
    NETWORK_MODE_SOFTAP,
    NETWORK_MODE_APSTA,
} network_mode_t;

typedef enum {
    APP_MODE_SERVER = 0,
    APP_MODE_FIELD,
    APP_MODE_EXPORT,
    APP_MODE_USB_EXPORT,
} app_mode_t;

typedef enum {
    RECORDING_KIND_RAW = 0,
    RECORDING_KIND_ANNOTATED,
} recording_kind_t;

typedef enum {
    TIME_SOURCE_UNSYNCED = 0,
    TIME_SOURCE_BROWSER,
    TIME_SOURCE_NTP,
} time_source_t;

typedef struct {
    int fd;
    bool valid;
    bool streaming;
    uint8_t *buffer[CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_count;
    uint32_t buffer_size;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t frame_rate;
    example_encoder_handle_t encoder;
    uint8_t *jpeg_buf;
    uint32_t jpeg_buf_size;
} camera_t;

typedef struct {
    uint32_t class_id;
    uint32_t score;
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
    char label[24];
} vision_detection_t;

typedef struct {
    uint32_t avg_luma;
    uint32_t avg_r;
    uint32_t avg_g;
    uint32_t avg_b;
    uint32_t motion_score;
    uint32_t edge_score;
    uint32_t unknown_score;
    uint32_t candidate_score;
    uint32_t box_min_score;
    uint32_t object_score;
    uint32_t object_count;
    uint32_t object_x;
    uint32_t object_y;
    uint32_t object_w;
    uint32_t object_h;
    uint32_t coke_score;
    uint32_t sprite_score;
    uint32_t detection_count;
    uint32_t raw_candidate_count;
    vision_detection_t detections[APP_MAX_DETECTIONS];
    uint32_t top_k_count;
    tiny_cls_topk_t top_k[TINY_CLS_TOP_K];
    bool motion;
    char scene[16];
    char color[16];
    char label[24];
    char object[32];
    char model[32];
    int64_t inference_ms;
    int64_t analysis_ms;
} vision_result_t;

typedef struct {
    uint32_t seq;
    uint32_t size;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t sensor_fps;
    int64_t timestamp_ms;
    int64_t capture_ms;
    int64_t encode_ms;
    vision_result_t vision;
} frame_meta_t;

typedef struct {
    uint32_t seq;
    uint32_t jpeg_size;
    uint32_t width;
    uint32_t height;
    int64_t timestamp_ms;
    int64_t capture_ms;
    int64_t encode_ms;
    int64_t stored_ms;
    vision_result_t vision;
    recognition_method_t recognition_method;
    network_mode_t network_mode;
    int rssi_dbm;
    uint32_t model_bytes;
    uint32_t model_input_size;
    uint32_t free_heap;
    uint32_t min_free_heap;
    uint32_t free_psram;
    uint32_t min_free_psram;
    char source[16];
    char snapshot[96];
} history_record_t;

typedef struct {
    history_record_t record;
    uint8_t *jpeg;
    uint32_t jpeg_size;
} history_item_t;

typedef struct {
    char label[24];
    uint32_t count;
} label_count_t;

typedef struct {
    frame_meta_t meta;
    uint8_t *jpeg;
    uint32_t jpeg_size;
    recording_kind_t kind;
    recognition_method_t method;
    bool finalize;
    SemaphoreHandle_t finalize_done;
} recording_item_t;

typedef struct {
    char addr[48];
    int64_t last_seen_ms;
} web_client_slot_t;

typedef struct {
    avi_mjpeg_writer_t *writer;
    FILE *meta_file;
    recording_kind_t kind;
    char name[96];
    char meta_name[96];
    char uri[128];
    char meta_uri[128];
    char part_path[384];
    char final_path[384];
    recognition_method_t method;
    char model[48];
    int64_t start_ms;
    int64_t last_ms;
    uint64_t start_epoch_ms;
    time_source_t time_source;
    uint32_t frames;
    uint32_t hit_frames;
    uint32_t detection_total;
    uint64_t bytes;
    label_count_t labels[RECORDING_LABEL_BUCKETS];
} recording_segment_t;

typedef struct {
    char dataset[DATASET_NAME_MAX];
    char run_id[80];
    recognition_method_t method;
    uint32_t limit;
    uint32_t stride;
} dataset_run_request_t;

typedef struct {
    bool queued;
    bool running;
    bool done;
    bool cancel;
    char dataset[DATASET_NAME_MAX];
    char run_id[80];
    char result_uri[160];
    char summary_uri[160];
    char last_error[128];
    recognition_method_t method;
    uint32_t limit;
    uint32_t stride;
    uint32_t processed;
    uint32_t ok_frames;
    uint32_t failed_frames;
    uint32_t detection_total;
    uint32_t avg_analysis_ms;
    uint32_t p95_analysis_ms;
    uint32_t max_analysis_ms;
    uint32_t last_frame_index;
    char last_overlay_uri[192];
    int64_t started_ms;
    int64_t finished_ms;
    label_count_t labels[RECORDING_LABEL_BUCKETS];
} dataset_run_status_t;

typedef struct {
    bool requested;
    bool running;
    bool cancelled;
    uint32_t target_ms;
    uint32_t input_segments;
    uint32_t processed_segments;
    uint32_t output_segments;
    char last_error[128];
} recording_resegment_status_t;

typedef enum {
    RECORDING_CLEANUP_IDLE = 0,
    RECORDING_CLEANUP_QUEUED,
    RECORDING_CLEANUP_RUNNING,
    RECORDING_CLEANUP_SUCCEEDED,
    RECORDING_CLEANUP_FAILED,
} recording_cleanup_state_t;

typedef struct {
    recording_cleanup_state_t state;
    uint32_t job_id;
    uint32_t total_files;
    uint32_t deleted_files;
    uint32_t remaining_files;
    uint32_t error_count;
    int first_errno;
    uint64_t freed_bytes;
    int64_t queued_ms;
    int64_t started_ms;
    int64_t finished_ms;
    char message[128];
} recording_cleanup_status_t;

typedef struct {
    uint32_t job_id;
} recording_cleanup_request_t;

typedef struct {
    bool valid;
    char run_id[80];
    char dataset[DATASET_NAME_MAX];
    uint32_t frame_index;
    vision_result_t vision;
    uint32_t source_w;
    uint32_t source_h;
} dataset_frame_cache_t;

typedef enum {
    STORAGE_SERVICE_IDLE = 0,
    STORAGE_SERVICE_STOPPING_NETWORK,
    STORAGE_SERVICE_HOSTED_DOWN,
    STORAGE_SERVICE_MOUNTING,
    STORAGE_SERVICE_AVAILABLE,
    STORAGE_SERVICE_UNMOUNTING,
    STORAGE_SERVICE_RESTORING_NETWORK,
    STORAGE_SERVICE_ERROR,
} storage_service_mode_t;

/*
 * Every operation that can change the application mode or TF ownership must
 * reserve this single admission token before it is queued or started.  The
 * request flags below are only wake-up/event state; they never grant ownership
 * by themselves.
 */
typedef enum {
    STORAGE_TRANSITION_NONE = 0,
    STORAGE_TRANSITION_MAINTENANCE,
    STORAGE_TRANSITION_RETRY,
    STORAGE_TRANSITION_FIELD,
    STORAGE_TRANSITION_EXPORT,
    STORAGE_TRANSITION_USB_EXPORT,
    STORAGE_TRANSITION_USB_RESTORE,
    STORAGE_TRANSITION_RECORDING_CLEANUP,
} storage_transition_t;

typedef struct {
    uint32_t hold_ms;
    bool format_requested;
} storage_service_request_t;

typedef struct {
    bool vision_enabled;
    bool history_enabled;
    bool recording_enabled;
    recognition_method_t recognition_method;
    power_state_t power_state;
} storage_runtime_snapshot_t;

typedef struct {
    SemaphoreHandle_t done;
    volatile uint32_t refs;
    bool publish_result;
    esp_err_t err;
    uint32_t id;
    validation_sample_t sample;
    recognition_method_t method;
    uint32_t box_min_score;
    vision_result_t vision;
    uint32_t source_w;
    uint32_t source_h;
    uint32_t jpeg_size;
    int64_t queued_ms;
    int64_t started_ms;
    int64_t completed_ms;
} validation_context_t;

typedef enum {
    VALIDATION_JOB_QUEUED = 0,
    VALIDATION_JOB_RUNNING,
    VALIDATION_JOB_SUCCEEDED,
    VALIDATION_JOB_FAILED,
} validation_job_state_t;

/*
 * /validate 页面使用的“最近一次验证结果”缓存。
 * HTTP 请求 /api/validate/run 会等待 inference_task 跑完模型；随后 /api/validate/overlay.svg
 * 只需要读取这里的结果并画 SVG，不再重复推理，避免一个页面刷新触发多次 YOLO。
 */
typedef struct {
    bool valid;
    uint32_t id;
    validation_job_state_t state;
    esp_err_t err;
    validation_sample_t sample;
    recognition_method_t method;
    uint32_t box_min_score;
    vision_result_t vision;
    uint32_t source_w;
    uint32_t source_h;
    uint32_t jpeg_size;
    int64_t queued_ms;
    int64_t started_ms;
    int64_t completed_ms;
} validation_cache_t;

/*
 * 推理任务队列只保存“要分析的一帧 JPEG 副本”和当时的元数据。
 * 摄像头任务发布图传后立刻返回采集循环，模型推理由 inference_task 独立完成。
 * 队列长度在 app_main() 中固定为 1：如果上一帧还没推理完，新的抽帧会被丢弃而不是堆积，避免内存被长耗时模型拖垮。
 */
typedef struct {
    uint8_t *jpeg;
    uint32_t jpeg_size;
    frame_meta_t meta;
    recognition_method_t method;
    /*
     * validation=true 表示这不是摄像头实时帧，而是 /validate 内嵌样例图。
     * 两种来源复用同一个队列和同一个 inference_task，能保证验证页面测到的是真实板端
     * JPEG 解码、letterbox、ESP-DL 推理、NMS 和坐标映射链路，而不是 PC 端或网页端模拟。
     */
    bool validation;
    validation_sample_t validation_sample;
    validation_context_t *validation_ctx;
    uint32_t box_min_score;
    int64_t queued_ms;
} inference_job_t;

typedef esp_err_t (*async_req_handler_t)(httpd_req_t *req);

typedef struct {
    httpd_req_t *req;
    async_req_handler_t handler;
} async_req_t;

typedef struct {
    uint32_t clients;
    uint32_t errors;
    uint64_t frames_total;
    uint64_t bytes_total;
    uint32_t fps_x100;
} stream_stats_snapshot_t;

static const char *TAG = "wifi_camera_web";
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_eth_netif;
static esp_eth_handle_t s_eth_handle;
static esp_eth_netif_glue_handle_t s_eth_glue;
static httpd_handle_t s_server;
static bool s_http_server_ready;
static QueueHandle_t s_camera_cmd_queue;
static QueueHandle_t s_async_req_queue;
static QueueHandle_t s_history_queue;
static QueueHandle_t s_recording_queue;
static QueueHandle_t s_inference_queue;
static QueueHandle_t s_netmode_queue;
static QueueHandle_t s_dataset_run_queue;
static QueueHandle_t s_storage_service_queue;
static QueueHandle_t s_recording_cleanup_queue;
static SemaphoreHandle_t s_async_worker_ready;
static SemaphoreHandle_t s_recording_finalize_done;
static SemaphoreHandle_t s_frame_lock;
static SemaphoreHandle_t s_history_lock;
static SemaphoreHandle_t s_validation_lock;
static SemaphoreHandle_t s_dataset_lock;
static SemaphoreHandle_t s_storage_lock;
static TaskHandle_t s_async_worker_handles[CONFIG_APP_MAX_STREAM_CLIENTS];
static TaskHandle_t s_camera_task_handle;
static TaskHandle_t s_inference_task_handle;
static TaskHandle_t s_history_task_handle;
static bool s_history_worker_busy;
static TaskHandle_t s_recording_task_handle;
static TaskHandle_t s_enrichment_task_handle;
static TaskHandle_t s_network_task_handle;
static TaskHandle_t s_eth_fallback_task_handle;
static TaskHandle_t s_dataset_task_handle;
static TaskHandle_t s_storage_service_task_handle;
static TaskHandle_t s_recording_cleanup_task_handle;

static camera_t s_camera = {
    .fd = -1,
};

static int s_retry_num;
static char s_ip_addr[16] = "0.0.0.0";
static char s_sta_ip_addr[16] = "0.0.0.0";
static char s_ap_ip_addr[16] = CONFIG_APP_AP_STATIC_IP;
static char s_eth_ip_addr[16] = "0.0.0.0";
static bool s_wifi_started;
static bool s_wifi_initialized;
static bool s_wifi_handlers_registered;
static bool s_eth_handlers_registered;
static volatile bool s_wifi_runtime_ready;
static volatile bool s_eth_runtime_ready;
static volatile bool s_eth_started;
static volatile bool s_eth_link_up;
static volatile bool s_eth_got_ip;
static volatile bool s_eth_static_fallback;
static int64_t s_eth_link_up_ms;
#if CONFIG_ESP_HOSTED_ENABLED && CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
/*
 * Hosted owns the global SDMMC host while Slot 1 is active. Slot 0 reuses that
 * host in SERVER_MODE, then initializes the host itself after Hosted is shut
 * down for FIELD_MODE.
 */
static volatile bool s_hosted_sdmmc_host_active = true;
#endif
static volatile uint32_t s_wifi_init_failures;
static char s_wifi_last_error[96] = "not started";
static char s_eth_last_error[96] = "disabled";
static volatile bool s_network_active;
static volatile bool s_network_shutdown_for_idle;
static volatile bool s_field_mode_requested;
static volatile bool s_export_mode_requested;
static volatile bool s_usb_export_requested;
static volatile bool s_usb_restore_requested;
static volatile bool s_usb_restore_manual_requested;
static volatile bool s_usb_restore_auto_blocked;
static volatile bool s_usb_host_seen_during_export;
static volatile bool s_wifi_reconfigure_requested;
static volatile bool s_usb_auto_export_suppressed;
static volatile bool s_mdns_started;
static volatile app_mode_t s_app_mode = APP_MODE_SERVER;
static volatile bool s_storage_quiescing;
static int64_t s_last_network_activity_ms;
static int64_t s_last_web_client_activity_ms;
static int64_t s_network_boot_window_until_ms;
static volatile bool s_field_idle_pause_latched = true;
static volatile bool s_storage_mount_allowed;
static bool s_video_hw_ready;
static volatile power_state_t s_power_state = CONFIG_APP_BOOT_STANDBY ? POWER_STATE_STANDBY : POWER_STATE_STARTING;
static volatile bool s_vision_enabled = true;
static volatile bool s_history_enabled = CONFIG_APP_HISTORY_ENABLE;
static volatile bool s_recording_enabled = CONFIG_APP_RECORDING_ENABLE;
static volatile uint32_t s_segment_sequence = 0;
static char s_current_segment_base[64] = {0};
static volatile uint32_t s_box_min_score = CONFIG_APP_CAN_BOX_MIN_SCORE;
static volatile uint32_t s_stream_max_fps = CONFIG_APP_STREAM_MAX_FPS;
static volatile uint32_t s_inference_interval_ms = CONFIG_APP_INFERENCE_INTERVAL_MS;
static volatile uint32_t s_history_sample_interval_ms = CONFIG_APP_HISTORY_SAMPLE_INTERVAL_MS;
static volatile uint32_t s_jpeg_quality = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
static volatile uint32_t s_recording_segment_ms = CONFIG_APP_RECORDING_SEGMENT_MS;
static volatile uint32_t s_field_idle_timeout_ms = CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS;
static volatile bool s_field_auto_enable = true;
static volatile recognition_method_t s_recognition_method = CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 6 ? RECOGNITION_METHOD_FISH31 :
                                                            (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 5 ? RECOGNITION_METHOD_TINYCLS :
                                                             (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 4 ? RECOGNITION_METHOD_COCO :
                                                              (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 3 ? RECOGNITION_METHOD_YOLO11 :
                                                               (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 2 ? RECOGNITION_METHOD_YOLO26 :
                                                                (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 0 ? RECOGNITION_METHOD_OFF :
                                                                 RECOGNITION_METHOD_MLP)))));
static volatile network_mode_t s_network_mode = CONFIG_APP_DEFAULT_NETWORK_MODE == 2 ? NETWORK_MODE_APSTA :
                                                (CONFIG_APP_DEFAULT_NETWORK_MODE == 1 ? NETWORK_MODE_SOFTAP :
                                                 NETWORK_MODE_STA);
static volatile bool s_rescue_ap_active;
static char s_camera_error[96] = "starting";
static char s_storage_status[128] = "not mounted";
static char s_sd_mount_mode[24] = "none";
static char s_sd_last_error[96] = "";
static volatile uint32_t s_sd_attempts;
static volatile int s_sd_last_error_code;
static volatile bool s_storage_write_verified;
static volatile bool s_storage_io_latched;
static volatile int s_storage_last_errno;
static int64_t s_storage_write_verified_ms;
static volatile bool s_storage_retry_requested;
static volatile uint32_t s_sd_format_count;
static volatile storage_service_mode_t s_storage_service_mode = STORAGE_SERVICE_IDLE;
static char s_storage_service_status[128] = "idle";
static volatile uint32_t s_storage_service_runs;
static volatile int s_storage_service_last_error_code;
static volatile bool s_storage_service_last_mount_ok;
static volatile storage_transition_t s_storage_transition_owner = STORAGE_TRANSITION_NONE;
static char s_storage_service_last_mode[24] = "none";
static volatile bool s_storage_boot_probe_queued;
static bool s_field_session_started;
static volatile bool s_time_sync_initialized;
static volatile time_source_t s_time_source = TIME_SOURCE_UNSYNCED;

static uint8_t *s_latest_jpeg;
static uint32_t s_frame_capacity;
static bool s_have_frame;
static frame_meta_t s_latest_meta;
/*
 * YOLO 推理是异步完成的，尤其 YOLO26 可能需要二十秒左右。
 * 如果只把结果写在 latest_meta 里，摄像头任务发布新帧时可能用 waiting 状态覆盖刚完成的结果。
 * 因此这里单独保存“最近一次真正完成的 YOLO 结果”，供后续视频帧复用。
 */
static vision_result_t s_last_completed_yolo_vision;
static recognition_method_t s_last_completed_yolo_method = RECOGNITION_METHOD_OFF;
static bool s_have_completed_yolo_vision;

static bool s_sd_mounted;
static sd_pwr_ctrl_handle_t s_sd_pwr_ctrl;
static spi_host_device_t s_sd_spi_host = SPI2_HOST;
static sdmmc_card_t *s_sd_card;
static sdmmc_card_t *s_usb_sd_card;
static wl_handle_t s_flash_wl_handle = WL_INVALID_HANDLE;
static char s_storage_backend[20] = "none";
static char s_router_ssid[33] = WIFI_SSID;
static char s_router_password[65] = WIFI_PASSWORD;
static volatile bool s_storage_flash_fallback_enabled;
static uint64_t s_sd_total_bytes;
static uint64_t s_sd_free_bytes;
static uint32_t s_boot_id;
static history_record_t *s_history_records;
static uint32_t s_history_head;
static uint32_t s_history_count;
static int64_t s_last_history_ms;
static validation_cache_t s_validation_last;
static volatile uint32_t s_validation_runs;
static volatile uint32_t s_validation_errors;
static volatile uint32_t s_validation_id;
static volatile uint32_t s_validation_active_jobs;
static dataset_run_status_t s_dataset_status;
static dataset_frame_cache_t *s_dataset_frame_cache;

static uint8_t s_prev_luma_grid[VISION_GRID_N];
static bool s_prev_luma_valid;

static volatile uint32_t s_requests;
static volatile uint32_t s_frames_total;
static volatile uint32_t s_capture_errors;
static volatile uint32_t s_frame_drops;
static uint32_t s_stream_clients;
static volatile uint32_t s_file_download_clients;
static uint32_t s_stream_errors;
static uint64_t s_stream_frames_total;
static uint64_t s_stream_bytes_total;
static volatile uint32_t s_capture_fps_x100;
static uint32_t s_stream_fps_x100;
static volatile uint32_t s_standby_requests;
static volatile uint32_t s_wake_requests;
static volatile uint32_t s_reconnect_count;
static volatile uint32_t s_ap_clients;
static volatile uint32_t s_netmode_switches;
static volatile uint32_t s_inference_frames_total;
static volatile uint32_t s_inference_dropped_frames;
static volatile uint32_t s_inference_jobs_queued;
static volatile uint32_t s_inference_jobs_completed;
static volatile uint32_t s_inference_queue_drops;
static volatile bool s_inference_worker_busy;
static int64_t s_last_inference_ms;
static volatile uint32_t s_history_saved;
static volatile uint32_t s_history_queued;
static volatile uint32_t s_history_dropped;
static volatile uint32_t s_history_files_deleted;
static volatile uint32_t s_history_sd_errors;
static volatile uint32_t s_recording_segments;
static volatile uint32_t s_recording_frames;
static volatile uint32_t s_recording_queued;
static volatile uint32_t s_recording_dropped;
static volatile uint32_t s_recording_files_deleted;
static volatile uint32_t s_recording_sd_errors;
static volatile uint32_t s_recording_zero_frame_archives;
static volatile uint32_t s_recording_summary_count;
static volatile uint32_t s_recording_current_frames;
static volatile uint64_t s_recording_bytes;
static volatile uint64_t s_recording_current_bytes;
static int64_t s_last_recording_frame_ms;
static volatile bool s_recording_reset_requested = false;
static char s_recording_current_uri[128];
static char s_usb_last_error[96] = "not initialized";
static volatile bool s_usb_storage_ready;
static bool s_usb_sd_card_initialized;
static bool s_usb_sdmmc_host_initialized;
static bool s_usb_sdmmc_slot_initialized;
static bool s_storage_vfs_cleanup_pending;
static bool s_sdmmc_slot_cleanup_pending;
static bool s_sdspi_bus_cleanup_pending;
static bool s_usb_prev_vision_enabled;
static bool s_usb_prev_history_enabled;
static bool s_usb_prev_recording_enabled;
static recognition_method_t s_usb_prev_recognition_method;
static power_state_t s_usb_prev_power_state;
static web_client_slot_t s_web_clients[APP_WEB_CLIENT_SLOTS];
static portMUX_TYPE s_web_client_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_web_client_count;
static portMUX_TYPE s_stream_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_http_activity_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_http_stopping;
static uint32_t s_async_active_requests;
static portMUX_TYPE s_resegment_mux = portMUX_INITIALIZER_UNLOCKED;
static recording_resegment_status_t s_resegment_status;
static portMUX_TYPE s_recording_cleanup_mux = portMUX_INITIALIZER_UNLOCKED;
static recording_cleanup_status_t s_recording_cleanup_status;
static volatile uint32_t s_recording_cleanup_job_sequence;

static uint32_t s_capture_fps_window_count;
static uint32_t s_stream_fps_window_count;
static uint32_t s_inference_fps_window_count;
static uint32_t s_inference_fps_x100;
static uint32_t s_min_free_heap;
static uint32_t s_min_free_psram;
static int64_t s_capture_fps_window_start_ms;
static int64_t s_stream_fps_window_start_ms;
static int64_t s_inference_fps_window_start_ms;

static uint8_t *alloc_psram_buffer(uint32_t size);
static int wifi_rssi(void);
static void sample_memory_stats(uint32_t *free_heap, uint32_t *min_free_heap,
                                uint32_t *free_psram, uint32_t *min_free_psram);
static bool inference_worker_busy(void);
static bool queue_inference_job(const uint8_t *jpeg, uint32_t jpeg_size,
                                 const frame_meta_t *meta, recognition_method_t method);
static bool recording_maybe_queue(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta);
static bool network_mode_has_sta(network_mode_t mode);
static bool network_mode_has_ap(network_mode_t mode);
static void mark_network_activity(void);
static void open_network_access_window(const char *reason);
static uint32_t web_client_count(int64_t now_ms);
static void record_http_request(httpd_req_t *req);
static void record_http_poll(httpd_req_t *req);
static esp_err_t eth_init_runtime(void);
static void log_acceleration_status(void);
static void dataset_status_copy(dataset_run_status_t *out);
static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end);
static esp_err_t config_get_handler(httpd_req_t *req);
static esp_err_t reject_non_post_method(httpd_req_t *req);
static esp_err_t send_customer_action_json(httpd_req_t *req, const char *status,
                                           const char *reason, const char *message,
                                           const char *action);
static esp_err_t queue_async_request(httpd_req_t *req, async_req_handler_t handler);
static bool file_download_reader_try_begin(void);
static void file_download_reader_end(void);
static esp_err_t send_file_download_unavailable(httpd_req_t *req);
static void cancel_dataset_for_storage_handoff(void);
static esp_err_t wait_for_usb_quiescence(uint32_t timeout_ms);
static uint32_t remove_recording_index_rows(const char *name, bool *failed);
static int delete_recording_files_by_name(const char *name, uint64_t *freed_bytes, bool *failed);
static esp_err_t cleanup_recording_temp_files(void);
static void recording_cleanup_status_copy(recording_cleanup_status_t *out);
static bool recording_cleanup_active(void);
static bool query_confirm_delete(httpd_req_t *req, char *query, size_t query_size);
static bool query_u32(const char *query, const char *key, uint32_t min_value,
                      uint32_t max_value, uint32_t *out_value);
static bool json_get_int64_field(const char *line, const char *key, int64_t *out);
static bool json_get_u32_field(const char *line, const char *key, uint32_t *out);
static bool json_get_string_field(const char *line, const char *key, char *out, size_t out_size);
static esp_err_t sync_and_close_file(FILE **file_ptr, bool sync_to_media,
                                     int *error_number);
static void storage_latch_io_error(const char *operation, int error_number);
static bool storage_errno_is_media_failure(int error_number);
static bool storage_backend_is_tf(void);
static bool storage_usb_owned(void);
static bool usb_auto_export_allowed_mode(app_mode_t mode);

static const char *recognition_method_name(recognition_method_t method)
{
    switch (method) {
    case RECOGNITION_METHOD_OFF:
        return "off";
    case RECOGNITION_METHOD_MLP:
        return "mlp";
    case RECOGNITION_METHOD_YOLO26:
        return "yolo26";
    case RECOGNITION_METHOD_YOLO11:
        return "yolo11";
    case RECOGNITION_METHOD_COCO:
        return "coco";
    case RECOGNITION_METHOD_TINYCLS:
        return "tinycls";
    case RECOGNITION_METHOD_FISH31:
        return "fish31";
    default:
        return "unknown";
    }
}

static const char *network_mode_name(network_mode_t mode)
{
    switch (mode) {
    case NETWORK_MODE_STA:
        return "sta";
    case NETWORK_MODE_SOFTAP:
        return "softap";
    case NETWORK_MODE_APSTA:
        return "apsta";
    default:
        return "unknown";
    }
}

static const char *app_mode_name(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SERVER:
        return "server";
    case APP_MODE_FIELD:
        return "field";
    case APP_MODE_EXPORT:
        return "export";
    case APP_MODE_USB_EXPORT:
        return "usb_export";
    default:
        return "unknown";
    }
}

static uint32_t active_model_bytes(void)
{
    recognition_method_t method = s_recognition_method;
    if (method == RECOGNITION_METHOD_MLP) {
        return (uint32_t)MLP_MODEL_BYTES;
    }
    if (method == RECOGNITION_METHOD_YOLO26) {
        return yolo26_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_YOLO11) {
        return yolo11_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return coco_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return tiny_cls_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return fish31_espdl_model_bytes();
    }
    return 0;
}

static uint32_t model_bytes_for_method(recognition_method_t method)
{
    if (method == RECOGNITION_METHOD_MLP) {
        return (uint32_t)MLP_MODEL_BYTES;
    }
    if (method == RECOGNITION_METHOD_YOLO26) {
        return yolo26_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_YOLO11) {
        return yolo11_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return coco_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return tiny_cls_espdl_model_bytes();
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return fish31_espdl_model_bytes();
    }
    return 0;
}

static const char *model_name_for_method(recognition_method_t method)
{
    if (method == RECOGNITION_METHOD_MLP) {
        return CAN_CLASSIFIER_MODEL_NAME;
    }
    if (method == RECOGNITION_METHOD_YOLO26) {
        return YOLO26_MODEL_NAME;
    }
    if (method == RECOGNITION_METHOD_YOLO11) {
        return YOLO11_MODEL_NAME;
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return COCO_MODEL_NAME;
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return TINYCLS_MODEL_NAME;
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return FISH31_MODEL_NAME;
    }
    return recognition_method_name(method);
}

static uint32_t model_input_size_for_method(recognition_method_t method)
{
    if (method == RECOGNITION_METHOD_FISH31) {
        return FISH31_INPUT_SIZE;
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return TINYCLS_INPUT_SIZE;
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return COCO_INPUT_SIZE;
    }
    if (method == RECOGNITION_METHOD_YOLO11) {
        return YOLO11_INPUT_SIZE;
    }
    if (method == RECOGNITION_METHOD_YOLO26) {
        return YOLO26_INPUT_SIZE;
    }
    if (method == RECOGNITION_METHOD_MLP) {
        return 16;
    }
    return 0;
}

static uint32_t model_class_count_for_method(recognition_method_t method)
{
    if (method == RECOGNITION_METHOD_COCO) {
        return COCO_ESPDL_CLASS_COUNT;
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return TINY_CLS_CLASS_COUNT;
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return FISH31_CLASS_COUNT;
    }
    if (method == RECOGNITION_METHOD_YOLO11 || method == RECOGNITION_METHOD_YOLO26) {
        return 2;
    }
    if (method == RECOGNITION_METHOD_MLP) {
        return 3;
    }
    return 0;
}

static uint32_t active_yolo_input_size(void)
{
    return model_input_size_for_method(s_recognition_method);
}

static bool active_yolo_available(void)
{
    recognition_method_t method = s_recognition_method;
    if (method == RECOGNITION_METHOD_YOLO11) {
        return yolo11_espdl_available();
    }
    if (method == RECOGNITION_METHOD_YOLO26) {
        return yolo26_espdl_available();
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return coco_espdl_available();
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return tiny_cls_espdl_available();
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return fish31_espdl_available();
    }
    return false;
}

static recognition_method_t preferred_recognition_method(void)
{
    if (fish31_espdl_available()) {
        return RECOGNITION_METHOD_FISH31;
    }
    if (tiny_cls_espdl_available()) {
        return RECOGNITION_METHOD_TINYCLS;
    }
    if (coco_espdl_available()) {
        return RECOGNITION_METHOD_COCO;
    }
    return RECOGNITION_METHOD_MLP;
}

static recognition_method_t configured_default_recognition_method(void)
{
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 6) {
        return RECOGNITION_METHOD_FISH31;
    }
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 5) {
        return RECOGNITION_METHOD_TINYCLS;
    }
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 4) {
        return RECOGNITION_METHOD_COCO;
    }
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 3) {
        return RECOGNITION_METHOD_YOLO11;
    }
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 2) {
        return RECOGNITION_METHOD_YOLO26;
    }
    if (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 0) {
        return RECOGNITION_METHOD_OFF;
    }
    return RECOGNITION_METHOD_MLP;
}

static recognition_method_t recognition_method_or_fallback(recognition_method_t method)
{
#if !CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE
    if (method == RECOGNITION_METHOD_MLP ||
        method == RECOGNITION_METHOD_YOLO11 ||
        method == RECOGNITION_METHOD_YOLO26) {
        return preferred_recognition_method();
    }
#endif
    if (method == RECOGNITION_METHOD_TINYCLS && !tiny_cls_espdl_available()) {
        return preferred_recognition_method();
    }
    if (method == RECOGNITION_METHOD_FISH31 && !fish31_espdl_available()) {
        return preferred_recognition_method();
    }
    if (method == RECOGNITION_METHOD_COCO && !coco_espdl_available()) {
        return preferred_recognition_method();
    }
    if (method == RECOGNITION_METHOD_YOLO11 && !yolo11_espdl_available()) {
        return preferred_recognition_method();
    }
    if (method == RECOGNITION_METHOD_YOLO26 && !yolo26_espdl_available()) {
        return preferred_recognition_method();
    }
    return method;
}

static void recording_time_slug(uint64_t epoch_ms, int64_t now_ms, char *out, size_t out_size)
{
    if (!out || out_size == 0) {
        return;
    }
    if (epoch_ms >= APP_MIN_VALID_EPOCH_MS && epoch_ms <= APP_MAX_VALID_EPOCH_MS) {
        time_t sec = (time_t)(epoch_ms / 1000ULL);
        struct tm tm_value = {0};
        localtime_r(&sec, &tm_value);
        if (strftime(out, out_size, "%Y%m%d_%H%M%S", &tm_value) > 0) {
            return;
        }
    }
    snprintf(out, out_size, "b%08" PRIx32 "_t%010" PRId64, s_boot_id, now_ms);
}

static recognition_method_t recognition_method_from_text_hint(const char *text)
{
    if (!text) {
        return RECOGNITION_METHOD_OFF;
    }
    if (strstr(text, "fish31") || strstr(text, "fish")) {
        return RECOGNITION_METHOD_FISH31;
    }
    if (strstr(text, "tinycls") || strstr(text, "tiny")) {
        return RECOGNITION_METHOD_TINYCLS;
    }
    if (strstr(text, "coco")) {
        return RECOGNITION_METHOD_COCO;
    }
    return RECOGNITION_METHOD_OFF;
}

static bool json_writer_append_detections(json_writer_t *writer,
                                          const vision_result_t *vision)
{
    if (!writer) {
        return false;
    }
    json_writer_appendf(writer, "[");
    uint32_t count = vision ? vision->detection_count : 0;
    if (count > APP_MAX_DETECTIONS) {
        count = APP_MAX_DETECTIONS;
    }
    for (uint32_t i = 0; i < count; i++) {
        const vision_detection_t *d = &vision->detections[i];
        json_writer_appendf(writer, "%s{\"label\":", i == 0 ? "" : ",");
        json_writer_append_escaped_string(writer, d->label);
        json_writer_appendf(writer,
                            ",\"class_id\":%" PRIu32 ",\"score\":%" PRIu32 ","
                            "\"x\":%" PRIu32 ",\"y\":%" PRIu32 ","
                            "\"w\":%" PRIu32 ",\"h\":%" PRIu32 "}",
                            d->class_id, d->score, d->x, d->y, d->w, d->h);
    }
    json_writer_appendf(writer, "]");
    return json_writer_ok(writer);
}

static bool detections_to_json(char *buf, size_t size, const vision_result_t *vision)
{
    if (!buf || size == 0) {
        return false;
    }
    json_writer_t writer;
    json_writer_init(&writer, buf, size);
    return json_writer_append_detections(&writer, vision);
}

static bool json_writer_append_top_k(json_writer_t *writer,
                                     const vision_result_t *vision)
{
    if (!writer) {
        return false;
    }
    json_writer_appendf(writer, "[");
    uint32_t count = vision ? vision->top_k_count : 0;
    if (count > TINY_CLS_TOP_K) {
        count = TINY_CLS_TOP_K;
    }
    for (uint32_t i = 0; i < count; i++) {
        const tiny_cls_topk_t *item = &vision->top_k[i];
        json_writer_appendf(writer, "%s{\"label\":", i == 0 ? "" : ",");
        json_writer_append_escaped_string(writer, item->label);
        json_writer_appendf(writer, ",\"class_id\":%" PRIu32 ",\"score\":%" PRIu32 "}",
                            item->class_id, item->score);
    }
    json_writer_appendf(writer, "]");
    return json_writer_ok(writer);
}

static bool top_k_to_json(char *buf, size_t size, const vision_result_t *vision)
{
    if (!buf || size == 0) {
        return false;
    }
    json_writer_t writer;
    json_writer_init(&writer, buf, size);
    return json_writer_append_top_k(&writer, vision);
}

static size_t base64_encoded_len(size_t len)
{
    return ((len + 2U) / 3U) * 4U;
}

static void base64_encode(char *dst, const uint8_t *src, size_t len)
{
    static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t out = 0;
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = ((uint32_t)src[i]) << 16;
        bool have_b = i + 1 < len;
        bool have_c = i + 2 < len;
        if (have_b) {
            v |= ((uint32_t)src[i + 1]) << 8;
        }
        if (have_c) {
            v |= src[i + 2];
        }
        dst[out++] = table[(v >> 18) & 0x3f];
        dst[out++] = table[(v >> 12) & 0x3f];
        dst[out++] = have_b ? table[(v >> 6) & 0x3f] : '=';
        dst[out++] = have_c ? table[v & 0x3f] : '=';
    }
    dst[out] = '\0';
}

/*
 * Ethernet file export benefits from fewer fread/send cycles. Keep the chunk
 * bounded, but use a larger window than the old Hosted SDIO-oriented path.
 */
#define HTTP_SAFE_CHUNK_BYTES ((size_t)CONFIG_APP_FILE_DOWNLOAD_CHUNK_BYTES)
#define HTTP_STREAM_SEND_CHUNK_BYTES 8192U

static esp_err_t http_send_chunk_part(httpd_req_t *req, const char *data, size_t len)
{
    if (len > 0 && !data) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > HTTP_SAFE_CHUNK_BYTES) {
            n = HTTP_SAFE_CHUNK_BYTES;
        }
        esp_err_t ret = httpd_resp_send_chunk(req, data + off, n);
        if (ret != ESP_OK) {
            return ret;
        }
        off += n;
        mark_network_activity();
    }
    return ESP_OK;
}

static esp_err_t http_send_stream_frame_chunked(httpd_req_t *req,
                                                const uint8_t *frame,
                                                size_t frame_len)
{
    if (frame_len > 0 && !frame) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *tx = (uint8_t *)heap_caps_malloc(HTTP_STREAM_SEND_CHUNK_BYTES,
                                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!tx) {
        tx = (uint8_t *)malloc(HTTP_STREAM_SEND_CHUNK_BYTES);
    }
    if (!tx) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = ESP_OK;
    size_t off = 0;
    while (off < frame_len) {
        size_t n = frame_len - off;
        if (n > HTTP_STREAM_SEND_CHUNK_BYTES) {
            n = HTTP_STREAM_SEND_CHUNK_BYTES;
        }
        memcpy(tx, frame + off, n);
        ret = httpd_resp_send_chunk(req, (const char *)tx, n);
        if (ret != ESP_OK) {
            break;
        }
        off += n;
        mark_network_activity();
    }

    free(tx);
    return ret;
}

static esp_err_t http_send_buffer_chunked(httpd_req_t *req, const char *data, size_t len)
{
    /*
     * These responses already live in one contiguous buffer, so advertise a
     * fixed Content-Length. This avoids a burst of tiny chunk headers and the
     * terminating chunk on the Hosted SDIO link.
     */
    return httpd_resp_send(req, data, len);
}

static esp_err_t http_send_cstr_chunked(httpd_req_t *req, const char *text)
{
    return http_send_buffer_chunked(req, text ? text : "", text ? strlen(text) : 0);
}

static bool svg_append_escaped_text(json_writer_t *writer, const char *text)
{
    if (!writer || !text) {
        return false;
    }
    const char *run = text;
    for (const char *p = text; *p; p++) {
        const char *entity = NULL;
        switch (*p) {
        case '&': entity = "&amp;"; break;
        case '<': entity = "&lt;"; break;
        case '>': entity = "&gt;"; break;
        case '"': entity = "&quot;"; break;
        case '\'': entity = "&apos;"; break;
        default: break;
        }
        if (!entity) {
            continue;
        }
        size_t run_len = (size_t)(p - run);
        if ((run_len > 0 &&
             !json_writer_appendf(writer, "%.*s", (int)run_len, run)) ||
            !json_writer_appendf(writer, "%s", entity)) {
            return false;
        }
        run = p + 1;
    }
    return json_writer_appendf(writer, "%s", run);
}

static esp_err_t send_overlay_svg_response(httpd_req_t *req,
                                           const uint8_t *jpeg,
                                           size_t jpeg_size,
                                           uint32_t source_w,
                                           uint32_t source_h,
                                           const vision_result_t *vision)
{
    if (!jpeg || jpeg_size == 0 || source_w == 0 || source_h == 0 || !vision) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad overlay input");
    }

    const char *data_prefix = "data:image/jpeg;base64,";
    size_t data_uri_len = strlen(data_prefix) + base64_encoded_len(jpeg_size) + 1U;
    char *data_uri = (char *)alloc_psram_buffer(data_uri_len);
    if (!data_uri) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no image data uri buffer");
    }
    strcpy(data_uri, data_prefix);
    base64_encode(data_uri + strlen(data_prefix), jpeg, jpeg_size);

    const size_t svg_cap = data_uri_len + 8192U;
    char *svg = (char *)alloc_psram_buffer(svg_cap);
    if (!svg) {
        free(data_uri);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no svg buffer");
    }

    json_writer_t svg_writer;
    json_writer_init(&svg_writer, svg, svg_cap);
    json_writer_appendf(
        &svg_writer,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %" PRIu32 " %" PRIu32 "\">"
        "<image href=\"%s\" x=\"0\" y=\"0\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" preserveAspectRatio=\"none\"/>"
        "<rect x=\"0\" y=\"0\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" fill=\"none\" stroke=\"#000\" stroke-width=\"2\"/>",
        source_w, source_h, data_uri, source_w, source_h, source_w, source_h);
    free(data_uri);

    uint32_t count = vision->detection_count;
    if (count > APP_MAX_DETECTIONS) {
        count = APP_MAX_DETECTIONS;
    }
    for (uint32_t i = 0; i < count && json_writer_ok(&svg_writer); i++) {
        const vision_detection_t *d = &vision->detections[i];
        uint32_t text_y = d->y > 24 ? d->y - 8 : d->y + d->h + 28;
        if (text_y > source_h - 4) {
            text_y = source_h > 8 ? source_h - 8 : source_h;
        }
        json_writer_appendf(
            &svg_writer,
            "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" "
            "fill=\"none\" stroke=\"#64d68a\" stroke-width=\"4\"/>"
            "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"430\" height=\"34\" fill=\"#000\" fill-opacity=\"0.75\"/>"
            "<text x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" fill=\"#64d68a\" font-size=\"24\" font-family=\"Arial,sans-serif\">",
            d->x, d->y, d->w, d->h,
            d->x, text_y > 28 ? text_y - 28 : 0, d->x + 8, text_y);
        svg_append_escaped_text(&svg_writer, d->label);
        json_writer_appendf(&svg_writer,
                            " %" PRIu32 "%% / threshold %" PRIu32 "%%</text>",
                            d->score, vision->box_min_score);
    }

    bool draw_candidate = count == 0 && vision->candidate_score > 0 &&
                          vision->object_w > 0 && vision->object_h > 0;
    if (draw_candidate && json_writer_ok(&svg_writer)) {
        uint32_t text_y = vision->object_y > 24 ? vision->object_y - 8 :
                          vision->object_y + vision->object_h + 28;
        if (text_y > source_h - 4) {
            text_y = source_h > 8 ? source_h - 8 : source_h;
        }
        json_writer_appendf(
            &svg_writer,
            "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" "
            "fill=\"none\" stroke=\"#ffcc66\" stroke-width=\"4\" stroke-dasharray=\"10 8\"/>"
            "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"520\" height=\"34\" fill=\"#000\" fill-opacity=\"0.75\"/>"
            "<text x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" fill=\"#ffcc66\" font-size=\"24\" font-family=\"Arial,sans-serif\">",
            vision->object_x, vision->object_y, vision->object_w,
            vision->object_h, vision->object_x,
            text_y > 28 ? text_y - 28 : 0, vision->object_x + 8, text_y);
        svg_append_escaped_text(&svg_writer, vision->label);
        json_writer_appendf(&svg_writer,
                            " %" PRIu32 "%% / threshold %" PRIu32 "%%</text>",
                            vision->candidate_score, vision->box_min_score);
    } else if (count == 0 && json_writer_ok(&svg_writer)) {
        if (vision->top_k_count > 0) {
            json_writer_appendf(
                &svg_writer,
                "<rect x=\"16\" y=\"16\" width=\"620\" height=\"150\" rx=\"8\" fill=\"#000\" fill-opacity=\"0.75\"/>"
                "<text x=\"28\" y=\"46\" fill=\"#ffcc66\" font-size=\"24\" font-family=\"Arial,sans-serif\">classification ");
            svg_append_escaped_text(&svg_writer, vision->label);
            json_writer_appendf(&svg_writer,
                                " %" PRIu32 "%% / threshold %" PRIu32 "%%</text>",
                                vision->candidate_score, vision->box_min_score);
            uint32_t bars = vision->top_k_count > TINY_CLS_TOP_K ? TINY_CLS_TOP_K : vision->top_k_count;
            for (uint32_t i = 0; i < bars && json_writer_ok(&svg_writer); i++) {
                const tiny_cls_topk_t *item = &vision->top_k[i];
                uint32_t bar_w = item->score > 100 ? 280 : (item->score * 280U) / 100U;
                uint32_t y = 72 + i * 28;
                json_writer_appendf(
                    &svg_writer,
                    "<text x=\"28\" y=\"%" PRIu32 "\" fill=\"#e8f2f8\" font-size=\"18\" font-family=\"Arial,sans-serif\">#%" PRIu32 " ",
                    y, i + 1);
                svg_append_escaped_text(&svg_writer, item->label);
                json_writer_appendf(
                    &svg_writer,
                    "</text><rect x=\"250\" y=\"%" PRIu32 "\" width=\"280\" height=\"14\" rx=\"4\" fill=\"#23303a\"/>"
                    "<rect x=\"250\" y=\"%" PRIu32 "\" width=\"%" PRIu32 "\" height=\"14\" rx=\"4\" fill=\"#64d68a\"/>"
                    "<text x=\"542\" y=\"%" PRIu32 "\" fill=\"#e8f2f8\" font-size=\"18\" font-family=\"Arial,sans-serif\">%" PRIu32 "%%</text>",
                    y - 15, y - 15, bar_w, y, item->score);
            }
        } else {
            json_writer_appendf(
                &svg_writer,
                "<rect x=\"16\" y=\"16\" width=\"560\" height=\"42\" fill=\"#000\" fill-opacity=\"0.75\"/>"
                "<text x=\"28\" y=\"46\" fill=\"#ffcc66\" font-size=\"24\" font-family=\"Arial,sans-serif\">"
                "no candidate / threshold %" PRIu32 "%%</text>",
                vision->box_min_score);
        }
    }
    json_writer_appendf(&svg_writer, "</svg>");
    if (!json_writer_ok(&svg_writer)) {
        free(svg);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "overlay SVG exceeds its safe response buffer");
    }

    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, svg);
    free(svg);
    return ret;
}

static const char s_validate_html[] =
"<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>板端验证可视化</title><style>"
":root{color-scheme:dark;--bg:#0d1117;--panel:#151b22;--line:#2d3740;--text:#eef3f7;--muted:#9aa8b3;--accent:#7cc7ff;--ok:#64d68a;--bad:#ff7b7b}"
"*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,'Segoe UI',sans-serif;background:var(--bg);color:var(--text)}"
"main{width:min(1120px,100%);margin:0 auto;padding:18px;display:grid;gap:16px}.top{display:flex;justify-content:space-between;gap:12px;align-items:center;flex-wrap:wrap}.title{font-size:22px;font-weight:750}.sub{color:var(--muted);font-size:13px}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:14px}.card,.panel{border:1px solid var(--line);border-radius:8px;background:var(--panel);overflow:hidden}.sample{height:290px;display:grid;place-items:center;background:#000}.sample img{width:100%;height:100%;object-fit:contain}.hint{padding:12px;color:#c7d2dc}.actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:10px}"
"button,select,input,a{border:1px solid var(--line);background:#22303a;color:var(--text);border-radius:8px;padding:9px 13px;text-decoration:none;font-weight:650}input{width:82px}button{cursor:pointer}button:disabled,select:disabled,input:disabled{opacity:.55;cursor:not-allowed}button:hover,a:hover,select:hover,input:hover{border-color:var(--accent)}"
".panel{padding:12px}.result{display:grid;grid-template-columns:minmax(0,1fr) 330px;gap:14px;align-items:start}.boxed{background:#050708;border:1px solid var(--line);border-radius:8px;min-height:260px;display:grid;place-items:center;overflow:hidden}.boxed img{width:100%;height:auto;display:block}.kv{display:grid;gap:8px}.row{border-top:1px solid var(--line);padding-top:8px}.ok{color:var(--ok)}.bad{color:var(--bad)}pre{white-space:pre-wrap;color:#c7d2dc;font-size:12px;margin:0}.videoProgress{min-width:110px;font-variant-numeric:tabular-nums}"
"@media(max-width:840px){.result{grid-template-columns:1fr}.sample{height:240px}}"
"</style></head><body><main>"
"<section class=\"top\"><div><div class=\"title\">板端验证可视化 Validation Lab</div>"
"<div class=\"sub\">点击“板端推理”后会立即获得任务编号，页面通过短请求显示排队和推理进度；模型较慢时不会阻塞其他 Web 功能，也无需重启设备。</div></div>"
"<a href=\"/\">返回首页</a></section>"
"<section class=\"panel\"><div class=\"actions\"><label>识别方法 <select id=\"method\" onchange=\"onMethodChange()\"><option value=\"fish31\">Fish31 MobileNetV3 224</option><option value=\"tinycls\">TinyCNN Marine 192</option><option value=\"coco\">COCO YOLO11n 320 fast</option></select></label><label>验证阈值 <input id=\"valBox\" type=\"number\" min=\"1\" max=\"100\" value=\"50\"></label><span class=\"sub\" id=\"methodHint\"></span></div></section>"
"<section class=\"panel\"><div class=\"title\" id=\"videoTitle\">模型视频验证</div><div class=\"actions\"><button id=\"videoStart\" onclick=\"startVideoVal()\">运行 16 帧板端推理</button><button id=\"videoPlay\" onclick=\"toggleVideoPlayback()\" disabled>播放</button><span class=\"videoProgress\" id=\"videoProgress\">0 / 16</span><a href=\"/api/datasets\" target=\"_blank\">查看数据集</a></div><div class=\"sub\" id=\"videoVal\"></div></section>"
"<section class=\"grid\" id=\"sampleGrid\"></section>"
"<section class=\"result\"><div class=\"boxed\"><img id=\"boxed\" alt=\"boxed validation result\"><div id=\"empty\" class=\"sub\">等待点击样例图进行板端推理</div></div><aside class=\"panel\"><div class=\"title\">推理结果</div><div class=\"kv\" id=\"summary\"></div><pre id=\"raw\"></pre></aside></section>"
"</main><script>"
"function esc(v){return String(v==null?'':v).replace(/[&<>\"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[m]))}"
"function cname(o){let m={unknown:'unknown',plastic_bottle:'plastic bottle',foam:'foam',buoy:'buoy',net:'net',ship_part:'ship part'};return m[o]||o||'unknown'}"
"function detRows(ds){if(!ds||!ds.length)return '<div class=\"row\">正式检测框：0</div>';return '<div class=\"row\">正式检测框 '+ds.length+'</div>'+ds.map((d,i)=>'<div class=\"row\"><b>#'+(i+1)+'</b> '+cname(d.label)+' '+d.score+'% <span class=\"sub\">x='+d.x+' y='+d.y+' w='+d.w+' h='+d.h+'</span></div>').join('')}"
"function topRows(top){if(!top||!top.length)return '';return '<div class=\"row\">Top-K '+top.map(x=>cname(x.label)+' '+x.score+'%').join(' | ')+'</div>'}"
"const METHOD_DEMOS={fish31:{hint:'Fish31 使用标准鱼类/水下背景候选，经 ESP32-P4 板端筛选后只嵌入 Top-1 正确且高置信的 4 张不同样例；分类可视化显示 Top-1 横幅和 Top-K，不画检测框。',videoTitle:'Fish31 鱼类分类视频验证',videoHint:'固件内置 16 帧 Fish31 板端实测样例，循环覆盖 4 张非 unknown 分类图；运行时逐帧显示分类横幅和 Top-K。',dataset:'fish31_video_demo',samples:[['fish31_01','/validate/fish31_01.jpg','Fish31 标准候选 01：板端 Top-1 通过'],['fish31_02','/validate/fish31_02.jpg','Fish31 标准候选 02：板端 Top-1 通过'],['fish31_03','/validate/fish31_03.jpg','Fish31 标准候选 03：板端 Top-1 通过'],['fish31_04','/validate/fish31_04.jpg','Fish31 标准候选 04：板端 Top-1 通过']]},tinycls:{hint:'TinyCNN Marine 使用 LaRS/PoTATO/本地候选经 ESP32-P4 板端筛选后的高置信样例；只展示 Top-1 正确、非 unknown 的 4 张不同图片。分类可视化显示 Top-1 横幅和 Top-K，不画检测框。',videoTitle:'TinyCNN marine 视频验证',videoHint:'固件内置 16 帧板端实测非 unknown 样例循环；运行时逐帧显示 TinyCNN 分类横幅和 Top-K。',dataset:'tinycls_marine_demo',samples:[['tiny_01','/validate/tiny_01.jpg','TinyCNN 标准候选 01：非 unknown，板端 Top-1 通过'],['tiny_02','/validate/tiny_02.jpg','TinyCNN 标准候选 02：非 unknown，板端 Top-1 通过'],['tiny_03','/validate/tiny_03.jpg','TinyCNN 标准候选 03：非 unknown，板端 Top-1 通过'],['tiny_04','/validate/tiny_04.jpg','TinyCNN 标准候选 04：非 unknown，板端 Top-1 通过']]},coco:{hint:'COCO 使用通用目标检测数据：person / bottle / cup / chair 等；COCO 可视化显示检测框，分类模型不会使用检测框。',videoTitle:'COCO 真实视频验证',videoHint:'固件内置商店过道连续视频帧；运行时逐帧显示 COCO 检测框，完成后以 1 FPS 循环播放。',dataset:'coco_video_demo',samples:[['demo_01','/validate/demo_01.jpg','COCO 经典 01：多人、餐桌、杯子、碗、披萨'],['demo_02','/validate/demo_02.jpg','COCO 经典 02：多人、椅子、瓶子、杯子、餐桌'],['demo_03','/validate/demo_03.jpg','COCO 经典 03：多人、椅子、瓶子、餐桌、雨伞'],['demo_04','/validate/demo_04.jpg','COCO 经典 04：多人、雨伞、碗、杯子、餐桌']]}};"
"let validationBusy=false,validationSeq=0,videoBusy=false,videoRunId='',videoDataset='',videoFrames=[],videoFrameIndex=0,videoTimer=null,videoPollToken=0;"
"function currentDemo(){return METHOD_DEMOS[method.value]||METHOD_DEMOS.fish31}"
"function renderSamples(){let d=currentDemo();methodHint.textContent=d.hint;videoTitle.textContent=d.videoTitle;videoVal.textContent=d.videoHint;sampleGrid.innerHTML=d.samples.map(s=>'<div class=\"card\"><div class=\"sample\"><img src=\"'+esc(s[1])+'\" alt=\"'+esc(s[0])+'\"></div><div class=\"hint\">'+esc(s[2])+'<div class=\"actions\"><button class=\"valBtn\" onclick=\"runVal(\\''+esc(s[0])+'\\')\">板端推理</button><a href=\"'+esc(s[1])+'\" target=\"_blank\">打开原图</a></div></div></div>').join('');setBusy(false)}"
"function onMethodChange(){clearVideoFrames();renderSamples();summary.innerHTML='';raw.textContent='';boxed.removeAttribute('src');empty.style.display='block';empty.textContent='等待点击样例图进行板端推理'}"
"function setBusy(b){validationBusy=b;document.querySelectorAll('.valBtn').forEach(x=>x.disabled=b||videoBusy);method.disabled=b||videoBusy;valBox.disabled=b||videoBusy;videoStart.disabled=b||videoBusy;videoPlay.disabled=b||videoBusy||!videoFrames.some(Boolean)}"
"function setVideoBusy(b){videoBusy=b;document.querySelectorAll('.valBtn').forEach(x=>x.disabled=b||validationBusy);method.disabled=b||validationBusy;valBox.disabled=b||validationBusy;videoStart.disabled=b||validationBusy;videoPlay.disabled=b||!videoFrames.some(Boolean)}"
"function stopVideoPlayback(){if(videoTimer){clearInterval(videoTimer);videoTimer=null}videoPlay.textContent='播放'}"
"function clearVideoFrames(){stopVideoPlayback();videoFrames.forEach(x=>{if(x)URL.revokeObjectURL(x)});videoFrames=[];videoFrameIndex=0;videoProgress.textContent='0 / 16';videoPlay.disabled=true}"
"function showVideoFrame(i){if(!videoFrames.length)return;let n=videoFrames.length;for(let step=0;step<n;step++){let k=(i+step+n)%n;if(videoFrames[k]){videoFrameIndex=k;boxed.src=videoFrames[k];empty.style.display='none';videoProgress.textContent=(k+1)+' / '+n;return}}}"
"function startVideoPlayback(){if(!videoFrames.some(Boolean))return;stopVideoPlayback();showVideoFrame(videoFrameIndex);videoPlay.textContent='暂停';videoTimer=setInterval(()=>showVideoFrame(videoFrameIndex+1),1000)}"
"function toggleVideoPlayback(){if(videoTimer)stopVideoPlayback();else startVideoPlayback()}"
"function videoOverlayUri(index){return '/api/dataset/frame.svg?run_id='+encodeURIComponent(videoRunId)+'&dataset='+encodeURIComponent(videoDataset)+'&index='+index+'&ts='+Date.now()}"
"async function ensureVideoFrame(index,token){let slot=index-1;if(videoFrames[slot])return videoFrames[slot];let r=await fetch(videoOverlayUri(index),{cache:'no-store'});if(!r.ok)throw new Error('overlay '+index+' HTTP '+r.status);let blob=await r.blob();if(token!==videoPollToken)return '';let url=URL.createObjectURL(blob);videoFrames[slot]=url;return url}"
"function sleepMs(ms){return new Promise(resolve=>setTimeout(resolve,ms))}"
"function valErr(j,status){if(status===409)return '设备正在执行另一项验证或视频分析，请等待完成后重试，无需重启';if(status===410)return '结果已被另一浏览器的新任务替换，请重新运行';if(status===503)return '设备正在切换模式或内存暂时不足，请等待稳定、关闭视频流后重试';return (j&&(j.message||j.error))||(status?'HTTP '+status:'未知错误')}"
"function renderVal(j,dt){empty.style.display='none';boxed.src=j.overlay+'&ts='+Date.now();let v=j.vision||{},ds=v.detections||j.detections||[],top=v.top_k||j.top_k||[];summary.innerHTML='<div class=\"row '+(j.matched?'ok':'bad')+'\">期望 '+esc(j.expected)+'，识别 '+cname(v.object)+'，'+(j.matched?'命中':'未命中')+'</div>'+'<div class=\"row\">方法 '+esc(j.method)+' | 模型 '+esc(v.model)+' | 模型 '+Math.round((j.model_bytes||0)/1024)+' KB | 输入 '+(j.model_input_size||0)+'</div>'+'<div class=\"row\">候选分 '+v.candidate_score+' | 验证阈值 '+v.box_min_score+' | NMS '+j.nms_threshold+' | raw候选 '+j.raw_candidate_count+'</div>'+'<div class=\"row\">推理 '+v.inference_ms+' ms | 分析 '+v.analysis_ms+' ms | 网页等待 '+dt+' ms</div>'+'<div class=\"row\">原图 '+j.source_w+'x'+j.source_h+' | JPEG '+Math.round(j.jpeg_bytes/1024)+' KB</div>'+topRows(top)+detRows(ds);raw.textContent=JSON.stringify(j,null,2)}"
"async function pollVal(id,seq,t){let failures=0;for(let i=0;i<240;i++){if(seq!==validationSeq)return null;await sleepMs(i?500:150);try{let r=await fetch('/api/validate/status?id='+encodeURIComponent(id)+'&ts='+Date.now(),{cache:'no-store'}),text=await r.text(),j={};try{j=text?JSON.parse(text):{}}catch(e){j={error:text}}if(!r.ok){let e=new Error(valErr(j,r.status));e.fatal=r.status<500;throw e}failures=0;raw.textContent=JSON.stringify(j,null,2);if(j.state==='queued'||j.state==='running'){let phase=j.state==='queued'?'已排队，等待推理资源':'板端正在推理，Web 服务仍可正常使用';summary.innerHTML='<div class=\"row\">任务 #'+id+'：'+phase+'</div><div class=\"row sub\">已等待 '+Math.round((performance.now()-t)/1000)+' 秒，请勿重复提交或重启设备。</div>';continue}return j}catch(e){if(e.fatal)throw e;failures++;if(failures>5)throw new Error('状态查询连续失败，任务可能仍在设备上运行：'+e.message);summary.innerHTML='<div class=\"row\">网络暂时不稳定，正在继续查询任务 #'+id+'（'+failures+'/5）</div>';await sleepMs(Math.min(4000,500*Math.pow(2,failures-1)))}}throw new Error('任务 #'+id+' 超过 2 分钟仍未完成，设备可能仍在推理；无需重启，请稍后重新运行或检查状态')}"
"async function runVal(sample){if(validationBusy||videoBusy)return;stopVideoPlayback();let seq=++validationSeq;setBusy(true);summary.innerHTML='<div class=\"row\">正在提交板端推理任务，提交后页面会通过短请求查看进度。</div>';raw.textContent='';boxed.removeAttribute('src');empty.style.display='block';let m=method.value,threshold=Math.max(1,Math.min(100,Number(valBox.value)||50)),t=performance.now();try{let q=new URLSearchParams({sample:sample,method:m,box_min_score:String(threshold)}),r=await fetch('/api/validate/run',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString(),cache:'no-store'}),text=await r.text(),job={};try{job=text?JSON.parse(text):{}}catch(e){job={error:text}}if(!r.ok||!job.ok||!job.id)throw new Error(valErr(job,r.status));summary.innerHTML='<div class=\"row\">任务 #'+job.id+' 已提交，等待板端推理。</div>';let j=await pollVal(job.id,seq,t);if(!j||seq!==validationSeq)return;if(j.state==='failed'||!j.ok){summary.innerHTML='<div class=\"row bad\">验证失败：'+esc(j.message||j.error||'板端推理失败')+'</div><div class=\"row\">Web 服务仍可使用，无需重启设备。</div>';raw.textContent=JSON.stringify(j,null,2);return}renderVal(j,Math.round(performance.now()-t))}catch(e){if(seq===validationSeq){summary.innerHTML='<div class=\"row bad\">'+esc(e.message||e)+'</div><div class=\"row\">若任务已经提交，刷新页面不会取消板端任务，也不需要重启设备。</div>';raw.textContent=String(e)}}finally{if(seq===validationSeq)setBusy(false)}}"
"function labelSummary(labels){return (labels||[]).map(x=>esc(x.label)+' '+x.count).join('，')||'无目标'}"
"async function startVideoVal(){if(videoBusy||validationBusy)return;clearVideoFrames();videoPollToken++;let token=videoPollToken;let d=currentDemo();videoDataset=d.dataset;let m=method.value;setVideoBusy(true);boxed.removeAttribute('src');empty.style.display='block';empty.textContent='正在等待第一帧板端推理结果';summary.innerHTML='<div class=\"row\">正在启动 '+esc(d.videoTitle)+'...</div>';raw.textContent='';try{let r=await fetch('/api/dataset/run/start?dataset='+encodeURIComponent(videoDataset)+'&method='+encodeURIComponent(m)+'&limit=16&stride=1',{method:'POST',cache:'no-store'});let text=await r.text();let j={};try{j=JSON.parse(text)}catch(e){}if(!r.ok||!j.ok)throw new Error(j.error||text||('HTTP '+r.status));videoRunId=j.run_id;videoVal.textContent='已排队 '+videoRunId+' | '+d.videoHint;setTimeout(()=>pollVideoVal(token),200)}catch(e){videoVal.textContent='启动失败：'+e;summary.innerHTML='<div class=\"row bad\">视频验证启动失败</div>';setVideoBusy(false)}}"
"async function preloadVideoFrames(s,token){for(let i=0;i<s.processed;i++){if(token!==videoPollToken)return;let index=1+i*(s.stride||1);await ensureVideoFrame(index,token)}if(token!==videoPollToken)return;videoPlay.disabled=false;videoFrameIndex=0;startVideoPlayback()}"
"async function pollVideoVal(token){if(token!==videoPollToken||!videoRunId)return;try{let r=await fetch('/api/dataset/run/status?ts='+Date.now(),{cache:'no-store'});let s=await r.json();if(s.run_id!==videoRunId)throw new Error('run_id mismatch');let links=(!s.running&&s.result_uri)?' | <a href=\"'+esc(s.result_uri)+'\" target=\"_blank\">JSONL</a> | <a href=\"'+esc(s.summary_uri)+'\" target=\"_blank\">summary</a>':'';videoVal.innerHTML='状态 '+esc(s.state||'idle')+' | 方法 '+esc(s.method||'')+' | 帧 '+s.processed+'/'+s.limit+' | ok '+s.ok_frames+' | 失败 '+s.failed_frames+' | 平均 '+s.avg_analysis_ms+' ms | P95 '+s.p95_analysis_ms+' ms | 最大 '+s.max_analysis_ms+' ms | 目标 '+s.detection_total+' | '+labelSummary(s.labels)+' | '+esc(s.error||'')+links;videoProgress.textContent=s.processed+' / '+s.limit;summary.innerHTML='<div class=\"row '+((s.failed_frames||s.error)?'bad':'ok')+'\">视频板端推理 '+s.ok_frames+'/'+s.limit+'</div><div class=\"row\">目标 '+s.detection_total+' | '+labelSummary(s.labels)+'</div><div class=\"row\">平均 '+s.avg_analysis_ms+' ms | P95 '+s.p95_analysis_ms+' ms | 最大 '+s.max_analysis_ms+' ms</div>';raw.textContent=JSON.stringify(s,null,2);if(s.last_frame_index>0){let url=await ensureVideoFrame(s.last_frame_index,token);if(url&&token===videoPollToken){boxed.src=url;empty.style.display='none';videoProgress.textContent=s.last_frame_index+' / '+s.limit}}if(s.queued||s.running){setTimeout(()=>pollVideoVal(token),400);return}if(s.done){await preloadVideoFrames(s,token);setVideoBusy(false);return}setVideoBusy(false)}catch(e){if(token===videoPollToken){videoVal.textContent='视频验证状态错误：'+e;summary.innerHTML='<div class=\"row bad\">视频验证失败</div>';setVideoBusy(false)}}}"
"renderSamples();"
"</script></body></html>";

extern const uint8_t validate_demo_01_jpg_start[] asm("_binary_demo_01_jpg_start");
extern const uint8_t validate_demo_01_jpg_end[] asm("_binary_demo_01_jpg_end");
extern const uint8_t validate_demo_02_jpg_start[] asm("_binary_demo_02_jpg_start");
extern const uint8_t validate_demo_02_jpg_end[] asm("_binary_demo_02_jpg_end");
extern const uint8_t validate_demo_03_jpg_start[] asm("_binary_demo_03_jpg_start");
extern const uint8_t validate_demo_03_jpg_end[] asm("_binary_demo_03_jpg_end");
extern const uint8_t validate_demo_04_jpg_start[] asm("_binary_demo_04_jpg_start");
extern const uint8_t validate_demo_04_jpg_end[] asm("_binary_demo_04_jpg_end");
extern const uint8_t validate_tiny_01_jpg_start[] asm("_binary_tiny_01_jpg_start");
extern const uint8_t validate_tiny_01_jpg_end[] asm("_binary_tiny_01_jpg_end");
extern const uint8_t validate_tiny_02_jpg_start[] asm("_binary_tiny_02_jpg_start");
extern const uint8_t validate_tiny_02_jpg_end[] asm("_binary_tiny_02_jpg_end");
extern const uint8_t validate_tiny_03_jpg_start[] asm("_binary_tiny_03_jpg_start");
extern const uint8_t validate_tiny_03_jpg_end[] asm("_binary_tiny_03_jpg_end");
extern const uint8_t validate_tiny_04_jpg_start[] asm("_binary_tiny_04_jpg_start");
extern const uint8_t validate_tiny_04_jpg_end[] asm("_binary_tiny_04_jpg_end");
extern const uint8_t validate_fish31_01_jpg_start[] asm("_binary_fish31_01_jpg_start");
extern const uint8_t validate_fish31_01_jpg_end[] asm("_binary_fish31_01_jpg_end");
extern const uint8_t validate_fish31_02_jpg_start[] asm("_binary_fish31_02_jpg_start");
extern const uint8_t validate_fish31_02_jpg_end[] asm("_binary_fish31_02_jpg_end");
extern const uint8_t validate_fish31_03_jpg_start[] asm("_binary_fish31_03_jpg_start");
extern const uint8_t validate_fish31_03_jpg_end[] asm("_binary_fish31_03_jpg_end");
extern const uint8_t validate_fish31_04_jpg_start[] asm("_binary_fish31_04_jpg_start");
extern const uint8_t validate_fish31_04_jpg_end[] asm("_binary_fish31_04_jpg_end");

#define DECLARE_COCO_VIDEO_FRAME(n) \
    extern const uint8_t coco_video_frame_##n##_jpg_start[] asm("_binary_frame_" #n "_jpg_start"); \
    extern const uint8_t coco_video_frame_##n##_jpg_end[] asm("_binary_frame_" #n "_jpg_end")
DECLARE_COCO_VIDEO_FRAME(00001);
DECLARE_COCO_VIDEO_FRAME(00002);
DECLARE_COCO_VIDEO_FRAME(00003);
DECLARE_COCO_VIDEO_FRAME(00004);
DECLARE_COCO_VIDEO_FRAME(00005);
DECLARE_COCO_VIDEO_FRAME(00006);
DECLARE_COCO_VIDEO_FRAME(00007);
DECLARE_COCO_VIDEO_FRAME(00008);
DECLARE_COCO_VIDEO_FRAME(00009);
DECLARE_COCO_VIDEO_FRAME(00010);
DECLARE_COCO_VIDEO_FRAME(00011);
DECLARE_COCO_VIDEO_FRAME(00012);
DECLARE_COCO_VIDEO_FRAME(00013);
DECLARE_COCO_VIDEO_FRAME(00014);
DECLARE_COCO_VIDEO_FRAME(00015);
DECLARE_COCO_VIDEO_FRAME(00016);
#undef DECLARE_COCO_VIDEO_FRAME

#define DECLARE_TINYCLS_VIDEO_FRAME(n) \
    extern const uint8_t tinycls_video_frame_##n##_jpg_start[] asm("_binary_tiny_frame_" #n "_jpg_start"); \
    extern const uint8_t tinycls_video_frame_##n##_jpg_end[] asm("_binary_tiny_frame_" #n "_jpg_end")
DECLARE_TINYCLS_VIDEO_FRAME(00001);
DECLARE_TINYCLS_VIDEO_FRAME(00002);
DECLARE_TINYCLS_VIDEO_FRAME(00003);
DECLARE_TINYCLS_VIDEO_FRAME(00004);
DECLARE_TINYCLS_VIDEO_FRAME(00005);
DECLARE_TINYCLS_VIDEO_FRAME(00006);
DECLARE_TINYCLS_VIDEO_FRAME(00007);
DECLARE_TINYCLS_VIDEO_FRAME(00008);
DECLARE_TINYCLS_VIDEO_FRAME(00009);
DECLARE_TINYCLS_VIDEO_FRAME(00010);
DECLARE_TINYCLS_VIDEO_FRAME(00011);
DECLARE_TINYCLS_VIDEO_FRAME(00012);
DECLARE_TINYCLS_VIDEO_FRAME(00013);
DECLARE_TINYCLS_VIDEO_FRAME(00014);
DECLARE_TINYCLS_VIDEO_FRAME(00015);
DECLARE_TINYCLS_VIDEO_FRAME(00016);
#undef DECLARE_TINYCLS_VIDEO_FRAME

#define DECLARE_FISH31_VIDEO_FRAME(n) \
    extern const uint8_t fish31_video_frame_##n##_jpg_start[] asm("_binary_fish31_frame_" #n "_jpg_start"); \
    extern const uint8_t fish31_video_frame_##n##_jpg_end[] asm("_binary_fish31_frame_" #n "_jpg_end")
DECLARE_FISH31_VIDEO_FRAME(00001);
DECLARE_FISH31_VIDEO_FRAME(00002);
DECLARE_FISH31_VIDEO_FRAME(00003);
DECLARE_FISH31_VIDEO_FRAME(00004);
DECLARE_FISH31_VIDEO_FRAME(00005);
DECLARE_FISH31_VIDEO_FRAME(00006);
DECLARE_FISH31_VIDEO_FRAME(00007);
DECLARE_FISH31_VIDEO_FRAME(00008);
DECLARE_FISH31_VIDEO_FRAME(00009);
DECLARE_FISH31_VIDEO_FRAME(00010);
DECLARE_FISH31_VIDEO_FRAME(00011);
DECLARE_FISH31_VIDEO_FRAME(00012);
DECLARE_FISH31_VIDEO_FRAME(00013);
DECLARE_FISH31_VIDEO_FRAME(00014);
DECLARE_FISH31_VIDEO_FRAME(00015);
DECLARE_FISH31_VIDEO_FRAME(00016);
#undef DECLARE_FISH31_VIDEO_FRAME

static esp_err_t ioctl_to_esp(int rc)
{
    return rc == 0 ? ESP_OK : ESP_FAIL;
}

static const char *power_state_name(power_state_t state)
{
    switch (state) {
    case POWER_STATE_STARTING:
        return "starting";
    case POWER_STATE_RUNNING:
        return "running";
    case POWER_STATE_STOPPING:
        return "stopping";
    case POWER_STATE_STANDBY:
        return "standby";
    case POWER_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "interrupt_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "watchdog";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    case ESP_RST_UNKNOWN:
    default:
        return "unknown";
    }
}

static const char *time_source_name(time_source_t source)
{
    switch (source) {
    case TIME_SOURCE_NTP:
        return "ntp";
    case TIME_SOURCE_BROWSER:
        return "browser";
    default:
        return "unsynced";
    }
}

static uint64_t wall_clock_epoch_ms(void)
{
    struct timeval tv = {0};
    if (gettimeofday(&tv, NULL) != 0 || tv.tv_sec < 0) {
        return 0;
    }
    uint64_t epoch_ms = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
    return epoch_ms >= APP_MIN_VALID_EPOCH_MS && epoch_ms <= APP_MAX_VALID_EPOCH_MS ?
           epoch_ms : 0;
}

static void time_sync_notification_cb(struct timeval *tv)
{
    if (!tv) {
        return;
    }
    uint64_t epoch_ms = (uint64_t)tv->tv_sec * 1000ULL + (uint64_t)tv->tv_usec / 1000ULL;
    if (epoch_ms >= APP_MIN_VALID_EPOCH_MS && epoch_ms <= APP_MAX_VALID_EPOCH_MS) {
        s_time_source = TIME_SOURCE_NTP;
        ESP_LOGI(TAG, "system time synchronized by NTP: epoch_ms=%" PRIu64, epoch_ms);
    }
}

static esp_err_t init_time_sync(void)
{
    if (s_time_sync_initialized) {
        return ESP_OK;
    }

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.sync_cb = time_sync_notification_cb;
    esp_err_t ret = esp_netif_sntp_init(&config);
    if (ret == ESP_OK) {
        s_time_sync_initialized = true;
        ESP_LOGI(TAG, "SNTP initialized with pool.ntp.org");
    } else if (ret == ESP_ERR_INVALID_STATE) {
        s_time_sync_initialized = true;
        ret = ESP_OK;
    }
    return ret;
}

static void set_power_state(power_state_t state)
{
    s_power_state = state;
}

static void set_camera_error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_camera_error, sizeof(s_camera_error), fmt, args);
    va_end(args);
}

static void set_storage_status(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_storage_status, sizeof(s_storage_status), fmt, args);
    va_end(args);
}

static const char *storage_service_mode_name(storage_service_mode_t mode)
{
    switch (mode) {
    case STORAGE_SERVICE_IDLE:
        return "idle";
    case STORAGE_SERVICE_STOPPING_NETWORK:
        return "stopping_network";
    case STORAGE_SERVICE_HOSTED_DOWN:
        return "hosted_down";
    case STORAGE_SERVICE_MOUNTING:
        return "mounting";
    case STORAGE_SERVICE_AVAILABLE:
        return "available";
    case STORAGE_SERVICE_UNMOUNTING:
        return "unmounting";
    case STORAGE_SERVICE_RESTORING_NETWORK:
        return "restoring_network";
    case STORAGE_SERVICE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

static void set_storage_service_state(storage_service_mode_t mode, const char *fmt, ...)
{
    s_storage_service_mode = mode;
    va_list args;
    va_start(args, fmt);
    vsnprintf(s_storage_service_status, sizeof(s_storage_service_status), fmt, args);
    va_end(args);
    ESP_LOGI(TAG, "storage service: %s - %s",
             storage_service_mode_name(mode), s_storage_service_status);
}

static const char *storage_transition_name(storage_transition_t transition)
{
    switch (transition) {
    case STORAGE_TRANSITION_NONE:
        return "none";
    case STORAGE_TRANSITION_MAINTENANCE:
        return "maintenance";
    case STORAGE_TRANSITION_RETRY:
        return "retry";
    case STORAGE_TRANSITION_FIELD:
        return "field";
    case STORAGE_TRANSITION_EXPORT:
        return "export";
    case STORAGE_TRANSITION_USB_EXPORT:
        return "usb_export";
    case STORAGE_TRANSITION_USB_RESTORE:
        return "usb_restore";
    case STORAGE_TRANSITION_RECORDING_CLEANUP:
        return "recording_cleanup";
    default:
        return "unknown";
    }
}

static storage_transition_t storage_transition_owner(void)
{
    return __atomic_load_n(&s_storage_transition_owner, __ATOMIC_ACQUIRE);
}

static bool storage_transition_active(void)
{
    return storage_transition_owner() != STORAGE_TRANSITION_NONE;
}

static bool storage_transition_try_acquire(storage_transition_t transition)
{
    if (transition == STORAGE_TRANSITION_NONE) {
        return false;
    }
    storage_transition_t expected = STORAGE_TRANSITION_NONE;
    return __atomic_compare_exchange_n(&s_storage_transition_owner, &expected, transition,
                                       false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static void storage_transition_release(storage_transition_t transition)
{
    storage_transition_t expected = transition;
    if (!__atomic_compare_exchange_n(&s_storage_transition_owner, &expected,
                                     STORAGE_TRANSITION_NONE, false,
                                     __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        ESP_LOGE(TAG, "storage transition release mismatch: expected=%s actual=%s",
                 storage_transition_name(transition), storage_transition_name(expected));
    }
}

static bool storage_request_pending(const volatile bool *request)
{
    return __atomic_load_n(request, __ATOMIC_ACQUIRE);
}

static void storage_request_set(volatile bool *request)
{
    __atomic_store_n(request, true, __ATOMIC_RELEASE);
}

static bool storage_request_take(volatile bool *request)
{
    return __atomic_exchange_n(request, false, __ATOMIC_ACQ_REL);
}

/* Keep the event pending when another transition owns admission. */
static bool storage_transition_reserve_event(storage_transition_t transition,
                                             volatile bool *request)
{
    storage_request_set(request);
    storage_transition_t owner = storage_transition_owner();
    if (owner == transition) {
        return true;
    }
    if (owner == STORAGE_TRANSITION_NONE &&
        storage_transition_try_acquire(transition)) {
        return true;
    }
    return storage_transition_owner() == transition;
}

static bool storage_mode_request_pending(void)
{
    return storage_request_pending(&s_field_mode_requested) ||
           storage_request_pending(&s_export_mode_requested) ||
           storage_request_pending(&s_usb_export_requested) ||
           storage_request_pending(&s_usb_restore_requested);
}

static bool storage_any_request_pending(void)
{
    return storage_request_pending(&s_storage_retry_requested) ||
           storage_mode_request_pending();
}

static void field_idle_pause_latch(void)
{
    __atomic_store_n(&s_field_idle_pause_latched, true, __ATOMIC_RELEASE);
}

static void field_idle_reanchor_after_pause(int64_t now_ms)
{
    __atomic_store_n(&s_last_network_activity_ms, now_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_last_web_client_activity_ms, now_ms, __ATOMIC_RELEASE);
}

static void fill_vision_disabled(vision_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->box_min_score = s_box_min_score;
    strlcpy(result->scene, "disabled", sizeof(result->scene));
    strlcpy(result->color, "none", sizeof(result->color));
    strlcpy(result->label, "off", sizeof(result->label));
    strlcpy(result->object, "unknown", sizeof(result->object));
    strlcpy(result->model, "off", sizeof(result->model));
}

static const char *fourcc_to_str(uint32_t fourcc, char out[5])
{
    out[0] = fourcc & 0xff;
    out[1] = (fourcc >> 8) & 0xff;
    out[2] = (fourcc >> 16) & 0xff;
    out[3] = (fourcc >> 24) & 0xff;
    out[4] = '\0';
    return out;
}

static uint8_t scale5(uint16_t v)
{
    return (uint8_t)((v * 255U) / 31U);
}

static uint8_t scale6(uint16_t v)
{
    return (uint8_t)((v * 255U) / 63U);
}

static bool sample_pixel(const uint8_t *data, uint32_t data_size, uint32_t width, uint32_t height,
                         uint32_t pixel_format, uint32_t x, uint32_t y,
                         uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *luma)
{
    if (!data || x >= width || y >= height) {
        return false;
    }

    if (pixel_format == V4L2_PIX_FMT_RGB565 || pixel_format == V4L2_PIX_FMT_RGB565X) {
        size_t off = ((size_t)y * width + x) * 2;
        if (off + 1 >= data_size) {
            return false;
        }
        uint16_t p = pixel_format == V4L2_PIX_FMT_RGB565 ?
                     ((uint16_t)data[off] | ((uint16_t)data[off + 1] << 8)) :
                     (((uint16_t)data[off] << 8) | (uint16_t)data[off + 1]);
        *r = scale5((p >> 11) & 0x1f);
        *g = scale6((p >> 5) & 0x3f);
        *b = scale5(p & 0x1f);
    } else if (pixel_format == V4L2_PIX_FMT_GREY || pixel_format == V4L2_PIX_FMT_SBGGR8) {
        size_t off = (size_t)y * width + x;
        if (off >= data_size) {
            return false;
        }
        *r = data[off];
        *g = data[off];
        *b = data[off];
    } else if (pixel_format == V4L2_PIX_FMT_RGB24) {
        size_t off = ((size_t)y * width + x) * 3;
        if (off + 2 >= data_size) {
            return false;
        }
        *r = data[off];
        *g = data[off + 1];
        *b = data[off + 2];
    } else if (pixel_format == V4L2_PIX_FMT_YUYV || pixel_format == V4L2_PIX_FMT_UYVY) {
        size_t pair = ((size_t)y * width + (x & ~1U)) * 2;
        if (pair + 3 >= data_size) {
            return false;
        }
        uint8_t yv;
        if (pixel_format == V4L2_PIX_FMT_YUYV) {
            yv = (x & 1U) ? data[pair + 2] : data[pair];
        } else {
            yv = (x & 1U) ? data[pair + 3] : data[pair + 1];
        }
        *r = yv;
        *g = yv;
        *b = yv;
    } else {
        return false;
    }

    *luma = (uint8_t)(((uint32_t)(*r) * 30U + (uint32_t)(*g) * 59U + (uint32_t)(*b) * 11U) / 100U);
    return true;
}

static uint32_t score_from_prob(float prob)
{
    int score = (int)(prob * 100.0f + 0.5f);
    if (score < 0) {
        return 0;
    }
    if (score > 100) {
        return 100;
    }
    return (uint32_t)score;
}

static void softmax3(const float logits[CAN_CLASSIFIER_CLASSES], float probs[CAN_CLASSIFIER_CLASSES])
{
    float max_logit = logits[0];
    for (int i = 1; i < CAN_CLASSIFIER_CLASSES; i++) {
        if (logits[i] > max_logit) {
            max_logit = logits[i];
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < CAN_CLASSIFIER_CLASSES; i++) {
        probs[i] = expf(logits[i] - max_logit);
        sum += probs[i];
    }
    if (sum <= 0.0f) {
        probs[CAN_CLASS_UNKNOWN] = 1.0f;
        probs[CAN_CLASS_COKE] = 0.0f;
        probs[CAN_CLASS_SPRITE] = 0.0f;
        return;
    }
    for (int i = 0; i < CAN_CLASSIFIER_CLASSES; i++) {
        probs[i] /= sum;
    }
}

static bool run_can_mlp_window(const uint8_t *data, uint32_t data_size, uint32_t width,
                               uint32_t height, uint32_t pixel_format, uint32_t x0,
                               uint32_t y0, uint32_t win_w, uint32_t win_h,
                               float probs[CAN_CLASSIFIER_CLASSES])
{
    if (!data || !width || !height || !win_w || !win_h || x0 >= width || y0 >= height) {
        return false;
    }

    float input[CAN_CLASSIFIER_INPUTS];
    uint32_t idx = 0;
    for (uint32_t iy = 0; iy < CAN_CLASSIFIER_INPUT_H; iy++) {
        uint32_t y = y0 + (iy * win_h + win_h / 2) / CAN_CLASSIFIER_INPUT_H;
        if (y >= height) {
            y = height - 1;
        }
        for (uint32_t ix = 0; ix < CAN_CLASSIFIER_INPUT_W; ix++) {
            uint32_t x = x0 + (ix * win_w + win_w / 2) / CAN_CLASSIFIER_INPUT_W;
            if (x >= width) {
                x = width - 1;
            }

            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            uint8_t luma = 0;
            if (!sample_pixel(data, data_size, width, height, pixel_format, x, y, &r, &g, &b, &luma)) {
                return false;
            }
            input[idx++] = (float)r * CAN_CLASSIFIER_INPUT_SCALE + CAN_CLASSIFIER_INPUT_OFFSET;
            input[idx++] = (float)g * CAN_CLASSIFIER_INPUT_SCALE + CAN_CLASSIFIER_INPUT_OFFSET;
            input[idx++] = (float)b * CAN_CLASSIFIER_INPUT_SCALE + CAN_CLASSIFIER_INPUT_OFFSET;
        }
    }

    float hidden[CAN_CLASSIFIER_HIDDEN];
    for (int h = 0; h < CAN_CLASSIFIER_HIDDEN; h++) {
        float sum = CAN_CLASSIFIER_B1[h];
        for (int i = 0; i < CAN_CLASSIFIER_INPUTS; i++) {
            sum += CAN_CLASSIFIER_W1[h][i] * input[i];
        }
        hidden[h] = sum > 0.0f ? sum : 0.0f;
    }

    float logits[CAN_CLASSIFIER_CLASSES];
    for (int c = 0; c < CAN_CLASSIFIER_CLASSES; c++) {
        float sum = CAN_CLASSIFIER_B2[c];
        for (int h = 0; h < CAN_CLASSIFIER_HIDDEN; h++) {
            sum += CAN_CLASSIFIER_W2[c][h] * hidden[h];
        }
        logits[c] = sum;
    }
    softmax3(logits, probs);
    return true;
}

static void update_best_can_window(const float probs[CAN_CLASSIFIER_CLASSES], uint32_t x,
                                   uint32_t y, uint32_t w, uint32_t h, float *best_score,
                                   int *best_class, uint32_t *best_x, uint32_t *best_y,
                                   uint32_t *best_w, uint32_t *best_h,
                                   float best_probs[CAN_CLASSIFIER_CLASSES])
{
    int cls = probs[CAN_CLASS_COKE] >= probs[CAN_CLASS_SPRITE] ? CAN_CLASS_COKE : CAN_CLASS_SPRITE;
    float score = probs[cls];
    if (score <= *best_score) {
        return;
    }

    *best_score = score;
    *best_class = cls;
    *best_x = x;
    *best_y = y;
    *best_w = w;
    *best_h = h;
    for (int i = 0; i < CAN_CLASSIFIER_CLASSES; i++) {
        best_probs[i] = probs[i];
    }
}

/*
 * 多尺度滑窗识别：把每个候选窗口缩放为 16x16 RGB，送入一个单隐藏层 MLP。
 * candidate_score 会记录最高候选分；只有分数达到 Kconfig 阈值时才画框并写入历史目标。
 */
static void classify_can_candidate(const uint8_t *data, uint32_t data_size, uint32_t width,
                                   uint32_t height, uint32_t pixel_format, vision_result_t *result)
{
    int64_t inference_start_us = esp_timer_get_time();
    float best_probs[CAN_CLASSIFIER_CLASSES] = {1.0f, 0.0f, 0.0f};
    float best_score = 0.0f;
    int best_class = CAN_CLASS_UNKNOWN;
    uint32_t best_x = 0;
    uint32_t best_y = 0;
    uint32_t best_w = width;
    uint32_t best_h = height;
    const uint32_t min_side = width < height ? width : height;
    const uint32_t scales[] = {35, 50, 68, 88};

    result->box_min_score = s_box_min_score;
    if (!data || !width || !height || min_side < 32) {
        result->inference_ms = (esp_timer_get_time() - inference_start_us) / 1000;
        return;
    }

    float probs[CAN_CLASSIFIER_CLASSES];
    if (run_can_mlp_window(data, data_size, width, height, pixel_format, 0, 0,
                           width, height, probs)) {
        update_best_can_window(probs, 0, 0, width, height, &best_score,
                               &best_class, &best_x, &best_y, &best_w, &best_h, best_probs);
    }

    for (size_t si = 0; si < sizeof(scales) / sizeof(scales[0]); si++) {
        uint32_t side = (min_side * scales[si]) / 100U;
        if (side < 32) {
            side = 32;
        }
        if (side > width) {
            side = width;
        }
        if (side > height) {
            side = height;
        }

        uint32_t step = side / 2U;
        if (step < 24) {
            step = 24;
        }
        uint32_t max_y = height - side;
        for (uint32_t y = 0;;) {
            uint32_t max_x = width - side;
            for (uint32_t x = 0;;) {
                if (run_can_mlp_window(data, data_size, width, height, pixel_format,
                                       x, y, side, side, probs)) {
                    update_best_can_window(probs, x, y, side, side, &best_score,
                                           &best_class, &best_x, &best_y, &best_w,
                                           &best_h, best_probs);
                }
                if (x == max_x) {
                    break;
                }
                x = (x + step >= max_x) ? max_x : x + step;
            }
            if (y == max_y) {
                break;
            }
            y = (y + step >= max_y) ? max_y : y + step;
        }
    }

    result->coke_score = score_from_prob(best_probs[CAN_CLASS_COKE]);
    result->sprite_score = score_from_prob(best_probs[CAN_CLASS_SPRITE]);
    result->unknown_score = score_from_prob(best_probs[CAN_CLASS_UNKNOWN]);
    result->candidate_score = score_from_prob(best_score);
    result->object_score = 0;

    const float box_min_prob = (float)s_box_min_score / 100.0f;
    if (best_class != CAN_CLASS_UNKNOWN && best_score >= box_min_prob &&
        best_score > best_probs[CAN_CLASS_UNKNOWN]) {
        strlcpy(result->object, CAN_CLASSIFIER_LABELS[best_class], sizeof(result->object));
        result->object_score = result->candidate_score;
        result->object_count = 1;
        result->object_x = best_x;
        result->object_y = best_y;
        result->object_w = best_w;
        result->object_h = best_h;
        result->detection_count = 1;
        result->raw_candidate_count = 1;
        result->detections[0].class_id = (uint32_t)best_class;
        result->detections[0].score = result->object_score;
        result->detections[0].x = best_x;
        result->detections[0].y = best_y;
        result->detections[0].w = best_w;
        result->detections[0].h = best_h;
        strlcpy(result->detections[0].label, result->object, sizeof(result->detections[0].label));
    } else {
        result->raw_candidate_count = best_score > 0.0f ? 1 : 0;
        strlcpy(result->object, CAN_CLASSIFIER_LABELS[CAN_CLASS_UNKNOWN], sizeof(result->object));
    }
    result->inference_ms = (esp_timer_get_time() - inference_start_us) / 1000;
}

/*
 * YOLO 板端入口：
 * - PC 端训练 YOLO26n/YOLO11n 后导出 raw-head ONNX，再用 ESP-PPQ/ESP-DL 量化为 P4 `.espdl`。
 * - 当前客户主路径加载 Fish31/TinyCNN/COCO；历史实验后端只保留为 legacy 代码。
 * - C++ 桥接层负责 JPEG/RGB 解码、letterbox 预处理、ESP-DL 推理、NMS 后处理和坐标映射。
 * - C 主程序只关心统一的 vision_result_t，这样网页、历史记录和 API 不需要知道底层是 MLP、YOLO26 还是 YOLO11。
 */
static bool recognition_method_is_yolo(recognition_method_t method)
{
    return method == RECOGNITION_METHOD_YOLO26 ||
           method == RECOGNITION_METHOD_YOLO11 ||
           method == RECOGNITION_METHOD_COCO;
}

static bool recognition_method_uses_jpeg_inference(recognition_method_t method)
{
    return recognition_method_is_yolo(method) ||
           method == RECOGNITION_METHOD_TINYCLS ||
           method == RECOGNITION_METHOD_FISH31;
}

static void fill_yolo_pending(vision_result_t *result, recognition_method_t method)
{
    const char *model = model_name_for_method(method);
    const char *label = "yolo26-waiting";
    if (method == RECOGNITION_METHOD_YOLO11) {
        label = "yolo11-waiting";
    } else if (method == RECOGNITION_METHOD_COCO) {
        label = "coco-waiting";
    }
    strlcpy(result->model, model, sizeof(result->model));
    strlcpy(result->object, CAN_CLASSIFIER_LABELS[CAN_CLASS_UNKNOWN], sizeof(result->object));
    strlcpy(result->label, label, sizeof(result->label));
    result->unknown_score = 100;
    result->box_min_score = s_box_min_score;
}

static void fill_tinycls_pending(vision_result_t *result)
{
    strlcpy(result->model, TINYCLS_MODEL_NAME, sizeof(result->model));
    strlcpy(result->object, TINY_CLS_LABELS[TINY_CLS_UNKNOWN], sizeof(result->object));
    strlcpy(result->label, "tinycls-waiting", sizeof(result->label));
    result->unknown_score = 100;
    result->box_min_score = s_box_min_score;
}

static void fill_fish31_pending(vision_result_t *result)
{
    strlcpy(result->model, FISH31_MODEL_NAME, sizeof(result->model));
    strlcpy(result->object, "fish31-waiting", sizeof(result->object));
    strlcpy(result->label, "fish31-waiting", sizeof(result->label));
    result->unknown_score = 0;
    result->box_min_score = s_box_min_score;
}

static bool is_completed_yolo_result(const vision_result_t *vision, recognition_method_t method)
{
    if (!vision || !recognition_method_is_yolo(method)) {
        return false;
    }
    const char *model = model_name_for_method(method);
    const char *waiting = "yolo26-waiting";
    if (method == RECOGNITION_METHOD_YOLO11) {
        waiting = "yolo11-waiting";
    } else if (method == RECOGNITION_METHOD_COCO) {
        waiting = "coco-waiting";
    }
    return strcmp(vision->model, model) == 0 &&
           strcmp(vision->label, waiting) != 0 &&
           strstr(vision->label, "error") == NULL &&
           (vision->inference_ms > 0 || vision->object_count > 0 || strcmp(vision->label, "no-object") == 0);
}

static bool is_completed_inference_result(const vision_result_t *vision, recognition_method_t method)
{
    if (recognition_method_is_yolo(method)) {
        return is_completed_yolo_result(vision, method);
    }
    if (!vision || (method != RECOGNITION_METHOD_TINYCLS && method != RECOGNITION_METHOD_FISH31)) {
        return false;
    }
    const char *model = method == RECOGNITION_METHOD_FISH31 ? FISH31_MODEL_NAME : TINYCLS_MODEL_NAME;
    const char *waiting = method == RECOGNITION_METHOD_FISH31 ? "fish31-waiting" : "tinycls-waiting";
    return strcmp(vision->model, model) == 0 &&
           strcmp(vision->label, waiting) != 0 &&
           strstr(vision->label, "error") == NULL &&
           (vision->inference_ms > 0 || vision->top_k_count > 0);
}

static uint32_t detection_right(const vision_detection_t *d)
{
    return d->x + d->w;
}

static uint32_t detection_bottom(const vision_detection_t *d)
{
    return d->y + d->h;
}

static bool detections_should_merge(const vision_detection_t *a, const vision_detection_t *b)
{
    if (!a || !b || a->class_id != b->class_id || a->w == 0 || a->h == 0 || b->w == 0 || b->h == 0) {
        return false;
    }
    bool x_overlap = a->x <= detection_right(b) && b->x <= detection_right(a);
    bool y_overlap = a->y <= detection_bottom(b) && b->y <= detection_bottom(a);
    if (x_overlap && y_overlap) {
        return true;
    }

    int32_t ax = (int32_t)a->x + (int32_t)a->w / 2;
    int32_t ay = (int32_t)a->y + (int32_t)a->h / 2;
    int32_t bx = (int32_t)b->x + (int32_t)b->w / 2;
    int32_t by = (int32_t)b->y + (int32_t)b->h / 2;
    uint32_t max_w = a->w > b->w ? a->w : b->w;
    uint32_t max_h = a->h > b->h ? a->h : b->h;
    return labs(ax - bx) < (long)(max_w + 48U) && labs(ay - by) < (long)(max_h + 48U);
}

static void detection_union_inplace(vision_detection_t *dst, const vision_detection_t *src)
{
    uint32_t x1 = dst->x < src->x ? dst->x : src->x;
    uint32_t y1 = dst->y < src->y ? dst->y : src->y;
    uint32_t x2 = detection_right(dst) > detection_right(src) ? detection_right(dst) : detection_right(src);
    uint32_t y2 = detection_bottom(dst) > detection_bottom(src) ? detection_bottom(dst) : detection_bottom(src);
    dst->x = x1;
    dst->y = y1;
    dst->w = x2 > x1 ? x2 - x1 : 0;
    dst->h = y2 > y1 ? y2 - y1 : 0;
    if (src->score > dst->score) {
        dst->score = src->score;
        strlcpy(dst->label, src->label, sizeof(dst->label));
    }
}

static void expand_detection_box(vision_detection_t *d, uint32_t source_w, uint32_t source_h,
                                 uint32_t x_percent, uint32_t y_percent)
{
    if (!d || d->w == 0 || d->h == 0 || source_w == 0 || source_h == 0) {
        return;
    }
    uint32_t pad_x = (d->w * x_percent) / 100U;
    uint32_t pad_y = (d->h * y_percent) / 100U;
    uint32_t x1 = d->x > pad_x ? d->x - pad_x : 0;
    uint32_t y1 = d->y > pad_y ? d->y - pad_y : 0;
    uint32_t x2 = detection_right(d) + pad_x;
    uint32_t y2 = detection_bottom(d) + pad_y;
    if (x2 > source_w) {
        x2 = source_w;
    }
    if (y2 > source_h) {
        y2 = source_h;
    }
    d->x = x1;
    d->y = y1;
    d->w = x2 > x1 ? x2 - x1 : 0;
    d->h = y2 > y1 ? y2 - y1 : 0;
}

static uint32_t refine_validation_candidates(const vision_detection_t *candidates, uint32_t candidate_count,
                                             vision_detection_t *refined,
                                             uint32_t source_w, uint32_t source_h)
{
    bool used[APP_MAX_DETECTIONS] = {0};
    uint32_t in_count = candidate_count > APP_MAX_DETECTIONS ? APP_MAX_DETECTIONS : candidate_count;
    uint32_t out_count = 0;
    for (uint32_t i = 0; i < in_count && out_count < APP_MAX_DETECTIONS; i++) {
        if (used[i] || candidates[i].w == 0 || candidates[i].h == 0 || candidates[i].score < 10) {
            continue;
        }
        vision_detection_t merged = candidates[i];
        used[i] = true;
        bool changed = true;
        while (changed) {
            changed = false;
            for (uint32_t j = 0; j < in_count; j++) {
                if (used[j] || candidates[j].score < 10 || candidates[j].w == 0 || candidates[j].h == 0) {
                    continue;
                }
                if (detections_should_merge(&merged, &candidates[j])) {
                    detection_union_inplace(&merged, &candidates[j]);
                    used[j] = true;
                    changed = true;
                }
            }
        }
        expand_detection_box(&merged, source_w, source_h, 36, 52);
        refined[out_count++] = merged;
    }

    for (uint32_t i = 0; i + 1 < out_count; i++) {
        for (uint32_t j = i + 1; j < out_count; j++) {
            if (refined[j].score > refined[i].score) {
                vision_detection_t tmp = refined[i];
                refined[i] = refined[j];
                refined[j] = tmp;
            }
        }
    }
    return out_count;
}

static void apply_yolo_detection_common(const vision_detection_t *candidates,
                                        uint32_t candidate_count,
                                        uint32_t raw_candidate_count,
                                        int64_t inference_ms,
                                        const char *model,
                                        uint32_t source_w,
                                        uint32_t source_h,
                                        uint32_t box_min_score,
                                        bool validation_refine,
                                        vision_result_t *result)
{
    result->box_min_score = box_min_score;
    result->candidate_score = 0;
    result->unknown_score = 100;
    result->object_score = 0;
    result->object_count = 0;
    result->detection_count = 0;
    result->raw_candidate_count = raw_candidate_count;
    result->coke_score = 0;
    result->sprite_score = 0;
    result->inference_ms = inference_ms;
    memset(result->detections, 0, sizeof(result->detections));
    strlcpy(result->model, model, sizeof(result->model));

    /*
     * 历史 YOLO26/YOLO11 只作为 legacy 对照路径保留，客户主路径不再启用。
     * 两个桥接层已经输出 ESP-DL 后处理/NMS 后的 Top-N 候选框。这里根据网页可调
     * box_min_score 过滤正式检测框，同时保留最高候选分用于调参观察。
     */
    vision_detection_t refined[APP_MAX_DETECTIONS] = {0};
    uint32_t count = candidate_count;
    if (count > APP_MAX_DETECTIONS) {
        count = APP_MAX_DETECTIONS;
    }
    if (validation_refine && candidates && count > 0) {
        count = refine_validation_candidates(candidates, count, refined, source_w, source_h);
        candidates = refined;
    }

    if (count > 0 && candidates) {
        const vision_detection_t *best = &candidates[0];
        result->candidate_score = best->score;
        result->unknown_score = best->score >= 100 ? 0 : 100 - best->score;
        result->object_x = best->x;
        result->object_y = best->y;
        result->object_w = best->w;
        result->object_h = best->h;
    }

    for (uint32_t i = 0; i < count && candidates; i++) {
        const vision_detection_t *cand = &candidates[i];
        if (cand->score < box_min_score || cand->w == 0 || cand->h == 0) {
            continue;
        }
        vision_detection_t *dst = &result->detections[result->detection_count++];
        *dst = *cand;
        if (strstr(dst->label, "coke") || strstr(dst->label, "cola")) {
            if (dst->score > result->coke_score) {
                result->coke_score = dst->score;
            }
        } else if (strstr(dst->label, "sprite")) {
            if (dst->score > result->sprite_score) {
                result->sprite_score = dst->score;
            }
        }
        if (result->detection_count >= APP_MAX_DETECTIONS) {
            break;
        }
    }

    if (result->detection_count > 0) {
        const vision_detection_t *best_hit = &result->detections[0];
        strlcpy(result->object, best_hit->label, sizeof(result->object));
        strlcpy(result->label, best_hit->label, sizeof(result->label));
        result->object_score = best_hit->score;
        result->object_count = result->detection_count;
        result->object_x = best_hit->x;
        result->object_y = best_hit->y;
        result->object_w = best_hit->w;
        result->object_h = best_hit->h;
    } else {
        strlcpy(result->object, CAN_CLASSIFIER_LABELS[CAN_CLASS_UNKNOWN], sizeof(result->object));
        if (count > 0 && candidates && candidates[0].label[0]) {
            strlcpy(result->label, candidates[0].label, sizeof(result->label));
            strlcat(result->label, "-low", sizeof(result->label));
        } else {
            strlcpy(result->label, "no-object", sizeof(result->label));
        }
    }
}

static void apply_tinycls_result(const tiny_cls_espdl_result_t *cls,
                                 uint32_t min_score,
                                 vision_result_t *result)
{
    if (!cls || !result) {
        return;
    }
    strlcpy(result->model, TINYCLS_MODEL_NAME, sizeof(result->model));
    strlcpy(result->label, cls->label, sizeof(result->label));
    strlcpy(result->object, cls->label, sizeof(result->object));
    result->candidate_score = cls->score;
    result->object_score = cls->score;
    result->box_min_score = min_score;
    result->object_count = (cls->class_id != TINY_CLS_UNKNOWN && cls->score >= min_score) ? 1 : 0;
    result->object_x = 0;
    result->object_y = 0;
    result->object_w = 0;
    result->object_h = 0;
    result->detection_count = 0;
    result->raw_candidate_count = cls->top_k_count;
    result->top_k_count = cls->top_k_count > TINY_CLS_TOP_K ? TINY_CLS_TOP_K : cls->top_k_count;
    result->unknown_score = cls->class_id == TINY_CLS_UNKNOWN ? cls->score : 0;
    memset(result->detections, 0, sizeof(result->detections));
    memset(result->top_k, 0, sizeof(result->top_k));
    for (uint32_t i = 0; i < result->top_k_count; i++) {
        result->top_k[i] = cls->top_k[i];
        if (cls->top_k[i].class_id == TINY_CLS_UNKNOWN) {
            result->unknown_score = cls->top_k[i].score;
        }
    }
    if (cls->class_id == TINY_CLS_UNKNOWN) {
        result->object_count = 0;
        strlcpy(result->object, TINY_CLS_LABELS[TINY_CLS_UNKNOWN], sizeof(result->object));
    }
    result->inference_ms = cls->inference_ms;
}

static void classify_tinycls_frame(const uint8_t *data,
                                   uint32_t data_size,
                                   uint32_t width,
                                   uint32_t height,
                                   uint32_t pixel_format,
                                   vision_result_t *result)
{
    tiny_cls_espdl_result_t cls = {0};
    strlcpy(result->model, TINYCLS_MODEL_NAME, sizeof(result->model));

    esp_err_t err = tiny_cls_espdl_classify_frame(data, data_size, width, height, pixel_format, &cls);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TinyCNN inference failed: %s", esp_err_to_name(err));
        strlcpy(result->label, "tinycls-error", sizeof(result->label));
        strlcpy(result->object, TINY_CLS_LABELS[TINY_CLS_UNKNOWN], sizeof(result->object));
        result->unknown_score = 100;
        result->box_min_score = s_box_min_score;
        return;
    }

    apply_tinycls_result(&cls, s_box_min_score, result);
}

static esp_err_t classify_validation_tinycls_jpeg(const uint8_t *jpeg,
                                                  uint32_t jpeg_size,
                                                  uint32_t box_min_score,
                                                  vision_result_t *result,
                                                  uint32_t *source_w,
                                                  uint32_t *source_h)
{
    if (!jpeg || !jpeg_size || !result || !source_w || !source_h) {
        return ESP_ERR_INVALID_ARG;
    }

    *source_w = 0;
    *source_h = 0;
    tiny_cls_espdl_result_t cls = {0};
    strlcpy(result->model, TINYCLS_MODEL_NAME, sizeof(result->model));
    esp_err_t err = tiny_cls_espdl_classify_validation_jpeg(jpeg, jpeg_size, &cls);
    if (err != ESP_OK) {
        strlcpy(result->label, "tinycls-error", sizeof(result->label));
        strlcpy(result->object, TINY_CLS_LABELS[TINY_CLS_UNKNOWN], sizeof(result->object));
        result->unknown_score = 100;
        result->box_min_score = box_min_score;
        return err;
    }

    apply_tinycls_result(&cls, box_min_score, result);
    *source_w = cls.source_w;
    *source_h = cls.source_h;
    return ESP_OK;
}

static void apply_fish31_result(const fish31_espdl_result_t *cls,
                                uint32_t min_score,
                                vision_result_t *result)
{
    if (!cls || !result) {
        return;
    }
    strlcpy(result->model, FISH31_MODEL_NAME, sizeof(result->model));
    strlcpy(result->label, cls->label, sizeof(result->label));
    strlcpy(result->object, cls->label, sizeof(result->object));
    result->candidate_score = cls->score;
    result->object_score = cls->score;
    result->box_min_score = min_score;
    result->object_count = cls->score >= min_score ? 1 : 0;
    result->object_x = 0;
    result->object_y = 0;
    result->object_w = 0;
    result->object_h = 0;
    result->detection_count = 0;
    result->raw_candidate_count = cls->top_k_count;
    result->top_k_count = cls->top_k_count > FISH31_TOP_K ? FISH31_TOP_K : cls->top_k_count;
    result->unknown_score = 0;
    memset(result->detections, 0, sizeof(result->detections));
    memset(result->top_k, 0, sizeof(result->top_k));
    for (uint32_t i = 0; i < result->top_k_count; i++) {
        result->top_k[i].class_id = cls->top_k[i].class_id;
        result->top_k[i].score = cls->top_k[i].score;
        strlcpy(result->top_k[i].label, cls->top_k[i].label, sizeof(result->top_k[i].label));
    }
    result->inference_ms = cls->inference_ms;
}

static void classify_fish31_frame(const uint8_t *data,
                                  uint32_t data_size,
                                  uint32_t width,
                                  uint32_t height,
                                  uint32_t pixel_format,
                                  vision_result_t *result)
{
    fish31_espdl_result_t cls = {0};
    strlcpy(result->model, FISH31_MODEL_NAME, sizeof(result->model));

    esp_err_t err = fish31_espdl_classify_frame(data, data_size, width, height, pixel_format, &cls);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Fish31 inference failed: %s", esp_err_to_name(err));
        strlcpy(result->label, "fish31-error", sizeof(result->label));
        strlcpy(result->object, "fish31-error", sizeof(result->object));
        result->unknown_score = 0;
        result->box_min_score = s_box_min_score;
        return;
    }

    apply_fish31_result(&cls, s_box_min_score, result);
}

static esp_err_t classify_validation_fish31_jpeg(const uint8_t *jpeg,
                                                 uint32_t jpeg_size,
                                                 uint32_t box_min_score,
                                                 vision_result_t *result,
                                                 uint32_t *source_w,
                                                 uint32_t *source_h)
{
    if (!jpeg || !jpeg_size || !result || !source_w || !source_h) {
        return ESP_ERR_INVALID_ARG;
    }

    *source_w = 0;
    *source_h = 0;
    fish31_espdl_result_t cls = {0};
    strlcpy(result->model, FISH31_MODEL_NAME, sizeof(result->model));
    esp_err_t err = fish31_espdl_classify_validation_jpeg(jpeg, jpeg_size, &cls);
    if (err != ESP_OK) {
        strlcpy(result->label, "fish31-error", sizeof(result->label));
        strlcpy(result->object, "fish31-error", sizeof(result->object));
        result->unknown_score = 0;
        result->box_min_score = box_min_score;
        return err;
    }

    apply_fish31_result(&cls, box_min_score, result);
    *source_w = cls.source_w;
    *source_h = cls.source_h;
    return ESP_OK;
}

static esp_err_t classify_validation_yolo_jpeg(const uint8_t *jpeg, uint32_t jpeg_size,
                                               recognition_method_t method,
                                               uint32_t box_min_score,
                                               vision_result_t *result,
                                               uint32_t *source_w,
                                               uint32_t *source_h)
{
    if (!jpeg || !jpeg_size || !result || !source_w || !source_h || !recognition_method_is_yolo(method)) {
        return ESP_ERR_INVALID_ARG;
    }

    *source_w = 0;
    *source_h = 0;
    fill_yolo_pending(result, method);

    if (method == RECOGNITION_METHOD_COCO) {
        coco_espdl_result_t det = {0};
        vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
        esp_err_t err = coco_espdl_detect_jpeg(jpeg, jpeg_size, &det);
        if (err != ESP_OK) {
            strlcpy(result->label, "coco-error", sizeof(result->label));
            result->unknown_score = 100;
            return err;
        }
        uint32_t count = det.detection_count > APP_MAX_DETECTIONS ? APP_MAX_DETECTIONS : det.detection_count;
        for (uint32_t i = 0; i < count; i++) {
            candidates[i].class_id = det.detections[i].class_id;
            candidates[i].score = det.detections[i].score;
            candidates[i].x = det.detections[i].x > 0 ? (uint32_t)det.detections[i].x : 0;
            candidates[i].y = det.detections[i].y > 0 ? (uint32_t)det.detections[i].y : 0;
            candidates[i].w = det.detections[i].w > 0 ? (uint32_t)det.detections[i].w : 0;
            candidates[i].h = det.detections[i].h > 0 ? (uint32_t)det.detections[i].h : 0;
            strlcpy(candidates[i].label, det.detections[i].label, sizeof(candidates[i].label));
        }
        apply_yolo_detection_common(candidates, count, det.raw_candidate_count,
                                    det.inference_ms, COCO_MODEL_NAME,
                                    det.source_w, det.source_h, box_min_score, false, result);
        *source_w = det.source_w;
        *source_h = det.source_h;
        return ESP_OK;
    }

    if (method == RECOGNITION_METHOD_YOLO11) {
        yolo11_espdl_result_t det = {0};
        vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
        esp_err_t err = yolo11_espdl_detect_jpeg(jpeg, jpeg_size, &det);
        if (err != ESP_OK) {
            strlcpy(result->label, "yolo11-error", sizeof(result->label));
            result->unknown_score = 100;
            return err;
        }
        uint32_t count = det.detection_count > APP_MAX_DETECTIONS ? APP_MAX_DETECTIONS : det.detection_count;
        for (uint32_t i = 0; i < count; i++) {
            candidates[i].class_id = det.detections[i].class_id;
            candidates[i].score = det.detections[i].score;
            candidates[i].x = det.detections[i].x > 0 ? (uint32_t)det.detections[i].x : 0;
            candidates[i].y = det.detections[i].y > 0 ? (uint32_t)det.detections[i].y : 0;
            candidates[i].w = det.detections[i].w > 0 ? (uint32_t)det.detections[i].w : 0;
            candidates[i].h = det.detections[i].h > 0 ? (uint32_t)det.detections[i].h : 0;
            strlcpy(candidates[i].label, det.detections[i].label, sizeof(candidates[i].label));
        }
        apply_yolo_detection_common(candidates, count, det.raw_candidate_count,
                                    det.inference_ms, YOLO11_MODEL_NAME,
                                    det.source_w, det.source_h, box_min_score, true, result);
        *source_w = det.source_w;
        *source_h = det.source_h;
        return ESP_OK;
    }

    yolo26_espdl_result_t det = {0};
    vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
    esp_err_t err = yolo26_espdl_detect_jpeg(jpeg, jpeg_size, &det);
    if (err != ESP_OK) {
        strlcpy(result->label, "yolo26-error", sizeof(result->label));
        result->unknown_score = 100;
        return err;
    }
    uint32_t count = det.detection_count > APP_MAX_DETECTIONS ? APP_MAX_DETECTIONS : det.detection_count;
    for (uint32_t i = 0; i < count; i++) {
        candidates[i].class_id = det.detections[i].class_id;
        candidates[i].score = det.detections[i].score;
        candidates[i].x = det.detections[i].x > 0 ? (uint32_t)det.detections[i].x : 0;
        candidates[i].y = det.detections[i].y > 0 ? (uint32_t)det.detections[i].y : 0;
        candidates[i].w = det.detections[i].w > 0 ? (uint32_t)det.detections[i].w : 0;
        candidates[i].h = det.detections[i].h > 0 ? (uint32_t)det.detections[i].h : 0;
        strlcpy(candidates[i].label, det.detections[i].label, sizeof(candidates[i].label));
    }
    apply_yolo_detection_common(candidates, count, det.raw_candidate_count,
                                det.inference_ms, YOLO26_MODEL_NAME,
                                det.source_w, det.source_h, box_min_score, true, result);
    *source_w = det.source_w;
    *source_h = det.source_h;
    return ESP_OK;
}

static void analyze_frame(const uint8_t *data, uint32_t data_size, uint32_t width, uint32_t height,
                          uint32_t pixel_format, vision_result_t *result)
{
    int64_t start_us = esp_timer_get_time();
    recognition_method_t method = s_vision_enabled ? s_recognition_method : RECOGNITION_METHOD_OFF;
    memset(result, 0, sizeof(*result));
    strlcpy(result->scene, "unknown", sizeof(result->scene));
    strlcpy(result->color, "neutral", sizeof(result->color));
    strlcpy(result->label, "no-model", sizeof(result->label));
    strlcpy(result->object, "unknown", sizeof(result->object));
    strlcpy(result->model, recognition_method_name(method), sizeof(result->model));
    result->box_min_score = s_box_min_score;

    if (method == RECOGNITION_METHOD_OFF) {
        strlcpy(result->scene, "disabled", sizeof(result->scene));
        strlcpy(result->color, "none", sizeof(result->color));
        strlcpy(result->label, "off", sizeof(result->label));
        strlcpy(result->model, "off", sizeof(result->model));
        result->analysis_ms = (esp_timer_get_time() - start_us) / 1000;
        return;
    }

    if (!data || !width || !height) {
        result->analysis_ms = (esp_timer_get_time() - start_us) / 1000;
        return;
    }

    uint32_t luma_sum = 0;
    uint32_t r_sum = 0;
    uint32_t g_sum = 0;
    uint32_t b_sum = 0;
    uint32_t edge_sum = 0;
    uint32_t motion_sum = 0;
    uint32_t samples = 0;
    uint8_t grid[VISION_GRID_N] = {0};

    for (uint32_t gy = 0; gy < VISION_GRID_H; gy++) {
        uint32_t y = (gy * height + height / 2) / VISION_GRID_H;
        if (y >= height) {
            y = height - 1;
        }
        for (uint32_t gx = 0; gx < VISION_GRID_W; gx++) {
            uint32_t x = (gx * width + width / 2) / VISION_GRID_W;
            if (x >= width) {
                x = width - 1;
            }
            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            uint8_t luma = 0;
            if (!sample_pixel(data, data_size, width, height, pixel_format, x, y, &r, &g, &b, &luma)) {
                continue;
            }

            uint32_t idx = gy * VISION_GRID_W + gx;
            grid[idx] = luma;
            luma_sum += luma;
            r_sum += r;
            g_sum += g;
            b_sum += b;
            samples++;

            if (gx > 0) {
                edge_sum += abs((int)luma - (int)grid[idx - 1]);
            }
            if (gy > 0) {
                edge_sum += abs((int)luma - (int)grid[idx - VISION_GRID_W]);
            }
            if (s_prev_luma_valid) {
                motion_sum += abs((int)luma - (int)s_prev_luma_grid[idx]);
            }
        }
    }

    if (!samples) {
        result->analysis_ms = (esp_timer_get_time() - start_us) / 1000;
        return;
    }

    memcpy(s_prev_luma_grid, grid, sizeof(s_prev_luma_grid));
    s_prev_luma_valid = true;

    result->avg_luma = luma_sum / samples;
    result->avg_r = r_sum / samples;
    result->avg_g = g_sum / samples;
    result->avg_b = b_sum / samples;
    result->motion_score = motion_sum / samples;
    result->edge_score = edge_sum / (samples > 1 ? samples - 1 : samples);
    result->motion = result->motion_score >= 12;

    if (result->avg_luma < 45) {
        strlcpy(result->scene, "dark", sizeof(result->scene));
    } else if (result->avg_luma > 190) {
        strlcpy(result->scene, "bright", sizeof(result->scene));
    } else {
        strlcpy(result->scene, "normal", sizeof(result->scene));
    }

    if (result->avg_r > result->avg_b + 24 && result->avg_r > result->avg_g + 10) {
        strlcpy(result->color, "warm", sizeof(result->color));
    } else if (result->avg_b > result->avg_r + 24 && result->avg_b > result->avg_g + 10) {
        strlcpy(result->color, "cool", sizeof(result->color));
    } else if (result->avg_g > result->avg_r + 18 && result->avg_g > result->avg_b + 18) {
        strlcpy(result->color, "green", sizeof(result->color));
    } else {
        strlcpy(result->color, "neutral", sizeof(result->color));
    }

    if (recognition_method_is_yolo(method)) {
        fill_yolo_pending(result, method);
    } else if (method == RECOGNITION_METHOD_TINYCLS) {
        classify_tinycls_frame(data, data_size, width, height, pixel_format, result);
    } else if (method == RECOGNITION_METHOD_FISH31) {
        classify_fish31_frame(data, data_size, width, height, pixel_format, result);
    } else {
        strlcpy(result->model, CAN_CLASSIFIER_MODEL_NAME, sizeof(result->model));
        classify_can_candidate(data, data_size, width, height, pixel_format, result);
    }

    if (result->object_count > 0) {
        strlcpy(result->label, result->object, sizeof(result->label));
    } else if (recognition_method_is_yolo(method)) {
        strlcpy(result->label,
                method == RECOGNITION_METHOD_YOLO11 ? "yolo11-waiting" :
                (method == RECOGNITION_METHOD_COCO ? "coco-waiting" : "yolo26-waiting"),
                sizeof(result->label));
    } else if (method == RECOGNITION_METHOD_TINYCLS || method == RECOGNITION_METHOD_FISH31) {
        if (result->label[0] == '\0') {
            strlcpy(result->label, result->object, sizeof(result->label));
        }
    } else if (strcmp(result->scene, "dark") == 0) {
        strlcpy(result->label, "low-light", sizeof(result->label));
    } else if (result->motion) {
        strlcpy(result->label, "motion", sizeof(result->label));
    } else if (result->edge_score > 35) {
        strlcpy(result->label, "textured", sizeof(result->label));
    } else {
        strlcpy(result->label, "static", sizeof(result->label));
    }

    result->analysis_ms = (esp_timer_get_time() - start_us) / 1000;
}

static void update_capture_fps(int64_t now_ms)
{
    if (s_capture_fps_window_start_ms == 0) {
        s_capture_fps_window_start_ms = now_ms;
    }

    s_capture_fps_window_count++;
    int64_t span = now_ms - s_capture_fps_window_start_ms;
    if (span >= 1000) {
        s_capture_fps_x100 = (uint32_t)((s_capture_fps_window_count * 100000LL) / span);
        s_capture_fps_window_count = 0;
        s_capture_fps_window_start_ms = now_ms;
    }
}

static void stream_stats_client_begin(void)
{
    taskENTER_CRITICAL(&s_stream_stats_mux);
    s_stream_clients++;
    taskEXIT_CRITICAL(&s_stream_stats_mux);
}

static void stream_stats_client_end(void)
{
    bool underflow = false;
    taskENTER_CRITICAL(&s_stream_stats_mux);
    if (s_stream_clients > 0) {
        s_stream_clients--;
    } else {
        underflow = true;
    }
    taskEXIT_CRITICAL(&s_stream_stats_mux);
    if (underflow) {
        ESP_LOGE(TAG, "stream client counter underflow prevented");
    }
}

static uint32_t stream_stats_client_count(void)
{
    taskENTER_CRITICAL(&s_stream_stats_mux);
    uint32_t clients = s_stream_clients;
    taskEXIT_CRITICAL(&s_stream_stats_mux);
    return clients;
}

static void stream_stats_record_error(void)
{
    taskENTER_CRITICAL(&s_stream_stats_mux);
    s_stream_errors++;
    taskEXIT_CRITICAL(&s_stream_stats_mux);
}

static void stream_stats_record_frame(uint32_t bytes, int64_t now_ms)
{
    taskENTER_CRITICAL(&s_stream_stats_mux);
    s_stream_frames_total++;
    s_stream_bytes_total += bytes;
    if (s_stream_fps_window_start_ms == 0) {
        s_stream_fps_window_start_ms = now_ms;
    }
    s_stream_fps_window_count++;
    int64_t span = now_ms - s_stream_fps_window_start_ms;
    if (span >= 1000) {
        s_stream_fps_x100 = (uint32_t)((s_stream_fps_window_count * 100000LL) / span);
        s_stream_fps_window_count = 0;
        s_stream_fps_window_start_ms = now_ms;
    }
    taskEXIT_CRITICAL(&s_stream_stats_mux);
}

static void stream_stats_get_snapshot(stream_stats_snapshot_t *snapshot)
{
    if (!snapshot) {
        return;
    }
    taskENTER_CRITICAL(&s_stream_stats_mux);
    snapshot->clients = s_stream_clients;
    snapshot->errors = s_stream_errors;
    snapshot->frames_total = s_stream_frames_total;
    snapshot->bytes_total = s_stream_bytes_total;
    snapshot->fps_x100 = s_stream_fps_x100;
    taskEXIT_CRITICAL(&s_stream_stats_mux);
}

static bool http_server_is_stopping(void)
{
    taskENTER_CRITICAL(&s_http_activity_mux);
    bool stopping = s_http_stopping;
    taskEXIT_CRITICAL(&s_http_activity_mux);
    return stopping;
}

static void http_server_set_stopping(bool stopping)
{
    taskENTER_CRITICAL(&s_http_activity_mux);
    s_http_stopping = stopping;
    taskEXIT_CRITICAL(&s_http_activity_mux);
}

static bool http_async_activity_try_begin(void)
{
    bool accepted = false;
    taskENTER_CRITICAL(&s_http_activity_mux);
    if (!s_http_stopping) {
        s_async_active_requests++;
        accepted = true;
    }
    taskEXIT_CRITICAL(&s_http_activity_mux);
    return accepted;
}

static void http_async_activity_end(void)
{
    bool underflow = false;
    taskENTER_CRITICAL(&s_http_activity_mux);
    if (s_async_active_requests > 0) {
        s_async_active_requests--;
    } else {
        underflow = true;
    }
    taskEXIT_CRITICAL(&s_http_activity_mux);
    if (underflow) {
        ESP_LOGE(TAG, "HTTP async activity counter underflow prevented");
    }
}

static uint32_t http_async_activity_count(void)
{
    taskENTER_CRITICAL(&s_http_activity_mux);
    uint32_t active = s_async_active_requests;
    taskEXIT_CRITICAL(&s_http_activity_mux);
    return active;
}

static void update_inference_fps(int64_t now_ms)
{
    if (s_inference_fps_window_start_ms == 0) {
        s_inference_fps_window_start_ms = now_ms;
    }

    s_inference_frames_total++;
    s_inference_fps_window_count++;
    int64_t span = now_ms - s_inference_fps_window_start_ms;
    if (span >= 1000) {
        s_inference_fps_x100 = (uint32_t)((s_inference_fps_window_count * 100000LL) / span);
        s_inference_fps_window_count = 0;
        s_inference_fps_window_start_ms = now_ms;
    }
}

static bool recognition_due(int64_t now_ms)
{
    /*
     * 推理间隔是“抽帧识别”的核心开关：视频可以持续推流，但识别任务按用户设置的周期运行。
     * 这能让 YOLO 在算力吃紧时保持 Web 响应和多客户端推流稳定；跳过的帧计入 dropped_inference_frames。
     */
    uint32_t interval_ms = s_inference_interval_ms;
    if (interval_ms == 0 || s_last_inference_ms == 0 ||
        now_ms - s_last_inference_ms >= (int64_t)interval_ms) {
        s_last_inference_ms = now_ms;
        return true;
    }

    s_inference_dropped_frames++;
    return false;
}

static bool recognition_interval_elapsed(int64_t now_ms)
{
    uint32_t interval_ms = s_inference_interval_ms;
    return interval_ms == 0 || s_last_inference_ms == 0 ||
           now_ms - s_last_inference_ms >= (int64_t)interval_ms;
}

static esp_err_t set_camera_jpeg_quality(camera_t *cam, int quality)
{
    if (cam->pixel_format != V4L2_PIX_FMT_JPEG) {
        return example_encoder_set_jpeg_quality(cam->encoder, quality);
    }

    struct v4l2_ext_controls controls = {0};
    struct v4l2_ext_control control[1] = {0};
    controls.ctrl_class = V4L2_CID_JPEG_CLASS;
    controls.count = 1;
    controls.controls = control;
    control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    control[0].value = quality;

    if (ioctl(cam->fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "sensor JPEG quality control is not supported");
    }
    return ESP_OK;
}

static void clear_latest_frame(void)
{
    if (!s_frame_lock) {
        return;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    s_have_frame = false;
    memset(&s_latest_meta, 0, sizeof(s_latest_meta));
    xSemaphoreGive(s_frame_lock);
}

static esp_err_t publish_frame(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta)
{
    if (!jpeg || !jpeg_size || !meta || !s_latest_jpeg) {
        return ESP_ERR_INVALID_ARG;
    }

    if (jpeg_size > s_frame_capacity) {
        s_frame_drops++;
        ESP_LOGW(TAG, "JPEG frame too large: %" PRIu32 " > %" PRIu32, jpeg_size, s_frame_capacity);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    memcpy(s_latest_jpeg, jpeg, jpeg_size);
    s_latest_meta = *meta;
    s_latest_meta.size = jpeg_size;
    s_have_frame = true;
    xSemaphoreGive(s_frame_lock);
    return ESP_OK;
}

static bool copy_latest_frame(uint8_t *dst, uint32_t dst_cap, frame_meta_t *meta)
{
    bool ok = false;

    if (!dst || !meta || !s_frame_lock) {
        return false;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    if (s_have_frame && s_latest_meta.size > 0 && s_latest_meta.size <= dst_cap) {
        memcpy(dst, s_latest_jpeg, s_latest_meta.size);
        *meta = s_latest_meta;
        ok = true;
    }
    xSemaphoreGive(s_frame_lock);
    return ok;
}

static bool copy_latest_meta(frame_meta_t *meta)
{
    bool ok = false;

    if (!meta || !s_frame_lock) {
        return false;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    if (s_have_frame) {
        *meta = s_latest_meta;
        ok = true;
    } else {
        memset(meta, 0, sizeof(*meta));
    }
    xSemaphoreGive(s_frame_lock);
    return ok;
}

static bool copy_completed_yolo_vision(vision_result_t *vision, recognition_method_t method)
{
    bool ok = false;

    if (!vision || !s_frame_lock || !recognition_method_uses_jpeg_inference(method)) {
        return false;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    if (s_have_completed_yolo_vision &&
        s_last_completed_yolo_method == method &&
        is_completed_inference_result(&s_last_completed_yolo_vision, method)) {
        *vision = s_last_completed_yolo_vision;
        ok = true;
    }
    xSemaphoreGive(s_frame_lock);
    return ok;
}

static void update_latest_vision_from_inference(const vision_result_t *vision,
                                                recognition_method_t method)
{
    if (!vision || !s_frame_lock || !recognition_method_uses_jpeg_inference(method)) {
        return;
    }

    /*
     * YOLO 推理结果通常比视频帧晚 1~20 秒返回。这里不要求 seq 仍然是当初那一帧：
     * 对于固定摄像头视角，最近一次检测结果继续叠加在当前画面上比“永远显示 waiting”更有用。
     * 后续如果做高速运动目标，可以在这里增加 seq/时间戳过期判断。
     */
    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    if (s_have_frame) {
        s_latest_meta.vision = *vision;
    }
    if (is_completed_inference_result(vision, method)) {
        s_last_completed_yolo_vision = *vision;
        s_last_completed_yolo_method = method;
        s_have_completed_yolo_vision = true;
    }
    xSemaphoreGive(s_frame_lock);
}

static esp_err_t update_sd_info(void)
{
    if (!s_sd_mounted) {
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
        return ESP_OK;
    }

    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;
    errno = 0;
    esp_err_t ret = esp_vfs_fat_info(CONFIG_APP_SD_MOUNT_POINT,
                                     &total_bytes, &free_bytes);
    if (ret != ESP_OK) {
        int info_errno = errno ? errno :
                         (ret == ESP_ERR_INVALID_STATE ? EINVAL : EIO);
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
        if (storage_backend_is_tf() &&
            storage_errno_is_media_failure(info_errno)) {
            s_recording_sd_errors++;
            storage_latch_io_error("TF capacity refresh", info_errno);
        }
        ESP_LOGE(TAG, "TF capacity refresh failed: %s errno=%d",
                 esp_err_to_name(ret), info_errno);
        errno = info_errno;
        return ret;
    }
    s_sd_total_bytes = total_bytes;
    s_sd_free_bytes = free_bytes;
    errno = 0;
    return ESP_OK;
}

static bool storage_backend_is_tf(void)
{
    return strcmp(s_storage_backend, "tf_sdmmc") == 0 ||
           strcmp(s_storage_backend, "tf_sdspi") == 0;
}

static bool storage_tf_ready(void)
{
    return s_sd_mounted && s_sd_card && storage_backend_is_tf();
}

static bool storage_acceptance_ok(void)
{
    return storage_tf_ready() && s_storage_write_verified && !s_storage_io_latched;
}

static void storage_clear_write_health(void)
{
    s_storage_write_verified = false;
    s_storage_write_verified_ms = 0;
}

static void storage_latch_io_error(const char *operation, int error_number)
{
    if (error_number == ENOSPC) {
        storage_clear_write_health();
        s_storage_last_errno = error_number;
        set_storage_status("TF card is full while %s", operation ? operation : "writing");
        return;
    }
    if (s_storage_io_latched) {
        return;
    }
    s_storage_io_latched = true;
    storage_clear_write_health();
    s_storage_last_errno = error_number ? error_number : EIO;
    set_storage_status("TF write disabled after %s failed: errno=%d",
                       operation ? operation : "I/O", s_storage_last_errno);
    ESP_LOGE(TAG,
             "TF runtime I/O failure latched; new recording writes are stopped: operation=%s errno=%d",
             operation ? operation : "unknown", s_storage_last_errno);
}

static bool storage_errno_is_media_failure(int error_number)
{
    switch (error_number) {
    case EIO:
    case ENOSPC:
    case ENODEV:
    case ENXIO:
    case EROFS:
    case ETIMEDOUT:
        return true;
    default:
        return false;
    }
}

typedef struct {
    esp_err_t result;
    int error_number;
    bool media_failure;
} storage_maintenance_result_t;

static void storage_maintenance_record_failure(
    storage_maintenance_result_t *result, const char *operation,
    int error_number, volatile uint32_t *error_counter)
{
    int saved_errno = error_number ? error_number : EIO;
    bool media_failure = storage_errno_is_media_failure(saved_errno);
    if (error_counter) {
        (*error_counter)++;
    }
    if (media_failure) {
        storage_latch_io_error(operation, saved_errno);
        ESP_LOGE(TAG, "%s failed: errno=%d",
                 operation ? operation : "storage maintenance", saved_errno);
    } else {
        ESP_LOGW(TAG, "%s degraded: errno=%d",
                 operation ? operation : "storage maintenance", saved_errno);
    }
    if (result && (result->result == ESP_OK ||
                   (media_failure && !result->media_failure))) {
        result->result = ESP_FAIL;
        result->error_number = saved_errno;
        result->media_failure = media_failure;
    }
}

static esp_err_t storage_maintenance_finish(
    const storage_maintenance_result_t *result)
{
    if (!result || result->result == ESP_OK) {
        errno = 0;
        return ESP_OK;
    }
    errno = result->error_number ? result->error_number : EIO;
    return result->result;
}

/*
 * AVI helpers also report allocation, argument, and size-limit failures.  Only
 * failures attributable to the file/card path may invalidate TF write health;
 * otherwise a transient heap or programming error would unnecessarily disable
 * all later recording writes until a storage retry.
 */
static bool recording_failure_is_storage_io(esp_err_t ret, int error_number)
{
    if (ret == ESP_OK || ret == ESP_ERR_NO_MEM || ret == ESP_ERR_INVALID_ARG ||
        ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_NOT_SUPPORTED) {
        return false;
    }
    switch (error_number) {
    case ENOMEM:
    case EMFILE:
    case ENFILE:
    case EINVAL:
    case EBADF:
        return false;
    default:
        break;
    }
    if (ret == ESP_ERR_INVALID_SIZE && error_number == 0) {
        return false;
    }
    /* FatFS/newlib can return ESP_FAIL with errno left at zero on a short I/O. */
    return ret == ESP_FAIL || error_number != 0;
}

static bool recording_latch_storage_failure(const char *operation,
                                             esp_err_t ret, int error_number)
{
    if (!recording_failure_is_storage_io(ret, error_number)) {
        return false;
    }
    s_recording_sd_errors++;
    storage_latch_io_error(operation, error_number ? error_number : EIO);
    return true;
}

static uint64_t recording_min_free_bytes(void)
{
    if (!storage_backend_is_tf() || s_sd_total_bytes == 0) {
        return 0;
    }
    uint64_t percent_bytes = s_sd_total_bytes * APP_RECORDING_MIN_FREE_PERCENT / 100U;
    return percent_bytes > APP_RECORDING_MIN_FREE_BYTES ? percent_bytes : APP_RECORDING_MIN_FREE_BYTES;
}

static void history_push_record(const history_record_t *record)
{
    if (!record || !s_history_lock || !s_history_records) {
        return;
    }

    xSemaphoreTake(s_history_lock, portMAX_DELAY);
    s_history_records[s_history_head] = *record;
    s_history_head = (s_history_head + 1) % CONFIG_APP_HISTORY_MAX_RECORDS;
    if (s_history_count < CONFIG_APP_HISTORY_MAX_RECORDS) {
        s_history_count++;
    }
    xSemaphoreGive(s_history_lock);
}

static esp_err_t ensure_dir(const char *path)
{
    if (!path || !path[0]) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    errno = 0;
    if (stat(path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return ESP_OK;
        }
        errno = ENOTDIR;
        return ESP_ERR_INVALID_STATE;
    }
    int stat_errno = errno ? errno : EIO;
    if (stat_errno != ENOENT) {
        errno = stat_errno;
        return ESP_FAIL;
    }

    errno = 0;
    if (mkdir(path, 0775) != 0 && errno != EEXIST) {
        int mkdir_errno = errno ? errno : EIO;
        errno = mkdir_errno;
        return ESP_FAIL;
    }

    /* EEXIST can be a concurrent creator or a regular-file conflict. Verify
     * the final object instead of silently accepting an unusable path. */
    memset(&st, 0, sizeof(st));
    errno = 0;
    if (stat(path, &st) != 0) {
        int verify_errno = errno ? errno : EIO;
        errno = verify_errno;
        return ESP_FAIL;
    }
    if (!S_ISDIR(st.st_mode)) {
        errno = ENOTDIR;
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static bool is_safe_snapshot_name(const char *name)
{
    if (!name || !name[0]) {
        return false;
    }

    for (const char *p = name; *p; p++) {
        bool ok = (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z') ||
                  (*p >= '0' && *p <= '9') || *p == '_' || *p == '-' || *p == '.';
        if (!ok) {
            return false;
        }
    }
    return strstr(name, "..") == NULL;
}

static esp_err_t report_index_rotation_failure(const char *index_name,
                                               const char *step,
                                               int error_number,
                                               volatile uint32_t *error_counter)
{
    int saved_errno = error_number ? error_number : EIO;
    if (error_counter) {
        (*error_counter)++;
    }

    char operation[80];
    snprintf(operation, sizeof(operation), "%s %s",
             index_name ? index_name : "JSONL index",
             step ? step : "rotation");
    if (storage_errno_is_media_failure(saved_errno)) {
        storage_latch_io_error(operation, saved_errno);
    } else {
        set_storage_status("%s failed: errno=%d; index write paused",
                           operation, saved_errno);
    }
    ESP_LOGE(TAG, "%s failed: errno=%d", operation, saved_errno);
    errno = saved_errno;
    return ESP_FAIL;
}

static esp_err_t rotate_index_if_needed(const char *path, const char *old_path,
                                        const char *index_name,
                                        volatile uint32_t *error_counter)
{
    if (!path || !old_path || !index_name) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    errno = 0;
    if (stat(path, &st) != 0) {
        int stat_errno = errno ? errno : EIO;
        if (stat_errno == ENOENT) {
            errno = 0;
            return ESP_OK;
        }
        return report_index_rotation_failure(index_name, "stat",
                                             stat_errno, error_counter);
    }

    const off_t max_bytes = (off_t)CONFIG_APP_HISTORY_INDEX_MAX_KB * 1024;
    if (st.st_size <= max_bytes) {
        return ESP_OK;
    }

    errno = 0;
    if (unlink(old_path) != 0) {
        int unlink_errno = errno ? errno : EIO;
        if (unlink_errno != ENOENT) {
            return report_index_rotation_failure(index_name, "old-index cleanup",
                                                 unlink_errno, error_counter);
        }
    }
    errno = 0;
    if (rename(path, old_path) != 0) {
        int rename_errno = errno ? errno : EIO;
        return report_index_rotation_failure(index_name, "rename",
                                             rename_errno, error_counter);
    }
    return ESP_OK;
}

static esp_err_t rotate_history_index_if_needed(void)
{
    return rotate_index_if_needed(HISTORY_JSONL_PATH, HISTORY_JSONL_OLD_PATH,
                                  "history index", &s_history_sd_errors);
}

static esp_err_t count_snapshot_files(uint32_t *out_count)
{
    if (!out_count) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    *out_count = 0;
    storage_maintenance_result_t result = {0};
    uint32_t count = 0;
    errno = 0;
    DIR *dir = opendir(HISTORY_SNAPSHOT_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "snapshot directory open",
                                           errno ? errno : EIO,
                                           &s_history_sd_errors);
        return storage_maintenance_finish(&result);
    }

    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "snapshot directory read", errno,
                    &s_history_sd_errors);
            }
            break;
        }
        if (!is_safe_snapshot_name(entry->d_name)) {
            continue;
        }
        char path[384];
        snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR,
                 entry->d_name);
        struct stat st = {0};
        errno = 0;
        if (stat(path, &st) != 0) {
            storage_maintenance_record_failure(
                &result, "snapshot count stat", errno ? errno : EIO,
                &s_history_sd_errors);
            continue;
        }
        if (S_ISREG(st.st_mode)) {
            count++;
        }
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "snapshot directory close",
                                           errno ? errno : EIO,
                                           &s_history_sd_errors);
    }
    *out_count = count;
    return storage_maintenance_finish(&result);
}

static esp_err_t find_oldest_snapshot(char *name, size_t name_size,
                                      bool *out_found)
{
    if (!name || name_size == 0 || !out_found) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    name[0] = '\0';
    *out_found = false;
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(HISTORY_SNAPSHOT_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "snapshot oldest directory open",
                                           errno ? errno : EIO,
                                           &s_history_sd_errors);
        return storage_maintenance_finish(&result);
    }

    bool found = false;
    time_t oldest_time = 0;
    char oldest_name[96] = {0};

    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "snapshot oldest directory read", errno,
                    &s_history_sd_errors);
            }
            break;
        }
        if (!is_safe_snapshot_name(entry->d_name)) {
            continue;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, entry->d_name);
        struct stat st = {0};
        errno = 0;
        if (stat(path, &st) != 0) {
            storage_maintenance_record_failure(
                &result, "snapshot oldest stat", errno ? errno : EIO,
                &s_history_sd_errors);
            continue;
        }
        if (!S_ISREG(st.st_mode)) {
            continue;
        }

        if (!found || st.st_mtime < oldest_time ||
            (st.st_mtime == oldest_time && strcmp(entry->d_name, oldest_name) < 0)) {
            found = true;
            oldest_time = st.st_mtime;
            strlcpy(oldest_name, entry->d_name, sizeof(oldest_name));
        }
    }

    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "snapshot oldest directory close",
                                           errno ? errno : EIO,
                                           &s_history_sd_errors);
    }
    if (found) {
        strlcpy(name, oldest_name, name_size);
    }
    *out_found = found;
    return storage_maintenance_finish(&result);
}

static esp_err_t cleanup_old_snapshots(void)
{
    if (!s_sd_mounted || CONFIG_APP_HISTORY_MAX_SNAPSHOTS <= 0) {
        return ESP_OK;
    }

    uint32_t count = 0;
    esp_err_t ret = count_snapshot_files(&count);
    if (ret != ESP_OK) {
        return ret;
    }
    while (count > CONFIG_APP_HISTORY_MAX_SNAPSHOTS) {
        char oldest[96] = {0};
        bool found = false;
        ret = find_oldest_snapshot(oldest, sizeof(oldest), &found);
        if (ret != ESP_OK) {
            return ret;
        }
        if (!found) {
            storage_maintenance_result_t result = {0};
            storage_maintenance_record_failure(
                &result, "snapshot retention layout", EINVAL,
                &s_history_sd_errors);
            return storage_maintenance_finish(&result);
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, oldest);
        errno = 0;
        if (unlink(path) == 0) {
            s_history_files_deleted++;
            count--;
        } else {
            int unlink_errno = errno ? errno : EIO;
            if (unlink_errno == ENOENT) {
                count--;
                continue;
            }
            storage_maintenance_result_t result = {0};
            storage_maintenance_record_failure(
                &result, "snapshot retention unlink", unlink_errno,
                &s_history_sd_errors);
            return storage_maintenance_finish(&result);
        }
    }
    return ESP_OK;
}

static bool has_suffix(const char *text, const char *suffix)
{
    if (!text || !suffix) {
        return false;
    }
    size_t text_len = strlen(text);
    size_t suffix_len = strlen(suffix);
    return text_len >= suffix_len && strcmp(text + text_len - suffix_len, suffix) == 0;
}

static bool is_safe_recording_name(const char *name)
{
    return is_safe_snapshot_name(name) &&
           (has_suffix(name, ".mjpg") || has_suffix(name, ".avi"));
}

static bool is_annotated_recording_name(const char *name)
{
    return name && (strncmp(name, "annotated_", 10) == 0 ||
                    strncmp(name, "ann_", 4) == 0);
}

static bool annotated_name_for_raw(const char *raw_name, char *out, size_t out_size)
{
    if (!raw_name || !out || out_size == 0 || strncmp(raw_name, "raw_", 4) != 0) {
        return false;
    }
    const char prefix[] = "annotated_";
    const char *suffix = raw_name + 4;
    size_t prefix_len = strlen(prefix);
    size_t suffix_len = strlen(suffix);
    if (prefix_len + suffix_len + 1 > out_size) {
        out[0] = '\0';
        return false;
    }
    memcpy(out, prefix, prefix_len);
    memcpy(out + prefix_len, suffix, suffix_len + 1);
    return true;
}

static bool legacy_annotated_name_for_raw(const char *raw_name, char *out, size_t out_size)
{
    if (!raw_name || !out || out_size == 0 || strncmp(raw_name, "raw_", 4) != 0) {
        return false;
    }
    const char prefix[] = "ann_";
    const char *suffix = raw_name + 4;
    size_t prefix_len = strlen(prefix);
    size_t suffix_len = strlen(suffix);
    if (prefix_len + suffix_len + 1 > out_size) {
        out[0] = '\0';
        return false;
    }
    memcpy(out, prefix, prefix_len);
    memcpy(out + prefix_len, suffix, suffix_len + 1);
    return true;
}

static bool raw_name_for_annotated(const char *annotated_name, char *out, size_t out_size)
{
    if (!annotated_name || !out || out_size == 0) {
        return false;
    }
    if (strncmp(annotated_name, "annotated_", 10) == 0) {
        const char prefix[] = "raw_";
        const char *suffix = annotated_name + 10;
        size_t prefix_len = strlen(prefix);
        size_t suffix_len = strlen(suffix);
        if (prefix_len + suffix_len + 1 > out_size) {
            out[0] = '\0';
            return false;
        }
        memcpy(out, prefix, prefix_len);
        memcpy(out + prefix_len, suffix, suffix_len + 1);
        return true;
    }
    if (strncmp(annotated_name, "ann_", 4) == 0) {
        const char prefix[] = "raw_";
        const char *suffix = annotated_name + 4;
        size_t prefix_len = strlen(prefix);
        size_t suffix_len = strlen(suffix);
        if (prefix_len + suffix_len + 1 > out_size) {
            out[0] = '\0';
            return false;
        }
        memcpy(out, prefix, prefix_len);
        memcpy(out + prefix_len, suffix, suffix_len + 1);
        return true;
    }
    return false;
}

static bool is_safe_recording_meta_name(const char *name)
{
    return is_safe_snapshot_name(name) && has_suffix(name, ".jsonl");
}

static bool is_safe_dataset_name(const char *name)
{
    if (!name || !name[0] || strlen(name) >= DATASET_NAME_MAX) {
        return false;
    }
    return is_safe_snapshot_name(name);
}

static bool dataset_relpath_depth_supported(const char *path)
{
    if (!path || !path[0]) {
        return false;
    }
    uint32_t directory_depth = 0;
    bool component_has_character = false;
    for (const char *p = path; *p; p++) {
        if (*p == '/' || *p == '\\') {
            if (!component_has_character ||
                directory_depth >= DATASET_RECOVERY_MAX_DIR_DEPTH) {
                return false;
            }
            directory_depth++;
            component_has_character = false;
        } else {
            component_has_character = true;
        }
    }
    return component_has_character;
}

static bool is_safe_dataset_relpath(const char *path)
{
    if (!path || !path[0] || strlen(path) >= DATASET_PATH_MAX || strstr(path, "..") ||
        !dataset_relpath_depth_supported(path)) {
        return false;
    }
    if (path[0] == '/' || path[0] == '\\') {
        return false;
    }
    for (const char *p = path; *p; p++) {
        bool ok = (*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z') ||
                  (*p >= '0' && *p <= '9') || *p == '_' || *p == '-' ||
                  *p == '.' || *p == '/' || *p == '\\';
        if (!ok) {
            return false;
        }
    }
    return has_suffix(path, ".jpg") || has_suffix(path, ".jpeg") ||
           has_suffix(path, ".json") || has_suffix(path, ".jsonl") ||
           has_suffix(path, ".txt");
}

static void meta_name_for_recording(const char *recording_name, char *meta_name, size_t meta_name_size)
{
    if (!recording_name || !meta_name || meta_name_size == 0) {
        return;
    }
    strlcpy(meta_name, recording_name, meta_name_size);
    char *dot = strrchr(meta_name, '.');
    if (dot) {
        *dot = '\0';
    }
    strlcat(meta_name, ".jsonl", meta_name_size);
}

static esp_err_t rotate_jsonl_if_needed(const char *path, const char *old_path,
                                        const char *index_name)
{
    return rotate_index_if_needed(path, old_path, index_name,
                                  &s_recording_sd_errors);
}

static esp_err_t count_recording_files(uint32_t *out_count)
{
    if (!out_count) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    *out_count = 0;
    storage_maintenance_result_t result = {0};
    uint32_t count = 0;
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "recording directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }

    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "recording directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_recording_name(entry->d_name)) {
            continue;
        }
        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        struct stat st = {0};
        errno = 0;
        if (stat(path, &st) != 0) {
            storage_maintenance_record_failure(
                &result, "recording count stat", errno ? errno : EIO,
                &s_recording_sd_errors);
            continue;
        }
        if (S_ISREG(st.st_mode)) {
            count++;
        }
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "recording directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    *out_count = count;
    return storage_maintenance_finish(&result);
}

static esp_err_t find_oldest_recording(char *name, size_t name_size,
                                       bool *out_found)
{
    if (!name || name_size == 0 || !out_found) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    name[0] = '\0';
    *out_found = false;
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "recording oldest directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }

    bool found = false;
    time_t oldest_time = 0;
    char oldest_name[96] = {0};
    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "recording oldest directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_recording_name(entry->d_name)) {
            continue;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        struct stat st = {0};
        errno = 0;
        if (stat(path, &st) != 0) {
            storage_maintenance_record_failure(
                &result, "recording oldest stat", errno ? errno : EIO,
                &s_recording_sd_errors);
            continue;
        }
        if (!S_ISREG(st.st_mode)) {
            continue;
        }

        if (!found || st.st_mtime < oldest_time ||
            (st.st_mtime == oldest_time && strcmp(entry->d_name, oldest_name) < 0)) {
            found = true;
            oldest_time = st.st_mtime;
            strlcpy(oldest_name, entry->d_name, sizeof(oldest_name));
        }
    }

    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "recording oldest directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    if (found) {
        strlcpy(name, oldest_name, name_size);
    }
    *out_found = found;
    return storage_maintenance_finish(&result);
}

static esp_err_t cleanup_old_recordings(void)
{
    if (!s_sd_mounted || CONFIG_APP_RECORDING_MAX_SEGMENTS <= 0) {
        return ESP_OK;
    }

    storage_maintenance_result_t result = {0};
    errno = 0;
    if (update_sd_info() != ESP_OK) {
        storage_maintenance_record_failure(
            &result, "recording retention capacity refresh",
            errno ? errno : EIO, NULL);
        if (result.media_failure) {
            return storage_maintenance_finish(&result);
        }
    }
    uint32_t count = 0;
    if (count_recording_files(&count) != ESP_OK) {
        return ESP_FAIL;
    }
    uint64_t min_free = recording_min_free_bytes();
    while (count > CONFIG_APP_RECORDING_MAX_SEGMENTS ||
           (min_free > 0 && s_sd_free_bytes > 0 && s_sd_free_bytes < min_free)) {
        char oldest[96] = {0};
        bool found = false;
        esp_err_t oldest_ret =
            find_oldest_recording(oldest, sizeof(oldest), &found);
        if (oldest_ret != ESP_OK) {
            return oldest_ret;
        }
        if (!found) {
            storage_maintenance_record_failure(
                &result, "recording retention layout", EINVAL,
                &s_recording_sd_errors);
            return storage_maintenance_finish(&result);
        }

        char path[384];
        struct stat st = {0};
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, oldest);
        errno = 0;
        if (stat(path, &st) != 0) {
            int stat_errno = errno ? errno : EIO;
            if (stat_errno == ENOENT) {
                count--;
                continue;
            }
            storage_maintenance_record_failure(
                &result, "recording retention stat", stat_errno,
                &s_recording_sd_errors);
            return storage_maintenance_finish(&result);
        }
        errno = 0;
        if (unlink(path) == 0) {
            char meta_name[96];
            char meta_path[384];
            meta_name_for_recording(oldest, meta_name, sizeof(meta_name));
            snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, meta_name);
            errno = 0;
            if (unlink(meta_path) != 0 && errno != ENOENT) {
                storage_maintenance_record_failure(
                    &result, "recording sidecar retention unlink",
                    errno ? errno : EIO, &s_recording_sd_errors);
                if (result.media_failure) {
                    return storage_maintenance_finish(&result);
                }
            }
            bool index_failed = false;
            errno = 0;
            remove_recording_index_rows(oldest, &index_failed);
            if (index_failed) {
                storage_maintenance_record_failure(
                    &result, "recording retention index update",
                    errno ? errno : EINVAL, &s_recording_sd_errors);
                if (result.media_failure) {
                    return storage_maintenance_finish(&result);
                }
            }
            s_recording_files_deleted++;
            count--;
            errno = 0;
            if (update_sd_info() != ESP_OK) {
                storage_maintenance_record_failure(
                    &result, "recording retention capacity refresh",
                    errno ? errno : EIO, NULL);
                if (result.media_failure) {
                    return storage_maintenance_finish(&result);
                }
            }
        } else {
            int unlink_errno = errno ? errno : EIO;
            if (unlink_errno == ENOENT) {
                count--;
                continue;
            }
            storage_maintenance_record_failure(
                &result, "recording retention unlink", unlink_errno,
                &s_recording_sd_errors);
            return storage_maintenance_finish(&result);
        }
    }
    return storage_maintenance_finish(&result);
}

static bool is_recording_cleanup_target(const char *name)
{
    if (!name || name[0] == '\0' || strcmp(name, ".") == 0 ||
        strcmp(name, "..") == 0) {
        return false;
    }
    /* Names come from readdir(), not a URL. FAT long names may legitimately
     * contain spaces or non-ASCII characters, so the Web path whitelist must
     * not make "delete all recordings" silently skip those files. */
    return has_suffix(name, ".avi") ||
           has_suffix(name, ".mjpg") ||
           has_suffix(name, ".jsonl") ||
           has_suffix(name, ".idx") ||
           has_suffix(name, ".part") ||
           has_suffix(name, ".new") ||
           has_suffix(name, ".prev") ||
           has_suffix(name, ".corrupt");
}

static const char *const s_recording_cleanup_indexes[] = {
    RECORDING_INDEX_PATH,
    RECORDING_INDEX_OLD_PATH,
    RECORDING_INDEX_TMP_PATH,
    RECORDING_INDEX_OLD_TMP_PATH,
    RECORDING_SUMMARY_PATH,
    RECORDING_SUMMARY_OLD_PATH,
    RECORDING_SUMMARY_TMP_PATH,
    RECORDING_SUMMARY_OLD_TMP_PATH,
    EVENT_INDEX_PATH,
    EVENT_INDEX_TMP_PATH,
    RECORDING_INDEX_PATH ".bak",
    RECORDING_INDEX_OLD_PATH ".bak",
    RECORDING_SUMMARY_PATH ".bak",
    RECORDING_SUMMARY_OLD_PATH ".bak",
    EVENT_INDEX_PATH ".bak",
};

static const char *recording_cleanup_state_name(recording_cleanup_state_t state)
{
    switch (state) {
    case RECORDING_CLEANUP_IDLE:
        return "idle";
    case RECORDING_CLEANUP_QUEUED:
        return "queued";
    case RECORDING_CLEANUP_RUNNING:
        return "running";
    case RECORDING_CLEANUP_SUCCEEDED:
        return "succeeded";
    case RECORDING_CLEANUP_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

static void recording_cleanup_status_copy(recording_cleanup_status_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    *out = s_recording_cleanup_status;
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
}

static bool recording_cleanup_active(void)
{
    bool active;
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    active = s_recording_cleanup_status.state == RECORDING_CLEANUP_QUEUED ||
             s_recording_cleanup_status.state == RECORDING_CLEANUP_RUNNING;
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
    return active;
}

static void recording_cleanup_status_set_queued(uint32_t job_id)
{
    recording_cleanup_status_t status = {
        .state = RECORDING_CLEANUP_QUEUED,
        .job_id = job_id,
        .queued_ms = esp_timer_get_time() / 1000,
    };
    strlcpy(status.message, "recording cleanup queued", sizeof(status.message));
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    s_recording_cleanup_status = status;
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
}

static void recording_cleanup_status_set_running(uint32_t job_id,
                                                 uint32_t total_files)
{
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    if (s_recording_cleanup_status.job_id == job_id) {
        s_recording_cleanup_status.state = RECORDING_CLEANUP_RUNNING;
        s_recording_cleanup_status.total_files = total_files;
        s_recording_cleanup_status.started_ms = esp_timer_get_time() / 1000;
        strlcpy(s_recording_cleanup_status.message, "recording cleanup running",
                sizeof(s_recording_cleanup_status.message));
    }
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
}

static void recording_cleanup_status_set_progress(uint32_t job_id,
                                                  uint32_t deleted_files,
                                                  uint64_t freed_bytes)
{
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    if (s_recording_cleanup_status.job_id == job_id &&
        s_recording_cleanup_status.state == RECORDING_CLEANUP_RUNNING) {
        s_recording_cleanup_status.deleted_files = deleted_files;
        s_recording_cleanup_status.freed_bytes = freed_bytes;
    }
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
}

static void recording_cleanup_status_finish(uint32_t job_id, bool ok,
                                            uint32_t deleted_files,
                                            uint64_t freed_bytes,
                                            uint32_t remaining_files,
                                            uint32_t error_count,
                                            int first_errno,
                                            const char *message)
{
    portENTER_CRITICAL(&s_recording_cleanup_mux);
    if (s_recording_cleanup_status.job_id == job_id) {
        s_recording_cleanup_status.state = ok ? RECORDING_CLEANUP_SUCCEEDED :
                                               RECORDING_CLEANUP_FAILED;
        s_recording_cleanup_status.deleted_files = deleted_files;
        s_recording_cleanup_status.freed_bytes = freed_bytes;
        s_recording_cleanup_status.remaining_files = remaining_files;
        s_recording_cleanup_status.error_count = error_count;
        s_recording_cleanup_status.first_errno = first_errno;
        s_recording_cleanup_status.finished_ms = esp_timer_get_time() / 1000;
        strlcpy(s_recording_cleanup_status.message,
                message ? message : (ok ? "recordings cleaned" : "recording cleanup failed"),
                sizeof(s_recording_cleanup_status.message));
    }
    portEXIT_CRITICAL(&s_recording_cleanup_mux);
}

static void recording_cleanup_note_error(uint32_t *errors,
                                         int *first_error_number,
                                         const char *operation,
                                         int error_number)
{
    int saved_errno = error_number ? error_number : EIO;
    if (errors) {
        (*errors)++;
    }
    if (first_error_number && *first_error_number == 0) {
        *first_error_number = saved_errno;
    }
    s_recording_sd_errors++;
    if (storage_errno_is_media_failure(saved_errno)) {
        storage_latch_io_error(operation, saved_errno);
        ESP_LOGE(TAG, "%s failed: errno=%d",
                 operation ? operation : "recording cleanup", saved_errno);
    } else {
        ESP_LOGW(TAG, "%s degraded: errno=%d",
                 operation ? operation : "recording cleanup", saved_errno);
    }
}

static uint32_t count_recording_cleanup_targets(uint32_t *errors,
                                                int *first_error_number)
{
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        int open_errno = errno ? errno : EIO;
        if (open_errno != ENOENT) {
            recording_cleanup_note_error(errors, first_error_number,
                                         "recording cleanup verify open",
                                         open_errno);
        }
        errno = first_error_number && *first_error_number ?
                *first_error_number : 0;
        return 0;
    }
    uint32_t count = 0;
    struct dirent *entry;
    errno = 0;
    while ((entry = readdir(dir)) != NULL) {
        if (is_recording_cleanup_target(entry->d_name)) {
            count++;
        }
    }
    if (errno != 0) {
        int read_errno = errno;
        recording_cleanup_note_error(errors, first_error_number,
                                     "recording cleanup verify read",
                                     read_errno);
    }
    errno = 0;
    if (closedir(dir) != 0) {
        recording_cleanup_note_error(errors, first_error_number,
                                     "recording cleanup verify close",
                                     errno ? errno : EIO);
    }
    errno = first_error_number && *first_error_number ?
            *first_error_number : 0;
    return count;
}

static uint32_t count_recording_cleanup_all_targets(uint32_t *errors,
                                                    int *first_error_number)
{
    uint32_t count = count_recording_cleanup_targets(errors, first_error_number);
    for (size_t i = 0;
         i < sizeof(s_recording_cleanup_indexes) /
                 sizeof(s_recording_cleanup_indexes[0]);
         i++) {
        struct stat st = {0};
        errno = 0;
        if (stat(s_recording_cleanup_indexes[i], &st) == 0) {
            if (S_ISREG(st.st_mode)) {
                count++;
            } else {
                recording_cleanup_note_error(errors, first_error_number,
                                             "recording cleanup index layout",
                                             EISDIR);
            }
        } else if (errno != ENOENT) {
            recording_cleanup_note_error(errors, first_error_number,
                                         "recording cleanup index stat",
                                         errno ? errno : EIO);
        }
    }
    return count;
}

static uint32_t cleanup_recording_dir_all(uint64_t *freed_bytes,
                                           uint32_t *remaining_files,
                                           uint32_t *error_count,
                                           int *first_error_number,
                                           uint32_t cleanup_job_id)
{
    if (!s_sd_mounted) {
        if (remaining_files) {
            *remaining_files = 0;
        }
        if (error_count) {
            *error_count = 1;
        }
        if (first_error_number) {
            *first_error_number = ENODEV;
        }
        storage_latch_io_error("recording cleanup without mounted TF", ENODEV);
        errno = ENODEV;
        return 0;
    }
    uint32_t errors = 0;
    int first_errno = 0;
    uint32_t deleted = 0;
    bool media_failure = false;
    size_t batch_size = RECORDING_CLEANUP_BATCH_MAX;
    char (*names)[RECORDING_CLEANUP_NAME_SIZE] =
        heap_caps_malloc(batch_size * RECORDING_CLEANUP_NAME_SIZE,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!names) {
        batch_size = RECORDING_CLEANUP_BATCH_FALLBACK;
        names = malloc(batch_size * RECORDING_CLEANUP_NAME_SIZE);
    }
    if (!names) {
        recording_cleanup_note_error(&errors, &first_errno,
                                     "recording cleanup scan buffer", ENOMEM);
        goto cleanup_verify;
    }
    for (;;) {
        size_t name_count = 0;
        errno = 0;
        DIR *dir = opendir(RECORDING_DIR);
        if (!dir) {
            int open_errno = errno ? errno : EIO;
            if (open_errno != ENOENT) {
                recording_cleanup_note_error(
                    &errors, &first_errno, "recording cleanup directory open",
                    open_errno);
                media_failure |= storage_errno_is_media_failure(open_errno);
            }
            break;
        }
        struct dirent *entry = NULL;
        bool scan_failed = false;
        errno = 0;
        while (name_count < batch_size && (entry = readdir(dir)) != NULL) {
            if (is_recording_cleanup_target(entry->d_name)) {
                strlcpy(names[name_count++], entry->d_name,
                        RECORDING_CLEANUP_NAME_SIZE);
            }
        }
        if (name_count < batch_size && entry == NULL && errno != 0) {
            int read_errno = errno;
            scan_failed = true;
            recording_cleanup_note_error(
                &errors, &first_errno, "recording cleanup directory read",
                read_errno);
            media_failure |= storage_errno_is_media_failure(read_errno);
        }
        errno = 0;
        if (closedir(dir) != 0) {
            int close_errno = errno ? errno : EIO;
            scan_failed = true;
            recording_cleanup_note_error(
                &errors, &first_errno, "recording cleanup directory close",
                close_errno);
            media_failure |= storage_errno_is_media_failure(close_errno);
        }
        if (media_failure) {
            break;
        }
        if (name_count == 0) {
            break;
        }

        uint32_t deleted_this_pass = 0;
        for (size_t i = 0; i < name_count; i++) {
            char path[384];
            size_t path_len = strlcpy(path, RECORDING_DIR "/", sizeof(path));
            if (path_len >= sizeof(path) ||
                strlcat(path, names[i], sizeof(path)) >= sizeof(path)) {
                recording_cleanup_note_error(
                    &errors, &first_errno, "recording cleanup path construction",
                    ENAMETOOLONG);
                continue;
            }
            struct stat st = {0};
            errno = 0;
            if (stat(path, &st) != 0) {
                int stat_errno = errno ? errno : EIO;
                if (stat_errno != ENOENT) {
                    recording_cleanup_note_error(
                        &errors, &first_errno, "recording cleanup target stat",
                        stat_errno);
                    media_failure |= storage_errno_is_media_failure(stat_errno);
                }
                if (media_failure) {
                    break;
                }
                continue;
            }
            if (!S_ISREG(st.st_mode)) {
                recording_cleanup_note_error(
                    &errors, &first_errno, "recording cleanup target layout",
                    EISDIR);
                continue;
            }
            errno = 0;
            if (unlink(path) != 0) {
                int unlink_errno = errno ? errno : EIO;
                if (unlink_errno != ENOENT) {
                    recording_cleanup_note_error(
                        &errors, &first_errno, "recording cleanup target unlink",
                        unlink_errno);
                    media_failure |= storage_errno_is_media_failure(unlink_errno);
                }
                if (media_failure) {
                    break;
                }
                continue;
            }
            deleted++;
            deleted_this_pass++;
            if (freed_bytes && st.st_size > 0) {
                *freed_bytes += (uint64_t)st.st_size;
            }
        }
        recording_cleanup_status_set_progress(
            cleanup_job_id, deleted, freed_bytes ? *freed_bytes : 0);
        if (media_failure) {
            break;
        }
        if (deleted_this_pass == 0) {
            break;
        }
        if (scan_failed) {
            break;
        }
    }
    free(names);
    names = NULL;

    for (size_t i = 0;
         i < sizeof(s_recording_cleanup_indexes) /
                 sizeof(s_recording_cleanup_indexes[0]);
         i++) {
        if (media_failure) {
            break;
        }
        struct stat st = {0};
        errno = 0;
        if (stat(s_recording_cleanup_indexes[i], &st) == 0) {
            if (!S_ISREG(st.st_mode)) {
                recording_cleanup_note_error(
                    &errors, &first_errno, "recording cleanup index layout",
                    EISDIR);
                continue;
            }
            errno = 0;
            if (unlink(s_recording_cleanup_indexes[i]) != 0) {
                int unlink_errno = errno ? errno : EIO;
                if (unlink_errno != ENOENT) {
                    recording_cleanup_note_error(
                        &errors, &first_errno, "recording cleanup index unlink",
                        unlink_errno);
                    media_failure |= storage_errno_is_media_failure(unlink_errno);
                }
                continue;
            }
            deleted++;
            if (freed_bytes && st.st_size > 0) {
                *freed_bytes += (uint64_t)st.st_size;
            }
            recording_cleanup_status_set_progress(
                cleanup_job_id, deleted, freed_bytes ? *freed_bytes : 0);
        } else {
            int stat_errno = errno ? errno : EIO;
            if (stat_errno != ENOENT) {
                recording_cleanup_note_error(
                    &errors, &first_errno, "recording cleanup index stat",
                    stat_errno);
                media_failure |= storage_errno_is_media_failure(stat_errno);
            }
        }
    }

cleanup_verify:
    free(names);
    uint32_t remaining = count_recording_cleanup_all_targets(
        &errors, &first_errno);
    if (remaining == 0 && errors == 0) {
        s_recording_segments = 0;
        s_recording_frames = 0;
        s_recording_bytes = 0;
        s_recording_queued = 0;
        s_recording_dropped = 0;
        s_recording_files_deleted = 0;
        s_recording_zero_frame_archives = 0;
        s_recording_summary_count = 0;
        s_segment_sequence = 0;
        s_current_segment_base[0] = '\0';
    }
    if (remaining_files) {
        *remaining_files = remaining;
    }
    if (error_count) {
        *error_count = errors;
    }
    if (first_error_number) {
        *first_error_number = first_errno;
    }
    ESP_LOGI(TAG, "recording dir cleanup: deleted=%" PRIu32
             " remaining=%" PRIu32 " errors=%" PRIu32,
              deleted, remaining, errors);
    errno = first_errno;
    return deleted;
}

static void recording_cleanup_task(void *arg)
{
    (void)arg;
    while (true) {
        recording_cleanup_request_t request = {0};
        if (xQueueReceive(s_recording_cleanup_queue, &request,
                          portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint32_t deleted = 0;
        uint64_t freed = 0;
        uint32_t remaining = 0;
        uint32_t errors = 0;
        int first_errno = 0;

        if (storage_transition_owner() !=
            STORAGE_TRANSITION_RECORDING_CLEANUP) {
            recording_cleanup_status_finish(
                request.job_id, false, 0, 0, 0, 1, EBUSY,
                "recording cleanup admission was lost; retry from Web");
            continue;
        }
        if (__atomic_load_n(&s_file_download_clients,
                            __ATOMIC_ACQUIRE) > 0) {
            recording_cleanup_status_finish(
                request.job_id, false, 0, 0, 0, 1, EBUSY,
                "a file transfer started before cleanup; wait for it to finish and retry");
            storage_transition_release(
                STORAGE_TRANSITION_RECORDING_CLEANUP);
            field_idle_pause_latch();
            continue;
        }
        if (xSemaphoreTake(s_storage_lock,
                           pdMS_TO_TICKS(RECORDING_CLEANUP_LOCK_TIMEOUT_MS)) !=
            pdTRUE) {
            recording_cleanup_status_finish(
                request.job_id, false, 0, 0, 0, 1, ETIMEDOUT,
                "storage is busy; wait for the current operation and retry cleanup");
            storage_transition_release(
                STORAGE_TRANSITION_RECORDING_CLEANUP);
            field_idle_pause_latch();
            continue;
        }

        if (!s_sd_mounted || s_app_mode != APP_MODE_SERVER ||
            s_storage_quiescing || storage_usb_owned()) {
            first_errno = !s_sd_mounted ? ENODEV : EBUSY;
            errors = 1;
        } else if (__atomic_load_n(&s_file_download_clients,
                                   __ATOMIC_ACQUIRE) > 0) {
            first_errno = EBUSY;
            errors = 1;
        }

        uint32_t total_files = 0;
        if (errors == 0) {
            total_files = count_recording_cleanup_all_targets(
                &errors, &first_errno);
        }
        recording_cleanup_status_set_running(request.job_id, total_files);
        if (errors == 0) {
            deleted = cleanup_recording_dir_all(
                &freed, &remaining, &errors, &first_errno,
                request.job_id);
        } else {
            remaining = total_files;
        }
        xSemaphoreGive(s_storage_lock);

        if (errors == 0) {
            errno = 0;
            if (update_sd_info() != ESP_OK) {
                first_errno = errno ? errno : EIO;
                errors++;
            }
        }
        bool ok = remaining == 0 && errors == 0;
        if (deleted > 0) {
            s_recording_files_deleted += deleted;
        }
        recording_cleanup_status_finish(
            request.job_id, ok, deleted, freed, remaining, errors,
            first_errno,
            ok ? "recordings cleaned" :
                 "cleanup incomplete; check TF health and retry from Web");
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        field_idle_pause_latch();
        ESP_LOGI(TAG,
                 "recording cleanup job=%" PRIu32 " finished ok=%u deleted=%" PRIu32
                 " remaining=%" PRIu32 " errors=%" PRIu32 " freed=%" PRIu64,
                 request.job_id, (unsigned)ok, deleted, remaining, errors,
                 freed);
    }
}

static esp_err_t cleanup_recording_temp_files(void)
{
    if (!s_sd_mounted) {
        return ESP_OK;
    }
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "recording temp directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }
    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "recording temp directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_snapshot_name(entry->d_name)) {
            continue;
        }
        /* 删除所有 .part、.part.corrupt、.idx */
        if (has_suffix(entry->d_name, ".new") ||
            has_suffix(entry->d_name, ".prev") ||
            has_suffix(entry->d_name, ".idx")) {
            char path[384];
            snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
            errno = 0;
            if (unlink(path) != 0 && errno != ENOENT) {
                storage_maintenance_record_failure(
                    &result, "recording temp unlink", errno ? errno : EIO,
                    &s_recording_sd_errors);
            }
            continue;
        }
        /* 删除没有对应 .avi 的孤立 .jsonl */
        if (has_suffix(entry->d_name, ".jsonl")) {
            char avi_name[128];
            strlcpy(avi_name, entry->d_name, sizeof(avi_name));
            size_t len = strlen(avi_name);
            if (len > 6 && strcmp(avi_name + len - 6, ".jsonl") == 0) {
                avi_name[len - 6] = '\0';
                strlcat(avi_name, ".avi", sizeof(avi_name));
            }
            char avi_path[384];
            snprintf(avi_path, sizeof(avi_path), "%s/%s", RECORDING_DIR, avi_name);
            struct stat st = {0};
            errno = 0;
            int stat_ret = stat(avi_path, &st);
            int stat_errno = stat_ret == 0 ? 0 : (errno ? errno : EIO);
            if (stat_ret != 0 && stat_errno != ENOENT) {
                storage_maintenance_record_failure(
                    &result, "recording sidecar pair stat", stat_errno,
                    &s_recording_sd_errors);
                if (result.media_failure) {
                    continue;
                }
            }
            bool keep_for_recovery = false;
            if (stat_ret != 0 && stat_errno == ENOENT) {
                char part_path[416];
                struct stat part_st = {0};
                snprintf(part_path, sizeof(part_path), "%s.part", avi_path);
                if (stat(part_path, &part_st) == 0 && S_ISREG(part_st.st_mode)) {
                    keep_for_recovery = true;
                }
                snprintf(part_path, sizeof(part_path), "%s.part.corrupt", avi_path);
                if (stat(part_path, &part_st) == 0 && S_ISREG(part_st.st_mode)) {
                    keep_for_recovery = true;
                }
            }
            if (!keep_for_recovery &&
                ((stat_ret != 0 && stat_errno == ENOENT) ||
                 (stat_ret == 0 && st.st_size == 0))) {
                char path[384];
                snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
                errno = 0;
                if (unlink(path) != 0 && errno != ENOENT) {
                    storage_maintenance_record_failure(
                        &result, "orphan recording sidecar unlink",
                        errno ? errno : EIO, &s_recording_sd_errors);
                }
            }
        }
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "recording temp directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    return storage_maintenance_finish(&result);
}

static void preserve_failed_recording_part(const char *part_path)
{
    if (!part_path || part_path[0] == '\0') {
        return;
    }
    char corrupt_path[416];
    int len = snprintf(corrupt_path, sizeof(corrupt_path), "%s.corrupt", part_path);
    if (len < 0 || (size_t)len >= sizeof(corrupt_path)) {
        ESP_LOGE(TAG, "cannot preserve failed AVI part; path too long: %s", part_path);
        return;
    }
    errno = 0;
    if (rename(part_path, corrupt_path) == 0) {
        ESP_LOGW(TAG, "preserved unrecoverable AVI part as %s", corrupt_path);
        return;
    }
    int rename_errno = errno ? errno : EIO;
    if (rename_errno == EEXIST) {
        ESP_LOGW(TAG,
                 "unrecoverable AVI part remains at %s because %s already exists",
                 part_path, corrupt_path);
        return;
    }
    if (rename_errno == ENOENT) {
        return;
    }
    ESP_LOGE(TAG, "failed to preserve unrecoverable AVI part %s: errno=%d",
             part_path, rename_errno);
}

static esp_err_t move_recording_recovery_file(const char *src_path,
                                              const char *src_name,
                                              const char *tag)
{
    if (!src_path || !src_name || !tag) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    struct stat st = {0};
    errno = 0;
    if (stat(src_path, &st) != 0) {
        int stat_errno = errno ? errno : EIO;
        if (stat_errno == ENOENT) {
            errno = 0;
            return ESP_OK;
        }
        errno = stat_errno;
        return ESP_FAIL;
    }
    if (!S_ISREG(st.st_mode)) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t dir_ret = ensure_dir(RECORDING_RECOVERY_DIR);
    if (dir_ret != ESP_OK) {
        return dir_ret;
    }

    char dest_path[448];
    int len = snprintf(dest_path, sizeof(dest_path), "%s/%s.%s",
                       RECORDING_RECOVERY_DIR, src_name, tag);
    if (len < 0 || (size_t)len >= sizeof(dest_path)) {
        errno = ENAMETOOLONG;
        return ESP_ERR_INVALID_SIZE;
    }
    errno = 0;
    if (rename(src_path, dest_path) == 0) {
        ESP_LOGW(TAG, "archived recording recovery file %s -> %s",
                 src_name, dest_path);
        return ESP_OK;
    }
    int rename_errno = errno ? errno : EIO;
    if (rename_errno == EEXIST) {
        uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
        len = snprintf(dest_path, sizeof(dest_path), "%s/%s.%" PRIu64 ".%s",
                       RECORDING_RECOVERY_DIR, src_name, now_ms, tag);
        if (len < 0 || (size_t)len >= sizeof(dest_path)) {
            errno = ENAMETOOLONG;
            return ESP_ERR_INVALID_SIZE;
        }
        errno = 0;
        if (rename(src_path, dest_path) == 0) {
            ESP_LOGW(TAG, "archived recording recovery file %s -> %s",
                     src_name, dest_path);
            return ESP_OK;
        }
        rename_errno = errno ? errno : EIO;
    }
    errno = rename_errno;
    return ESP_FAIL;
}

static esp_err_t archive_zero_frame_recording_part(const char *part_path,
                                                   const char *part_name,
                                                   const char *final_name)
{
    if (!part_path || !part_name || !final_name) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGW(TAG, "archiving zero-frame recording part %s", part_name);
    esp_err_t ret = move_recording_recovery_file(part_path, part_name,
                                                 "zero_frame");
    int saved_errno = ret == ESP_OK ? 0 : (errno ? errno : EIO);

    char meta_name[128];
    strlcpy(meta_name, final_name, sizeof(meta_name));
    size_t len = strlen(meta_name);
    if (len > 4 && strcmp(meta_name + len - 4, ".avi") == 0) {
        meta_name[len - 4] = '\0';
        strlcat(meta_name, ".jsonl", sizeof(meta_name));
        char meta_path[384];
        snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, meta_name);
        esp_err_t meta_ret = move_recording_recovery_file(meta_path, meta_name,
                                                          "zero_frame");
        if (ret == ESP_OK && meta_ret != ESP_OK) {
            ret = meta_ret;
            saved_errno = errno ? errno : EIO;
        }
    }
    if (ret == ESP_OK) {
        __atomic_add_fetch(&s_recording_zero_frame_archives, 1,
                           __ATOMIC_ACQ_REL);
        errno = 0;
    } else {
        errno = saved_errno ? saved_errno : EIO;
    }
    return ret;
}

static bool recording_recovery_error_is_zero_frame(esp_err_t ret, int error_number)
{
    if (ret == ESP_ERR_NOT_FOUND && error_number != ENOENT) {
        return true;
    }
#ifdef ENODATA
    if (error_number == ENODATA) {
        return true;
    }
#endif
    return false;
}

static esp_err_t recording_file_ready(const char *name, bool *out_ready)
{
    if (!is_safe_recording_name(name) || !out_ready) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    *out_ready = false;
    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    struct stat st = {0};
    errno = 0;
    if (stat(path, &st) != 0) {
        int stat_errno = errno ? errno : EIO;
        if (stat_errno == ENOENT) {
            errno = 0;
            return ESP_OK;
        }
        errno = stat_errno;
        return ESP_FAIL;
    }
    *out_ready = S_ISREG(st.st_mode) && st.st_size > 0;
    return ESP_OK;
}

static esp_err_t recording_has_paired_file(const char *name, bool *out_paired)
{
    if (!is_safe_recording_name(name) || !out_paired) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    *out_paired = false;
    char pair[96] = {0};
    if (is_annotated_recording_name(name)) {
        if (!raw_name_for_annotated(name, pair, sizeof(pair))) {
            errno = EINVAL;
            return ESP_ERR_INVALID_ARG;
        }
        return recording_file_ready(pair, out_paired);
    }
    if (annotated_name_for_raw(name, pair, sizeof(pair))) {
        bool ready = false;
        esp_err_t ret = recording_file_ready(pair, &ready);
        if (ret != ESP_OK || ready) {
            *out_paired = ready;
            return ret;
        }
    }
    if (!legacy_annotated_name_for_raw(name, pair, sizeof(pair))) {
        return ESP_OK;
    }
    return recording_file_ready(pair, out_paired);
}

static esp_err_t __attribute__((unused))
purge_unpaired_recording_files(uint64_t *freed_bytes, uint32_t *out_deleted)
{
    if (out_deleted) {
        *out_deleted = 0;
    }
    if (!s_sd_mounted) {
        return ESP_OK;
    }
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "unpaired recording directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }
    uint32_t deleted = 0;
    uint32_t removed_rows = 0;
    bool failed = false;
    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "unpaired recording directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_recording_name(entry->d_name) ||
            !has_suffix(entry->d_name, ".avi")) {
            continue;
        }
        bool paired = false;
        errno = 0;
        if (recording_has_paired_file(entry->d_name, &paired) != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording pair stat", errno ? errno : EIO,
                &s_recording_sd_errors);
            continue;
        }
        if (paired) {
            continue;
        }
        bool delete_failed = false;
        errno = 0;
        int ret = delete_recording_files_by_name(entry->d_name, freed_bytes,
                                                 &delete_failed);
        if (ret > 0) {
            deleted += (uint32_t)ret;
        }
        bool row_failed = false;
        int delete_errno = delete_failed ? (errno ? errno : EINVAL) : 0;
        errno = 0;
        uint32_t rows = remove_recording_index_rows(entry->d_name, &row_failed);
        removed_rows += rows;
        failed |= delete_failed || row_failed;
        if (delete_failed) {
            storage_maintenance_record_failure(
                &result, "unpaired recording delete", delete_errno,
                &s_recording_sd_errors);
        }
        if (row_failed) {
            storage_maintenance_record_failure(
                &result, "unpaired recording index update",
                errno ? errno : EINVAL, &s_recording_sd_errors);
        }
        ESP_LOGW(TAG, "removed unpaired recording %s (files=%d rows=%" PRIu32 ")",
                 entry->d_name, ret, rows);
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "unpaired recording directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    if (deleted > 0 || removed_rows > 0) {
        errno = 0;
        if (update_sd_info() != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "unpaired recording capacity refresh",
                errno ? errno : EIO, NULL);
        }
        ESP_LOGW(TAG,
                 "unpaired recording cleanup complete: files=%" PRIu32
                 " rows=%" PRIu32 " failed=%s",
                  deleted, removed_rows, failed ? "true" : "false");
    }
    if (out_deleted) {
        *out_deleted = deleted;
    }
    return storage_maintenance_finish(&result);
}

static void label_count_add_with_unknown(label_count_t *labels, const char *label, bool include_unknown)
{
    if (!labels || !label || !label[0] || (!include_unknown && strcmp(label, "unknown") == 0)) {
        return;
    }

    for (uint32_t i = 0; i < RECORDING_LABEL_BUCKETS; i++) {
        if (labels[i].count > 0 && strcmp(labels[i].label, label) == 0) {
            labels[i].count++;
            return;
        }
    }

    for (uint32_t i = 0; i < RECORDING_LABEL_BUCKETS; i++) {
        if (labels[i].count == 0) {
            strlcpy(labels[i].label, label, sizeof(labels[i].label));
            labels[i].count = 1;
            return;
        }
    }
}

static void label_count_add(label_count_t *labels, const char *label)
{
    label_count_add_with_unknown(labels, label, false);
}

static bool label_counts_to_json(char *buf, size_t size, const label_count_t *labels)
{
    if (!buf || size == 0 || !labels) {
        return false;
    }

    json_writer_t writer;
    json_writer_init(&writer, buf, size);
    json_writer_appendf(&writer, "[");
    bool first = true;
    for (uint32_t i = 0; i < RECORDING_LABEL_BUCKETS && json_writer_ok(&writer); i++) {
        if (!labels[i].count) {
            continue;
        }
        json_writer_appendf(&writer, "%s{\"label\":", first ? "" : ",");
        json_writer_append_escaped_string(&writer, labels[i].label);
        json_writer_appendf(&writer, ",\"count\":%" PRIu32 "}", labels[i].count);
        first = false;
    }
    json_writer_appendf(&writer, "]");
    bool ok = json_writer_ok(&writer);
    if (!ok && size >= 3) {
        strlcpy(buf, "[]", size);
    }
    return ok;
}

static void recording_segment_add_vision(recording_segment_t *seg, const vision_result_t *vision)
{
    if (!seg || !vision) {
        return;
    }

    if (vision->detection_count > 0) {
        seg->hit_frames++;
        seg->detection_total += vision->detection_count;
        for (uint32_t i = 0; i < vision->detection_count && i < APP_MAX_DETECTIONS; i++) {
            label_count_add(seg->labels, vision->detections[i].label);
        }
    } else if (vision->object_count > 0) {
        seg->hit_frames++;
        seg->detection_total += vision->object_count;
        label_count_add(seg->labels, vision->object);
    }
}

static esp_err_t append_recording_segment_jsonl(const recording_segment_t *seg)
{
    if (!seg || !seg->name[0] || seg->frames == 0) {
        return ESP_OK;
    }

    if (rotate_jsonl_if_needed(RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH,
                               "recording index") != ESP_OK) {
        return ESP_FAIL;
    }
    FILE *file = fopen(RECORDING_INDEX_PATH, "a");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        if (!recording_latch_storage_failure("recording index open",
                                             ESP_FAIL, open_errno)) {
            s_recording_sd_errors++;
        }
        set_storage_status("open recording index failed: errno=%d", open_errno);
        return ESP_FAIL;
    }

    char labels[512];
    if (!label_counts_to_json(labels, sizeof(labels), seg->labels)) {
        int close_errno = 0;
        if (sync_and_close_file(&file, false, &close_errno) != ESP_OK) {
            recording_latch_storage_failure("recording index close",
                                            ESP_FAIL, close_errno);
        }
        return ESP_ERR_INVALID_SIZE;
    }
    char raw_name[96] = {0};
    char annotated_name[96] = {0};
    if (seg->kind == RECORDING_KIND_RAW) {
        strlcpy(raw_name, seg->name, sizeof(raw_name));
        annotated_name_for_raw(seg->name, annotated_name, sizeof(annotated_name));
    } else {
        strlcpy(annotated_name, seg->name, sizeof(annotated_name));
        raw_name_for_annotated(seg->name, raw_name, sizeof(raw_name));
    }
    uint64_t end_epoch_ms = seg->start_epoch_ms;
    if (end_epoch_ms > 0 && seg->last_ms > seg->start_ms) {
        end_epoch_ms += (uint64_t)(seg->last_ms - seg->start_ms);
    }
    errno = 0;
    int write_ret = fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"kind\":\"%s\",\"container\":\"avi\",\"codec\":\"mjpeg\","
            "\"name\":\"%s\",\"uri\":\"%s\",\"meta\":\"%s\",\"meta_uri\":\"%s\","
            "\"method\":\"%s\",\"model\":\"%s\",\"raw_name\":\"%s\",\"annotated_name\":\"%s\","
            "\"annotated_uri\":\"%s\",\"manifest_uri\":\"%s?name=%s\","
            "\"first_frame_overlay_uri\":\"%s?name=%s&frame=1\",\"storage_backend\":\"%s\","
            "\"start_ms\":%" PRId64 ",\"end_ms\":%" PRId64
            ",\"start_epoch_ms\":%" PRIu64 ",\"end_epoch_ms\":%" PRIu64
            ",\"clock_source\":\"%s\""
            ",\"duration_ms\":%" PRId64 ",\"frames\":%" PRIu32 ",\"bytes\":%" PRIu64
            ",\"hit_frames\":%" PRIu32 ",\"detection_total\":%" PRIu32 ",\"complete\":true,\"labels\":%s}\n",
            (uint32_t)APP_JSONL_INDEX_VERSION,
            seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
            seg->name, seg->uri, seg->meta_name, seg->meta_uri,
            recognition_method_name(seg->method), seg->model, raw_name, annotated_name,
            seg->kind == RECORDING_KIND_ANNOTATED ? seg->uri : "",
            RECORDING_MANIFEST_URI, seg->name,
            RECORDING_FRAME_SVG_URI, seg->name, s_storage_backend,
            seg->start_ms, seg->last_ms,
            seg->start_epoch_ms, end_epoch_ms, time_source_name(seg->time_source),
            seg->last_ms > seg->start_ms ? seg->last_ms - seg->start_ms : 0,
            seg->frames, seg->bytes, seg->hit_frames, seg->detection_total, labels);
    int write_errno = write_ret < 0 ? (errno ? errno : EIO) : 0;
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, true, &close_errno);
    if (write_ret < 0 || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        recording_latch_storage_failure("recording index append",
                                        ESP_FAIL, saved_errno);
        ESP_LOGE(TAG, "recording index append failed: name=%s errno=%d",
                 seg->name, saved_errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t append_recording_summary_jsonl(const recording_segment_t *seg)
{
    if (!seg || !seg->name[0] || seg->frames == 0) {
        return ESP_OK;
    }

    if (rotate_jsonl_if_needed(RECORDING_SUMMARY_PATH,
                               RECORDING_SUMMARY_OLD_PATH,
                               "recording summary") != ESP_OK) {
        return ESP_FAIL;
    }
    FILE *file = fopen(RECORDING_SUMMARY_PATH, "a");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        if (!recording_latch_storage_failure("recording summary open",
                                             ESP_FAIL, open_errno)) {
            s_recording_sd_errors++;
        }
        set_storage_status("open summary index failed: errno=%d", open_errno);
        return ESP_FAIL;
    }

    char labels[512];
    if (!label_counts_to_json(labels, sizeof(labels), seg->labels)) {
        int close_errno = 0;
        if (sync_and_close_file(&file, false, &close_errno) != ESP_OK) {
            recording_latch_storage_failure("recording summary close",
                                            ESP_FAIL, close_errno);
        }
        return ESP_ERR_INVALID_SIZE;
    }
    char raw_name[96] = {0};
    char annotated_name[96] = {0};
    if (seg->kind == RECORDING_KIND_RAW) {
        strlcpy(raw_name, seg->name, sizeof(raw_name));
        annotated_name_for_raw(seg->name, annotated_name, sizeof(annotated_name));
    } else {
        strlcpy(annotated_name, seg->name, sizeof(annotated_name));
        raw_name_for_annotated(seg->name, raw_name, sizeof(raw_name));
    }
    uint64_t end_epoch_ms = seg->start_epoch_ms;
    if (end_epoch_ms > 0 && seg->last_ms > seg->start_ms) {
        end_epoch_ms += (uint64_t)(seg->last_ms - seg->start_ms);
    }
    errno = 0;
    int write_ret = fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"period_ms\":%" PRIu32 ",\"kind\":\"%s\","
            "\"segment\":\"%s\",\"uri\":\"%s\",\"annotated_uri\":\"%s\",\"manifest_uri\":\"%s?name=%s\","
            "\"method\":\"%s\",\"model\":\"%s\",\"raw_name\":\"%s\",\"annotated_name\":\"%s\","
            "\"first_frame_overlay_uri\":\"%s?name=%s&frame=1\",\"storage_backend\":\"%s\","
            "\"start_ms\":%" PRId64
            ",\"end_ms\":%" PRId64 ",\"start_epoch_ms\":%" PRIu64
            ",\"end_epoch_ms\":%" PRIu64 ",\"clock_source\":\"%s\""
            ",\"frames\":%" PRIu32 ",\"hit_frames\":%" PRIu32
            ",\"detection_total\":%" PRIu32 ",\"labels\":%s}\n",
            (uint32_t)APP_JSONL_INDEX_VERSION, s_recording_segment_ms,
            seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
            seg->name, seg->uri,
            seg->kind == RECORDING_KIND_ANNOTATED ? seg->uri : "",
            RECORDING_MANIFEST_URI, seg->name,
            recognition_method_name(seg->method), seg->model, raw_name, annotated_name,
            RECORDING_FRAME_SVG_URI, seg->name, s_storage_backend, seg->start_ms, seg->last_ms,
            seg->start_epoch_ms, end_epoch_ms, time_source_name(seg->time_source),
            seg->frames, seg->hit_frames, seg->detection_total, labels);
    int write_errno = write_ret < 0 ? (errno ? errno : EIO) : 0;
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, true, &close_errno);
    if (write_ret < 0 || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        recording_latch_storage_failure("recording summary append",
                                        ESP_FAIL, saved_errno);
        ESP_LOGE(TAG, "recording summary append failed: name=%s errno=%d",
                 seg->name, saved_errno);
        return ESP_FAIL;
    }
    s_recording_summary_count++;
    return ESP_OK;
}

static void recording_close_segment(recording_segment_t *seg)
{
    if (!seg || (!seg->writer && !seg->meta_file)) {
        return;
    }

    esp_err_t close_ret = ESP_OK;
    int close_errno = 0;
    if (seg->writer) {
        if (seg->frames > 0) {
            uint64_t duration_ms = seg->last_ms > seg->start_ms ?
                (uint64_t)(seg->last_ms - seg->start_ms) : 0;
            avi_mjpeg_writer_set_duration_ms(seg->writer, duration_ms);
            errno = 0;
            close_ret = avi_mjpeg_writer_close(seg->writer);
            close_errno = errno;
            if (close_ret != ESP_OK) {
                recording_latch_storage_failure("AVI finalize/close",
                                                close_ret, close_errno);
            }
        } else {
            avi_mjpeg_writer_abort(seg->writer);
            unlink(seg->part_path);
        }
    }
    if (seg->meta_file) {
        esp_err_t meta_close_ret = ESP_OK;
        int meta_close_errno = 0;
        errno = 0;
        if (fflush(seg->meta_file) != 0) {
            meta_close_ret = ESP_FAIL;
            meta_close_errno = errno;
        } else {
            errno = 0;
            if (fsync(fileno(seg->meta_file)) != 0) {
                meta_close_ret = ESP_FAIL;
                meta_close_errno = errno;
            }
        }
        errno = 0;
        if (fclose(seg->meta_file) != 0) {
            int fclose_errno = errno ? errno : EIO;
            if (meta_close_ret == ESP_OK ||
                !recording_failure_is_storage_io(meta_close_ret, meta_close_errno)) {
                meta_close_ret = ESP_FAIL;
                meta_close_errno = fclose_errno;
            }
        }
        if (meta_close_ret != ESP_OK) {
            recording_latch_storage_failure("recording metadata finalize/close",
                                            meta_close_ret, meta_close_errno);
            ESP_LOGE(TAG, "recording metadata finalize failed: name=%s errno=%d",
                     seg->meta_name, meta_close_errno);
        }
    }
    seg->writer = NULL;
    seg->meta_file = NULL;

    if (seg->frames == 0 || close_ret != ESP_OK) {
        char path[384];
        if (seg->frames == 0) {
            ESP_LOGE(TAG,
                     "discarding empty recording segment after write failures: kind=%s name=%s",
                     seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                     seg->name);
            unlink(seg->part_path);
            unlink(seg->final_path);
        } else {
            ESP_LOGE(TAG,
                     "recording segment finalize failed: kind=%s name=%s frames=%" PRIu32
                     " bytes=%" PRIu64 " error=%s errno=%d storage_io=%d",
                     seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                     seg->name, seg->frames, seg->bytes, esp_err_to_name(close_ret),
                     close_errno,
                     recording_failure_is_storage_io(close_ret, close_errno));
        }
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, seg->meta_name);
        if (seg->frames == 0) {
            unlink(path);
        }
    } else {
        struct stat st;
        if (stat(seg->final_path, &st) == 0 && st.st_size > 0) {
            seg->bytes = (uint64_t)st.st_size;
        } else {
            ESP_LOGW(TAG, "recording size stat failed: name=%s errno=%d",
                     seg->name, errno);
        }
        if (storage_acceptance_ok()) {
            esp_err_t index_ret = append_recording_segment_jsonl(seg);
            esp_err_t summary_ret = index_ret == ESP_OK ?
                append_recording_summary_jsonl(seg) : index_ret;
            if (index_ret == ESP_OK && summary_ret == ESP_OK) {
                cleanup_old_recordings();
            } else {
                ESP_LOGE(TAG,
                         "recording closed but index publication failed: name=%s index=%s summary=%s; recovery will reconcile after TF retry",
                         seg->name, esp_err_to_name(index_ret),
                         esp_err_to_name(summary_ret));
            }
        } else {
            ESP_LOGW(TAG,
                     "recording indexes deferred because TF write health is not accepted: %s",
                     seg->name);
        }
        update_sd_info();
        s_recording_segments++;
        ESP_LOGI(TAG,
                 "recording segment closed: kind=%s name=%s frames=%" PRIu32
                 " bytes=%" PRIu64 " duration_ms=%" PRId64,
                 seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                 seg->name, seg->frames, seg->bytes,
                 seg->last_ms > seg->start_ms ? seg->last_ms - seg->start_ms : 0);
    }

    s_recording_current_frames = 0;
    s_recording_current_bytes = 0;
    s_recording_current_uri[0] = '\0';
    memset(seg, 0, sizeof(*seg));
}

static esp_err_t recording_open_segment(recording_segment_t *seg, int64_t now_ms,
                                        recording_kind_t kind,
                                        uint32_t width, uint32_t height)
{
    if (!seg || !s_sd_mounted || !width || !height) {
        return ESP_FAIL;
    }

    errno = 0;
    esp_err_t dir_ret = ensure_dir(RECORDING_DIR);
    if (dir_ret != ESP_OK) {
        int dir_errno = errno;
        recording_latch_storage_failure("recording directory create",
                                        dir_ret, dir_errno);
        ESP_LOGE(TAG, "recording mkdir failed: ret=%s errno=%d storage_io=%d",
                 esp_err_to_name(dir_ret), dir_errno,
                 recording_failure_is_storage_io(dir_ret, dir_errno));
        return ESP_FAIL;
    }

    memset(seg, 0, sizeof(*seg));
    seg->kind = kind;
    seg->start_ms = now_ms;
    seg->last_ms = now_ms;
    seg->start_epoch_ms = wall_clock_epoch_ms();
    seg->time_source = seg->start_epoch_ms > 0 ? s_time_source : TIME_SOURCE_UNSYNCED;
    seg->method = recognition_method_or_fallback(s_recognition_method);
    if (seg->method == RECOGNITION_METHOD_OFF ||
        seg->method == RECOGNITION_METHOD_MLP ||
        seg->method == RECOGNITION_METHOD_YOLO11 ||
        seg->method == RECOGNITION_METHOD_YOLO26) {
        seg->method = fish31_espdl_available() ? RECOGNITION_METHOD_FISH31 :
                      recognition_method_or_fallback(preferred_recognition_method());
    }
    strlcpy(seg->model, model_name_for_method(seg->method), sizeof(seg->model));
    char slug[40];
    recording_time_slug(seg->start_epoch_ms, now_ms, slug, sizeof(slug));
    if (kind == RECORDING_KIND_RAW) {
        s_segment_sequence++;
        if (s_segment_sequence == 0) {
            s_segment_sequence = 1;
        }
        snprintf(s_current_segment_base, sizeof(s_current_segment_base), "%03" PRIu32 "_%s_%s",
                 s_segment_sequence, slug, recognition_method_name(seg->method));
        snprintf(seg->name, sizeof(seg->name), "raw_%s.avi", s_current_segment_base);
    } else {
        if (s_current_segment_base[0]) {
            snprintf(seg->name, sizeof(seg->name), "annotated_%s.avi", s_current_segment_base);
        } else {
            snprintf(seg->name, sizeof(seg->name), "annotated_%s_%s.avi",
                     slug, recognition_method_name(seg->method));
        }
    }
    meta_name_for_recording(seg->name, seg->meta_name, sizeof(seg->meta_name));
    snprintf(seg->uri, sizeof(seg->uri), RECORDING_URI_PREFIX "%s", seg->name);
    snprintf(seg->meta_uri, sizeof(seg->meta_uri), RECORDING_META_URI_PREFIX "%s", seg->meta_name);

    snprintf(seg->final_path, sizeof(seg->final_path), "%s/%s", RECORDING_DIR, seg->name);
    snprintf(seg->part_path, sizeof(seg->part_path), "%s/%s.part", RECORDING_DIR, seg->name);
    uint32_t fps = CONFIG_APP_FIELD_RECORDING_MAX_FPS;
    if (fps == 0) {
        fps = 1;
    }
    errno = 0;
    esp_err_t ret = avi_mjpeg_writer_open(&seg->writer, seg->part_path, seg->final_path,
                                          width, height, fps);
    int open_errno = errno;
    if (ret != ESP_OK) {
        recording_latch_storage_failure("AVI open/header sync", ret, open_errno);
        ESP_LOGE(TAG, "open AVI failed: ret=%s errno=%d storage_io=%d",
                 esp_err_to_name(ret), open_errno,
                 recording_failure_is_storage_io(ret, open_errno));
        memset(seg, 0, sizeof(*seg));
        return ret;
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, seg->meta_name);
    errno = 0;
    seg->meta_file = fopen(path, "w");
    if (!seg->meta_file) {
        int meta_open_errno = errno;
        recording_latch_storage_failure("recording metadata open",
                                        ESP_FAIL, meta_open_errno);
        avi_mjpeg_writer_abort(seg->writer);
        seg->writer = NULL;
        unlink(seg->part_path);
        ESP_LOGE(TAG, "open recording meta failed: errno=%d storage_io=%d",
                 meta_open_errno,
                 recording_failure_is_storage_io(ESP_FAIL, meta_open_errno));
        memset(seg, 0, sizeof(*seg));
        return ESP_FAIL;
    }

    strlcpy(s_recording_current_uri, seg->uri, sizeof(s_recording_current_uri));
    s_recording_current_frames = 0;
    s_recording_current_bytes = 0;
    ESP_LOGI(TAG, "recording segment opened: kind=%s name=%s %" PRIu32 "x%" PRIu32
             " fps=%" PRIu32,
             kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
             seg->name, width, height, fps);
    return ESP_OK;
}

static esp_err_t append_recording_event(const recording_segment_t *seg,
                                        const recording_item_t *item,
                                        uint32_t frame_index,
                                        int64_t now_ms)
{
    if (!seg || !item || seg->kind != RECORDING_KIND_ANNOTATED) {
        return ESP_OK;
    }
    FILE *file = fopen(EVENT_INDEX_PATH, "a");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        recording_latch_storage_failure("recording event open",
                                        ESP_FAIL, open_errno);
        return ESP_FAIL;
    }
    char detections[1280];
    if (!detections_to_json(detections, sizeof(detections), &item->meta.vision)) {
        s_recording_sd_errors++;
        ESP_LOGE(TAG, "event detection JSON exceeded safe buffer; row skipped");
        int close_errno = 0;
        if (sync_and_close_file(&file, false, &close_errno) != ESP_OK) {
            recording_latch_storage_failure("recording event close",
                                            ESP_FAIL, close_errno);
        }
        return ESP_ERR_INVALID_SIZE;
    }
    errno = 0;
    int write_ret = fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"session\":\"b%08" PRIx32 "\","
            "\"segment\":\"%s\",\"uri\":\"%s\",\"frame_index\":%" PRIu32 ","
            "\"seq\":%" PRIu32 ",\"time_ms\":%" PRId64 ",\"epoch_ms\":%" PRIu64
            ",\"best_label\":\"%s\","
            "\"best_score\":%" PRIu32 ",\"detection_count\":%" PRIu32 ",\"detections\":%s}\n",
            (uint32_t)APP_JSONL_INDEX_VERSION, s_boot_id, seg->name, seg->uri,
            frame_index, item->meta.seq, now_ms,
            seg->start_epoch_ms > 0 && now_ms >= seg->start_ms ?
                seg->start_epoch_ms + (uint64_t)(now_ms - seg->start_ms) : 0,
            item->meta.vision.label,
            item->meta.vision.object_score, item->meta.vision.detection_count, detections);
    int write_errno = write_ret < 0 ? (errno ? errno : EIO) : 0;
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, false, &close_errno);
    if (write_ret < 0 || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        recording_latch_storage_failure("recording event append",
                                        ESP_FAIL, saved_errno);
        ESP_LOGE(TAG, "recording event append failed: segment=%s frame=%" PRIu32
                 " errno=%d", seg->name, frame_index, saved_errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void recording_write_item(recording_segment_t *seg, const recording_item_t *item)
{
    if (!seg || !item || !item->jpeg || !item->jpeg_size ||
        !storage_acceptance_ok()) {
        return;
    }

    int64_t now_ms = item->meta.timestamp_ms ? item->meta.timestamp_ms : esp_timer_get_time() / 1000;
    uint32_t segment_ms = s_recording_segment_ms;
    if (!seg->writer ||
        (segment_ms > 0 && now_ms - seg->start_ms >= (int64_t)segment_ms)) {
        recording_close_segment(seg);
        if (recording_open_segment(seg, now_ms, item->kind,
                                   item->meta.width, item->meta.height) != ESP_OK) {
            return;
        }
    }

    errno = 0;
    esp_err_t write_ret = avi_mjpeg_writer_add_frame(seg->writer, item->jpeg, item->jpeg_size);
    int write_errno = errno;
    if (write_ret != ESP_OK) {
        bool storage_io = recording_latch_storage_failure("AVI frame write",
                                                          write_ret, write_errno);
        ESP_LOGE(TAG,
                 "AVI frame write failed: kind=%s name=%s seq=%" PRIu32
                 " jpeg=%" PRIu32 " error=%s errno=%d storage_io=%d%s",
                 item->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                 seg->name, item->meta.seq, item->jpeg_size,
                 esp_err_to_name(write_ret), write_errno, storage_io,
                 storage_io ? "; recording writes stopped" : "");
        return;
    }

    seg->frames++;
    seg->last_ms = now_ms;
    seg->bytes += item->jpeg_size;
    recording_segment_add_vision(seg, &item->meta.vision);
    if (seg->meta_file) {
        char detections[1280];
        char top_k[512];
        bool arrays_ok = detections_to_json(detections, sizeof(detections), &item->meta.vision) &&
                         top_k_to_json(top_k, sizeof(top_k), &item->meta.vision);
        if (!arrays_ok) {
            ESP_LOGE(TAG, "recording metadata arrays exceeded safe buffers: name=%s frame=%" PRIu32,
                     seg->meta_name, seg->frames);
        } else {
            errno = 0;
            int meta_ret = fprintf(seg->meta_file,
                "{\"index_version\":%" PRIu32 ",\"segment\":\"%s\",\"segment_uri\":\"%s\","
                "\"kind\":\"%s\","
                "\"meta_uri\":\"%s\",\"overlay_uri\":\"%s?name=%s&frame=%" PRIu32 "\","
                "\"storage_backend\":\"%s\",\"frame_index\":%" PRIu32 ",\"seq\":%" PRIu32
                ",\"time_ms\":%" PRId64 ",\"epoch_ms\":%" PRIu64
                ",\"jpeg_bytes\":%" PRIu32
                ",\"width\":%" PRIu32 ",\"height\":%" PRIu32
                ",\"method\":\"%s\",\"model\":\"%s\",\"label\":\"%s\",\"object\":\"%s\""
                ",\"object_count\":%" PRIu32 ",\"top_k\":%s"
                ",\"box_min_score\":%" PRIu32 ",\"best_score\":%" PRIu32
                ",\"candidate_score\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
                ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64
                ",\"detection_count\":%" PRIu32 ",\"detections\":%s}\n",
                (uint32_t)APP_JSONL_INDEX_VERSION, seg->name, seg->uri,
                seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw", seg->meta_uri,
                RECORDING_FRAME_SVG_URI, seg->name, seg->frames, s_storage_backend,
                seg->frames, item->meta.seq, now_ms,
                seg->start_epoch_ms > 0 && now_ms >= seg->start_ms ?
                    seg->start_epoch_ms + (uint64_t)(now_ms - seg->start_ms) : 0,
                item->jpeg_size,
                item->meta.width, item->meta.height,
                recognition_method_name(seg->method), item->meta.vision.model,
                item->meta.vision.label, item->meta.vision.object,
                item->meta.vision.object_count, top_k,
                item->meta.vision.box_min_score,
                item->meta.vision.object_score, item->meta.vision.candidate_score,
                item->meta.vision.raw_candidate_count,
                item->meta.vision.inference_ms, item->meta.vision.analysis_ms,
                item->meta.vision.detection_count, detections);
            if (meta_ret < 0) {
                int meta_errno = errno;
                recording_latch_storage_failure("recording metadata write",
                                                ESP_FAIL, meta_errno);
                ESP_LOGE(TAG, "recording metadata write failed: name=%s frame=%" PRIu32
                         " errno=%d ferror=%d",
                         seg->meta_name, seg->frames, meta_errno, ferror(seg->meta_file));
            }
            if (storage_acceptance_ok() && (seg->frames % 16U) == 0) {
                errno = 0;
                if (fflush(seg->meta_file) != 0 || fsync(fileno(seg->meta_file)) != 0) {
                    int meta_errno = errno;
                    recording_latch_storage_failure("recording metadata sync",
                                                    ESP_FAIL, meta_errno);
                    ESP_LOGE(TAG, "recording metadata sync failed: name=%s frame=%" PRIu32
                             " errno=%d",
                             seg->meta_name, seg->frames, meta_errno);
                }
            }
        }
    }
    if (storage_acceptance_ok()) {
        append_recording_event(seg, item, seg->frames, now_ms);
    }
    s_recording_frames++;
    s_recording_bytes += item->jpeg_size;
    s_recording_current_frames = seg->frames;
    s_recording_current_bytes = seg->bytes;
    if (seg->frames == 1U || (seg->frames % 64U) == 0U) {
        ESP_LOGI(TAG,
                 "recording write progress: kind=%s name=%s frames=%" PRIu32
                 " bytes=%" PRIu64,
                 seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                 seg->name, seg->frames, seg->bytes);
    }
}

static bool recording_maybe_queue(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta)
{
    if (s_storage_quiescing || s_app_mode != APP_MODE_FIELD ||
        !CONFIG_APP_RECORDING_ENABLE || !s_recording_enabled || !s_sd_mounted ||
        !storage_acceptance_ok() ||
        !s_recording_queue || !jpeg || !jpeg_size || !meta || meta->seq == 0) {
        return false;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    uint32_t max_fps = CONFIG_APP_FIELD_RECORDING_MAX_FPS;
    int64_t min_interval_ms = max_fps > 0 ? 1000 / max_fps : 0;
    if (s_last_recording_frame_ms != 0 && now_ms - s_last_recording_frame_ms < min_interval_ms) {
        return false;
    }
    s_last_recording_frame_ms = now_ms;

    recording_item_t item = {
        .meta = *meta,
        .jpeg_size = jpeg_size,
        .kind = RECORDING_KIND_RAW,
        .method = recognition_method_or_fallback(s_recognition_method),
    };
    item.jpeg = alloc_psram_buffer(jpeg_size);
    if (!item.jpeg) {
        s_recording_dropped++;
        return false;
    }
    memcpy(item.jpeg, jpeg, jpeg_size);

    if (xQueueSend(s_recording_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        free(item.jpeg);
        s_recording_dropped++;
        return false;
    }
    s_recording_queued++;
    if (s_recording_queued == 1 || (s_recording_queued % 100U) == 0U) {
        ESP_LOGI(TAG, "raw recording queue progress: queued=%" PRIu32
                 " seq=%" PRIu32 " jpeg=%" PRIu32,
                 s_recording_queued, meta->seq, jpeg_size);
    }
    return true;
}

static const char *sd_error_hint(esp_err_t err)
{
    if (err == ESP_ERR_NOT_FOUND) {
        return "SD host unavailable: check TF host availability and ESP-Hosted transport";
    }
    if (err == ESP_ERR_TIMEOUT) {
        return "card no response: check SDMMC pins, card insertion, or LDO power";
    }
    if (err == ESP_ERR_INVALID_RESPONSE) {
        return "invalid card response: check signal wiring or bus width";
    }
    if (err == ESP_FAIL) {
        return "mount or filesystem failed";
    }
    return esp_err_to_name(err);
}

static void record_sd_mount_error(const char *mode, esp_err_t err)
{
    strlcpy(s_sd_mount_mode, mode ? mode : "none", sizeof(s_sd_mount_mode));
    s_sd_last_error_code = (int)err;
    snprintf(s_sd_last_error, sizeof(s_sd_last_error), "%s (%s)",
             esp_err_to_name(err), sd_error_hint(err));
    set_storage_status("%s failed: %s", s_sd_mount_mode, s_sd_last_error);
}

static esp_err_t recover_incomplete_recordings(void)
{
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "incomplete recording directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }
    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "incomplete recording directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_snapshot_name(entry->d_name) || !has_suffix(entry->d_name, ".avi.part")) {
            continue;
        }
        char part_path[384];
        char final_name[128];
        strlcpy(final_name, entry->d_name, sizeof(final_name));
        final_name[strlen(final_name) - strlen(".part")] = '\0';
        snprintf(part_path, sizeof(part_path), "%s/%s", RECORDING_DIR, entry->d_name);
        struct stat st = {0};
        errno = 0;
        if (stat(part_path, &st) != 0) {
            int stat_errno = errno ? errno : EIO;
            if (stat_errno != ENOENT) {
                storage_maintenance_record_failure(
                    &result, "incomplete recording stat", stat_errno,
                    &s_recording_sd_errors);
            }
            continue;
        }
        char final_path[384];
        snprintf(final_path, sizeof(final_path), "%s/%s", RECORDING_DIR, final_name);
        if (st.st_size <= (off_t)RECORDING_AVI_HEADER_BYTES) {
            esp_err_t archive_ret =
                archive_zero_frame_recording_part(part_path, entry->d_name,
                                                  final_name);
            if (archive_ret != ESP_OK) {
                storage_maintenance_record_failure(
                    &result, "zero-frame recording archive",
                    errno ? errno : EIO, &s_recording_sd_errors);
            }
            continue;
        }
        ESP_LOGW(TAG, "recovering incomplete recording part %s (%" PRId64 " bytes)",
                 entry->d_name, (int64_t)st.st_size);
        errno = 0;
        esp_err_t recover_ret = avi_mjpeg_recover_part(part_path, final_path);
        if (recover_ret == ESP_OK) {
            ESP_LOGW(TAG, "recovered incomplete recording %s -> %s",
                     entry->d_name, final_name);
            continue;
        }
        int recover_errno = errno;
        if (recover_errno == 0) {
            recover_errno =
                (recover_ret == ESP_ERR_INVALID_RESPONSE ||
                 recover_ret == ESP_ERR_INVALID_SIZE) ? EINVAL :
                (recover_ret == ESP_ERR_NO_MEM ? ENOMEM : EIO);
        }
        ESP_LOGE(TAG, "failed to recover incomplete recording %s: %s errno=%d",
                 entry->d_name, esp_err_to_name(recover_ret), recover_errno);
        if (recover_errno == ENOENT) {
            continue;
        }
        if (recording_recovery_error_is_zero_frame(recover_ret, recover_errno)) {
            esp_err_t archive_ret =
                archive_zero_frame_recording_part(part_path, entry->d_name,
                                                  final_name);
            if (archive_ret != ESP_OK) {
                storage_maintenance_record_failure(
                    &result, "zero-frame recording archive",
                    errno ? errno : EIO, &s_recording_sd_errors);
            }
            continue;
        }
        preserve_failed_recording_part(part_path);
        storage_maintenance_record_failure(
            &result, "incomplete recording recovery", recover_errno,
            &s_recording_sd_errors);
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "incomplete recording directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    if (!result.media_failure) {
        errno = 0;
        esp_err_t cleanup_ret = cleanup_recording_temp_files();
        if (cleanup_ret != ESP_OK && result.result == ESP_OK) {
            result.result = cleanup_ret;
            result.error_number = errno ? errno : EIO;
            result.media_failure =
                storage_errno_is_media_failure(result.error_number);
        }
    }
    return storage_maintenance_finish(&result);
}

static esp_err_t recording_index_contains_name(const char *name, bool *out_found)
{
    if (!name || !out_found) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    *out_found = false;
    const char *paths[] = {RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH};
    char needle[128];
    snprintf(needle, sizeof(needle), "\"name\":\"%s\"", name);
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        errno = ENOMEM;
        return ESP_ERR_NO_MEM;
    }
    bool found = false;
    int saved_errno = 0;
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]) && !found; i++) {
        errno = 0;
        FILE *file = fopen(paths[i], "r");
        if (!file) {
            int open_errno = errno ? errno : EIO;
            if (open_errno != ENOENT) {
                saved_errno = open_errno;
                break;
            }
            continue;
        }
        errno = 0;
        while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
            if (strstr(line, needle)) {
                found = true;
                break;
            }
        }
        if (!found && ferror(file)) {
            saved_errno = errno ? errno : EIO;
        }
        errno = 0;
        if (fclose(file) != 0 && saved_errno == 0) {
            saved_errno = errno ? errno : EIO;
        }
        if (saved_errno != 0) {
            break;
        }
    }
    free(line);
    if (saved_errno != 0) {
        errno = saved_errno;
        return ESP_FAIL;
    }
    *out_found = found;
    errno = 0;
    return ESP_OK;
}

typedef struct {
    char (*slots)[96];
    uint32_t capacity;
    uint32_t count;
    bool overflow;
} recording_name_set_t;

static uint32_t recording_name_hash(const char *name)
{
    uint32_t hash = 2166136261U;
    if (!name) {
        return hash;
    }
    while (*name) {
        hash ^= (uint8_t)*name++;
        hash *= 16777619U;
    }
    return hash;
}

static bool recording_name_set_init(recording_name_set_t *set, uint32_t capacity)
{
    if (!set || capacity == 0) {
        return false;
    }
    memset(set, 0, sizeof(*set));
    set->slots = (char (*)[96])heap_caps_calloc(capacity, sizeof(*set->slots),
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!set->slots) {
        set->slots = (char (*)[96])calloc(capacity, sizeof(*set->slots));
    }
    if (!set->slots) {
        return false;
    }
    set->capacity = capacity;
    return true;
}

static void recording_name_set_free(recording_name_set_t *set)
{
    if (!set) {
        return;
    }
    free(set->slots);
    memset(set, 0, sizeof(*set));
}

static bool recording_name_set_contains(const recording_name_set_t *set, const char *name)
{
    if (!set || !set->slots || !name || !name[0] || set->capacity == 0) {
        return false;
    }
    uint32_t hash = recording_name_hash(name);
    for (uint32_t probe = 0; probe < set->capacity; probe++) {
        uint32_t idx = (hash + probe) % set->capacity;
        if (!set->slots[idx][0]) {
            return false;
        }
        if (strcmp(set->slots[idx], name) == 0) {
            return true;
        }
    }
    return false;
}

static void recording_name_set_add(recording_name_set_t *set, const char *name)
{
    if (!set || !set->slots || !name || !name[0] || set->capacity == 0) {
        return;
    }
    uint32_t hash = recording_name_hash(name);
    for (uint32_t probe = 0; probe < set->capacity; probe++) {
        uint32_t idx = (hash + probe) % set->capacity;
        if (!set->slots[idx][0]) {
            strlcpy(set->slots[idx], name, sizeof(set->slots[idx]));
            set->count++;
            return;
        }
        if (strcmp(set->slots[idx], name) == 0) {
            return;
        }
    }
    set->overflow = true;
}

static esp_err_t recording_name_set_load(recording_name_set_t *set)
{
    if (!set || !set->slots) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    const char *paths[] = {RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH};
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        set->overflow = true;
        errno = ENOMEM;
        return ESP_ERR_NO_MEM;
    }
    int saved_errno = 0;
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        errno = 0;
        FILE *file = fopen(paths[i], "r");
        if (!file) {
            int open_errno = errno ? errno : EIO;
            if (open_errno != ENOENT) {
                saved_errno = open_errno;
                break;
            }
            continue;
        }
        errno = 0;
        while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
            char name[96] = {0};
            if (json_get_string_field(line, "name", name, sizeof(name))) {
                recording_name_set_add(set, name);
            }
        }
        if (ferror(file)) {
            saved_errno = errno ? errno : EIO;
        }
        errno = 0;
        if (fclose(file) != 0 && saved_errno == 0) {
            saved_errno = errno ? errno : EIO;
        }
        if (saved_errno != 0) {
            break;
        }
    }
    free(line);
    if (saved_errno != 0) {
        errno = saved_errno;
        return ESP_FAIL;
    }
    errno = 0;
    return ESP_OK;
}

static int64_t json_number_from_line(const char *line, const char *key)
{
    int64_t value = -1;
    return json_get_int64_field(line, key, &value) ? value : -1;
}

static bool recording_time_mark_from_name(const char *name, int64_t *time_ms)
{
    if (!name || !time_ms) {
        return false;
    }
    const char *mark = strstr(name, "_t");
    if (!mark || mark[2] < '0' || mark[2] > '9') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long long value = strtoull(mark + 2, &end, 10);
    if (errno == ERANGE || end == mark + 2 || value == 0 || value > INT64_MAX ||
        (*end != '\0' && *end != '_' && *end != '.')) {
        return false;
    }
    *time_ms = (int64_t)value;
    return true;
}

static esp_err_t recovered_segment_read_meta(recording_segment_t *seg,
                                             const char *path)
{
    if (!seg || !path) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }
    errno = 0;
    FILE *file = fopen(path, "r");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        if (open_errno == ENOENT) {
            errno = 0;
            return ESP_OK;
        }
        errno = open_errno;
        return ESP_FAIL;
    }
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        int saved_errno = ENOMEM;
        errno = 0;
        if (fclose(file) != 0 && storage_errno_is_media_failure(errno)) {
            saved_errno = errno ? errno : EIO;
        }
        errno = saved_errno;
        return saved_errno == ENOMEM ? ESP_ERR_NO_MEM : ESP_FAIL;
    }

    int64_t first_ms = -1;
    int64_t last_ms = -1;
    errno = 0;
    while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
        char text[64] = {0};
        if (seg->method == RECOGNITION_METHOD_OFF &&
            json_get_string_field(line, "method", text, sizeof(text))) {
            recognition_method_t method = recognition_method_from_text_hint(text);
            if (method != RECOGNITION_METHOD_OFF) {
                seg->method = method;
            }
        }
        if (!seg->model[0] && json_get_string_field(line, "model", text, sizeof(text))) {
            strlcpy(seg->model, text, sizeof(seg->model));
            recognition_method_t method = recognition_method_from_text_hint(text);
            if (seg->method == RECOGNITION_METHOD_OFF && method != RECOGNITION_METHOD_OFF) {
                seg->method = method;
            }
        }
        int64_t time_ms = json_number_from_line(line, "time_ms");
        if (time_ms >= 0) {
            if (first_ms < 0) {
                first_ms = time_ms;
            }
            last_ms = time_ms;
        }
        int64_t detection_count = json_number_from_line(line, "detection_count");
        if (detection_count > 0) {
            seg->hit_frames++;
            seg->detection_total += (uint32_t)detection_count;
        }
        const char *cursor = line;
        static const char label_key[] = "\"label\":\"";
        while ((cursor = strstr(cursor, label_key)) != NULL) {
            cursor += strlen(label_key);
            const char *end = strchr(cursor, '"');
            if (!end) {
                break;
            }
            size_t length = (size_t)(end - cursor);
            if (length > 0 && length < sizeof(seg->labels[0].label)) {
                char label[sizeof(seg->labels[0].label)];
                memcpy(label, cursor, length);
                label[length] = '\0';
                label_count_add(seg->labels, label);
            }
            cursor = end + 1;
        }
    }
    int saved_errno = ferror(file) ? (errno ? errno : EIO) : 0;
    free(line);
    errno = 0;
    if (fclose(file) != 0 && saved_errno == 0) {
        saved_errno = errno ? errno : EIO;
    }
    if (first_ms >= 0 && last_ms >= first_ms) {
        seg->start_ms = first_ms;
        seg->last_ms = last_ms;
    }
    if (saved_errno != 0) {
        errno = saved_errno;
        return ESP_FAIL;
    }
    errno = 0;
    return ESP_OK;
}

static esp_err_t reconcile_recording_indexes(void)
{
    storage_maintenance_result_t result = {0};
    errno = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        storage_maintenance_record_failure(&result, "recording reconcile directory open",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
        return storage_maintenance_finish(&result);
    }
    recording_name_set_t indexed = {0};
    bool have_indexed_set = recording_name_set_init(&indexed, 8192);
    if (have_indexed_set) {
        errno = 0;
        esp_err_t load_ret = recording_name_set_load(&indexed);
        if (load_ret != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording index cache read", errno ? errno : EIO,
                &s_recording_sd_errors);
            recording_name_set_free(&indexed);
            have_indexed_set = false;
        } else {
            ESP_LOGI(TAG, "recording reconcile loaded %" PRIu32 " indexed names%s",
                     indexed.count, indexed.overflow ? " (overflow)" : "");
        }
    } else {
        ESP_LOGW(TAG, "recording reconcile index cache unavailable; using slow lookup");
    }
    uint32_t scanned = 0;
    uint32_t added = 0;
    while (!result.media_failure) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                storage_maintenance_record_failure(
                    &result, "recording reconcile directory read", errno,
                    &s_recording_sd_errors);
            }
            break;
        }
        if (!is_safe_recording_name(entry->d_name) ||
            !has_suffix(entry->d_name, ".avi")) {
            continue;
        }
        scanned++;
        bool already_indexed = false;
        esp_err_t lookup_ret = ESP_OK;
        if (have_indexed_set) {
            already_indexed = recording_name_set_contains(&indexed, entry->d_name);
        } else {
            errno = 0;
            lookup_ret = recording_index_contains_name(entry->d_name,
                                                       &already_indexed);
        }
        if (!already_indexed && have_indexed_set && indexed.overflow) {
            errno = 0;
            lookup_ret = recording_index_contains_name(entry->d_name,
                                                       &already_indexed);
        }
        if (lookup_ret != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording index lookup", errno ? errno : EIO,
                &s_recording_sd_errors);
            continue;
        }
        if (already_indexed) {
            continue;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        avi_mjpeg_info_t info = {0};
        errno = 0;
        esp_err_t probe_ret = avi_mjpeg_probe(path, &info);
        if (probe_ret != ESP_OK) {
            int probe_errno = errno;
            if (probe_errno == 0) {
                probe_errno = probe_ret == ESP_ERR_NOT_FOUND ? ENOENT :
                              (probe_ret == ESP_FAIL ? EIO : EINVAL);
            }
            storage_maintenance_record_failure(
                &result, "recording AVI probe", probe_errno,
                &s_recording_sd_errors);
            continue;
        }
        recording_segment_t *seg = (recording_segment_t *)calloc(1, sizeof(*seg));
        if (!seg) {
            storage_maintenance_record_failure(
                &result, "recording reconcile allocation", ENOMEM, NULL);
            break;
        }
        seg->kind = is_annotated_recording_name(entry->d_name) ?
            RECORDING_KIND_ANNOTATED : RECORDING_KIND_RAW;
        strlcpy(seg->name, entry->d_name, sizeof(seg->name));
        seg->method = recognition_method_from_text_hint(seg->name);
        if (seg->method == RECOGNITION_METHOD_OFF) {
            seg->method = RECOGNITION_METHOD_FISH31;
        }
        strlcpy(seg->model, model_name_for_method(seg->method), sizeof(seg->model));
        meta_name_for_recording(seg->name, seg->meta_name, sizeof(seg->meta_name));
        snprintf(seg->uri, sizeof(seg->uri), RECORDING_URI_PREFIX "%s", seg->name);
        snprintf(seg->meta_uri, sizeof(seg->meta_uri), RECORDING_META_URI_PREFIX "%s",
                 seg->meta_name);
        if (!recording_time_mark_from_name(seg->name, &seg->start_ms)) {
            seg->start_ms = 0;
        }
        seg->last_ms = seg->start_ms + (int64_t)info.duration_ms;
        seg->frames = info.frame_count;
        seg->bytes = info.file_bytes;

        char meta_path[384];
        snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, seg->meta_name);
        errno = 0;
        esp_err_t meta_ret = recovered_segment_read_meta(seg, meta_path);
        if (meta_ret != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording sidecar read", errno ? errno : EIO,
                &s_recording_sd_errors);
            if (result.media_failure) {
                free(seg);
                break;
            }
        }
        if (seg->last_ms > seg->start_ms) {
            uint64_t expected_duration_ms = (uint64_t)(seg->last_ms - seg->start_ms);
            uint64_t duration_delta_ms = info.duration_ms > expected_duration_ms ?
                info.duration_ms - expected_duration_ms :
                expected_duration_ms - info.duration_ms;
            if (duration_delta_ms > 5U) {
                errno = 0;
                esp_err_t retime_ret =
                    avi_mjpeg_retime_file(path, expected_duration_ms);
                if (retime_ret != ESP_OK) {
                    int retime_errno = errno ? errno :
                                       (retime_ret == ESP_FAIL ? EIO : EINVAL);
                    storage_maintenance_record_failure(
                        &result, "recording AVI retime", retime_errno,
                        &s_recording_sd_errors);
                    ESP_LOGW(TAG, "could not retime recovered AVI %s: %s",
                             seg->name, esp_err_to_name(retime_ret));
                    if (result.media_failure) {
                        free(seg);
                        break;
                    }
                }
            }
        }
        errno = 0;
        esp_err_t index_ret = already_indexed ? ESP_OK :
                              append_recording_segment_jsonl(seg);
        int index_errno = index_ret == ESP_OK ? 0 : (errno ? errno : EINVAL);
        errno = 0;
        esp_err_t summary_ret = index_ret == ESP_OK && !already_indexed ?
                                append_recording_summary_jsonl(seg) : index_ret;
        int summary_errno = summary_ret == ESP_OK ? 0 : (errno ? errno : EINVAL);
        if (index_ret != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording reconcile index append", index_errno, NULL);
        } else if (summary_ret != ESP_OK) {
            storage_maintenance_record_failure(
                &result, "recording reconcile summary append", summary_errno, NULL);
        }
        if (!already_indexed && index_ret == ESP_OK && summary_ret == ESP_OK) {
            if (have_indexed_set) {
                recording_name_set_add(&indexed, seg->name);
            }
            added++;
            ESP_LOGI(TAG,
                     "reconciled recording index: %s frames=%" PRIu32
                     " duration_ms=%" PRId64,
                     seg->name, seg->frames,
                     seg->last_ms > seg->start_ms ? seg->last_ms - seg->start_ms : 0);
        }
        free(seg);
        if (result.media_failure) {
            break;
        }
        if ((scanned % 64U) == 0U) {
            ESP_LOGI(TAG, "recording reconcile progress: scanned=%" PRIu32
                     " added=%" PRIu32, scanned, added);
        }
    }
    errno = 0;
    if (closedir(dir) != 0) {
        storage_maintenance_record_failure(&result, "recording reconcile directory close",
                                           errno ? errno : EIO,
                                           &s_recording_sd_errors);
    }
    recording_name_set_free(&indexed);
    ESP_LOGI(TAG, "recording reconcile complete: scanned=%" PRIu32
             " added=%" PRIu32, scanned, added);
    return storage_maintenance_finish(&result);
}

static esp_err_t report_field_session_failure(const char *step, int error_number)
{
    int saved_errno = error_number ? error_number : EIO;
    s_recording_sd_errors++;
    char operation[64];
    snprintf(operation, sizeof(operation), "field session %s",
             step ? step : "append");
    if (storage_errno_is_media_failure(saved_errno)) {
        storage_latch_io_error(operation, saved_errno);
    } else {
        set_storage_status("field session %s failed: errno=%d; FIELD entry aborted",
                           step ? step : "append", saved_errno);
    }
    ESP_LOGE(TAG, "field session %s failed: errno=%d",
             step ? step : "append", saved_errno);
    errno = saved_errno;
    return ESP_FAIL;
}

static esp_err_t append_field_session(void)
{
    if (s_field_session_started) {
        return ESP_OK;
    }

    errno = 0;
    FILE *file = fopen(SESSION_INDEX_PATH, "a");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        return report_field_session_failure("open", open_errno);
    }
    uint64_t epoch_ms = wall_clock_epoch_ms();
    errno = 0;
    int write_ret = fprintf(
        file,
        "{\"index_version\":%" PRIu32 ",\"session\":\"b%08" PRIx32 "\","
        "\"start_ms\":%" PRId64 ",\"start_epoch_ms\":%" PRIu64
        ",\"clock\":\"%s\","
        "\"mode\":\"field\",\"storage_backend\":\"%s\"}\n",
        (uint32_t)APP_JSONL_INDEX_VERSION, s_boot_id,
        esp_timer_get_time() / 1000, epoch_ms,
        epoch_ms > 0 ? time_source_name(s_time_source) : "boot_relative",
        s_storage_backend);
    int write_errno = write_ret < 0 ? (errno ? errno : EIO) : 0;
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, true, &close_errno);
    if (write_ret < 0 || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        return report_field_session_failure(
            write_ret < 0 ? "write" : "flush/fsync/close", saved_errno);
    }
    s_field_session_started = true;
    return ESP_OK;
}

static esp_err_t storage_write_selftest(void)
{
    static const char path[] = HISTORY_ROOT_DIR "/.write_test.tmp";
    enum {
        TEST_BYTES = 4096,
        TEST_CHUNK_BYTES = 512,
    };
    uint8_t buffer[TEST_CHUNK_BYTES];

    errno = 0;
    int fd = open(path, O_CREAT | O_TRUNC | O_WRONLY, 0644);
    if (fd < 0) {
        ESP_LOGE(TAG, "TF write self-test open failed: path=%s errno=%d", path, errno);
        return ESP_FAIL;
    }

    size_t offset = 0;
    while (offset < TEST_BYTES) {
        size_t chunk = TEST_BYTES - offset;
        if (chunk > sizeof(buffer)) {
            chunk = sizeof(buffer);
        }
        for (size_t i = 0; i < chunk; i++) {
            buffer[i] = (uint8_t)((offset + i) * 37U + 0x5aU);
        }
        size_t chunk_offset = 0;
        while (chunk_offset < chunk) {
            errno = 0;
            ssize_t written = write(fd, buffer + chunk_offset, chunk - chunk_offset);
            if (written <= 0) {
                ESP_LOGE(TAG,
                         "TF write self-test short write: offset=%u result=%d errno=%d",
                         (unsigned)(offset + chunk_offset), (int)written, errno);
                close(fd);
                unlink(path);
                return ESP_FAIL;
            }
            chunk_offset += (size_t)written;
        }
        offset += chunk;
    }
    errno = 0;
    if (fsync(fd) != 0) {
        ESP_LOGE(TAG, "TF write self-test fsync failed: errno=%d", errno);
        close(fd);
        unlink(path);
        return ESP_FAIL;
    }
    errno = 0;
    if (close(fd) != 0) {
        ESP_LOGE(TAG, "TF write self-test close failed: errno=%d", errno);
        unlink(path);
        return ESP_FAIL;
    }

    struct stat st = {0};
    errno = 0;
    if (stat(path, &st) != 0 || st.st_size != TEST_BYTES) {
        ESP_LOGE(TAG, "TF write self-test stat failed: size=%" PRId64 " errno=%d",
                 (int64_t)st.st_size, errno);
        unlink(path);
        return ESP_FAIL;
    }

    errno = 0;
    fd = open(path, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "TF write self-test reopen failed: errno=%d", errno);
        unlink(path);
        return ESP_FAIL;
    }
    offset = 0;
    while (offset < TEST_BYTES) {
        size_t chunk = TEST_BYTES - offset;
        if (chunk > sizeof(buffer)) {
            chunk = sizeof(buffer);
        }
        size_t chunk_offset = 0;
        while (chunk_offset < chunk) {
            errno = 0;
            ssize_t read_bytes = read(fd, buffer + chunk_offset, chunk - chunk_offset);
            if (read_bytes <= 0) {
                ESP_LOGE(TAG,
                         "TF write self-test short read: offset=%u result=%d errno=%d",
                         (unsigned)(offset + chunk_offset), (int)read_bytes, errno);
                close(fd);
                unlink(path);
                return ESP_FAIL;
            }
            chunk_offset += (size_t)read_bytes;
        }
        for (size_t i = 0; i < chunk; i++) {
            uint8_t expected = (uint8_t)((offset + i) * 37U + 0x5aU);
            if (buffer[i] != expected) {
                ESP_LOGE(TAG,
                         "TF write self-test data mismatch: offset=%u expected=%02x actual=%02x",
                         (unsigned)(offset + i), expected, buffer[i]);
                close(fd);
                unlink(path);
                return ESP_ERR_INVALID_CRC;
            }
        }
        offset += chunk;
    }
    int final_errno = 0;
    errno = 0;
    if (close(fd) != 0) {
        final_errno = errno ? errno : EIO;
        ESP_LOGE(TAG, "TF write self-test read close failed: errno=%d",
                 final_errno);
    }
    errno = 0;
    if (unlink(path) != 0) {
        int unlink_errno = errno ? errno : EIO;
        ESP_LOGE(TAG, "TF write self-test cleanup failed: errno=%d",
                 unlink_errno);
        if (final_errno == 0) {
            final_errno = unlink_errno;
        }
    }
    if (final_errno != 0) {
        errno = final_errno;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "TF write self-test passed: %u bytes fsync/reopen/verify",
             (unsigned)TEST_BYTES);
    return ESP_OK;
}

#if CONFIG_FATFS_USE_LABEL
static int storage_fresult_errno(FRESULT result)
{
    switch (result) {
    case FR_OK:
        return 0;
    case FR_DISK_ERR:
    case FR_INT_ERR:
    case FR_NO_FILESYSTEM:
    case FR_MKFS_ABORTED:
        return EIO;
    case FR_NOT_READY:
        return ENODEV;
    case FR_WRITE_PROTECTED:
        return EROFS;
    case FR_TIMEOUT:
        return ETIMEDOUT;
    case FR_NO_FILE:
    case FR_NO_PATH:
        return ENOENT;
    case FR_DENIED:
        return EACCES;
    case FR_EXIST:
        return EEXIST;
    case FR_INVALID_OBJECT:
        return EBADF;
    case FR_LOCKED:
        return EBUSY;
    case FR_NOT_ENOUGH_CORE:
        return ENOMEM;
    case FR_TOO_MANY_OPEN_FILES:
        return EMFILE;
    case FR_INVALID_NAME:
    case FR_INVALID_DRIVE:
    case FR_NOT_ENABLED:
    case FR_INVALID_PARAMETER:
    default:
        return EINVAL;
    }
}

static bool storage_fresult_is_media_failure(FRESULT result)
{
    switch (result) {
    case FR_DISK_ERR:
    case FR_INT_ERR:
    case FR_NOT_READY:
    case FR_WRITE_PROTECTED:
    case FR_NO_FILESYSTEM:
    case FR_MKFS_ABORTED:
    case FR_TIMEOUT:
        return true;
    default:
        return false;
    }
}

static esp_err_t report_volume_label_failure(const char *step, FRESULT result)
{
    int saved_errno = storage_fresult_errno(result);
    char operation[64];
    snprintf(operation, sizeof(operation), "volume label %s",
             step ? step : "operation");
    ESP_LOGW(TAG, "TF volume label %s failed: fresult=%d errno=%d",
             step ? step : "operation", (int)result, saved_errno);
    if (storage_fresult_is_media_failure(result)) {
        s_recording_sd_errors++;
        storage_latch_io_error(operation, saved_errno);
    } else {
        set_storage_status(
            "TF write verified; USB volume label %s skipped (fresult=%d errno=%d)",
            step ? step : "operation", (int)result, saved_errno);
    }
    errno = saved_errno;
    return ESP_FAIL;
}
#endif

static esp_err_t storage_ensure_usb_volume_label(void)
{
#if CONFIG_FATFS_USE_LABEL
    if (!storage_backend_is_tf()) {
        return ESP_OK;
    }
    if (!s_sd_card) {
        errno = EINVAL;
        set_storage_status("TF write verified; USB volume label skipped: card handle unavailable");
        return ESP_ERR_INVALID_STATE;
    }
    BYTE pdrv = ff_diskio_get_pdrv_card(s_sd_card);
    if (pdrv == 0xff) {
        ESP_LOGW(TAG, "cannot resolve TF FatFS drive for volume label");
        errno = EINVAL;
        set_storage_status("TF write verified; USB volume label skipped: drive unavailable");
        return ESP_ERR_INVALID_STATE;
    }
    char drive[8];
    char requested[24];
    char label[24] = {0};
    DWORD serial = 0;
    snprintf(drive, sizeof(drive), "%u:", (unsigned)pdrv);
    FRESULT result = f_getlabel(drive, label, &serial);
    if (result != FR_OK) {
        return report_volume_label_failure("read", result);
    }
    if (strcmp(label, "P4_BUOY") == 0) {
        return ESP_OK;
    }
    snprintf(requested, sizeof(requested), "%u:P4_BUOY", (unsigned)pdrv);
    result = f_setlabel(requested);
    if (result == FR_OK) {
        ESP_LOGI(TAG, "TF volume label set to P4_BUOY");
        return ESP_OK;
    }
    return report_volume_label_failure("update", result);
#else
    return ESP_OK;
#endif
}

typedef enum {
    DATASET_RECOVERY_ISSUE_NONE = 0,
    DATASET_RECOVERY_ISSUE_DEGRADED,
    DATASET_RECOVERY_ISSUE_STORAGE_IO,
} dataset_recovery_issue_t;

typedef struct {
    esp_err_t result;
    int error_number;
    dataset_recovery_issue_t issue;
    uint32_t recovered;
    uint32_t preserved;
    uint32_t scanned_entries;
    uint32_t skipped_subtrees;
    int64_t started_ms;
    bool stop_scan;
} dataset_recovery_report_t;

static bool dataset_errno_is_storage_io(int error_number)
{
    return error_number != ENOSPC &&
           storage_errno_is_media_failure(error_number);
}

static void dataset_recovery_record_issue(dataset_recovery_report_t *report,
                                          esp_err_t result, int error_number,
                                          bool storage_io)
{
    if (!report || result == ESP_OK) {
        return;
    }
    dataset_recovery_issue_t issue = storage_io ?
        DATASET_RECOVERY_ISSUE_STORAGE_IO : DATASET_RECOVERY_ISSUE_DEGRADED;
    /* Keep the first issue of a severity, but never hide a later real media I/O
     * failure behind an earlier path/layout warning. The esp_err_t and errno
     * are always replaced together, so callers never observe a mismatched pair. */
    if (report->issue == DATASET_RECOVERY_ISSUE_NONE ||
        (issue == DATASET_RECOVERY_ISSUE_STORAGE_IO &&
         report->issue != DATASET_RECOVERY_ISSUE_STORAGE_IO)) {
        report->result = result;
        report->error_number = error_number;
        report->issue = issue;
    }
}

static bool dataset_recovery_take_scan_budget(dataset_recovery_report_t *report,
                                              const char *dir_path)
{
    if (!report || report->stop_scan) {
        return false;
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (report->scanned_entries >= DATASET_RECOVERY_MAX_SCAN_ENTRIES ||
        now_ms - report->started_ms >= DATASET_RECOVERY_MAX_SCAN_MS) {
        report->stop_scan = true;
        report->skipped_subtrees++;
        dataset_recovery_record_issue(report, ESP_ERR_TIMEOUT, ETIMEDOUT, false);
        ESP_LOGW(TAG,
                 "dataset recovery scan budget exhausted at %s: entries=%" PRIu32
                 " elapsed_ms=%" PRId64 "; unscanned backups are preserved",
                 dir_path ? dir_path : DATASET_ROOT_DIR, report->scanned_entries,
                 now_ms - report->started_ms);
        return false;
    }
    report->scanned_entries++;
    return true;
}

static void recover_dataset_upload_backups_in_dir(
    const char *dir_path, const char *relative_prefix, uint32_t depth,
    dataset_recovery_report_t *report)
{
    if (!dir_path || !relative_prefix || !report || report->stop_scan) {
        return;
    }

    errno = 0;
    DIR *dir = opendir(dir_path);
    if (!dir) {
        int open_errno = errno ? errno : EIO;
        dataset_recovery_record_issue(
            report, ESP_FAIL, open_errno,
            dataset_errno_is_storage_io(open_errno));
        ESP_LOGE(TAG, "dataset recovery cannot open %s: errno=%d", dir_path,
                 open_errno);
        return;
    }

    while (!report->stop_scan) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno != 0) {
                int read_errno = errno;
                dataset_recovery_record_issue(
                    report, ESP_FAIL, read_errno,
                    dataset_errno_is_storage_io(read_errno));
                ESP_LOGE(TAG, "dataset recovery directory read failed: %s errno=%d",
                         dir_path, read_errno);
            }
            break;
        }
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (!dataset_recovery_take_scan_budget(report, dir_path)) {
            break;
        }
        if (!is_safe_snapshot_name(entry->d_name)) {
            ESP_LOGW(TAG,
                     "dataset recovery skipped unsafe entry under %s: %s; any backups remain unchanged",
                     dir_path, entry->d_name);
            report->skipped_subtrees++;
            if (has_suffix(entry->d_name, DATASET_UPLOAD_BACKUP_SUFFIX)) {
                report->preserved++;
            }
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_ARG, EINVAL, false);
            continue;
        }

        char child_path[512];
        char child_relative[DATASET_PATH_MAX + sizeof(DATASET_UPLOAD_BACKUP_SUFFIX)];
        int child_len = snprintf(child_path, sizeof(child_path), "%s/%s",
                                 dir_path, entry->d_name);
        int relative_len = relative_prefix[0] ?
            snprintf(child_relative, sizeof(child_relative), "%s/%s",
                     relative_prefix, entry->d_name) :
            snprintf(child_relative, sizeof(child_relative), "%s", entry->d_name);
        if (child_len < 0 || (size_t)child_len >= sizeof(child_path) ||
            relative_len < 0 || (size_t)relative_len >= sizeof(child_relative)) {
            ESP_LOGW(TAG,
                     "dataset recovery path exceeds safe buffer under %s; subtree preserved",
                     dir_path);
            report->skipped_subtrees++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_SIZE, ENAMETOOLONG, false);
            continue;
        }

        errno = 0;
        struct stat child_stat = {0};
        if (stat(child_path, &child_stat) != 0) {
            int stat_errno = errno ? errno : EIO;
            dataset_recovery_record_issue(
                report, ESP_FAIL, stat_errno,
                dataset_errno_is_storage_io(stat_errno));
            ESP_LOGE(TAG, "dataset recovery stat failed: %s errno=%d",
                     child_path, stat_errno);
            continue;
        }
        if (S_ISDIR(child_stat.st_mode)) {
            if (depth < DATASET_RECOVERY_MAX_DIR_DEPTH) {
                recover_dataset_upload_backups_in_dir(
                    child_path, child_relative, depth + 1U, report);
            } else {
                report->skipped_subtrees++;
                dataset_recovery_record_issue(
                    report, ESP_ERR_INVALID_SIZE, ENAMETOOLONG, false);
                ESP_LOGW(TAG,
                         "dataset recovery depth limit reached at %s; subtree and backups preserved",
                         child_path);
            }
            continue;
        }
        if (!S_ISREG(child_stat.st_mode) ||
            !has_suffix(entry->d_name, DATASET_UPLOAD_BACKUP_SUFFIX)) {
            continue;
        }

        size_t suffix_len = strlen(DATASET_UPLOAD_BACKUP_SUFFIX);
        size_t relative_size = strlen(child_relative);
        if (relative_size <= suffix_len) {
            report->preserved++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_ARG, EINVAL, false);
            continue;
        }
        char final_relative[DATASET_PATH_MAX];
        size_t final_relative_len = relative_size - suffix_len;
        if (final_relative_len >= sizeof(final_relative)) {
            report->preserved++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_SIZE, ENAMETOOLONG, false);
            continue;
        }
        memcpy(final_relative, child_relative, final_relative_len);
        final_relative[final_relative_len] = '\0';
        if (!is_safe_dataset_relpath(final_relative)) {
            ESP_LOGW(TAG, "preserving dataset backup with invalid or too-deep final path: %s",
                     child_path);
            report->preserved++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_ARG, EINVAL, false);
            continue;
        }

        char final_path[512];
        size_t child_path_len = strlen(child_path);
        if (child_path_len <= suffix_len ||
            child_path_len - suffix_len >= sizeof(final_path)) {
            report->preserved++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_SIZE, ENAMETOOLONG, false);
            continue;
        }
        size_t final_path_len = child_path_len - suffix_len;
        memcpy(final_path, child_path, final_path_len);
        final_path[final_path_len] = '\0';

        errno = 0;
        struct stat final_stat = {0};
        if (stat(final_path, &final_stat) == 0) {
            if (!S_ISREG(final_stat.st_mode)) {
                ESP_LOGW(TAG,
                         "dataset final path blocks recovery and is not a file; preserving %s",
                         child_path);
                dataset_recovery_record_issue(
                    report, ESP_ERR_INVALID_STATE, EISDIR, false);
                report->preserved++;
                continue;
            }
            /* The fsynced new file survived and the old backup is still useful
             * for manual recovery. Keep both; a later successful upload may
             * retire the stale backup transactionally. */
            ESP_LOGW(TAG, "dataset final and recovery backup both exist; preserving %s",
                     child_path);
            report->preserved++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_STATE, EEXIST, false);
            continue;
        }
        int final_stat_errno = errno ? errno : EIO;
        if (final_stat_errno != ENOENT) {
            dataset_recovery_record_issue(
                report, ESP_FAIL, final_stat_errno,
                dataset_errno_is_storage_io(final_stat_errno));
            report->preserved++;
            ESP_LOGE(TAG, "dataset final path stat failed: %s errno=%d",
                     final_path, final_stat_errno);
            continue;
        }

        off_t expected_size = child_stat.st_size;
        errno = 0;
        if (rename(child_path, final_path) != 0) {
            int rename_errno = errno ? errno : EIO;
            ESP_LOGE(TAG, "dataset backup recovery failed: %s -> %s errno=%d",
                     child_path, final_path, rename_errno);
            dataset_recovery_record_issue(
                report, ESP_FAIL, rename_errno,
                dataset_errno_is_storage_io(rename_errno));
            report->preserved++;
            continue;
        }
        memset(&final_stat, 0, sizeof(final_stat));
        errno = 0;
        int verify_stat_ret = stat(final_path, &final_stat);
        if (verify_stat_ret != 0 || !S_ISREG(final_stat.st_mode) ||
            final_stat.st_size != expected_size) {
            int verify_errno = verify_stat_ret != 0 && errno ? errno : EIO;
            ESP_LOGE(TAG, "dataset backup recovery verification failed: %s errno=%d",
                     final_path, verify_errno);
            /* Do not strand an unverified file under its normal dataset name.
             * Restore the transaction suffix whenever the filesystem still
             * permits it, so the next mount or USB maintenance pass can retry
             * and the Web dataset list cannot mistake it for valid content. */
            errno = 0;
            if (rename(final_path, child_path) == 0) {
                report->preserved++;
                ESP_LOGW(TAG, "dataset recovery rolled unverified file back to %s",
                         child_path);
            } else {
                int rollback_errno = errno ? errno : EIO;
                report->preserved++;
                ESP_LOGE(TAG,
                         "dataset recovery rollback failed: %s -> %s errno=%d",
                         final_path, child_path, rollback_errno);
                if (dataset_errno_is_storage_io(rollback_errno)) {
                    verify_errno = rollback_errno;
                }
            }
            dataset_recovery_record_issue(
                report, ESP_FAIL, verify_errno,
                dataset_errno_is_storage_io(verify_errno));
            continue;
        }
        ESP_LOGW(TAG, "recovered interrupted dataset upload: %s", final_path);
        report->recovered++;
    }

    errno = 0;
    if (closedir(dir) != 0) {
        int close_errno = errno ? errno : EIO;
        dataset_recovery_record_issue(
            report, ESP_FAIL, close_errno,
            dataset_errno_is_storage_io(close_errno));
        ESP_LOGE(TAG, "dataset recovery directory close failed: %s errno=%d",
                 dir_path, close_errno);
    }
}

static esp_err_t recover_dataset_upload_backups(dataset_recovery_report_t *report)
{
    if (!report) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(report, 0, sizeof(*report));
    report->result = ESP_OK;
    report->started_ms = esp_timer_get_time() / 1000;

    errno = 0;
    DIR *root = opendir(DATASET_ROOT_DIR);
    if (!root) {
        int open_errno = errno ? errno : EIO;
        dataset_recovery_record_issue(
            report, ESP_FAIL, open_errno,
            dataset_errno_is_storage_io(open_errno));
        ESP_LOGE(TAG, "dataset recovery cannot open root %s: errno=%d",
                 DATASET_ROOT_DIR, open_errno);
        errno = report->error_number;
        return report->result;
    }

    while (!report->stop_scan) {
        errno = 0;
        struct dirent *entry = readdir(root);
        if (!entry) {
            if (errno != 0) {
                int read_errno = errno;
                dataset_recovery_record_issue(
                    report, ESP_FAIL, read_errno,
                    dataset_errno_is_storage_io(read_errno));
                ESP_LOGE(TAG, "dataset root scan failed: errno=%d", read_errno);
            }
            break;
        }
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (!dataset_recovery_take_scan_budget(report, DATASET_ROOT_DIR)) {
            break;
        }
        if (!is_safe_dataset_name(entry->d_name)) {
            ESP_LOGW(TAG,
                     "dataset recovery skipped unsafe root entry: %s; any backups remain unchanged",
                     entry->d_name);
            report->skipped_subtrees++;
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_ARG, EINVAL, false);
            continue;
        }
        char dataset_dir[512];
        int path_len = snprintf(dataset_dir, sizeof(dataset_dir), "%s/%s",
                                DATASET_ROOT_DIR, entry->d_name);
        if (path_len < 0 || (size_t)path_len >= sizeof(dataset_dir)) {
            dataset_recovery_record_issue(
                report, ESP_ERR_INVALID_SIZE, ENAMETOOLONG, false);
            ESP_LOGW(TAG, "dataset root entry path is too long; preserving %s",
                     entry->d_name);
            continue;
        }
        errno = 0;
        struct stat dataset_stat = {0};
        if (stat(dataset_dir, &dataset_stat) != 0) {
            int stat_errno = errno ? errno : EIO;
            dataset_recovery_record_issue(
                report, ESP_FAIL, stat_errno,
                dataset_errno_is_storage_io(stat_errno));
            ESP_LOGE(TAG, "dataset root entry stat failed: %s errno=%d",
                     dataset_dir, stat_errno);
            continue;
        }
        if (!S_ISDIR(dataset_stat.st_mode)) {
            continue;
        }
        recover_dataset_upload_backups_in_dir(dataset_dir, "", 0, report);
    }
    errno = 0;
    if (closedir(root) != 0) {
        int close_errno = errno ? errno : EIO;
        dataset_recovery_record_issue(
            report, ESP_FAIL, close_errno,
            dataset_errno_is_storage_io(close_errno));
        ESP_LOGE(TAG, "dataset root close failed: errno=%d", close_errno);
    }
    if (report->recovered > 0 || report->preserved > 0 ||
        report->issue != DATASET_RECOVERY_ISSUE_NONE) {
        ESP_LOGW(TAG,
                 "dataset upload recovery: recovered=%" PRIu32
                 " preserved_backups=%" PRIu32 " scanned=%" PRIu32
                 " skipped_subtrees=%" PRIu32 " result=%s errno=%d storage_io=%d",
                 report->recovered, report->preserved, report->scanned_entries,
                 report->skipped_subtrees, esp_err_to_name(report->result),
                 report->error_number,
                 report->issue == DATASET_RECOVERY_ISSUE_STORAGE_IO);
    }
    if (report->result != ESP_OK) {
        errno = report->error_number;
    }
    return report->result;
}

static esp_err_t storage_prepare_dirs_after_mount(const char *mode)
{
    ESP_LOGI(TAG, "TF post-mount prepare begin: mode=%s app_mode=%s",
             mode ? mode : "unknown", app_mode_name(s_app_mode));
    bool capacity_info_degraded = false;
    int capacity_info_errno = 0;
    errno = 0;
    esp_err_t capacity_ret = update_sd_info();
    if (capacity_ret != ESP_OK) {
        capacity_info_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(capacity_info_errno)) {
            return capacity_ret;
        }
        capacity_info_degraded = true;
    }
    ESP_LOGI(TAG, "TF post-mount: capacity sampled%s",
             capacity_info_degraded ? " with a maintenance warning" : "");
    strlcpy(s_sd_mount_mode, mode, sizeof(s_sd_mount_mode));
    s_sd_last_error[0] = '\0';
    s_sd_last_error_code = 0;

    if (ensure_dir(HISTORY_ROOT_DIR) != ESP_OK ||
        ensure_dir(HISTORY_SNAPSHOT_DIR) != ESP_OK ||
        ensure_dir(RECORDING_DIR) != ESP_OK ||
        ensure_dir(DATASET_ROOT_DIR) != ESP_OK ||
        ensure_dir(DATASET_RUN_DIR) != ESP_OK) {
        set_storage_status("mkdir failed: errno=%d", errno);
        s_history_sd_errors++;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "TF post-mount: directories ready");
    esp_err_t write_test_ret = storage_write_selftest();
    if (write_test_ret != ESP_OK) {
        s_recording_sd_errors++;
        storage_latch_io_error("post-mount write verification", errno);
        set_storage_status("TF write self-test failed: %s", esp_err_to_name(write_test_ret));
        return write_test_ret;
    }
    dataset_recovery_report_t dataset_recovery = {0};
    esp_err_t dataset_recovery_ret =
        recover_dataset_upload_backups(&dataset_recovery);
    if (dataset_recovery.issue == DATASET_RECOVERY_ISSUE_STORAGE_IO) {
        storage_latch_io_error("dataset upload backup recovery",
                               dataset_recovery.error_number);
        set_storage_status(
            "dataset recovery failed: %s errno=%d; use USB to preserve .upload.bak",
            esp_err_to_name(dataset_recovery_ret), dataset_recovery.error_number);
        return dataset_recovery_ret != ESP_OK ? dataset_recovery_ret : ESP_FAIL;
    }
    bool dataset_recovery_degraded =
        dataset_recovery.issue == DATASET_RECOVERY_ISSUE_DEGRADED ||
        dataset_recovery.preserved > 0 || dataset_recovery.skipped_subtrees > 0;
    s_storage_io_latched = false;
    s_storage_last_errno = 0;
    s_storage_write_verified = true;
    s_storage_write_verified_ms = esp_timer_get_time() / 1000;
    errno = 0;
    esp_err_t label_ret = storage_ensure_usb_volume_label();
    int label_errno = label_ret == ESP_OK ? 0 : (errno ? errno : EIO);
    bool volume_label_degraded = label_ret != ESP_OK;
    if (volume_label_degraded && storage_backend_is_tf() &&
        !storage_acceptance_ok()) {
        /* A FatFS label request surfaced a real medium failure after the write
         * self-test. Do not let a later success message restore write health. */
        return label_ret;
    }
    ESP_LOGI(TAG, "TF post-mount: volume label checked%s",
             volume_label_degraded ? " with a maintenance warning" : "");

    if (s_app_mode == APP_MODE_SERVER) {
        ESP_LOGI(TAG, "TF post-mount: write verified; running bounded recording recovery in SERVER mode");
        bool recording_recovery_degraded = false;
        int recording_recovery_errno = 0;
        const char *recording_recovery_stage = "none";
        errno = 0;
        esp_err_t recording_recovery_ret = recover_incomplete_recordings();
        if (recording_recovery_ret != ESP_OK) {
            int helper_errno = errno ? errno : EIO;
            if (storage_errno_is_media_failure(helper_errno) ||
                !storage_acceptance_ok()) {
                return recording_recovery_ret;
            }
            recording_recovery_degraded = true;
            recording_recovery_errno = helper_errno;
            recording_recovery_stage = "part recovery";
        }
        errno = 0;
        recording_recovery_ret = reconcile_recording_indexes();
        if (recording_recovery_ret != ESP_OK) {
            int helper_errno = errno ? errno : EIO;
            if (storage_errno_is_media_failure(helper_errno) ||
                !storage_acceptance_ok()) {
                return recording_recovery_ret;
            }
            if (!recording_recovery_degraded) {
                recording_recovery_degraded = true;
                recording_recovery_errno = helper_errno;
                recording_recovery_stage = "index reconciliation";
            }
        }
        if (recording_recovery_degraded) {
            set_storage_status(
                "TF write verified on %s; recording recovery warning at %s errno=%d; inspect recordings via Web/USB",
                mode, recording_recovery_stage, recording_recovery_errno);
        } else if (capacity_info_degraded) {
            set_storage_status(
                "TF write verified on %s; capacity warning errno=%d; dataset=%s label=%s",
                mode, capacity_info_errno,
                dataset_recovery_degraded ? "warn" : "ok",
                volume_label_degraded ? "warn" : "ok");
        } else if (dataset_recovery_degraded && volume_label_degraded) {
            set_storage_status(
                "TF write verified on %s; dataset recovery warning %s/%d kept=%" PRIu32
                " skip=%" PRIu32 "; USB label warning errno=%d",
                mode,
                esp_err_to_name(dataset_recovery.result),
                dataset_recovery.error_number, dataset_recovery.preserved,
                dataset_recovery.skipped_subtrees, label_errno);
        } else if (dataset_recovery_degraded) {
            set_storage_status(
                "TF write verified on %s; dataset recovery warning %s/%d kept=%" PRIu32
                " skip=%" PRIu32 "; inspect .upload.bak via USB",
                mode, esp_err_to_name(dataset_recovery.result),
                dataset_recovery.error_number, dataset_recovery.preserved,
                dataset_recovery.skipped_subtrees);
        } else if (volume_label_degraded) {
            set_storage_status(
                "TF write verified on %s; USB volume label unavailable errno=%d; "
                "recording remains available",
                mode, label_errno);
        } else {
            set_storage_status("TF ready on %s; write verified; recovery available from Web", mode);
        }
        return ESP_OK;
    }

    esp_err_t field_prepare_ret = rotate_history_index_if_needed();
    if (field_prepare_ret != ESP_OK) {
        return field_prepare_ret;
    }
    field_prepare_ret = rotate_jsonl_if_needed(
        RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH, "recording index");
    if (field_prepare_ret != ESP_OK) {
        return field_prepare_ret;
    }
    field_prepare_ret = rotate_jsonl_if_needed(
        RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_OLD_PATH, "recording summary");
    if (field_prepare_ret != ESP_OK) {
        return field_prepare_ret;
    }
    bool field_maintenance_degraded = capacity_info_degraded;
    int field_maintenance_errno = capacity_info_errno;
    const char *field_maintenance_stage = capacity_info_degraded ?
                                          "capacity sample" : "none";
    errno = 0;
    field_prepare_ret = cleanup_old_snapshots();
    if (field_prepare_ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) ||
            !storage_acceptance_ok()) {
            return field_prepare_ret;
        }
        if (!field_maintenance_degraded) {
            field_maintenance_degraded = true;
            field_maintenance_errno = helper_errno;
            field_maintenance_stage = "snapshot cleanup";
        }
    }
    errno = 0;
    field_prepare_ret = cleanup_old_recordings();
    if (field_prepare_ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) ||
            !storage_acceptance_ok()) {
            return field_prepare_ret;
        }
        if (!field_maintenance_degraded) {
            field_maintenance_degraded = true;
            field_maintenance_errno = helper_errno;
            field_maintenance_stage = "recording cleanup";
        }
    }
    errno = 0;
    field_prepare_ret = recover_incomplete_recordings();
    if (field_prepare_ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) ||
            !storage_acceptance_ok()) {
            return field_prepare_ret;
        }
        if (!field_maintenance_degraded) {
            field_maintenance_degraded = true;
            field_maintenance_errno = helper_errno;
            field_maintenance_stage = "recording recovery";
        }
    }
    errno = 0;
    field_prepare_ret = reconcile_recording_indexes();
    if (field_prepare_ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) ||
            !storage_acceptance_ok()) {
            return field_prepare_ret;
        }
        if (!field_maintenance_degraded) {
            field_maintenance_degraded = true;
            field_maintenance_errno = helper_errno;
            field_maintenance_stage = "index reconciliation";
        }
    }
    field_prepare_ret = append_field_session();
    if (field_prepare_ret != ESP_OK) {
        return field_prepare_ret;
    }
    errno = 0;
    field_prepare_ret = update_sd_info();
    if (field_prepare_ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (!storage_errno_is_media_failure(helper_errno) &&
            !field_maintenance_degraded) {
            field_maintenance_degraded = true;
            field_maintenance_errno = helper_errno;
            field_maintenance_stage = "capacity refresh";
        }
    }
    if (!storage_acceptance_ok()) {
        ESP_LOGE(TAG, "FIELD preparation invalidated TF write health");
        errno = s_storage_last_errno ? s_storage_last_errno : EIO;
        return field_prepare_ret != ESP_OK ? field_prepare_ret : ESP_FAIL;
    }
    if (field_maintenance_degraded) {
        set_storage_status(
            "field recording active; warning=%s/%d dataset=%s label=%s",
            field_maintenance_stage, field_maintenance_errno,
            dataset_recovery_degraded ? "warn" : "ok",
            volume_label_degraded ? "warn" : "ok");
    } else if (dataset_recovery_degraded && volume_label_degraded) {
        set_storage_status(
            "field recording active on %s with dataset recovery warning %s/%d "
            "kept=%" PRIu32 " skip=%" PRIu32 "; USB label warning errno=%d",
            mode, esp_err_to_name(dataset_recovery.result),
            dataset_recovery.error_number, dataset_recovery.preserved,
            dataset_recovery.skipped_subtrees, label_errno);
    } else if (dataset_recovery_degraded) {
        set_storage_status(
            "field recording active with dataset recovery warning %s/%d kept=%" PRIu32
            " skip=%" PRIu32 "; inspect .upload.bak via USB",
            esp_err_to_name(dataset_recovery.result),
            dataset_recovery.error_number, dataset_recovery.preserved,
            dataset_recovery.skipped_subtrees);
    } else if (volume_label_degraded) {
        set_storage_status(
            "field recording active on %s; USB volume label unavailable errno=%d",
            mode, label_errno);
    } else {
        set_storage_status("mounted on %s; field recording enabled", mode);
    }
    return ESP_OK;
}

/*
 * ESP-Hosted initializes the P4 SDMMC controller for Slot 1 before app_main.
 * IDF 6.0 cannot initialize that controller a second time, so Slot 0 follows
 * Espressif's host_sdcard_with_hosted workaround and only initializes its slot.
 */
static esp_err_t storage_sdmmc_host_init_already_done(void)
{
    return ESP_OK;
}

static esp_err_t storage_sdmmc_host_deinit_keep_hosted(void)
{
    return ESP_OK;
}

static esp_err_t storage_reset_card_power(void)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << APP_SD_POWER_RESET_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&config);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_RETURN_ON_ERROR(gpio_set_level(APP_SD_POWER_RESET_PIN, 1), TAG,
                        "disable SD external power");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(gpio_set_level(APP_SD_POWER_RESET_PIN, 0), TAG,
                        "enable SD external power");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t storage_unmount_locked(const char *reason);

static esp_err_t __attribute__((unused)) storage_mount_sdmmc_width(int width,
                                                                   int max_freq_khz,
                                                                   bool format_if_failed)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_failed,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = max_freq_khz;
    host.unaligned_multi_block_rw_max_chunk_size = 8;
    host.pwr_ctrl_handle = s_sd_pwr_ctrl;
#if CONFIG_ESP_HOSTED_ENABLED && CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    if (s_hosted_sdmmc_host_active) {
        host.init = storage_sdmmc_host_init_already_done;
        host.deinit = storage_sdmmc_host_deinit_keep_hosted;
    }
#endif

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = width >= 4 ? 4 : 1;
    slot_config.clk = CONFIG_APP_SD_PIN_CLK;
    slot_config.cmd = CONFIG_APP_SD_PIN_CMD;
    slot_config.d0 = CONFIG_APP_SD_PIN_D0;
    if (slot_config.width == 4) {
        slot_config.d1 = CONFIG_APP_SD_PIN_D1;
        slot_config.d2 = CONFIG_APP_SD_PIN_D2;
        slot_config.d3 = CONFIG_APP_SD_PIN_D3;
    }
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    char mode[24];
    snprintf(mode, sizeof(mode), "sdmmc_%dbit", slot_config.width);
    s_sd_attempts++;
    ESP_LOGI(TAG,
             "TF mount attempt #%" PRIu32 ": %s slot=0 freq=%d kHz clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d ldo=%d format_if_failed=%d",
             s_sd_attempts, mode, max_freq_khz,
             CONFIG_APP_SD_PIN_CLK, CONFIG_APP_SD_PIN_CMD,
             CONFIG_APP_SD_PIN_D0, CONFIG_APP_SD_PIN_D1, CONFIG_APP_SD_PIN_D2,
             CONFIG_APP_SD_PIN_D3, CONFIG_APP_SD_LDO_IO_ID, format_if_failed);

    sdmmc_card_t *mounted_card = NULL;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &mounted_card);
    if (ret != ESP_OK) {
        /* The convenience mount API owns its temporary card allocation on
         * failure. Never leave an old or partially-consumed pointer visible to
         * later health checks or teardown attempts. */
        s_sd_card = NULL;
        /* In Hosted mode host.deinit is intentionally a no-op so Slot 1 stays
         * alive for the C6 link. The IDF mount helper therefore cannot release
         * Slot 0 when card initialization fails after slot creation. Explicitly
         * retire Slot 0 here; the operation is idempotent when failure happened
         * before slot initialization or the normal host teardown already ran. */
        esp_err_t slot_ret = sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
        if (slot_ret != ESP_OK && slot_ret != ESP_ERR_INVALID_ARG &&
            slot_ret != ESP_ERR_INVALID_STATE) {
            s_sdmmc_slot_cleanup_pending = true;
            ESP_LOGE(TAG, "TF mount failed and Slot 0 cleanup also failed: %s",
                     esp_err_to_name(slot_ret));
            record_sd_mount_error("sdmmc_slot_cleanup", slot_ret);
            return slot_ret;
        }
        ESP_LOGW(TAG, "TF mount failed on %s: %s - %s", mode, esp_err_to_name(ret), sd_error_hint(ret));
        record_sd_mount_error(mode, ret);
        return ret;
    }

    s_sd_card = mounted_card;
    s_sd_mounted = true;
    strlcpy(s_storage_backend, "tf_sdmmc", sizeof(s_storage_backend));
    ESP_LOGI(TAG, "TF card mounted at %s via SDMMC %d-bit", CONFIG_APP_SD_MOUNT_POINT, slot_config.width);
    sdmmc_card_print_info(stdout, s_sd_card);
    ret = storage_prepare_dirs_after_mount(mode);
    if (ret != ESP_OK) {
        storage_unmount_locked("mount verification failed");
    }
    return ret;
}

static esp_err_t storage_mount_sdspi(spi_host_device_t spi_host, int max_freq_khz,
                                     bool format_if_failed)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_failed,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = spi_host;
    host.max_freq_khz = max_freq_khz;
    host.unaligned_multi_block_rw_max_chunk_size = 8;
    host.pwr_ctrl_handle = s_sd_pwr_ctrl;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_APP_SD_PIN_CMD,
        .miso_io_num = CONFIG_APP_SD_PIN_D0,
        .sclk_io_num = CONFIG_APP_SD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32 * 1024,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        set_storage_status("spi%d init failed: %s", (int)spi_host + 1, esp_err_to_name(ret));
        record_sd_mount_error("sdspi_init", ret);
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_APP_SD_PIN_D3;
    slot_config.host_id = host.slot;
    slot_config.wait_for_miso = 100;

    char mode[24];
    snprintf(mode, sizeof(mode), "sdspi%d", (int)spi_host + 1);
    s_sd_attempts++;
    ESP_LOGI(TAG,
             "TF mount attempt #%" PRIu32 ": %s freq=%d kHz mosi/cmd=%d miso/d0=%d sclk=%d cs/d3=%d ldo=%d format_if_failed=%d",
             s_sd_attempts, mode, max_freq_khz,
             CONFIG_APP_SD_PIN_CMD, CONFIG_APP_SD_PIN_D0,
             CONFIG_APP_SD_PIN_CLK, CONFIG_APP_SD_PIN_D3, CONFIG_APP_SD_LDO_IO_ID, format_if_failed);

    sdmmc_card_t *mounted_card = NULL;
    ret = esp_vfs_fat_sdspi_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config,
                                  &mount_config, &mounted_card);
    if (ret != ESP_OK) {
        s_sd_card = NULL;
        ESP_LOGW(TAG, "TF mount failed on %s: %s - %s", mode, esp_err_to_name(ret), sd_error_hint(ret));
        record_sd_mount_error(mode, ret);
        spi_bus_free(host.slot);
        return ret;
    }

    s_sd_card = mounted_card;
    s_sd_mounted = true;
    s_sd_spi_host = spi_host;
    strlcpy(s_storage_backend, "tf_sdspi", sizeof(s_storage_backend));
    ESP_LOGI(TAG, "TF card mounted at %s via SDSPI%d", CONFIG_APP_SD_MOUNT_POINT, (int)spi_host + 1);
    sdmmc_card_print_info(stdout, s_sd_card);
    ret = storage_prepare_dirs_after_mount(mode);
    if (ret != ESP_OK) {
        storage_unmount_locked("mount verification failed");
    }
    return ret;
}

static esp_err_t __attribute__((unused)) storage_mount_flash_fallback(void)
{
    if (s_flash_wl_handle != WL_INVALID_HANDLE) {
        s_sd_mounted = true;
        strlcpy(s_storage_backend, "flash_fat", sizeof(s_storage_backend));
        return ESP_OK;
    }

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 8,
        .allocation_unit_size = 4096,
        .use_one_fat = true,
    };

    ESP_LOGW(TAG,
             "TF unavailable; mounting internal flash FAT partition '%s' at %s as online fallback storage",
             CONFIG_APP_FLASH_STORAGE_LABEL, CONFIG_APP_SD_MOUNT_POINT);
    esp_err_t ret = esp_vfs_fat_spiflash_mount_rw_wl(CONFIG_APP_SD_MOUNT_POINT,
                                                     CONFIG_APP_FLASH_STORAGE_LABEL,
                                                     &mount_config,
                                                     &s_flash_wl_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "flash fallback mount failed: %s", esp_err_to_name(ret));
        record_sd_mount_error("flash_fat", ret);
        return ret;
    }

    s_sd_mounted = true;
    s_sd_card = NULL;
    strlcpy(s_storage_backend, "flash_fat", sizeof(s_storage_backend));
    esp_err_t prep = storage_prepare_dirs_after_mount("flash_fat");
    if (prep == ESP_OK) {
        set_storage_status("mounted on flash_fat fallback; TF unavailable");
        ESP_LOGI(TAG, "flash fallback storage mounted at %s", CONFIG_APP_SD_MOUNT_POINT);
    }
    return prep;
}

static esp_err_t storage_mount_internal(bool format_if_failed)
{
    bool locked = false;
    if (s_storage_lock) {
        xSemaphoreTake(s_storage_lock, portMAX_DELAY);
        locked = true;
    }

    esp_err_t final_ret = ESP_OK;
    if (!CONFIG_APP_SD_ENABLE) {
        set_storage_status("disabled");
        final_ret = ESP_ERR_NOT_SUPPORTED;
        goto out;
    }

    if (s_sd_mounted) {
        final_ret = storage_acceptance_ok() ? ESP_OK : ESP_ERR_INVALID_STATE;
        goto out;
    }

    esp_err_t ret = ESP_FAIL;
    storage_clear_write_health();
    s_storage_io_latched = false;
    s_storage_last_errno = 0;

    /*
     * Each candidate is a complete mount transaction. A failed candidate is
     * fully released before the next one starts; callers must not wrap this in
     * another retry loop. Resource/state errors stop immediately because a
     * different bus width cannot repair exhausted or leaked resources.
     */
    enum { CANDIDATE_SDMMC, CANDIDATE_SDSPI };
    typedef struct {
        int kind;
        int width;
        int freq_khz;
    } mount_candidate_t;
    mount_candidate_t candidates[3];
    size_t candidate_count = 0;
#if CONFIG_APP_SD_USE_SDMMC
    candidates[candidate_count++] = (mount_candidate_t) {
        .kind = CANDIDATE_SDMMC,
        .width = CONFIG_APP_SD_BUS_WIDTH,
        .freq_khz = CONFIG_APP_SD_MAX_FREQ_KHZ,
    };
    if (CONFIG_APP_SD_BUS_WIDTH > 1) {
        candidates[candidate_count++] = (mount_candidate_t) {
            .kind = CANDIDATE_SDMMC,
            .width = 1,
            .freq_khz = CONFIG_APP_SD_MAX_FREQ_KHZ < 10000 ?
                        CONFIG_APP_SD_MAX_FREQ_KHZ : 10000,
        };
    }
#endif
    candidates[candidate_count++] = (mount_candidate_t) {
        .kind = CANDIDATE_SDSPI,
        .width = 1,
        .freq_khz = CONFIG_APP_SD_MAX_FREQ_KHZ < 5000 ?
                    CONFIG_APP_SD_MAX_FREQ_KHZ : 5000,
    };

    for (size_t candidate = 0; candidate < candidate_count; candidate++) {
        ret = storage_unmount_locked("prepare next mount candidate");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "TF mount blocked by incomplete teardown: %s",
                     esp_err_to_name(ret));
            break;
        }
        sd_pwr_ctrl_ldo_config_t ldo_config = {
            .ldo_chan_id = CONFIG_APP_SD_LDO_IO_ID,
        };
        ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_pwr_ctrl);
        if (ret != ESP_OK) {
            set_storage_status("SD power init failed: %s", esp_err_to_name(ret));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = storage_reset_card_power();
        if (ret != ESP_OK) {
            esp_err_t cleanup_ret = storage_unmount_locked("card power reset failed");
            if (cleanup_ret != ESP_OK) {
                ret = cleanup_ret;
            }
            break;
        }

        if (candidates[candidate].kind == CANDIDATE_SDMMC) {
            ret = storage_mount_sdmmc_width(candidates[candidate].width,
                                            candidates[candidate].freq_khz,
                                            format_if_failed);
        } else {
            ret = storage_mount_sdspi(SPI2_HOST, candidates[candidate].freq_khz,
                                      format_if_failed);
        }
        if (ret == ESP_OK) {
            break;
        }
        esp_err_t cleanup_ret = storage_unmount_locked("mount candidate failed");
        if (cleanup_ret != ESP_OK) {
            ret = cleanup_ret;
            break;
        }
        if (ret == ESP_ERR_NO_MEM || ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "TF mount stopped after resource/state error: %s",
                     esp_err_to_name(ret));
            break;
        }
    }
    if (ret != ESP_OK) {
        s_history_sd_errors++;
        storage_clear_write_health();
    }
    final_ret = ret;

out:
    if (locked) {
        xSemaphoreGive(s_storage_lock);
    }
    return final_ret;
}

static esp_err_t storage_mount(void)
{
    return storage_mount_internal(false);
}

static void storage_mark_application_volume_detached(void)
{
    s_sd_card = NULL;
    s_sd_mounted = false;
    s_sd_total_bytes = 0;
    s_sd_free_bytes = 0;
    storage_clear_write_health();
    strlcpy(s_storage_backend, "none", sizeof(s_storage_backend));
}

static esp_err_t storage_retry_vfs_cleanup_locked(void)
{
    if (!s_storage_vfs_cleanup_pending) {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_unregister_path(CONFIG_APP_SD_MOUNT_POINT);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_storage_vfs_cleanup_pending = false;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "pending FAT VFS path cleanup failed: %s", esp_err_to_name(ret));
    set_storage_status("FAT VFS teardown is incomplete: %s; storage remount is blocked",
                       esp_err_to_name(ret));
    return ret;
}

static esp_err_t storage_unmount_locked(const char *reason)
{
    esp_err_t pending_vfs_ret = storage_retry_vfs_cleanup_locked();
    if (pending_vfs_ret != ESP_OK) {
        return pending_vfs_ret;
    }
    if (s_sdmmc_slot_cleanup_pending) {
        esp_err_t cleanup_ret = sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
        if (cleanup_ret != ESP_OK && cleanup_ret != ESP_ERR_INVALID_ARG &&
            cleanup_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "pending SDMMC slot0 cleanup failed: %s",
                     esp_err_to_name(cleanup_ret));
            set_storage_status("SDMMC teardown is incomplete: %s; storage remount is blocked",
                               esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
        s_sdmmc_slot_cleanup_pending = false;
    }
    if (s_sdspi_bus_cleanup_pending) {
        esp_err_t cleanup_ret = spi_bus_free(s_sd_spi_host);
        if (cleanup_ret != ESP_OK && cleanup_ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "pending SPI bus cleanup failed: %s",
                     esp_err_to_name(cleanup_ret));
            set_storage_status("SPI teardown is incomplete: %s; storage remount is blocked",
                               esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
        s_sdspi_bus_cleanup_pending = false;
    }

    esp_err_t result = ESP_OK;
    if (s_sd_mounted && s_sd_card) {
        bool was_sdmmc = strcmp(s_storage_backend, "tf_sdmmc") == 0;
        bool was_sdspi = strcmp(s_storage_backend, "tf_sdspi") == 0;
        ESP_LOGI(TAG, "Unmounting TF card: %s", reason ? reason : "no reason");
        esp_err_t unmount_ret = esp_vfs_fat_sdcard_unmount(
            CONFIG_APP_SD_MOUNT_POINT, s_sd_card);

        /* IDF 6.0 removes the diskio registration, deinitializes the host and
         * may free sdmmc_card_t before its final VFS path unregister call. Its
         * return value therefore cannot be used as proof that the card pointer
         * is still owned by the application. Retire it unconditionally. */
        storage_mark_application_volume_detached();
        if (unmount_ret != ESP_OK) {
            result = unmount_ret;
            s_storage_vfs_cleanup_pending = true;
            ESP_LOGE(TAG,
                     "TF VFS unmount reported %s after ownership may have been consumed; application handle retired",
                     esp_err_to_name(unmount_ret));
            esp_err_t cleanup_ret = storage_retry_vfs_cleanup_locked();
            if (cleanup_ret != ESP_OK) {
                ESP_LOGE(TAG, "TF VFS cleanup remains pending after unmount error: %s",
                         esp_err_to_name(cleanup_ret));
            }
        }

        if (was_sdmmc) {
            esp_err_t slot_ret = sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_0);
            if (slot_ret != ESP_OK &&
                slot_ret != ESP_ERR_INVALID_ARG &&
                slot_ret != ESP_ERR_INVALID_STATE) {
                s_sdmmc_slot_cleanup_pending = true;
                ESP_LOGE(TAG, "SDMMC slot0 explicit release failed: %s",
                         esp_err_to_name(slot_ret));
                if (result == ESP_OK) {
                    result = slot_ret;
                }
            }
        }
        if (was_sdspi) {
            esp_err_t bus_ret = spi_bus_free(s_sd_spi_host);
            if (bus_ret != ESP_OK && bus_ret != ESP_ERR_INVALID_STATE) {
                s_sdspi_bus_cleanup_pending = true;
                ESP_LOGE(TAG, "SPI bus free failed: %s", esp_err_to_name(bus_ret));
                if (result == ESP_OK) {
                    result = bus_ret;
                }
            }
        }
    } else if (s_sd_mounted && s_flash_wl_handle != WL_INVALID_HANDLE) {
        ESP_LOGI(TAG, "Unmounting flash fallback storage: %s", reason ? reason : "no reason");
        esp_err_t unmount_ret = esp_vfs_fat_spiflash_unmount_rw_wl(
            CONFIG_APP_SD_MOUNT_POINT, s_flash_wl_handle);
        /* The flash helper likewise tears down diskio/WL before returning a
         * possible final VFS unregister error. Do not reuse the old handle. */
        s_flash_wl_handle = WL_INVALID_HANDLE;
        storage_mark_application_volume_detached();
        if (unmount_ret != ESP_OK) {
            result = unmount_ret;
            s_storage_vfs_cleanup_pending = true;
            ESP_LOGE(TAG, "flash fallback VFS unmount reported %s; old handle retired",
                      esp_err_to_name(unmount_ret));
            (void)storage_retry_vfs_cleanup_locked();
        }
    }
    if (s_sd_pwr_ctrl) {
        esp_err_t ldo_ret = sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
        if (ldo_ret != ESP_OK) {
            ESP_LOGE(TAG, "SD LDO release failed: %s", esp_err_to_name(ldo_ret));
            if (result == ESP_OK) {
                result = ldo_ret;
            }
        } else {
            s_sd_pwr_ctrl = NULL;
        }
    }
    if (result == ESP_OK) {
        set_storage_status("unmounted for %s", reason ? reason : "storage switch");
    } else {
        set_storage_status(
            "storage detached but teardown needs attention: %s; Web retry remains available",
            esp_err_to_name(result));
    }
    return result;
}

static esp_err_t storage_unmount(const char *reason)
{
    bool locked = false;
    if (s_storage_lock) {
        xSemaphoreTake(s_storage_lock, portMAX_DELAY);
        locked = true;
    }

    esp_err_t ret = storage_unmount_locked(reason);

    if (locked) {
        xSemaphoreGive(s_storage_lock);
    }
    return ret;
}

static void history_record_to_json(char *buf, size_t size, const history_record_t *record)
{
    if (!buf || size == 0 || !record) {
        return;
    }

    char fallback_dets[] = "[]";
    char *dets = (char *)alloc_psram_buffer(1280);
    if (!dets) {
        dets = fallback_dets;
    }
    if (dets != fallback_dets && !detections_to_json(dets, 1280, &record->vision)) {
        strlcpy(dets, fallback_dets, 1280);
    }

    json_writer_t writer;
    json_writer_init(&writer, buf, size);
    json_writer_appendf(&writer, "{\"index_version\":%" PRIu32 ",\"storage_backend\":",
                        (uint32_t)APP_JSONL_INDEX_VERSION);
    json_writer_append_escaped_string(&writer, s_storage_backend);
    json_writer_appendf(&writer,
                        ",\"seq\":%" PRIu32 ",\"time_ms\":%" PRId64
                        ",\"stored_ms\":%" PRId64 ",\"source\":",
                        record->seq, record->timestamp_ms, record->stored_ms);
    json_writer_append_escaped_string(&writer, record->source[0] ? record->source : "camera");
    json_writer_appendf(&writer,
                        ",\"width\":%" PRIu32 ",\"height\":%" PRIu32
                        ",\"jpeg_bytes\":%" PRIu32 ",\"capture_ms\":%" PRId64
                        ",\"encode_ms\":%" PRId64 ",\"inference_ms\":%" PRId64
                        ",\"analysis_ms\":%" PRId64 ",\"recognition_method\":",
                        record->width, record->height, record->jpeg_size,
                        record->capture_ms, record->encode_ms,
                        record->vision.inference_ms, record->vision.analysis_ms);
    json_writer_append_escaped_string(&writer,
                                      recognition_method_name(record->recognition_method));
    json_writer_appendf(&writer, ",\"network_mode\":");
    json_writer_append_escaped_string(&writer, network_mode_name(record->network_mode));
    json_writer_appendf(&writer,
                        ",\"rssi_dbm\":%d,\"model_bytes\":%" PRIu32
                        ",\"model_input_size\":%" PRIu32 ",\"free_heap\":%" PRIu32
                        ",\"min_free_heap\":%" PRIu32 ",\"free_psram\":%" PRIu32
                        ",\"min_free_psram\":%" PRIu32 ",\"label\":",
                        record->rssi_dbm, record->model_bytes, record->model_input_size,
                        record->free_heap, record->min_free_heap,
                        record->free_psram, record->min_free_psram);
    json_writer_append_escaped_string(&writer, record->vision.label);
    json_writer_appendf(&writer, ",\"object\":");
    json_writer_append_escaped_string(&writer, record->vision.object);
    json_writer_appendf(&writer, ",\"scene\":");
    json_writer_append_escaped_string(&writer, record->vision.scene);
    json_writer_appendf(&writer, ",\"color\":");
    json_writer_append_escaped_string(&writer, record->vision.color);
    json_writer_appendf(&writer, ",\"model\":");
    json_writer_append_escaped_string(&writer, record->vision.model);
    json_writer_appendf(&writer,
                        ",\"motion\":%s,\"motion_score\":%" PRIu32
                        ",\"edge_score\":%" PRIu32 ",\"avg_luma\":%" PRIu32
                        ",\"avg_r\":%" PRIu32 ",\"avg_g\":%" PRIu32
                        ",\"avg_b\":%" PRIu32 ",\"object_score\":%" PRIu32
                        ",\"candidate_score\":%" PRIu32 ",\"box_min_score\":%" PRIu32
                        ",\"object_count\":%" PRIu32 ",\"detection_count\":%" PRIu32
                        ",\"raw_candidate_count\":%" PRIu32 ",\"best_score\":%" PRIu32
                        ",\"detections\":%s,\"object_x\":%" PRIu32
                        ",\"object_y\":%" PRIu32 ",\"object_w\":%" PRIu32
                        ",\"object_h\":%" PRIu32 ",\"coke_score\":%" PRIu32
                        ",\"sprite_score\":%" PRIu32 ",\"unknown_score\":%" PRIu32
                        ",\"snapshot\":",
                        record->vision.motion ? "true" : "false",
                        record->vision.motion_score, record->vision.edge_score,
                        record->vision.avg_luma, record->vision.avg_r, record->vision.avg_g,
                        record->vision.avg_b, record->vision.object_score,
                        record->vision.candidate_score, record->vision.box_min_score,
                        record->vision.object_count, record->vision.detection_count,
                        record->vision.raw_candidate_count, record->vision.object_score, dets,
                        record->vision.object_x, record->vision.object_y,
                        record->vision.object_w, record->vision.object_h,
                        record->vision.coke_score, record->vision.sprite_score,
                        record->vision.unknown_score);
    json_writer_append_escaped_string(&writer, record->snapshot);
    json_writer_appendf(&writer, "}");

    if (!json_writer_ok(&writer)) {
        json_writer_init(&writer, buf, size);
        json_writer_appendf(&writer,
                            "{\"index_version\":%" PRIu32
                            ",\"error\":\"history serialization overflow\"}",
                            (uint32_t)APP_JSONL_INDEX_VERSION);
        if (!json_writer_ok(&writer)) {
            buf[0] = '\0';
        }
    }
    if (dets != fallback_dets) {
        free(dets);
    }
}

static esp_err_t append_history_jsonl(const history_record_t *record)
{
    errno = 0;
    if (rotate_history_index_if_needed() != ESP_OK) {
        return ESP_FAIL;
    }

    errno = 0;
    FILE *file = fopen(HISTORY_JSONL_PATH, "a");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        s_history_sd_errors++;
        storage_latch_io_error("history index open", open_errno);
        set_storage_status("open history failed: errno=%d", open_errno);
        return ESP_FAIL;
    }

    char *line = (char *)alloc_psram_buffer(4096);
    if (!line) {
        int close_errno = 0;
        if (sync_and_close_file(&file, false, &close_errno) != ESP_OK) {
            storage_latch_io_error("history index close", close_errno);
        }
        return ESP_ERR_NO_MEM;
    }
    history_record_to_json(line, 4096, record);
    if (!line[0]) {
        free(line);
        int close_errno = 0;
        if (sync_and_close_file(&file, false, &close_errno) != ESP_OK) {
            storage_latch_io_error("history index close", close_errno);
        }
        return ESP_ERR_INVALID_SIZE;
    }
    errno = 0;
    int write_ret = fprintf(file, "%s\n", line);
    int write_errno = write_ret < 0 ? (errno ? errno : EIO) : 0;
    free(line);
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, true, &close_errno);
    if (write_ret < 0 || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        s_history_sd_errors++;
        storage_latch_io_error("history index append", saved_errno);
        ESP_LOGE(TAG, "history index append failed: errno=%d", saved_errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t save_snapshot_jpeg(const history_item_t *item, char *snapshot_uri, size_t snapshot_uri_size)
{
    if (!CONFIG_APP_HISTORY_SAVE_JPEG || !item->jpeg || !item->jpeg_size) {
        snapshot_uri[0] = '\0';
        return ESP_OK;
    }

    char name[96];
    snprintf(name, sizeof(name), "b%08" PRIx32 "_f%08" PRIx32 ".jpg", s_boot_id, item->record.seq);

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, name);
    errno = 0;
    FILE *file = fopen(path, "wb");
    if (!file) {
        int open_errno = errno ? errno : EIO;
        s_history_sd_errors++;
        storage_latch_io_error("history snapshot open", open_errno);
        set_storage_status("open snapshot failed: errno=%d", open_errno);
        return ESP_FAIL;
    }

    errno = 0;
    size_t written = fwrite(item->jpeg, 1, item->jpeg_size, file);
    int write_errno = written != item->jpeg_size ? (errno ? errno : EIO) : 0;
    int close_errno = 0;
    esp_err_t close_ret = sync_and_close_file(&file, true, &close_errno);
    if (written != item->jpeg_size || close_ret != ESP_OK) {
        int saved_errno = write_errno ? write_errno : close_errno;
        (void)unlink(path);
        s_history_sd_errors++;
        storage_latch_io_error("history snapshot write", saved_errno);
        set_storage_status("snapshot write failed: errno=%d", saved_errno);
        return ESP_FAIL;
    }

    strlcpy(snapshot_uri, SNAPSHOT_URI_PREFIX, snapshot_uri_size);
    strlcat(snapshot_uri, name, snapshot_uri_size);
    return ESP_OK;
}

static void history_store_item(history_item_t *item)
{
    if (!item) {
        return;
    }

    item->record.stored_ms = esp_timer_get_time() / 1000;
    item->record.snapshot[0] = '\0';

    if (s_sd_mounted) {
        esp_err_t snapshot_ret = save_snapshot_jpeg(
            item, item->record.snapshot, sizeof(item->record.snapshot));
        if (snapshot_ret == ESP_OK && storage_acceptance_ok()) {
            (void)append_history_jsonl(&item->record);
        }
        if ((s_history_saved % 10) == 0) {
            cleanup_old_snapshots();
            update_sd_info();
        }
    }

    history_push_record(&item->record);
    s_history_saved++;
}

static void history_maybe_queue(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta,
                                recognition_method_t method, const char *source, bool force_hit)
{
    if (s_app_mode != APP_MODE_FIELD ||
        !CONFIG_APP_HISTORY_ENABLE || !s_history_enabled ||
        !s_history_queue || !jpeg || !jpeg_size || !meta || meta->seq == 0) {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    bool hit = meta->vision.detection_count > 0 || meta->vision.object_count > 0;
    if (!hit) {
        return;
    }
    bool force = force_hit && hit;
    if (!force && s_last_history_ms != 0 &&
        now_ms - s_last_history_ms < (int64_t)s_history_sample_interval_ms) {
        return;
    }
    s_last_history_ms = now_ms;

    uint32_t free_heap = 0;
    uint32_t min_free_heap = 0;
    uint32_t free_psram = 0;
    uint32_t min_free_psram = 0;
    sample_memory_stats(&free_heap, &min_free_heap, &free_psram, &min_free_psram);

    history_item_t item = {
        .record = {
            .seq = meta->seq,
            .jpeg_size = jpeg_size,
            .width = meta->width,
            .height = meta->height,
            .timestamp_ms = meta->timestamp_ms,
            .capture_ms = meta->capture_ms,
            .encode_ms = meta->encode_ms,
            .vision = meta->vision,
            .recognition_method = method,
            .network_mode = s_network_mode,
            .rssi_dbm = wifi_rssi(),
            .model_bytes = model_bytes_for_method(method),
            .model_input_size = model_input_size_for_method(method),
            .free_heap = free_heap,
            .min_free_heap = min_free_heap,
            .free_psram = free_psram,
            .min_free_psram = min_free_psram,
        },
        .jpeg_size = CONFIG_APP_HISTORY_SAVE_JPEG ? jpeg_size : 0,
    };
    strlcpy(item.record.source, source ? source : "camera", sizeof(item.record.source));

    if (CONFIG_APP_HISTORY_SAVE_JPEG) {
        item.jpeg = alloc_psram_buffer(jpeg_size);
        if (!item.jpeg) {
            s_history_dropped++;
            return;
        }
        memcpy(item.jpeg, jpeg, jpeg_size);
    }

    if (xQueueSend(s_history_queue, &item, 0) != pdTRUE) {
        free(item.jpeg);
        s_history_dropped++;
        return;
    }
    s_history_queued++;
}

static bool inference_worker_busy(void)
{
    UBaseType_t queued = s_inference_queue ? uxQueueMessagesWaiting(s_inference_queue) : 0;
    return s_inference_worker_busy || queued > 0;
}

static bool queue_inference_job(const uint8_t *jpeg, uint32_t jpeg_size,
                                 const frame_meta_t *meta, recognition_method_t method)
{
    if (s_storage_quiescing || storage_transition_active()) {
        return false;
    }
    if (!s_inference_queue || !jpeg || !jpeg_size || !meta || !recognition_method_uses_jpeg_inference(method)) {
        return false;
    }

    inference_job_t job = {
        .jpeg_size = jpeg_size,
        .meta = *meta,
        .method = method,
        .queued_ms = esp_timer_get_time() / 1000,
    };

    job.jpeg = alloc_psram_buffer(jpeg_size);
    if (!job.jpeg) {
        s_inference_queue_drops++;
        s_inference_dropped_frames++;
        return false;
    }
    memcpy(job.jpeg, jpeg, jpeg_size);

    /*
     * 不阻塞摄像头任务：队列满就直接丢弃本次抽帧。
     * 这比排队等待更适合实时视频，因为用户更关心“最新状态”，而不是几十秒前的旧帧结果。
     */
    if (xQueueSend(s_inference_queue, &job, 0) != pdTRUE) {
        free(job.jpeg);
        s_inference_queue_drops++;
        s_inference_dropped_frames++;
        return false;
    }

    s_inference_jobs_queued++;
    return true;
}

static validation_context_t *validation_context_create(void)
{
    validation_context_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return NULL;
    }
    ctx->done = xSemaphoreCreateBinary();
    if (!ctx->done) {
        free(ctx);
        return NULL;
    }
    ctx->refs = 1;
    return ctx;
}

static void validation_context_retain(validation_context_t *ctx)
{
    if (ctx) {
        __atomic_add_fetch(&ctx->refs, 1, __ATOMIC_RELAXED);
    }
}

static void validation_context_release(validation_context_t *ctx)
{
    if (!ctx || __atomic_sub_fetch(&ctx->refs, 1, __ATOMIC_ACQ_REL) != 0) {
        return;
    }
    if (ctx->done) {
        vSemaphoreDelete(ctx->done);
    }
    free(ctx);
}

static void validation_cache_set_state(validation_context_t *ctx,
                                       validation_job_state_t state,
                                       esp_err_t err)
{
    if (!ctx || !ctx->publish_result || !s_validation_lock) {
        return;
    }
    if (state == VALIDATION_JOB_RUNNING && ctx->started_ms == 0) {
        ctx->started_ms = esp_timer_get_time() / 1000;
    }

    xSemaphoreTake(s_validation_lock, portMAX_DELAY);
    if (state == VALIDATION_JOB_QUEUED) {
        validation_cache_t fresh = {
            .valid = true,
            .id = ctx->id,
            .state = state,
            .err = err,
            .sample = ctx->sample,
            .method = ctx->method,
            .box_min_score = ctx->box_min_score,
            .jpeg_size = ctx->jpeg_size,
            .queued_ms = ctx->queued_ms,
        };
        s_validation_last = fresh;
    } else if (s_validation_last.valid && s_validation_last.id == ctx->id) {
        s_validation_last.state = state;
        s_validation_last.err = err;
        s_validation_last.started_ms = ctx->started_ms;
        if (state == VALIDATION_JOB_FAILED) {
            s_validation_last.completed_ms = ctx->completed_ms;
        }
    }
    xSemaphoreGive(s_validation_lock);
}

static void validation_cache_update(validation_context_t *ctx)
{
    if (!ctx || !ctx->publish_result || !s_validation_lock) {
        return;
    }

    xSemaphoreTake(s_validation_lock, portMAX_DELAY);
    if (s_validation_last.valid && s_validation_last.id == ctx->id) {
        validation_cache_t completed = {
            .valid = true,
            .id = ctx->id,
            .state = ctx->err == ESP_OK ?
                     VALIDATION_JOB_SUCCEEDED : VALIDATION_JOB_FAILED,
            .err = ctx->err,
            .sample = ctx->sample,
            .method = ctx->method,
            .box_min_score = ctx->box_min_score,
            .vision = ctx->vision,
            .source_w = ctx->source_w,
            .source_h = ctx->source_h,
            .jpeg_size = ctx->jpeg_size,
            .queued_ms = ctx->queued_ms,
            .started_ms = ctx->started_ms,
            .completed_ms = ctx->completed_ms,
        };
        s_validation_last = completed;
    }
    xSemaphoreGive(s_validation_lock);
}

static esp_err_t generate_annotated_jpeg_from_vision(
    const uint8_t *jpeg, uint32_t jpeg_size,
    recognition_method_t method, const vision_result_t *vision,
    uint32_t source_w, uint32_t source_h,
    uint8_t **annotated_jpeg, size_t *annotated_size);

static void inference_task(void *arg)
{
    (void)arg;
    inference_job_t job;

    while (true) {
        if (xQueueReceive(s_inference_queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_inference_worker_busy = true;
        vision_result_t vision = job.meta.vision;

        if (job.validation) {
            /*
             * 板端验证样例不依赖摄像头是否 Wake：图片已经嵌入在固件 flash 中，
             * 因此 Standby 下也能验证模型是否能被加载、推理和后处理。
             * 这个分支故意跳过下面的 stale 检查，但仍然复用同一个 inference_task，
             * 方便和实时图传推理保持一致的耗时、内存和后处理行为。
             */
            validation_context_t *ctx = job.validation_ctx;
            validation_cache_set_state(ctx, VALIDATION_JOB_RUNNING, ESP_OK);
            uint32_t source_w = 0;
            uint32_t source_h = 0;
            int64_t start_us = esp_timer_get_time();
            uint32_t box_min_score = job.box_min_score ? job.box_min_score : s_box_min_score;
            esp_err_t err = ESP_OK;
            if (job.method == RECOGNITION_METHOD_TINYCLS) {
                err = classify_validation_tinycls_jpeg(job.jpeg, job.jpeg_size, box_min_score,
                                                       &vision, &source_w, &source_h);
            } else if (job.method == RECOGNITION_METHOD_FISH31) {
                err = classify_validation_fish31_jpeg(job.jpeg, job.jpeg_size, box_min_score,
                                                      &vision, &source_w, &source_h);
            } else {
                err = classify_validation_yolo_jpeg(job.jpeg, job.jpeg_size, job.method,
                                                    box_min_score,
                                                    &vision, &source_w, &source_h);
            }
            vision.analysis_ms = (esp_timer_get_time() - start_us) / 1000;

            if (ctx) {
                ctx->err = err;
                ctx->vision = vision;
                ctx->source_w = source_w;
                ctx->source_h = source_h;
                ctx->completed_ms = esp_timer_get_time() / 1000;
                validation_cache_update(ctx);
                if (vision.object_count > 0) {
                    frame_meta_t validation_meta = {
                        .seq = ctx->id,
                        .size = job.jpeg_size,
                        .width = source_w,
                        .height = source_h,
                        .timestamp_ms = ctx->completed_ms,
                        .vision = vision,
                    };
                    history_maybe_queue(job.jpeg, job.jpeg_size, &validation_meta, job.method,
                                        "validation", true);
                }
                xSemaphoreGive(ctx->done);
            }

            update_inference_fps(esp_timer_get_time() / 1000);
            s_inference_jobs_completed++;
            s_validation_runs++;
            if (err != ESP_OK) {
                s_validation_errors++;
            }
            s_last_inference_ms = esp_timer_get_time() / 1000;
            free(job.jpeg);
            if (ctx) {
                __atomic_sub_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
                validation_context_release(ctx);
            }
            s_inference_worker_busy = false;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /*
         * 如果用户在排队期间切换到待机/关闭识别/换模型，直接丢弃旧任务。
         * 这能避免“刚切到 MLP，上一帧 YOLO 结果又回来覆盖网页”的错觉。
         */
        bool stale = s_power_state != POWER_STATE_RUNNING ||
                     !s_vision_enabled ||
                     s_recognition_method != job.method ||
                     !recognition_method_uses_jpeg_inference(job.method);

        if (!stale) {
            int64_t start_us = esp_timer_get_time();
            uint32_t source_w = 0;
            uint32_t source_h = 0;
            if (job.method == RECOGNITION_METHOD_TINYCLS) {
                classify_validation_tinycls_jpeg(job.jpeg, job.jpeg_size, s_box_min_score,
                                                 &vision, &source_w, &source_h);
            } else if (job.method == RECOGNITION_METHOD_FISH31) {
                classify_validation_fish31_jpeg(job.jpeg, job.jpeg_size, s_box_min_score,
                                                &vision, &source_w, &source_h);
            } else {
                classify_validation_yolo_jpeg(job.jpeg, job.jpeg_size, job.method,
                                              s_box_min_score, &vision, &source_w, &source_h);
            }
            vision.analysis_ms = (esp_timer_get_time() - start_us) / 1000;

            job.meta.vision = vision;
            update_latest_vision_from_inference(&vision, job.method);

            /* FIELD 录像中只发布最新推理结果；每个 raw 帧的 annotated 图像由 recording_task
             * 使用当前 raw 图像和最近一次推理标签生成，确保 raw/annotated 帧数一致。 */
            if (s_recording_enabled && s_app_mode == APP_MODE_FIELD && s_sd_mounted) {
                recording_item_t ann_update = {
                    .meta = job.meta,
                    .kind = RECORDING_KIND_ANNOTATED,
                    .method = job.method,
                };
                ann_update.meta.width = source_w ? source_w : ann_update.meta.width;
                ann_update.meta.height = source_h ? source_h : ann_update.meta.height;
                if (xQueueSend(s_recording_queue, &ann_update, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "recording vision update dropped: seq=%" PRIu32,
                             ann_update.meta.seq);
                }
            }

            history_maybe_queue(job.jpeg, job.jpeg_size, &job.meta, job.method, "camera",
                                vision.object_count > 0);
            update_inference_fps(esp_timer_get_time() / 1000);
            s_inference_jobs_completed++;
        }

        free(job.jpeg);
        s_inference_worker_busy = false;

        /*
         * 让出 CPU 给 HTTP、Wi-Fi 和摄像头任务。YOLO26 单帧很慢，完成后主动让步能让界面恢复更顺滑。
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void camera_release_device(int fd, bool streaming)
{
    if (fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (streaming && ioctl(fd, VIDIOC_STREAMOFF, &type) != 0) {
            ESP_LOGW(TAG, "camera stream off failed: errno=%d", errno);
        }
    }

    if (s_camera.pixel_format != V4L2_PIX_FMT_JPEG) {
        if (s_camera.jpeg_buf && s_camera.encoder) {
            example_encoder_free_output_buffer(s_camera.encoder, s_camera.jpeg_buf);
        }
        if (s_camera.encoder) {
            example_encoder_deinit(s_camera.encoder);
        }
    }

    uint32_t buffer_count = s_camera.buffer_count;
    if (buffer_count > CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER) {
        buffer_count = CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    }
    for (uint32_t i = 0; i < buffer_count; i++) {
        if (s_camera.buffer[i] && s_camera.buffer[i] != MAP_FAILED) {
            munmap(s_camera.buffer[i], s_camera.buffer_size);
        }
    }

    if (fd >= 0 && buffer_count > 0) {
        struct v4l2_requestbuffers release = {
            .count = 0,
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_REQBUFS, &release) != 0) {
            ESP_LOGW(TAG, "camera buffer release failed: errno=%d", errno);
        }
    }
    if (fd >= 0) {
        close(fd);
    }
    memset(&s_camera, 0, sizeof(s_camera));
    s_camera.fd = -1;
}

static void camera_close(void)
{
    if (s_camera.valid || s_camera.fd >= 0) {
        camera_release_device(s_camera.fd, s_camera.streaming);
    }

    /*
     * Keep esp-video, SCCB, XCLK and the sensor initialized across standby.
     * Repeated full deinit/init cycles are not reliable on this board and were
     * the cause of wake failures after serving a recording download.
     */
    s_prev_luma_valid = false;
    clear_latest_frame();
    set_camera_error("standby");
}

static void camera_reset_video_hw_after_open_failure(esp_err_t open_ret, int saved_errno)
{
    if (!s_video_hw_ready) {
        return;
    }
    /*
     * DMA allocation failure is local to this open attempt. Keep esp-video
     * registered so the next wake can retry after buffers are released.
     * Only reset the full stack when the V4L2 device itself reports an invalid
     * registration/state error.
     */
    if (open_ret == ESP_ERR_NO_MEM ||
        (saved_errno != EINVAL && saved_errno != ENODEV && saved_errno != ENOENT)) {
        return;
    }

    ESP_LOGW(TAG,
             "resetting esp-video after camera open failure ret=%s errno=%d",
             esp_err_to_name(open_ret), saved_errno);
    esp_err_t deinit_ret = example_video_deinit();
    if (deinit_ret == ESP_OK) {
        s_video_hw_ready = false;
        s_prev_luma_valid = false;
        clear_latest_frame();
    } else {
        ESP_LOGW(TAG, "esp-video deinit after camera failure returned %s",
                 esp_err_to_name(deinit_ret));
    }
}

static esp_err_t camera_open(void)
{
    esp_err_t ret = ESP_FAIL;
    int fd = -1;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format format = {0};
    struct v4l2_streamparm sparm = {0};
    struct v4l2_requestbuffers req = {0};

    if (s_camera.valid) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "camera open DMA before: free=%u largest=%u minimum=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));

    if (!s_video_hw_ready) {
        ret = example_video_init();
        if (ret != ESP_OK) {
            set_camera_error("video init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        s_video_hw_ready = true;
    }

    fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        int saved_errno = errno ? errno : ENOENT;
        set_camera_error("open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
                         saved_errno);
        camera_reset_video_hw_after_open_failure(ESP_ERR_NOT_FOUND, saved_errno);
        return ESP_ERR_NOT_FOUND;
    }

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_G_FMT, &format)), fail, TAG, "failed to get camera format");

#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X) {
#if CONFIG_ESP_VIDEO_ENABLE_SWAP_BYTE
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_S_FMT, &format)), fail, TAG, "failed to switch RGB565 byte order");
#else
        ESP_GOTO_ON_ERROR(ESP_ERR_NOT_SUPPORTED, fail, TAG, "RGB565X needs CONFIG_ESP_VIDEO_ENABLE_SWAP_BYTE");
#endif
    }
#endif

    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_PARM, &sparm) == 0 && sparm.parm.capture.timeperframe.numerator) {
        s_camera.frame_rate = sparm.parm.capture.timeperframe.denominator /
                              sparm.parm.capture.timeperframe.numerator;
    }

    s_camera.fd = fd;
    s_camera.width = format.fmt.pix.width;
    s_camera.height = format.fmt.pix.height;
    s_camera.pixel_format = format.fmt.pix.pixelformat;

    /* Reserve the scarce JPEG/DMA resources before allocating V4L2 frame buffers. */
    if (s_camera.pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_config_t encoder_config = {
            .width = s_camera.width,
            .height = s_camera.height,
            .pixel_format = s_camera.pixel_format,
            .quality = (uint8_t)s_jpeg_quality,
        };
        ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &s_camera.encoder), fail,
                          TAG, "failed to init JPEG encoder");
        ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(
                              s_camera.encoder, &s_camera.jpeg_buf, &s_camera.jpeg_buf_size),
                          fail, TAG, "failed to alloc JPEG buffer");
    }

    req.count = CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_REQBUFS, &req)), fail, TAG, "failed to request video buffers");
    ESP_GOTO_ON_FALSE(req.count > 0, ESP_ERR_NO_MEM, fail, TAG, "video driver returned no buffers");
    s_camera.buffer_count = req.count > CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER ?
                            CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER : req.count;

    for (uint32_t i = 0; i < s_camera.buffer_count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_QUERYBUF, &buf)), fail, TAG, "failed to query buffer");

        s_camera.buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(s_camera.buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail, TAG, "failed to mmap buffer");
        s_camera.buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_QBUF, &buf)), fail, TAG, "failed to queue buffer");
    }

    ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(&s_camera, (int)s_jpeg_quality),
                      fail, TAG, "failed to set JPEG quality");
    ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(fd, VIDIOC_STREAMON, &type)), fail, TAG, "failed to stream on");

    s_camera.valid = true;
    s_camera.streaming = true;
    s_prev_luma_valid = false;
    set_camera_error("ok");

    char fourcc[5];
    ESP_LOGI(TAG, "camera ready: %" PRIu32 "x%" PRIu32 " fmt=%s sensor_fps=%" PRIu32,
             s_camera.width, s_camera.height, fourcc_to_str(s_camera.pixel_format, fourcc), s_camera.frame_rate);
    ESP_LOGI(TAG, "camera open DMA after: free=%u largest=%u minimum=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    return ESP_OK;

fail:
    int saved_errno = errno;
    esp_err_t open_ret = ret == ESP_OK ? ESP_FAIL : ret;
    set_camera_error("camera open failed: %s (errno=%d)",
                     esp_err_to_name(open_ret), saved_errno);
    camera_release_device(fd, false);
    camera_reset_video_hw_after_open_failure(open_ret, saved_errno);
    return open_ret;
}

static esp_err_t camera_capture_once(void)
{
    esp_err_t ret = ESP_OK;
    bool dequeued = false;
    struct v4l2_buffer buf = {0};
    uint8_t *jpeg_ptr = NULL;
    uint32_t jpeg_size = 0;
    vision_result_t vision = {0};
    int64_t encode_ms = 0;

    /*
     * 单帧处理顺序：
     * 1. 从 V4L2 摄像头队列取一帧；
     * 2. 根据 inference_interval_ms 和 inference_task 忙闲决定是否抽帧识别；
     * 3. MLP 很轻，仍在摄像头任务里同步跑；YOLO 只把 JPEG 副本投递给 inference_task；
     * 4. 立即更新 latest frame，HTTP 推流线程只复制最新 JPEG，不等待 YOLO 模型返回；
     * 5. YOLO 结果稍后由 inference_task 写回 latest meta，并按历史记录间隔入队保存。
     */
    int64_t start_us = esp_timer_get_time();
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl_to_esp(ioctl(s_camera.fd, VIDIOC_DQBUF, &buf)), done, TAG, "failed to dequeue frame");
    dequeued = true;

    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto done;
    }

    uint8_t *raw_ptr = s_camera.buffer[buf.index];
    uint32_t raw_size = buf.bytesused ? buf.bytesused : s_camera.buffer_size;
    bool vision_enabled = s_vision_enabled;
    recognition_method_t method = s_recognition_method;
    bool method_enabled = vision_enabled && method != RECOGNITION_METHOD_OFF;
    bool method_is_yolo = recognition_method_is_yolo(method);
    bool method_uses_jpeg_inference = method_is_yolo ||
                                      ((method == RECOGNITION_METHOD_TINYCLS ||
                                        method == RECOGNITION_METHOD_FISH31) &&
                                       s_camera.pixel_format == V4L2_PIX_FMT_JPEG);
    int64_t now_for_interval_ms = esp_timer_get_time() / 1000;
    bool yolo_busy = method_enabled && method_uses_jpeg_inference && inference_worker_busy();
    bool run_recognition = false;
    if (method_enabled) {
        if (yolo_busy) {
            if (recognition_interval_elapsed(now_for_interval_ms)) {
                s_inference_dropped_frames++;
            }
        } else {
            run_recognition = recognition_due(now_for_interval_ms);
        }
    }
    bool need_inference = run_recognition && method_uses_jpeg_inference;
    if (method_enabled && !run_recognition) {
        frame_meta_t previous = {0};
        if (method_uses_jpeg_inference && copy_completed_yolo_vision(&vision, method)) {
            /* 复用上一次完成的 YOLO 结果，让视频流在下一次慢速推理结束前仍有稳定标注。 */
        } else if (copy_latest_meta(&previous)) {
            vision = previous.vision;
        } else if (method_is_yolo) {
            fill_yolo_pending(&vision, method);
        } else if (method == RECOGNITION_METHOD_TINYCLS) {
            fill_tinycls_pending(&vision);
        } else if (method == RECOGNITION_METHOD_FISH31) {
            fill_fish31_pending(&vision);
        } else {
            fill_vision_disabled(&vision);
        }
    }

    if (s_camera.pixel_format == V4L2_PIX_FMT_JPEG) {
        jpeg_ptr = raw_ptr;
        jpeg_size = buf.bytesused;
        if (need_inference) {
            strlcpy(vision.scene, "jpeg", sizeof(vision.scene));
            strlcpy(vision.color, "unknown", sizeof(vision.color));
            if (method == RECOGNITION_METHOD_TINYCLS) {
                fill_tinycls_pending(&vision);
            } else if (method == RECOGNITION_METHOD_FISH31) {
                fill_fish31_pending(&vision);
            } else {
                fill_yolo_pending(&vision, method);
            }
        } else if (run_recognition) {
            strlcpy(vision.scene, "jpeg", sizeof(vision.scene));
            strlcpy(vision.color, "unknown", sizeof(vision.color));
            strlcpy(vision.label, "jpeg-only", sizeof(vision.label));
            strlcpy(vision.object, "unknown", sizeof(vision.object));
            strlcpy(vision.model, "jpeg-only", sizeof(vision.model));
            vision.box_min_score = s_box_min_score;
        } else {
            fill_vision_disabled(&vision);
        }
    } else {
        if (run_recognition) {
            analyze_frame(raw_ptr, raw_size, s_camera.width, s_camera.height, s_camera.pixel_format, &vision);
            if (method == RECOGNITION_METHOD_MLP ||
                method == RECOGNITION_METHOD_TINYCLS ||
                method == RECOGNITION_METHOD_FISH31) {
                update_inference_fps(esp_timer_get_time() / 1000);
            }
        } else if (!method_enabled) {
            fill_vision_disabled(&vision);
            s_prev_luma_valid = false;
        }
        int64_t encode_start_us = esp_timer_get_time();
        ESP_GOTO_ON_ERROR(example_encoder_process(s_camera.encoder, raw_ptr, s_camera.buffer_size,
                          s_camera.jpeg_buf, s_camera.jpeg_buf_size, &jpeg_size), done, TAG, "failed to encode JPEG");
        encode_ms = (esp_timer_get_time() - encode_start_us) / 1000;
        jpeg_ptr = s_camera.jpeg_buf;
    }

    if (need_inference && jpeg_ptr && jpeg_size > 0) {
        frame_meta_t previous = {0};
        if (copy_completed_yolo_vision(&vision, method)) {
            /* 新任务已经投递时，画面继续显示上一轮真实 YOLO 结果，避免 waiting 闪烁。 */
        } else if (copy_latest_meta(&previous) && is_completed_inference_result(&previous.vision, method)) {
            vision = previous.vision;
        }
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    frame_meta_t meta = {
        .seq = s_latest_meta.seq + 1,
        .size = jpeg_size,
        .width = s_camera.width,
        .height = s_camera.height,
        .pixel_format = s_camera.pixel_format,
        .sensor_fps = s_camera.frame_rate,
        .timestamp_ms = now_ms,
        .capture_ms = (esp_timer_get_time() - start_us) / 1000,
        .encode_ms = encode_ms,
        .vision = vision,
    };

    ret = publish_frame(jpeg_ptr, jpeg_size, &meta);
    if (ret == ESP_OK) {
        bool recording_queued = recording_maybe_queue(jpeg_ptr, jpeg_size, &meta);
        if (need_inference && (s_app_mode != APP_MODE_FIELD || recording_queued)) {
            queue_inference_job(jpeg_ptr, jpeg_size, &meta, method);
        } else if (run_recognition) {
            history_maybe_queue(jpeg_ptr, jpeg_size, &meta, method, "camera",
                                meta.vision.object_count > 0);
        }
        s_frames_total++;
        update_capture_fps(now_ms);
    }

done:
    if (dequeued) {
        ioctl(s_camera.fd, VIDIOC_QBUF, &buf);
    }
    if (ret != ESP_OK) {
        s_capture_errors++;
    }
    return ret;
}

static void camera_task(void *arg)
{
    bool desired_running = s_app_mode == APP_MODE_FIELD;
    camera_cmd_t cmd;
    uint32_t open_retry_ms = 250;
    uint32_t consecutive_capture_failures = 0;

    set_power_state(desired_running ? POWER_STATE_STARTING : POWER_STATE_STANDBY);
    if (!desired_running) {
        set_camera_error("standby");
    }

    while (true) {
        while (xQueueReceive(s_camera_cmd_queue, &cmd, 0) == pdTRUE) {
            desired_running = (cmd == CAMERA_CMD_WAKE);
            set_power_state(desired_running ? POWER_STATE_STARTING : POWER_STATE_STOPPING);
        }

        if (!desired_running) {
            if (s_camera.valid || s_video_hw_ready) {
                camera_close();
            }
            set_power_state(POWER_STATE_STANDBY);
            open_retry_ms = 250;
            consecutive_capture_failures = 0;

            if (xQueueReceive(s_camera_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
                desired_running = (cmd == CAMERA_CMD_WAKE);
                set_power_state(desired_running ? POWER_STATE_STARTING : POWER_STATE_STANDBY);
            }
            continue;
        }

        if (!s_camera.valid) {
            set_power_state(POWER_STATE_STARTING);
            esp_err_t ret = camera_open();
            if (ret != ESP_OK) {
                set_power_state(POWER_STATE_ERROR);
                ESP_LOGE(TAG, "camera start failed: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(open_retry_ms));
                if (open_retry_ms < 5000) {
                    open_retry_ms *= 2;
                    if (open_retry_ms > 5000) {
                        open_retry_ms = 5000;
                    }
                }
                continue;
            }
            open_retry_ms = 250;
            consecutive_capture_failures = 0;
            set_power_state(POWER_STATE_RUNNING);
        }

        esp_err_t ret = camera_capture_once();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "capture failed: %s", esp_err_to_name(ret));
            consecutive_capture_failures++;
            if (consecutive_capture_failures >= 8) {
                ESP_LOGW(TAG, "reopening camera after %" PRIu32 " consecutive capture failures",
                         consecutive_capture_failures);
                camera_release_device(s_camera.fd, s_camera.streaming);
                set_power_state(POWER_STATE_STARTING);
                consecutive_capture_failures = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            consecutive_capture_failures = 0;
        }
    }
}

static uint8_t *alloc_psram_buffer(uint32_t size)
{
    uint8_t *buf = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t *)malloc(size);
    }
    return buf;
}

static int wifi_rssi(void)
{
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;
}

static void sample_memory_stats(uint32_t *free_heap, uint32_t *min_free_heap,
                                uint32_t *free_psram, uint32_t *min_free_psram)
{
    uint32_t heap_now = (uint32_t)esp_get_free_heap_size();
    uint32_t psram_now = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (s_min_free_heap == 0 || heap_now < s_min_free_heap) {
        s_min_free_heap = heap_now;
    }
    if (s_min_free_psram == 0 || psram_now < s_min_free_psram) {
        s_min_free_psram = psram_now;
    }

    if (free_heap) {
        *free_heap = heap_now;
    }
    if (min_free_heap) {
        *min_free_heap = s_min_free_heap;
    }
    if (free_psram) {
        *free_psram = psram_now;
    }
    if (min_free_psram) {
        *min_free_psram = s_min_free_psram;
    }
}

static void save_setting_u8(const char *key, uint8_t value)
{
    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, key, value);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static uint32_t clamp_u32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t format_version;
    uint16_t blob_size;
    uint32_t generation;
    uint8_t recognition_method;
    uint8_t network_mode;
    uint8_t flags;
    uint8_t reserved;
    uint32_t box_min_score;
    uint32_t stream_max_fps;
    uint32_t inference_interval_ms;
    uint32_t history_sample_interval_ms;
    uint32_t jpeg_quality;
    uint32_t recording_segment_ms;
    uint32_t field_idle_timeout_ms;
    char router_ssid[33];
    char router_password[65];
    uint8_t reserved_tail[2];
    uint32_t crc32;
} runtime_config_blob_t;

_Static_assert(sizeof(runtime_config_blob_t) == 148,
               "runtime config NVS blob layout changed");

static const char *runtime_config_slot_key(uint8_t slot)
{
    return slot == 0 ? RUNTIME_CONFIG_SLOT0_KEY : RUNTIME_CONFIG_SLOT1_KEY;
}

static uint32_t runtime_config_crc32(const void *data, size_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static bool runtime_config_blob_valid(const runtime_config_blob_t *blob)
{
    if (!blob || blob->magic != RUNTIME_CONFIG_BLOB_MAGIC ||
        blob->format_version != RUNTIME_CONFIG_BLOB_VERSION ||
        blob->blob_size != sizeof(*blob) || blob->generation == 0 ||
        blob->recognition_method > RECOGNITION_METHOD_FISH31 ||
        blob->network_mode > NETWORK_MODE_APSTA ||
        (blob->flags & ~RUNTIME_CONFIG_FLAGS_MASK) != 0 ||
        blob->reserved != 0 || blob->reserved_tail[0] != 0 ||
        blob->reserved_tail[1] != 0 ||
        blob->box_min_score < 50 || blob->box_min_score > 100 ||
        blob->stream_max_fps < 1 || blob->stream_max_fps > 30 ||
        blob->inference_interval_ms > 600000 ||
        blob->history_sample_interval_ms < 250 ||
        blob->history_sample_interval_ms > 600000 ||
        blob->jpeg_quality < 1 || blob->jpeg_quality > 100 ||
        blob->recording_segment_ms < APP_RECORDING_SEGMENT_MIN_MS ||
        blob->recording_segment_ms > APP_RECORDING_SEGMENT_MAX_MS ||
        blob->field_idle_timeout_ms < APP_FIELD_IDLE_TIMEOUT_MIN_MS ||
        blob->field_idle_timeout_ms > APP_FIELD_IDLE_TIMEOUT_MAX_MS ||
        !memchr(blob->router_ssid, '\0', sizeof(blob->router_ssid)) ||
        !memchr(blob->router_password, '\0', sizeof(blob->router_password))) {
        return false;
    }
    return blob->crc32 ==
           runtime_config_crc32(blob, offsetof(runtime_config_blob_t, crc32));
}

static esp_err_t runtime_config_read_slot(nvs_handle_t nvs, uint8_t slot,
                                          runtime_config_blob_t *blob)
{
    if (slot > 1 || !blob) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t size = sizeof(*blob);
    esp_err_t ret = nvs_get_blob(nvs, runtime_config_slot_key(slot), blob, &size);
    if (ret != ESP_OK) {
        return ret;
    }
    if (size != sizeof(*blob) || !runtime_config_blob_valid(blob)) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static bool runtime_config_generation_newer(uint32_t lhs, uint32_t rhs)
{
    return (int32_t)(lhs - rhs) > 0;
}

static esp_err_t runtime_config_select_blob(nvs_handle_t nvs,
                                            runtime_config_blob_t *selected,
                                            uint8_t *selected_slot,
                                            bool *active_marker_needs_repair)
{
    runtime_config_blob_t slots[2];
    bool valid[2] = {
        runtime_config_read_slot(nvs, 0, &slots[0]) == ESP_OK,
        runtime_config_read_slot(nvs, 1, &slots[1]) == ESP_OK,
    };
    uint8_t active = UINT8_MAX;
    if (nvs_get_u8(nvs, RUNTIME_CONFIG_ACTIVE_KEY, &active) != ESP_OK ||
        active > 1) {
        /* A valid slot is not committed until its active marker is durable. */
        return ESP_ERR_NOT_FOUND;
    }
    int chosen;
    if (valid[active]) {
        chosen = active;
    } else if (valid[active ^ 1U]) {
        chosen = active ^ 1U;
    } else {
        return ESP_ERR_NOT_FOUND;
    }
    *selected = slots[chosen];
    *selected_slot = (uint8_t)chosen;
    if (active_marker_needs_repair) {
        *active_marker_needs_repair = active != chosen;
    }
    return ESP_OK;
}

static void runtime_config_blob_from_globals(runtime_config_blob_t *blob)
{
    memset(blob, 0, sizeof(*blob));
    blob->magic = RUNTIME_CONFIG_BLOB_MAGIC;
    blob->format_version = RUNTIME_CONFIG_BLOB_VERSION;
    blob->blob_size = sizeof(*blob);
    blob->recognition_method = (uint8_t)s_recognition_method;
    blob->network_mode = (uint8_t)s_network_mode;
    blob->flags = (s_vision_enabled ? RUNTIME_CONFIG_FLAG_VISION : 0) |
                  (s_history_enabled ? RUNTIME_CONFIG_FLAG_HISTORY : 0) |
                  (s_recording_enabled ? RUNTIME_CONFIG_FLAG_RECORDING : 0) |
                  (s_field_auto_enable ? RUNTIME_CONFIG_FLAG_FIELD_AUTO : 0);
    blob->box_min_score = s_box_min_score;
    blob->stream_max_fps = s_stream_max_fps;
    blob->inference_interval_ms = s_inference_interval_ms;
    blob->history_sample_interval_ms = s_history_sample_interval_ms;
    blob->jpeg_quality = s_jpeg_quality;
    blob->recording_segment_ms = s_recording_segment_ms;
    blob->field_idle_timeout_ms = s_field_idle_timeout_ms;
    strlcpy(blob->router_ssid, s_router_ssid, sizeof(blob->router_ssid));
    strlcpy(blob->router_password, s_router_password,
            sizeof(blob->router_password));
}

static void runtime_config_apply_blob(const runtime_config_blob_t *blob)
{
    s_recognition_method = recognition_method_or_fallback(
        (recognition_method_t)blob->recognition_method);
    s_network_mode = (network_mode_t)blob->network_mode;
    s_vision_enabled = (blob->flags & RUNTIME_CONFIG_FLAG_VISION) != 0;
    s_history_enabled = (blob->flags & RUNTIME_CONFIG_FLAG_HISTORY) != 0;
    s_recording_enabled = (blob->flags & RUNTIME_CONFIG_FLAG_RECORDING) != 0;
    s_field_auto_enable = (blob->flags & RUNTIME_CONFIG_FLAG_FIELD_AUTO) != 0;
    s_box_min_score = blob->box_min_score;
    s_stream_max_fps = blob->stream_max_fps;
    s_inference_interval_ms = blob->inference_interval_ms;
    s_history_sample_interval_ms = blob->history_sample_interval_ms;
    s_jpeg_quality = blob->jpeg_quality;
    s_recording_segment_ms = blob->recording_segment_ms;
    s_field_idle_timeout_ms = blob->field_idle_timeout_ms;
    strlcpy(s_router_ssid, blob->router_ssid, sizeof(s_router_ssid));
    strlcpy(s_router_password, blob->router_password,
            sizeof(s_router_password));
}

static esp_err_t runtime_config_commit_blob(nvs_handle_t nvs,
                                            runtime_config_blob_t *blob)
{
    runtime_config_blob_t slots[2];
    bool valid[2] = {
        runtime_config_read_slot(nvs, 0, &slots[0]) == ESP_OK,
        runtime_config_read_slot(nvs, 1, &slots[1]) == ESP_OK,
    };
    uint8_t active = UINT8_MAX;
    bool active_valid = nvs_get_u8(nvs, RUNTIME_CONFIG_ACTIVE_KEY, &active) == ESP_OK &&
                        active <= 1 && valid[active];
    int current = active_valid ? active : -1;
    if (current < 0 && valid[0] && valid[1]) {
        current = runtime_config_generation_newer(slots[1].generation,
                                                  slots[0].generation) ? 1 : 0;
    } else if (current < 0 && valid[0]) {
        current = 0;
    } else if (current < 0 && valid[1]) {
        current = 1;
    }

    uint32_t newest_generation = 0;
    if (valid[0]) {
        newest_generation = slots[0].generation;
    }
    if (valid[1] && (!newest_generation ||
                     runtime_config_generation_newer(slots[1].generation,
                                                     newest_generation))) {
        newest_generation = slots[1].generation;
    }
    blob->generation = newest_generation + 1;
    if (blob->generation == 0) {
        blob->generation = 1;
    }
    blob->crc32 = runtime_config_crc32(blob,
                                       offsetof(runtime_config_blob_t, crc32));
    if (!runtime_config_blob_valid(blob)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t target = current < 0 ? 0 : ((uint8_t)current ^ 1U);
    esp_err_t ret = nvs_set_blob(nvs, runtime_config_slot_key(target), blob,
                                 sizeof(*blob));
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_commit(nvs);
    if (ret != ESP_OK) {
        return ret;
    }

    runtime_config_blob_t verified;
    ret = runtime_config_read_slot(nvs, target, &verified);
    if (ret != ESP_OK || memcmp(&verified, blob, sizeof(verified)) != 0) {
        return ret == ESP_OK ? ESP_ERR_INVALID_STATE : ret;
    }

    ret = nvs_set_u8(nvs, RUNTIME_CONFIG_ACTIVE_KEY, target);
    if (ret != ESP_OK) {
        return ret;
    }
    return nvs_commit(nvs);
}

static void json_escape_string(char *out, size_t out_size, const char *in)
{
    if (!out || out_size == 0) {
        return;
    }
    size_t off = 0;
    if (!in) {
        out[0] = '\0';
        return;
    }
    for (const unsigned char *p = (const unsigned char *)in;
         *p && off + 1 < out_size; p++) {
        if ((*p == '"' || *p == '\\') && off + 2 < out_size) {
            out[off++] = '\\';
            out[off++] = (char)*p;
        } else if (*p >= 0x20) {
            out[off++] = (char)*p;
        }
    }
    out[off] = '\0';
}

static uint32_t network_access_window_ms(void)
{
    uint32_t configured = s_field_idle_timeout_ms;
    uint32_t boot_window = CONFIG_APP_NETWORK_BOOT_WINDOW_MS;
    if (configured < APP_NETWORK_ACCESS_GRACE_MIN_MS) {
        configured = APP_NETWORK_ACCESS_GRACE_MIN_MS;
    }
    return boot_window > configured ? configured : boot_window;
}

static void load_runtime_settings(void)
{
    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }

    runtime_config_blob_t blob;
    uint8_t selected_slot = 0;
    bool repair_active_marker = false;
    if (runtime_config_select_blob(nvs, &blob, &selected_slot,
                                   &repair_active_marker) == ESP_OK) {
        runtime_config_apply_blob(&blob);
        if (repair_active_marker) {
            esp_err_t repair_ret = nvs_set_u8(nvs, RUNTIME_CONFIG_ACTIVE_KEY,
                                              selected_slot);
            if (repair_ret == ESP_OK) {
                repair_ret = nvs_commit(nvs);
            }
            if (repair_ret != ESP_OK) {
                ESP_LOGW(TAG, "could not repair runtime config active slot: %s",
                         esp_err_to_name(repair_ret));
            }
        }
        nvs_close(nvs);
        return;
    }

    uint32_t version = 0;
    if (nvs_get_u32(nvs, "version", &version) != ESP_OK || version != SETTINGS_VERSION) {
        /*
         * NVS 会保留现场调参结果。升级到本版本后需要重置一次运行参数：
         * - 网络恢复 AP+STA，保证手机固定访问 192.168.4.1；
         * - 新 TF 卡到位后恢复历史记录和分段录像默认开启；
         * - 主识别路线恢复 COCO，避免旧实验模型继续占用演示入口。
         */
        s_network_mode = CONFIG_APP_DEFAULT_NETWORK_MODE == 2 ? NETWORK_MODE_APSTA :
                         (CONFIG_APP_DEFAULT_NETWORK_MODE == 1 ? NETWORK_MODE_SOFTAP :
                          NETWORK_MODE_STA);
        s_recognition_method = recognition_method_or_fallback(configured_default_recognition_method());
        s_vision_enabled = s_recognition_method != RECOGNITION_METHOD_OFF;
        s_history_enabled = CONFIG_APP_HISTORY_ENABLE;
        s_recording_enabled = CONFIG_APP_RECORDING_ENABLE;
        s_box_min_score = CONFIG_APP_CAN_BOX_MIN_SCORE;
        s_stream_max_fps = CONFIG_APP_STREAM_MAX_FPS;
        s_inference_interval_ms = CONFIG_APP_INFERENCE_INTERVAL_MS;
        s_history_sample_interval_ms = CONFIG_APP_HISTORY_SAMPLE_INTERVAL_MS;
        s_jpeg_quality = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
        s_recording_segment_ms = clamp_u32(CONFIG_APP_RECORDING_SEGMENT_MS,
                                           APP_RECORDING_SEGMENT_MIN_MS,
                                           APP_RECORDING_SEGMENT_MAX_MS);
        s_field_idle_timeout_ms = clamp_u32(CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS,
                                            APP_FIELD_IDLE_TIMEOUT_MIN_MS,
                                            APP_FIELD_IDLE_TIMEOUT_MAX_MS);
        s_field_auto_enable = true;
        strlcpy(s_router_ssid, WIFI_SSID, sizeof(s_router_ssid));
        strlcpy(s_router_password, WIFI_PASSWORD, sizeof(s_router_password));
        runtime_config_blob_from_globals(&blob);
        esp_err_t migrate_ret = runtime_config_commit_blob(nvs, &blob);
        if (migrate_ret != ESP_OK) {
            ESP_LOGW(TAG, "could not persist default runtime config: %s",
                     esp_err_to_name(migrate_ret));
        }
        nvs_close(nvs);
        return;
    }

    uint8_t method = (uint8_t)s_recognition_method;
    if (nvs_get_u8(nvs, "method", &method) == ESP_OK && method <= RECOGNITION_METHOD_FISH31) {
        s_recognition_method = recognition_method_or_fallback((recognition_method_t)method);
        s_vision_enabled = method != RECOGNITION_METHOD_OFF;
    }

    uint8_t vision = s_vision_enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "vision", &vision) == ESP_OK) {
        s_vision_enabled = vision != 0;
    }

    uint8_t history = s_history_enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "history", &history) == ESP_OK) {
        s_history_enabled = history != 0;
    }

    uint8_t recording = s_recording_enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "recording", &recording) == ESP_OK) {
        s_recording_enabled = recording != 0;
    }

    uint8_t netmode = (uint8_t)s_network_mode;
    if (nvs_get_u8(nvs, "netmode", &netmode) == ESP_OK && netmode <= NETWORK_MODE_APSTA) {
        s_network_mode = (network_mode_t)netmode;
    }

    uint32_t value = 0;
    if (nvs_get_u32(nvs, "box_min", &value) == ESP_OK) {
        s_box_min_score = clamp_u32(value, 50, 100);
    }
    if (nvs_get_u32(nvs, "stream_fps", &value) == ESP_OK) {
        s_stream_max_fps = clamp_u32(value, 1, 30);
    }
    if (nvs_get_u32(nvs, "inf_ms", &value) == ESP_OK) {
        s_inference_interval_ms = clamp_u32(value, 0, 600000);
    }
    if (nvs_get_u32(nvs, "hist_ms", &value) == ESP_OK) {
        s_history_sample_interval_ms = clamp_u32(value, 250, 600000);
    }
    if (nvs_get_u32(nvs, "jpeg_q", &value) == ESP_OK) {
        s_jpeg_quality = clamp_u32(value, 1, 100);
    }
    if (nvs_get_u32(nvs, "seg_ms", &value) == ESP_OK) {
        s_recording_segment_ms = clamp_u32(value, APP_RECORDING_SEGMENT_MIN_MS,
                                           APP_RECORDING_SEGMENT_MAX_MS);
    } else {
        s_recording_segment_ms = clamp_u32(CONFIG_APP_RECORDING_SEGMENT_MS,
                                           APP_RECORDING_SEGMENT_MIN_MS,
                                           APP_RECORDING_SEGMENT_MAX_MS);
    }
    if (nvs_get_u32(nvs, "idle_ms", &value) == ESP_OK) {
        s_field_idle_timeout_ms = clamp_u32(value, APP_FIELD_IDLE_TIMEOUT_MIN_MS,
                                            APP_FIELD_IDLE_TIMEOUT_MAX_MS);
    } else {
        s_field_idle_timeout_ms = clamp_u32(CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS,
                                            APP_FIELD_IDLE_TIMEOUT_MIN_MS,
                                            APP_FIELD_IDLE_TIMEOUT_MAX_MS);
    }
    uint8_t field_auto = s_field_auto_enable ? 1 : 0;
    if (nvs_get_u8(nvs, "field_auto", &field_auto) == ESP_OK) {
        s_field_auto_enable = field_auto != 0;
    }
    size_t str_len = sizeof(s_router_ssid);
    if (nvs_get_str(nvs, "router_ssid", s_router_ssid, &str_len) != ESP_OK) {
        strlcpy(s_router_ssid, WIFI_SSID, sizeof(s_router_ssid));
    }
    str_len = sizeof(s_router_password);
    if (nvs_get_str(nvs, "router_pass", s_router_password, &str_len) != ESP_OK) {
        strlcpy(s_router_password, WIFI_PASSWORD, sizeof(s_router_password));
    }
    runtime_config_blob_from_globals(&blob);
    esp_err_t migrate_ret = runtime_config_commit_blob(nvs, &blob);
    if (migrate_ret != ESP_OK) {
        ESP_LOGW(TAG, "could not migrate legacy runtime config: %s",
                 esp_err_to_name(migrate_ret));
    }
    nvs_close(nvs);
}

static void mark_network_activity(void)
{
    __atomic_store_n(&s_last_network_activity_ms, esp_timer_get_time() / 1000,
                     __ATOMIC_RELEASE);
}

static bool http_remote_addr(httpd_req_t *req, char *addr, size_t addr_size)
{
    if (!req || !addr || addr_size == 0) {
        return false;
    }
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd < 0) {
        return false;
    }

    struct sockaddr_storage peer = {0};
    socklen_t peer_len = sizeof(peer);
    if (getpeername(sockfd, (struct sockaddr *)&peer, &peer_len) != 0) {
        return false;
    }
    if (peer.ss_family == AF_INET) {
        const struct sockaddr_in *in = (const struct sockaddr_in *)&peer;
        return inet_ntop(AF_INET, &in->sin_addr, addr, addr_size) != NULL;
    }
#if LWIP_IPV6
    if (peer.ss_family == AF_INET6) {
        const struct sockaddr_in6 *in6 = (const struct sockaddr_in6 *)&peer;
        return inet_ntop(AF_INET6, &in6->sin6_addr, addr, addr_size) != NULL;
    }
#endif
    return false;
}

static uint32_t web_client_count_locked(int64_t now_ms)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < APP_WEB_CLIENT_SLOTS; i++) {
        if (s_web_clients[i].last_seen_ms > 0 &&
            now_ms - s_web_clients[i].last_seen_ms <= APP_WEB_CLIENT_TIMEOUT_MS) {
            count++;
        }
    }
    s_web_client_count = count;
    return count;
}

static uint32_t web_client_count(int64_t now_ms)
{
    uint32_t count;
    taskENTER_CRITICAL(&s_web_client_mux);
    count = web_client_count_locked(now_ms);
    taskEXIT_CRITICAL(&s_web_client_mux);
    return count;
}

static void clear_web_clients(void)
{
    taskENTER_CRITICAL(&s_web_client_mux);
    memset(s_web_clients, 0, sizeof(s_web_clients));
    s_web_client_count = 0;
    taskEXIT_CRITICAL(&s_web_client_mux);
}

static void record_http_client(httpd_req_t *req)
{
    char addr[48] = {0};
    if (!http_remote_addr(req, addr, sizeof(addr))) {
        strlcpy(addr, "unknown", sizeof(addr));
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    __atomic_store_n(&s_last_web_client_activity_ms, now_ms, __ATOMIC_RELEASE);

    taskENTER_CRITICAL(&s_web_client_mux);
    int exact = -1;
    int free_or_expired = -1;
    int oldest = -1;
    int64_t oldest_ms = INT64_MAX;
    for (uint32_t i = 0; i < APP_WEB_CLIENT_SLOTS; i++) {
        if (s_web_clients[i].last_seen_ms <= 0 ||
            now_ms - s_web_clients[i].last_seen_ms > APP_WEB_CLIENT_TIMEOUT_MS) {
            if (free_or_expired < 0) {
                free_or_expired = (int)i;
            }
            continue;
        }
        if (strcmp(s_web_clients[i].addr, addr) == 0) {
            exact = (int)i;
            break;
        }
        if (s_web_clients[i].last_seen_ms < oldest_ms) {
            oldest_ms = s_web_clients[i].last_seen_ms;
            oldest = (int)i;
        }
    }
    int replace = exact >= 0 ? exact :
                  (free_or_expired >= 0 ? free_or_expired : oldest);
    if (replace < 0) {
        replace = 0;
    }
    strlcpy(s_web_clients[replace].addr, addr, sizeof(s_web_clients[replace].addr));
    s_web_clients[replace].last_seen_ms = now_ms;
    web_client_count_locked(now_ms);
    taskEXIT_CRITICAL(&s_web_client_mux);
}

static void open_network_access_window(const char *reason)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    uint32_t window_ms = network_access_window_ms();
    __atomic_store_n(&s_last_network_activity_ms, now_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_last_web_client_activity_ms, now_ms, __ATOMIC_RELEASE);
    s_network_boot_window_until_ms = now_ms + window_ms;
    ESP_LOGI(TAG, "network access window opened for %" PRIu32 " ms%s%s",
             window_ms,
             reason && reason[0] ? ": " : "",
             reason && reason[0] ? reason : "");
}

static void record_http_request(httpd_req_t *req)
{
    s_requests++;
    record_http_client(req);
    mark_network_activity();
}

static void record_http_poll(httpd_req_t *req)
{
    s_requests++;
    record_http_client(req);
}

static void resegment_status_copy(recording_resegment_status_t *out)
{
    if (!out) {
        return;
    }
    taskENTER_CRITICAL(&s_resegment_mux);
    *out = s_resegment_status;
    taskEXIT_CRITICAL(&s_resegment_mux);
}

static void resegment_status_update(bool running, bool requested, bool cancelled,
                                    uint32_t input_segments, uint32_t processed_segments,
                                    uint32_t output_segments, const char *message)
{
    taskENTER_CRITICAL(&s_resegment_mux);
    s_resegment_status.running = running;
    s_resegment_status.requested = requested;
    s_resegment_status.cancelled = cancelled;
    s_resegment_status.input_segments = input_segments;
    s_resegment_status.processed_segments = processed_segments;
    s_resegment_status.output_segments = output_segments;
    if (message) {
        strlcpy(s_resegment_status.last_error, message, sizeof(s_resegment_status.last_error));
    }
    taskEXIT_CRITICAL(&s_resegment_mux);
}

static void log_acceleration_status(void)
{
    int cpu_mhz = 0;
    int l2_cache_kb = 0;
    int l2_line_bytes = 0;
#ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
    cpu_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
#endif
#ifdef CONFIG_CACHE_L2_CACHE_SIZE
    l2_cache_kb = CONFIG_CACHE_L2_CACHE_SIZE / 1024;
#endif
#ifdef CONFIG_CACHE_L2_CACHE_LINE_SIZE
    l2_line_bytes = CONFIG_CACHE_L2_CACHE_LINE_SIZE;
#endif

#ifdef CONFIG_IDF_TARGET_ESP32P4
    const char *espdl_kernel = "ESP32-P4 INT8 ISA kernels";
    const char *detect_runtime = "RUNTIME_MODE_MULTI_CORE";
#else
    const char *espdl_kernel = "generic ESP-DL kernels";
    const char *detect_runtime = "single-core runtime";
#endif

#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    const char *jpeg_encode = "hardware JPEG encoder";
#else
    const char *jpeg_encode = "software JPEG encoder";
#endif

#if CONFIG_SOC_JPEG_DECODE_SUPPORTED
    const char *jpeg_decode_hw = "available";
#else
    const char *jpeg_decode_hw = "not available";
#endif

    ESP_LOGI(TAG,
             "Acceleration: ESP-DL=%s, detect_runtime=%s, CPU=%dMHz, L2=%dKB, line=%dB",
             espdl_kernel, detect_runtime, cpu_mhz, l2_cache_kb, l2_line_bytes);
    ESP_LOGI(TAG,
             "Acceleration: camera JPEG encode=%s, SOC JPEG decode=%s, COCO validation decode=ESP-DL sw_decode_jpeg",
             jpeg_encode, jpeg_decode_hw);
}

static const char s_customer_index_html_v2[] __attribute__((unused)) =
"<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>P4 Buoy Vision</title><style>"
":root{color-scheme:light;--bg:#f6f7f9;--panel:#fff;--line:#d7dde5;--text:#17202a;--muted:#667085;--ok:#147a44;--bad:#b42318;--accent:#146c94;--warn:#a15c07}"
"*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif}"
"main{width:min(1180px,100%);margin:0 auto;padding:16px;display:grid;gap:14px}.top{display:flex;justify-content:space-between;gap:12px;align-items:center;flex-wrap:wrap}"
".title{font-size:24px;font-weight:760}.small{font-size:18px}.sub,.hint,.muted{color:var(--muted);font-size:13px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}"
".panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}.rows{display:grid;gap:8px;margin-top:10px}.row{display:grid;grid-template-columns:126px minmax(0,1fr);gap:10px;border-top:1px solid var(--line);padding-top:8px;align-items:center}"
".row:first-child{border-top:0;padding-top:0}.label{font-weight:700;color:#344054}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}"
".actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:10px}button,select,input,a.btn{border:1px solid var(--line);border-radius:7px;background:#fff;color:var(--text);padding:9px 11px;font:inherit;font-weight:650;text-decoration:none}"
"button,a.btn{cursor:pointer;background:#edf6f9}button:hover,a.btn:hover,select:hover,input:hover{border-color:var(--accent)}button:disabled{opacity:.45;cursor:not-allowed}input,select{width:100%}"
"label{display:grid;gap:5px;font-size:13px;color:#344054}.form{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-top:10px}.toggle{display:flex;gap:8px;align-items:center}.toggle input{width:auto}"
".preview{display:none;margin-top:12px}.preview img{display:block;width:100%;max-height:70vh;object-fit:contain;background:#111;border-radius:6px}.hidden{display:none}"
".records .row{grid-template-columns:minmax(0,1fr) 300px}.record-title{font-weight:760}.record-actions{display:flex;gap:6px;flex-wrap:wrap;justify-content:flex-end}.record-actions a,.record-actions button,.record-actions span{min-width:76px;text-align:center}"
".bar{height:8px;border-radius:999px;background:#e8edf2;overflow:hidden;margin-top:8px}.bar>i{display:block;height:100%;width:0;background:var(--accent)}.storage-flow{display:none;margin-top:10px}.storage-flow.active,.storage-flow.done{display:block}.storage-flow.active .bar>i{width:35%;animation:flow 1.1s ease-in-out infinite alternate}.storage-flow.done .bar>i{width:100%;animation:none}@keyframes flow{from{margin-left:0}to{margin-left:65%}}@media(max-width:760px){.records .row,.row{grid-template-columns:1fr}.record-actions{justify-content:flex-start}.title{font-size:21px}}"
"</style></head><body><main>"
"<section class=\"top\"><div><div class=\"title\">P4 Buoy Vision</div><div class=\"sub\" id=\"modeLine\">正在加载...</div></div><div class=\"actions\"><button onclick=\"loadAll()\">刷新</button><button onclick=\"rebootDevice()\">重启设备</button></div></section>"
"<section class=\"grid\"><div class=\"panel\"><div class=\"title small\">连接地址</div><div class=\"rows\" id=\"netRows\"></div></div><div class=\"panel\"><div class=\"title small\">采集状态</div><div class=\"rows\" id=\"stateRows\"></div></div></section>"
"<section class=\"panel\"><div class=\"title small\">现场查看</div><div class=\"hint\">默认不打开实时图传，需要时再开启。</div><div class=\"actions\"><button id=\"previewBtn\" onclick=\"togglePreview()\">打开实时图传</button><button onclick=\"toggleModelPanel()\">模型切换</button></div><div id=\"modelPanel\" class=\"panel hidden\" style=\"margin-top:12px\"><div class=\"form\"><label>推理模型<select id=\"modelSelect\"><option value=\"fish31\">Fish31</option><option value=\"tinycls\">TinyCNN</option><option value=\"coco\">COCO</option></select></label></div><div class=\"actions\"><button onclick=\"saveModel()\">保存模型</button><a class=\"btn\" href=\"/validate\" target=\"_blank\">手机验证</a></div><div class=\"hint\" id=\"modelMsg\">Fish31 为默认模型。</div></div><div id=\"previewBox\" class=\"preview\"><img id=\"previewStream\" alt=\"camera stream\"></div></section>"
"<section class=\"panel\"><div class=\"title small\">用户设置</div><div class=\"form\">"
"<label>录像片段时长(秒)<input id=\"segmentSec\" type=\"number\" min=\"5\" max=\"14400\" step=\"1\" required disabled></label>"
"<label>无连接进入采集(秒)<input id=\"idleSec\" type=\"number\" min=\"10\" max=\"86400\" step=\"1\" required disabled></label>"
"<label>网络模式<select id=\"netMode\" disabled><option value=\"apsta\">AP+STA</option><option value=\"softap\">SoftAP</option><option value=\"sta\">STA</option></select></label>"
"<label>路由器 SSID<input id=\"routerSsid\" maxlength=\"32\" autocomplete=\"off\" disabled></label>"
"<label>路由器密码<input id=\"routerPass\" maxlength=\"64\" type=\"password\" autocomplete=\"new-password\" disabled></label>"
"<label class=\"toggle\"><input id=\"clearRouterPass\" type=\"checkbox\" disabled>清除已保存密码（仅用于开放网络）</label>"
"<label class=\"toggle\"><input id=\"fieldAuto\" type=\"checkbox\" disabled>自动进入野外采集</label>"
"</div><div class=\"actions\"><button id=\"saveConfigBtn\" onclick=\"applyCustomerConfig()\" disabled>保存设置</button><button onclick=\"enterFieldMode()\">立即进入野外录像</button><button onclick=\"usbExportNow()\">USB 立即导出</button><button onclick=\"usbRestore()\">USB 恢复存储</button><button id=\"retryStorageBtn\" onclick=\"retryStorage()\">检查并重试 TF</button></div><div class=\"hint\" id=\"configMsg\">正在读取设备当前设置；读取成功后才可修改和保存。</div></section>"
"<section class=\"panel\"><div class=\"top\"><div><div class=\"title small\">录像记录</div><div class=\"hint\" id=\"recordMeta\">--</div></div><button id=\"clearRecordingsBtn\" onclick=\"clearRecordings()\">清空录像记录</button></div><div class=\"rows records\" id=\"recordRows\"></div></section>"
"</main><script>"
"window.onerror=function(m){let e=document.getElementById('modeLine');if(e)e.textContent='页面脚本错误：'+m};const modeLine=document.getElementById('modeLine'),netRows=document.getElementById('netRows'),stateRows=document.getElementById('stateRows'),segmentSec=document.getElementById('segmentSec'),idleSec=document.getElementById('idleSec'),routerSsid=document.getElementById('routerSsid'),routerPass=document.getElementById('routerPass'),clearRouterPass=document.getElementById('clearRouterPass'),netMode=document.getElementById('netMode'),modelSelect=document.getElementById('modelSelect'),fieldAuto=document.getElementById('fieldAuto'),saveConfigBtn=document.getElementById('saveConfigBtn'),configMsg=document.getElementById('configMsg'),modelPanel=document.getElementById('modelPanel'),modelMsg=document.getElementById('modelMsg'),previewBtn=document.getElementById('previewBtn'),previewBox=document.getElementById('previewBox'),previewStream=document.getElementById('previewStream'),recordMeta=document.getElementById('recordMeta'),recordRows=document.getElementById('recordRows'),configInputs=[segmentSec,idleSec,routerSsid,routerPass,clearRouterPass,netMode,fieldAuto];let lastStatus=null,lastGroups=[],configReady=false,configDirty=false,configSaving=false,statusLoading=false,recordsLoading=false;function setConfigEnabled(enabled){let active=enabled&&!configSaving;configInputs.forEach(e=>e.disabled=!active);saveConfigBtn.disabled=!active}function markConfigDirty(){if(!configReady)return;configDirty=true;configMsg.textContent='设置已修改；自动刷新不会覆盖输入，点击“保存设置”后生效'}configInputs.forEach(e=>{e.addEventListener('input',markConfigDirty);e.addEventListener('change',markConfigDirty)});function esc(v){return String(v==null?'':v).replace(/[&<>\\\"]/g,m=>m==='&'?'&amp;':m==='<'?'&lt;':m==='>'?'&gt;':'&quot;')}"
"function fetchWithTimeout(url,opt,ms){if(!window.AbortController)return fetch(url,opt||{});let c=new AbortController(),o=Object.assign({},opt||{}, {signal:c.signal}),t=setTimeout(()=>c.abort(),ms||4500);return fetch(url,o).finally(()=>clearTimeout(t))}"
"function yes(v){return v?'<span class=\"ok\">是</span>':'<span class=\"bad\">否</span>'}function link(u){return u?'<a href=\"'+esc(u)+'\" target=\"_blank\">'+esc(u)+'</a>':'--'}"
"function keep(id,v){let e=document.getElementById(id);if(e&&!configDirty&&document.activeElement!==e)e.value=v}function row(k,v){return '<div class=\"row\"><div class=\"label\">'+esc(k)+'</div><div>'+v+'</div></div>'}"
"function fieldCountdown(c){if(!c.field_auto_enable)return '<span class=\"muted\">已关闭</span>';if(c.field_idle_paused)return '<span class=\"warn\">暂停：'+esc(c.field_idle_pause_reason||'设备正在处理其他任务')+'</span>';let ms=Number(c.field_idle_remaining_ms);return ms>=0?'<span class=\"ok\">运行中，'+Math.ceil(ms/1000)+' 秒后进入采集</span>':'<span class=\"muted\">等待计时</span>'}"
"function fmtBytes(v){v=Number(v)||0;return v>=1048576?Math.round(v/1048576)+' MB':Math.round(v/1024)+' KB'}function fmtSec(v){v=Number(v)||0;return v?Math.round(v/1000)+' 秒':'--'}"
"function modelName(m){return m==='tinycls'?'TinyCNN':(m==='coco'?'COCO':'Fish31')}function cameraPowerLabel(s){let p=s&&s.power_mode;if(p==='running')return '运行中';if(p==='error')return '异常'+(s.camera_error?'：'+s.camera_error:'');return '待机'}function recordingLine(s){let done=Math.floor(Number(s&&s.recording_segments||0)/2),cur=Number(s&&s.recording_current_frames||0);return '已完成 '+done+' 段 / 当前片段 '+cur+' 帧'}function recTime(x){let t=Number(x&&x.start_epoch_ms)||0;return t?new Date(t).toLocaleString():('boot '+Math.round((Number(x&&x.start_ms)||0)/1000)+'s')}"
"function recInfo(x){return x?fmtSec(x.duration_ms)+' | '+(x.frames||0)+' 帧 | '+fmtBytes(x.bytes)+' | '+modelName(x.method):'--'}"
"function labelSummary(a){return (a||[]).slice(0,3).map(x=>esc(x.label)+' x'+(x.count||0)).join(' / ')||'--'}"
"async function syncTime(){await fetch('/api/time/sync?epoch_ms='+Date.now(),{method:'POST',cache:'no-store'}).catch(()=>{})}"
"async function loadAll(){await loadStatus();await loadRecords()}async function loadStatus(){if(statusLoading)return;statusLoading=true;try{let r=await fetchWithTimeout('/api/status?ts='+Date.now(),{cache:'no-store'},4500);if(!r.ok)throw new Error('HTTP '+r.status);let s=await r.json(),c=s.config||{};lastStatus=s;let clients=Number(s.client_count||s.web_clients||0),segMaxSec=Math.max(5,Math.floor(Number(c.recording_segment_max_ms||14400000)/1000));segmentSec.max=String(segMaxSec);modeLine.textContent='模式 '+(s.app_mode||'--')+' | 摄像头 '+cameraPowerLabel(s)+' | 网络 '+(s.network_mode||'--')+' | 模型 '+modelName(s.recognition_method);netRows.innerHTML=row('热点(AP)',link(s.ap_url))+row('路由器(STA)',link(s.sta_url))+row('网线(ETH)',link(s.eth_url))+row('mDNS',link(s.mdns_url))+row('客户端',String(clients));let tfOk=!!s.storage_acceptance_ok,tfText=tfOk?'写入验证通过':(s.storage_io_latched?'检测到写入故障，已停止录像写入':'尚未通过写入验证'),zf=Number(s.recording_zero_frame_archives||0);stateRows.innerHTML=row('TF存储',yes(tfOk)+' '+esc(tfText)+' | '+esc(s.storage_status||''))+row('USB主机已配置',s.usb_host_connected?'<span class=\"ok\">是</span>':'<span class=\"muted\">否</span>')+row('USB占用',esc(s.usb_storage_owner||'none')+' '+esc(s.usb_last_error||''))+row('录像',esc(recordingLine(s)))+row('录像片段时长',Math.round(Number(c.recording_segment_ms||0)/1000)+' 秒')+(zf>0?row('恢复归档','已归档 '+zf+' 个 0 帧临时文件，录像列表不受影响；清空录像会一并删除'):'');if(!configDirty){keep('segmentSec',Math.round((c.recording_segment_ms||60000)/1000));keep('idleSec',Math.round((c.field_idle_timeout_ms||300000)/1000));keep('routerSsid',c.router_ssid||'');netMode.value=s.network_mode||'apsta';fieldAuto.checked=!!c.field_auto_enable}modelSelect.value=s.recognition_method||'fish31';routerPass.placeholder=c.router_password_set?'已保存，留空不修改':'8-64 位，开放路由器可留空';if(!configReady){configReady=true;configMsg.textContent='已读取设备当前设置；修改后点击“保存设置”，以设备回显值为准'}setConfigEnabled(true);updateRecordProgress()}catch(e){modeLine.textContent='状态读取失败：'+(e&&e.name==='AbortError'?'设备响应超时，可能正在切换存储或进入野外录像':(e&&e.message?e.message:e));if(!configReady)configMsg.textContent='暂时无法读取设备当前设置，已禁止保存；页面会自动重试'}finally{statusLoading=false}}"
"async function postConfig(q){let r=await fetchWithTimeout('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q.toString(),cache:'no-store'},8000),t=await r.text(),j=null;try{j=t?JSON.parse(t):{}}catch(e){}if(!r.ok){let m=j&&(j.message||j.error)?(j.message||j.error):(t||('HTTP '+r.status));if(j&&j.action)m+='；'+j.action;throw new Error(m)}return j||{}}"
"async function applyCustomerConfig(){if(!configReady){configMsg.textContent='尚未读取到设备当前设置，暂不能保存；请等待页面自动重试';return}if(!segmentSec.reportValidity()||!idleSec.reportValidity()){configMsg.textContent='请先修正标红的时间参数';return}if(clearRouterPass.checked&&routerPass.value.length){configMsg.textContent='不能同时填写新密码和清除密码；请只选择一种操作';return}let seg=Number(segmentSec.value),idle=Number(idleSec.value);if(!Number.isInteger(seg)||!Number.isInteger(idle)){configMsg.textContent='倒计时和片段时长必须是整数秒';return}let q=new URLSearchParams();q.set('recording_segment_ms',String(seg*1000));q.set('field_idle_timeout_ms',String(idle*1000));q.set('field_auto_enable',fieldAuto.checked?'1':'0');q.set('network_mode',netMode.value);q.set('router_ssid',routerSsid.value);if(clearRouterPass.checked)q.set('router_password','');else if(routerPass.value.length>0)q.set('router_password',routerPass.value);configSaving=true;setConfigEnabled(false);configMsg.textContent='正在保存并核对设备实际配置...';let saved=false;try{let j=await postConfig(q);saved=true;configDirty=false;let auto=j.field_auto_enable?'开启':'关闭';configMsg.textContent='已生效：自动采集 '+auto+'，倒计时 '+Math.round(j.field_idle_timeout_ms/1000)+' 秒，录像片段 '+Math.round(j.recording_segment_ms/1000)+' 秒（已有录像保持不变）'+(j.network_mode?'，网络 '+j.network_mode:'');await loadStatus()}catch(e){configMsg.textContent='保存未确认：'+(e&&e.message?e.message:e)+'；页面会自动重连，状态恢复后请核对并再次保存'}finally{if(saved){routerPass.value='';clearRouterPass.checked=false}configSaving=false;setConfigEnabled(configReady)}}"
"function toggleModelPanel(){modelPanel.classList.toggle('hidden')}async function saveModel(){let q=new URLSearchParams();q.set('method',modelSelect.value);modelMsg.textContent='正在切换模型...';try{let j=await postConfig(q);modelMsg.textContent='模型已保存：'+modelName(j.recognition_method);await loadStatus()}catch(e){modelMsg.textContent='模型保存失败：'+(e&&e.message?e.message:e)}}"
"async function enterFieldMode(){configMsg.textContent='正在检查 TF 存储状态...'}"
"async function usbExportNow(){configMsg.textContent='正在准备 USB U 盘...'}"
"async function usbRestore(){configMsg.textContent='正在恢复 TF 存储...'}async function rebootDevice(){if(!confirm('确认重启设备？'))return;await fetch('/api/system/reboot?confirm=REBOOT',{method:'POST',cache:'no-store'}).catch(()=>{});configMsg.textContent='设备正在重启'}"
"async function waitCameraReady(){let lastErr='';for(let i=0;i<40;i++){let r=await fetch('/api/status?ts='+Date.now(),{cache:'no-store'}).catch(()=>null);if(r&&r.ok){let s=await r.json().catch(()=>null);if(s){lastStatus=s;if(s.power_mode==='running'&&(Number(s.last_jpeg_bytes)||0)>0)return s;if(s.power_mode==='error')lastErr=s.camera_error||'camera error';configMsg.textContent=lastErr?'摄像头重试中：'+lastErr:'正在等待摄像头画面...'}}await new Promise(resolve=>setTimeout(resolve,500))}throw new Error(lastErr||'camera wake timeout')}async function togglePreview(){let box=previewBox,img=previewStream,on=box.style.display==='block';if(on){previewBtn.disabled=true;previewBtn.textContent='正在关闭...';configMsg.textContent='正在关闭实时图传并让摄像头待机...';await fetch('/api/power?cmd=standby',{method:'POST',cache:'no-store'}).catch(()=>{});await new Promise(resolve=>setTimeout(resolve,300));img.removeAttribute('src');box.style.display='none';previewBtn.textContent='打开实时图传';previewBtn.disabled=false;setTimeout(loadAll,600);return}previewBtn.disabled=true;box.style.display='block';previewBtn.textContent='正在打开...';configMsg.textContent='正在唤醒摄像头...';img.onerror=()=>{configMsg.textContent='实时图传连接中断，请稍后重试';};try{await fetch('/api/power?cmd=wake',{method:'POST',cache:'no-store'}).catch(()=>{});img.src='/stream?ts='+Date.now();previewBtn.textContent='关闭实时图传';previewBtn.disabled=false;configMsg.textContent='实时图传正在打开，首帧出来前可能需要几秒';waitCameraReady().then(()=>{configMsg.textContent='实时图传已打开'}).catch(e=>{configMsg.textContent='摄像头仍在启动：'+(e&&e.message?e.message:e)})}catch(e){img.removeAttribute('src');box.style.display='none';previewBtn.textContent='打开实时图传';previewBtn.disabled=false;configMsg.textContent='实时图传打开失败：'+(e&&e.message?e.message:e)}finally{setTimeout(loadAll,600)}}"
"function pairFallback(records){let raws=(records||[]).filter(x=>x.kind==='raw'),anns=(records||[]).filter(x=>x.kind==='annotated');return raws.map(raw=>{let expected=raw.name&&raw.name.startsWith('raw_')?'annotated_'+raw.name.slice(4):'',legacy=raw.name&&raw.name.startsWith('raw_')?'ann_'+raw.name.slice(4):'';let ann=anns.find(a=>a.name===expected||a.name===legacy)||null;return {raw:raw,annotated:ann,method:raw.method||'fish31',model:raw.model||'',fill_state:ann?'ready':'missing',needs_rebuild:!ann}}).reverse()}"
"function rawOf(g){return g.raw||g.data&&g.data.raw}function annOf(g){return g.annotated||g.data&&g.data.annotated}function groupMethod(g){return g.method||(rawOf(g)&&rawOf(g).method)||'fish31'}"
"function renderRecord(g){let raw=rawOf(g),ann=annOf(g),base=raw||ann;if(!base)return '';let method=groupMethod(g),need=g.needs_rebuild||!ann;let status=need?'<span class=\"warn\">需补帧/重建</span>':'<span class=\"ok\">已生成</span>';let actions=(raw?'<a class=\"btn\" href=\"'+esc(raw.uri)+'\" target=\"_blank\">原视频</a>':'<span class=\"muted\">原视频</span>')+(ann?'<a class=\"btn\" href=\"'+esc(ann.uri)+'\" target=\"_blank\">推理视频</a>':'<span class=\"muted\">推理视频</span>')+(raw?'<button data-fill=\"'+esc(raw.name)+'\">补帧</button>':'');let progress=raw?'<div class=\"bar\" data-progress=\"'+esc(raw.name)+'\"><i></i></div><div class=\"sub\" data-progress-text=\"'+esc(raw.name)+'\"></div>':'';return '<div class=\"row\"><div><div class=\"record-title\">'+esc(recTime(base))+' | '+modelName(method)+'</div><div class=\"sub\">原视频 '+recInfo(raw)+'</div><div class=\"sub\">推理视频 '+recInfo(ann)+' | '+status+'</div><div class=\"sub\">'+labelSummary((ann||raw||{}).labels)+'</div>'+progress+'</div><div class=\"record-actions\">'+actions+'</div></div>'}"
"async function startFill(name){let r=await fetch('/api/recording/enrich?name='+encodeURIComponent(name),{method:'POST',cache:'no-store'}).catch(()=>null);configMsg.textContent=r&&r.ok?'补帧任务已加入队列':'补帧启动失败';setTimeout(loadAll,800)}"
"function updateRecordProgress(){let e=lastStatus&&lastStatus.enrichment;if(!e)return;document.querySelectorAll('[data-progress]').forEach(bar=>{let name=bar.getAttribute('data-progress'),i=bar.querySelector('i'),txt=Array.from(document.querySelectorAll('[data-progress-text]')).find(x=>x.getAttribute('data-progress-text')===name);if(e.raw_name===name&&(e.running||e.frame_count)){let pct=e.frame_count?Math.round((e.output_frames||0)*100/e.frame_count):0;i.style.width=pct+'%';if(txt)txt.textContent='补帧 '+(e.output_frames||0)+' / '+(e.frame_count||0)+' 帧 | stride '+(e.pass_stride||0)+' | '+(e.last_error||'')}else{i.style.width='0';if(txt)txt.textContent=''}})}"
"async function loadRecords(){if(recordsLoading)return;recordsLoading=true;try{let c=lastStatus&&lastStatus.recording_cleanup;if(c&&(c.queued||c.running)){recordMeta.textContent='正在后台清理录像：已删除 '+(c.deleted_files||0)+' / '+(c.total_files||'?')+' 个文件；Web 保持可用';recordRows.innerHTML='<div class=\"row\"><div class=\"warn\">正在清理录像，下载和补帧暂不可用；完成后列表会自动刷新。</div></div>';return}let r=await fetchWithTimeout('/api/recordings?limit=30&ts='+Date.now(),{cache:'no-store'},5000);if(!r.ok)throw new Error('HTTP '+r.status);let j=await r.json();recordMeta.textContent='TF '+(j.sd_mounted?'可用':'不可用')+' | '+(j.storage_status||'')+' | 当前 '+(j.current_uri||'--');lastGroups=(j.recording_groups&&j.recording_groups.length)?j.recording_groups:pairFallback(j.recordings||[]);recordRows.innerHTML=lastGroups.map(renderRecord).join('')||'<div class=\"row\"><div class=\"muted\">暂无录像记录</div></div>';document.querySelectorAll('[data-fill]').forEach(b=>b.onclick=()=>startFill(b.getAttribute('data-fill')));updateRecordProgress()}catch(e){recordMeta.textContent='录像记录读取失败：'+(e&&e.name==='AbortError'?'设备忙或正在切换存储，稍后自动重试':(e&&e.message?e.message:e))}finally{recordsLoading=false}}"
"function sleep(ms){return new Promise(resolve=>setTimeout(resolve,ms))}"
"function actionButton(fn){return Array.from(document.querySelectorAll('button')).find(b=>(b.getAttribute('onclick')||'').indexOf(fn+'()')>=0)}"
"function ensureStorageFlow(){let e=document.getElementById('storageFlow');if(!e){e=document.createElement('div');e.id='storageFlow';e.className='storage-flow';e.innerHTML='<div class=\"bar\"><i></i></div><div class=\"hint\" id=\"storageFlowText\"></div>';configMsg.parentNode.insertBefore(e,configMsg)}return e}"
"function storagePhase(s){s=s||{};let owner=s.usb_storage_owner||'none',svc=s.storage_service||{},txt=svc.status||s.storage_status||'';if(s.storage_quiescing||s.usb_export_requested||s.usb_restore_requested||s.storage_retry_requested){return {busy:1,cls:'warn',text:s.usb_restore_requested?'正在把 TF 恢复给设备，请等待 5-15 秒':(s.usb_export_requested?'正在准备 USB U 盘，正在停止采集并交接 TF':'正在检查 TF 并执行写入验证，请等待'),action:'请等待当前存储流程完成'}}if(owner==='usb'||s.app_mode==='usb_export'){let c=!!s.usb_host_connected;return {done:1,cls:'warn',text:c?'P4_BUOY 正在导出给电脑；复制完成后可点击“USB 恢复存储”，也可以拔掉 USB 自动恢复 TF':'USB 已断开或电脑已弹出，设备正在自动恢复 TF；若长时间未恢复可点击“USB 恢复存储”',action:c?'确认复制完成后点击“USB 恢复存储”，设备会从 Windows 移除 U 盘并恢复 TF':'等待自动恢复，或点击“USB 恢复存储”兜底'}}if(s.storage_acceptance_ok){return {done:1,cls:'ok',text:'TF 写入验证已通过，可以稳定录像和下载记录',action:''}}return {busy:0,cls:'bad',text:'TF 未通过写入验证：'+(txt||s.usb_last_error||'请检查卡片')+'；检查或更换 TF 后可在 Web 直接重试',action:'检查或更换 TF 卡，再点击“检查并重试 TF”'}}"
"function enhanceStatus(){let s=lastStatus||{},p=storagePhase(s),flow=ensureStorageFlow(),ft=document.getElementById('storageFlowText');flow.className='storage-flow '+(p.busy?'active':(p.done?'done':''));if(ft)ft.textContent=p.text;if(stateRows&&stateRows.innerHTML.indexOf('存储流程')<0)stateRows.innerHTML+=row('存储流程','<span class=\"'+p.cls+'\">'+esc(p.text)+'</span>');let usbOwner=(s.usb_storage_owner==='usb'||s.app_mode==='usb_export'),field=actionButton('enterFieldMode'),ue=actionButton('usbExportNow'),ur=actionButton('usbRestore'),sr=document.getElementById('retryStorageBtn');if(field){field.disabled=!!s.storage_quiescing||s.app_mode==='field'||s.network_shutdown_for_idle||usbOwner;field.title=field.disabled?(usbOwner?'先点击“USB 恢复存储”或拔掉 USB 线':(p.action||'当前状态不能进入野外录像')):''}if(ue){ue.disabled=!!s.storage_quiescing||usbOwner||s.app_mode!=='server';ue.title=ue.disabled?(s.app_mode==='export'?'以太网导出模式不能切换 USB；请先返回服务器模式':p.text):''}if(ur){ur.disabled=!!s.storage_quiescing||!!s.usb_restore_requested||!usbOwner;ur.title=ur.disabled?'当前状态不能恢复 TF':(s.usb_host_connected?'点击后会从 Windows 移除 P4_BUOY 并恢复 TF，请确认复制已完成':'USB 已断开，可安全恢复 TF 给设备')}if(sr){sr.disabled=!!s.storage_quiescing||usbOwner||!!s.storage_retry_requested;sr.title=sr.disabled?p.text:'重新挂载并执行真实写入、同步、读回验证'}}"
"const baseLoadStatus=loadStatus;loadStatus=async function(){await baseLoadStatus();enhanceStatus()};"
"async function readActionJson(url,opt){let r=await fetchWithTimeout(url,opt||{cache:'no-store'},8000).catch(e=>{if(e&&e.name==='AbortError')throw new Error('设备响应超时，可能正在切换存储或模式');return null});if(!r)throw new Error('网络连接失败');let t=await r.text().catch(()=>''),j=null;try{j=t?JSON.parse(t):null}catch(e){}if(!r.ok){throw new Error(j&&j.message?(j.message+(j.action?'；'+j.action:'')):(j&&j.error?j.error:(t||('HTTP '+r.status))))}return j||{}}"
"async function statusOnce(){let r=await fetchWithTimeout('/api/status?ts='+Date.now(),{cache:'no-store'},4500);let s=await r.json();lastStatus=s;enhanceStatus();return s}"
"async function waitStorageStable(ms){let start=Date.now(),s=lastStatus||{};while(Date.now()-start<ms){s=await statusOnce().catch(()=>s);if(!s.storage_quiescing&&!s.usb_export_requested&&!s.usb_restore_requested&&!s.storage_retry_requested)return s;configMsg.textContent=storagePhase(s).text;await sleep(1000)}throw new Error('存储流程等待超时；Web 仍在线，请检查卡片后再次重试')}"
"async function waitTfReady(ms){let start=Date.now(),s=lastStatus||{};while(Date.now()-start<ms){s=await statusOnce().catch(()=>s);if(s.storage_acceptance_ok&&s.usb_storage_owner!=='usb'&&!s.storage_quiescing)return s;configMsg.textContent='正在检查 TF 存储，请等待... '+storagePhase(s).text;await sleep(1000)}throw new Error('TF 未通过写入验证，请检查或更换卡片后点击“检查并重试 TF”')}"
"async function retryStorage(){let b=document.getElementById('retryStorageBtn');if(b)b.disabled=true;try{configMsg.textContent='正在重新挂载 TF 并执行写入/读回验证...';await readActionJson('/api/storage/retry?confirm=RETRY',{method:'POST',cache:'no-store'});await waitTfReady(60000);configMsg.textContent='TF 写入验证已通过，可以稳定录像'}catch(e){configMsg.textContent='TF 重试未通过：'+(e&&e.message?e.message:e)}finally{if(b)b.disabled=false;setTimeout(loadAll,800)}}"
"async function prepareFieldStorage(){let s=await waitStorageStable(45000);if(s.usb_storage_owner==='usb'||s.app_mode==='usb_export')throw new Error('TF 仍由 USB 导出给电脑；请先点击“USB 恢复存储”或拔掉 USB 线');if(!s.storage_acceptance_ok){configMsg.textContent='TF 尚未通过验证；Wi-Fi 保持连接，页面可能短暂重连，正在安全重试...';await readActionJson('/api/storage/retry?confirm=RETRY',{method:'POST',cache:'no-store'});s=await waitTfReady(60000)}return s}"
"enterFieldMode=async function(){if(!confirm('确认立即进入野外录像？设备会先核对 TF 写入状态。'))return;let b=actionButton('enterFieldMode');if(b)b.disabled=true;try{await prepareFieldStorage();configMsg.textContent='正在进入野外录像，Web 可能断开，设备开始写入 TF...';await readActionJson('/api/mode/field?confirm=FIELD',{method:'POST',cache:'no-store'});configMsg.textContent='正在进入野外录像'}catch(e){configMsg.textContent='进入野外录像未完成：'+(e&&e.message?e.message:e)}finally{setTimeout(loadAll,1200)}};"
"usbExportNow=async function(){try{configMsg.textContent='正在准备 USB U 盘，可能需要 5-15 秒...';let s=lastStatus||await statusOnce();if(!(s.usb_storage_owner==='usb'||s.app_mode==='usb_export'))await readActionJson('/api/mode/usb?confirm=USB',{method:'POST',cache:'no-store'});let start=Date.now();while(Date.now()-start<60000){s=await statusOnce().catch(()=>s);if(s.usb_storage_owner==='usb'||s.app_mode==='usb_export'){configMsg.textContent='P4_BUOY U 盘已交给电脑，Web 保持在线；复制完成后可点击“USB 恢复存储”或直接拔线';break}configMsg.textContent=storagePhase(s).text;await sleep(1000)}}catch(e){configMsg.textContent='USB 导出未完成：'+(e&&e.message?e.message:e)}finally{setTimeout(loadAll,800)}};"
"usbRestore=async function(){try{configMsg.textContent='正在从电脑移除 P4_BUOY 并把 TF 恢复给设备，可能需要 5-15 秒...';await readActionJson('/api/mode/usb/restore?confirm=RESTORE',{method:'POST',cache:'no-store'});await waitTfReady(60000);configMsg.textContent='TF 已恢复给设备并通过写入验证；USB 线可以继续插着，需要导出时再点“USB 立即导出”'}catch(e){configMsg.textContent='USB 恢复未完成：'+(e&&e.message?e.message:e)+'；可稍后重试，或拔掉 USB 后等待自动恢复'}finally{setTimeout(loadAll,800)}};"
"let recordingCleanupJobId=0;function renderRecordingCleanup(s){s=s||{};let c=s.recording_cleanup||{},b=document.getElementById('clearRecordingsBtn'),active=!!(c.queued||c.running),total=Number(c.total_files)||0,deleted=Number(c.deleted_files)||0;if(active){recordingCleanupJobId=Number(c.job_id)||recordingCleanupJobId;if(b){b.disabled=true;b.textContent='正在清理 '+deleted+(total?' / '+total:'')};recordMeta.textContent='正在后台清理录像：已删除 '+deleted+(total?' / '+total:'')+' 个文件；Web 和状态检查保持可用';recordRows.innerHTML='<div class=\"row\"><div class=\"warn\">正在清理录像，旧下载链接已暂停；完成后列表会自动刷新。</div></div>';configMsg.textContent='录像清理正在后台进行，请勿重启或重复点击；可以继续查看设备状态';[actionButton('enterFieldMode'),actionButton('usbExportNow'),actionButton('usbRestore'),document.getElementById('retryStorageBtn')].forEach(x=>{if(x){x.disabled=true;x.title='等待录像清理完成'}});setConfigEnabled(false);return}if(b){let transfers=Number(s.file_download_clients)||0,blocked=!!s.storage_quiescing||s.usb_storage_owner==='usb'||s.app_mode!=='server'||!s.sd_mounted||transfers>0;b.disabled=blocked;b.textContent='清空录像记录';b.title=transfers>0?'仍有文件下载或上传，请结束后再清理':(blocked?'当前存储状态不能清理录像':'')}if(recordingCleanupJobId&&Number(c.job_id)===recordingCleanupJobId&&c.done){if(c.ok){configMsg.textContent='清理完成：删除 '+deleted+' 个文件，释放 '+fmtBytes(c.freed_bytes||0)+'；目录复扫无残留'}else{configMsg.textContent='清理未完成：'+(c.message||'请检查 TF 健康状态')+'（剩余 '+(c.remaining_files||0)+'，错误 '+(c.errors||0)+'）；请先检查并重试 TF'}recordingCleanupJobId=0;setTimeout(loadRecords,200)}}"
"const loadStatusWithStorage=loadStatus;loadStatus=async function(){await loadStatusWithStorage();renderRecordingCleanup(lastStatus)};"
"async function pollRecordingCleanup(jobId){let deadline=Date.now()+600000,lastNetworkError='';while(Date.now()<deadline){try{let r=await fetch('/api/status?cleanup_job='+jobId+'&ts='+Date.now(),{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);let s=await r.json();lastStatus=s;enhanceStatus();renderRecordingCleanup(s);let c=s.recording_cleanup||{};if(Number(c.job_id)===Number(jobId)&&c.done){await loadRecords();return}}catch(e){lastNetworkError=e&&e.message?e.message:String(e)}await sleep(700)}configMsg.textContent='清理状态暂未确认'+(lastNetworkError?'：'+lastNetworkError:'')+'；设备会继续后台处理，请保持页面打开并查看状态，不要重启或重复点击'}"
"clearRecordings=async function(){if(!confirm('确认清空全部录像记录？清理会在后台执行，期间 Web 保持可用。'))return;let b=document.getElementById('clearRecordingsBtn');if(b){b.disabled=true;b.textContent='正在提交...'}configMsg.textContent='正在检查下载和存储状态...';let r=await fetch('/api/recordings?confirm=DELETE',{method:'DELETE',cache:'no-store'}).catch(()=>null);if(!r){configMsg.textContent='清理请求未确认；页面将检查设备状态，请勿立即重复点击';setTimeout(loadStatus,500);return}let t=await r.text().catch(()=>''),j=null;try{j=t?JSON.parse(t):null}catch(e){}if(!r.ok||!j||!j.ok){let m=j&&(j.message||j.error)?(j.message||j.error):(t||('HTTP '+r.status));if(j&&j.action)m+='；'+j.action;configMsg.textContent='暂未开始清理：'+m;setTimeout(loadStatus,500);return}recordingCleanupJobId=Number(j.job_id)||0;configMsg.textContent='录像清理已进入后台队列，Web 保持可用；正在读取进度...';pollRecordingCleanup(recordingCleanupJobId)};"
"setInterval(loadStatus,1000);setInterval(loadRecords,5000);syncTime().then(loadAll);"
"</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, s_customer_index_html_v2);
}

static bool export_mode_reject(httpd_req_t *req, const char *feature)
{
    if (s_app_mode != APP_MODE_EXPORT && s_app_mode != APP_MODE_USB_EXPORT) {
        return false;
    }
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_set_type(req, "text/plain");
    char msg[128];
    snprintf(msg, sizeof(msg), "%s disabled while %s is active",
             feature ? feature : "feature", app_mode_name(s_app_mode));
    httpd_resp_sendstr(req, msg);
    return true;
}

static esp_err_t validate_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (export_mode_reject(req, "validation")) {
        return ESP_OK;
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, s_validate_html);
}

static esp_err_t send_embedded_jpeg(httpd_req_t *req, const uint8_t *start, const uint8_t *end)
{
    if (!start || !end || end <= start) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "validation image missing");
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=3600");
    return http_send_buffer_chunked(req, (const char *)start, (size_t)(end - start));
}

static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end);

static esp_err_t validate_demo_jpg_handler(httpd_req_t *req)
{
    record_http_request(req);
    validation_sample_t sample = VALIDATION_SAMPLE_NONE;
    if (strcmp(req->uri, "/validate/demo_01.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_01;
    } else if (strcmp(req->uri, "/validate/demo_02.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_02;
    } else if (strcmp(req->uri, "/validate/demo_03.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_03;
    } else if (strcmp(req->uri, "/validate/demo_04.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_04;
    } else if (strcmp(req->uri, "/validate/tiny_01.jpg") == 0) {
        sample = VALIDATION_SAMPLE_TINY_01;
    } else if (strcmp(req->uri, "/validate/tiny_02.jpg") == 0) {
        sample = VALIDATION_SAMPLE_TINY_02;
    } else if (strcmp(req->uri, "/validate/tiny_03.jpg") == 0) {
        sample = VALIDATION_SAMPLE_TINY_03;
    } else if (strcmp(req->uri, "/validate/tiny_04.jpg") == 0) {
        sample = VALIDATION_SAMPLE_TINY_04;
    } else if (strcmp(req->uri, "/validate/fish31_01.jpg") == 0) {
        sample = VALIDATION_SAMPLE_FISH31_01;
    } else if (strcmp(req->uri, "/validate/fish31_02.jpg") == 0) {
        sample = VALIDATION_SAMPLE_FISH31_02;
    } else if (strcmp(req->uri, "/validate/fish31_03.jpg") == 0) {
        sample = VALIDATION_SAMPLE_FISH31_03;
    } else if (strcmp(req->uri, "/validate/fish31_04.jpg") == 0) {
        sample = VALIDATION_SAMPLE_FISH31_04;
    }

    const uint8_t *start = NULL;
    const uint8_t *end = NULL;
    if (!validation_sample_image(sample, &start, &end)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "validation image missing");
    }
    return send_embedded_jpeg(req, start, end);
}

static const char *validation_sample_name(validation_sample_t sample)
{
    switch (sample) {
    case VALIDATION_SAMPLE_COKE:
        return "coke";
    case VALIDATION_SAMPLE_SPRITE:
        return "sprite";
    case VALIDATION_SAMPLE_DEMO_01:
        return "demo_01";
    case VALIDATION_SAMPLE_DEMO_02:
        return "demo_02";
    case VALIDATION_SAMPLE_DEMO_03:
        return "demo_03";
    case VALIDATION_SAMPLE_DEMO_04:
        return "demo_04";
    case VALIDATION_SAMPLE_TINY_01:
        return "tiny_01";
    case VALIDATION_SAMPLE_TINY_02:
        return "tiny_02";
    case VALIDATION_SAMPLE_TINY_03:
        return "tiny_03";
    case VALIDATION_SAMPLE_TINY_04:
        return "tiny_04";
    case VALIDATION_SAMPLE_FISH31_01:
        return "fish31_01";
    case VALIDATION_SAMPLE_FISH31_02:
        return "fish31_02";
    case VALIDATION_SAMPLE_FISH31_03:
        return "fish31_03";
    case VALIDATION_SAMPLE_FISH31_04:
        return "fish31_04";
    default:
        return "none";
    }
}

static const char *validation_sample_expected(validation_sample_t sample)
{
    if (sample == VALIDATION_SAMPLE_DEMO_01 ||
        sample == VALIDATION_SAMPLE_DEMO_02 ||
        sample == VALIDATION_SAMPLE_DEMO_03 ||
        sample == VALIDATION_SAMPLE_DEMO_04) {
        return "object";
    }
    if (sample == VALIDATION_SAMPLE_TINY_01) {
        return VALIDATION_TINYCLS_01_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_TINY_02) {
        return VALIDATION_TINYCLS_02_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_TINY_03) {
        return VALIDATION_TINYCLS_03_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_TINY_04) {
        return VALIDATION_TINYCLS_04_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_01) {
        return VALIDATION_FISH31_01_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_02) {
        return VALIDATION_FISH31_02_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_03) {
        return VALIDATION_FISH31_03_LABEL;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_04) {
        return VALIDATION_FISH31_04_LABEL;
    }
    return "none";
}

static const char *validation_sample_image_uri(validation_sample_t sample)
{
    switch (sample) {
    case VALIDATION_SAMPLE_DEMO_01:
        return "/validate/demo_01.jpg";
    case VALIDATION_SAMPLE_DEMO_02:
        return "/validate/demo_02.jpg";
    case VALIDATION_SAMPLE_DEMO_03:
        return "/validate/demo_03.jpg";
    case VALIDATION_SAMPLE_DEMO_04:
        return "/validate/demo_04.jpg";
    case VALIDATION_SAMPLE_TINY_01:
        return "/validate/tiny_01.jpg";
    case VALIDATION_SAMPLE_TINY_02:
        return "/validate/tiny_02.jpg";
    case VALIDATION_SAMPLE_TINY_03:
        return "/validate/tiny_03.jpg";
    case VALIDATION_SAMPLE_TINY_04:
        return "/validate/tiny_04.jpg";
    case VALIDATION_SAMPLE_FISH31_01:
        return "/validate/fish31_01.jpg";
    case VALIDATION_SAMPLE_FISH31_02:
        return "/validate/fish31_02.jpg";
    case VALIDATION_SAMPLE_FISH31_03:
        return "/validate/fish31_03.jpg";
    case VALIDATION_SAMPLE_FISH31_04:
        return "/validate/fish31_04.jpg";
    default:
        return "";
    }
}

static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end)
{
    if (!start || !end) {
        return false;
    }
    if (sample == VALIDATION_SAMPLE_DEMO_01) {
        *start = validate_demo_01_jpg_start;
        *end = validate_demo_01_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_DEMO_02) {
        *start = validate_demo_02_jpg_start;
        *end = validate_demo_02_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_DEMO_03) {
        *start = validate_demo_03_jpg_start;
        *end = validate_demo_03_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_DEMO_04) {
        *start = validate_demo_04_jpg_start;
        *end = validate_demo_04_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_TINY_01) {
        *start = validate_tiny_01_jpg_start;
        *end = validate_tiny_01_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_TINY_02) {
        *start = validate_tiny_02_jpg_start;
        *end = validate_tiny_02_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_TINY_03) {
        *start = validate_tiny_03_jpg_start;
        *end = validate_tiny_03_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_TINY_04) {
        *start = validate_tiny_04_jpg_start;
        *end = validate_tiny_04_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_01) {
        *start = validate_fish31_01_jpg_start;
        *end = validate_fish31_01_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_02) {
        *start = validate_fish31_02_jpg_start;
        *end = validate_fish31_02_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_03) {
        *start = validate_fish31_03_jpg_start;
        *end = validate_fish31_03_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_FISH31_04) {
        *start = validate_fish31_04_jpg_start;
        *end = validate_fish31_04_jpg_end;
        return *end > *start;
    }
    return false;
}

static bool validation_sample_matched(validation_sample_t sample,
                                      recognition_method_t method,
                                      const vision_result_t *vision)
{
    if (!vision) {
        return false;
    }
    const char *expected = validation_sample_expected(sample);
    if (method == RECOGNITION_METHOD_TINYCLS || method == RECOGNITION_METHOD_FISH31) {
        return strcmp(vision->label, expected) == 0 || strcmp(vision->object, expected) == 0;
    }
    if (strcmp(expected, "object") == 0) {
        return vision->object_count > 0;
    }
    return vision->object_count > 0 && strcmp(vision->object, expected) == 0;
}

static bool validation_selftest_method_available(recognition_method_t method)
{
    if (method == RECOGNITION_METHOD_YOLO26) {
        return yolo26_espdl_available();
    }
    if (method == RECOGNITION_METHOD_YOLO11) {
        return yolo11_espdl_available();
    }
    if (method == RECOGNITION_METHOD_COCO) {
        return coco_espdl_available();
    }
    if (method == RECOGNITION_METHOD_TINYCLS) {
        return tiny_cls_espdl_available();
    }
    if (method == RECOGNITION_METHOD_FISH31) {
        return fish31_espdl_available();
    }
    return false;
}

static validation_sample_t parse_validation_sample(const char *text)
{
    if (strcmp(text, "demo_01") == 0 || strcmp(text, "demo1") == 0) {
        return VALIDATION_SAMPLE_DEMO_01;
    }
    if (strcmp(text, "demo_02") == 0 || strcmp(text, "demo2") == 0) {
        return VALIDATION_SAMPLE_DEMO_02;
    }
    if (strcmp(text, "demo_03") == 0 || strcmp(text, "demo3") == 0) {
        return VALIDATION_SAMPLE_DEMO_03;
    }
    if (strcmp(text, "demo_04") == 0 || strcmp(text, "demo4") == 0) {
        return VALIDATION_SAMPLE_DEMO_04;
    }
    if (strcmp(text, "tiny_01") == 0 || strcmp(text, "tiny1") == 0) {
        return VALIDATION_SAMPLE_TINY_01;
    }
    if (strcmp(text, "tiny_02") == 0 || strcmp(text, "tiny2") == 0) {
        return VALIDATION_SAMPLE_TINY_02;
    }
    if (strcmp(text, "tiny_03") == 0 || strcmp(text, "tiny3") == 0) {
        return VALIDATION_SAMPLE_TINY_03;
    }
    if (strcmp(text, "tiny_04") == 0 || strcmp(text, "tiny4") == 0) {
        return VALIDATION_SAMPLE_TINY_04;
    }
    if (strcmp(text, "fish31_01") == 0 || strcmp(text, "fish31_1") == 0) {
        return VALIDATION_SAMPLE_FISH31_01;
    }
    if (strcmp(text, "fish31_02") == 0 || strcmp(text, "fish31_2") == 0) {
        return VALIDATION_SAMPLE_FISH31_02;
    }
    if (strcmp(text, "fish31_03") == 0 || strcmp(text, "fish31_3") == 0) {
        return VALIDATION_SAMPLE_FISH31_03;
    }
    if (strcmp(text, "fish31_04") == 0 || strcmp(text, "fish31_4") == 0) {
        return VALIDATION_SAMPLE_FISH31_04;
    }
    return VALIDATION_SAMPLE_NONE;
}

static recognition_method_t parse_validation_method(const char *text)
{
    if (strcmp(text, "coco") == 0) {
        return RECOGNITION_METHOD_COCO;
    }
    if (strcmp(text, "tinycls") == 0 || strcmp(text, "tiny_cls") == 0) {
        return RECOGNITION_METHOD_TINYCLS;
    }
    if (strcmp(text, "fish31") == 0 || strcmp(text, "fish") == 0) {
        return RECOGNITION_METHOD_FISH31;
    }
    if (s_recognition_method == RECOGNITION_METHOD_FISH31 ||
        s_recognition_method == RECOGNITION_METHOD_TINYCLS ||
        s_recognition_method == RECOGNITION_METHOD_COCO) {
        return s_recognition_method;
    }
    return preferred_recognition_method();
}

static bool parse_validation_method_strict(const char *text, recognition_method_t *method)
{
    if (!text || !text[0] || !method) {
        return false;
    }
    if (strcmp(text, "coco") == 0) {
        *method = RECOGNITION_METHOD_COCO;
        return true;
    }
    if (strcmp(text, "tinycls") == 0 || strcmp(text, "tiny_cls") == 0) {
        *method = RECOGNITION_METHOD_TINYCLS;
        return true;
    }
    if (strcmp(text, "fish31") == 0 || strcmp(text, "fish") == 0) {
        *method = RECOGNITION_METHOD_FISH31;
        return true;
    }
    return false;
}

static esp_err_t validation_json_error(httpd_req_t *req, const char *status, const char *error)
{
    char json[256];
    json_writer_t writer;
    json_writer_init(&writer, json, sizeof(json));
    json_writer_appendf(&writer, "{\"ok\":false,\"error\":");
    json_writer_append_escaped_string(&writer, error ? error : "unknown validation error");
    json_writer_appendf(&writer, "}");
    if (!json_writer_ok(&writer)) {
        strlcpy(json, "{\"ok\":false,\"error\":\"validation error response overflow\"}",
                sizeof(json));
    }
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t validation_run_get_rejected_handler(httpd_req_t *req)
{
    record_http_request(req);
    httpd_resp_set_hdr(req, "Allow", "POST");
    return validation_json_error(req, "405 Method Not Allowed",
                                 "validation jobs must be started with POST");
}

static esp_err_t validation_run_post_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (export_mode_reject(req, "validation run")) {
        return ESP_OK;
    }
    if (http_server_is_stopping() || s_storage_quiescing ||
        storage_transition_active()) {
        return validation_json_error(req, "503 Service Unavailable",
                                     "the device is changing mode; wait for the web service to stabilize and retry");
    }
    char form[192] = {0};
    char sample_text[16] = {0};
    char method_text[16] = {0};
    char box_min_text[8] = {0};

    if (req->content_len == 0) {
        return validation_json_error(req, "400 Bad Request",
                                     "POST body must include sample, method, and box_min_score");
    }
    if (req->content_len >= sizeof(form)) {
        return validation_json_error(req, "413 Content Too Large",
                                     "validation request body is too large");
    }
    size_t received = 0;
    while (received < req->content_len) {
        int ret = httpd_req_recv(req, form + received, req->content_len - received);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            return validation_json_error(req, "408 Request Timeout",
                                         "validation request body timed out");
        }
        if (ret <= 0) {
            return ESP_FAIL;
        }
        received += (size_t)ret;
    }
    form[received] = '\0';
    esp_err_t sample_ret = httpd_query_key_value(form, "sample", sample_text, sizeof(sample_text));
    esp_err_t method_ret = httpd_query_key_value(form, "method", method_text, sizeof(method_text));
    esp_err_t box_ret = httpd_query_key_value(form, "box_min_score", box_min_text, sizeof(box_min_text));
    if (sample_ret != ESP_OK || method_ret != ESP_OK ||
        (box_ret != ESP_OK && box_ret != ESP_ERR_NOT_FOUND)) {
        return validation_json_error(req, "400 Bad Request",
                                     "sample and method are required; parameters must not be truncated");
    }

    validation_sample_t sample = parse_validation_sample(sample_text);
    if (sample == VALIDATION_SAMPLE_NONE) {
        return validation_json_error(req, "400 Bad Request", "sample must be fish31_01..fish31_04, tiny_01..tiny_04, or demo_01..demo_04");
    }

    recognition_method_t method;
    if (!parse_validation_method_strict(method_text, &method)) {
        return validation_json_error(req, "400 Bad Request",
                                     "method must be fish31, tinycls, or coco");
    }
    if (method == RECOGNITION_METHOD_YOLO26 && !yolo26_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "yolo26 model unavailable");
    }
    if (method == RECOGNITION_METHOD_YOLO11 && !yolo11_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "yolo11 model unavailable");
    }
    if (method == RECOGNITION_METHOD_COCO && !coco_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "coco model unavailable");
    }
    if (method == RECOGNITION_METHOD_TINYCLS && !tiny_cls_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "tinycls model unavailable");
    }
    if (method == RECOGNITION_METHOD_FISH31 && !fish31_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "fish31 model unavailable");
    }
    if (!recognition_method_uses_jpeg_inference(method)) {
        return validation_json_error(req, "400 Bad Request", "validation supports fish31, tinycls, or coco");
    }
    bool sample_matches_method =
        (method == RECOGNITION_METHOD_COCO &&
         sample >= VALIDATION_SAMPLE_DEMO_01 && sample <= VALIDATION_SAMPLE_DEMO_04) ||
        (method == RECOGNITION_METHOD_TINYCLS &&
         sample >= VALIDATION_SAMPLE_TINY_01 && sample <= VALIDATION_SAMPLE_TINY_04) ||
        (method == RECOGNITION_METHOD_FISH31 &&
         sample >= VALIDATION_SAMPLE_FISH31_01 && sample <= VALIDATION_SAMPLE_FISH31_04);
    if (!sample_matches_method) {
        return validation_json_error(req, "400 Bad Request",
                                     "sample does not belong to the selected validation method");
    }
    uint32_t validation_box_min_score = 50;
    if (box_min_text[0]) {
        if (!query_u32(form, "box_min_score", 1, 100, &validation_box_min_score)) {
            return validation_json_error(req, "400 Bad Request",
                                         "box_min_score must be an integer in range 1..100");
        }
    }

    /*
     * 这里不直接在 HTTP worker 里跑模型，而是把内嵌 JPEG 复制到 PSRAM 后投递到
     * inference_task。这样 /validate 的结果和实时摄像头抽帧走同一条板端推理链路，
     * 同时也避免长耗时 YOLO 把 HTTP 服务器的其它请求全部卡住。
     */
    const uint8_t *jpg_start = NULL;
    const uint8_t *jpg_end = NULL;
    if (!validation_sample_image(sample, &jpg_start, &jpg_end)) {
        return validation_json_error(req, "404 Not Found", "embedded validation image missing");
    }
    uint32_t jpeg_size = (uint32_t)(jpg_end - jpg_start);

    uint32_t expected_active = 0;
    if (!__atomic_compare_exchange_n(&s_validation_active_jobs, &expected_active, 1,
                                     false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        return validation_json_error(req, "409 Conflict",
                                     "another validation or dataset inference is running");
    }
    validation_context_t *ctx = validation_context_create();
    if (!ctx) {
        __atomic_sub_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
        return validation_json_error(req, "500 Internal Server Error", "no validation context");
    }
    ctx->publish_result = true;
    ctx->id = __atomic_add_fetch(&s_validation_id, 1, __ATOMIC_ACQ_REL);
    ctx->sample = sample;
    ctx->method = method;
    ctx->box_min_score = validation_box_min_score;
    ctx->jpeg_size = jpeg_size;
    ctx->queued_ms = esp_timer_get_time() / 1000;

    inference_job_t job = {
        .jpeg_size = jpeg_size,
        .method = method,
        .box_min_score = validation_box_min_score,
        .validation = true,
        .validation_sample = sample,
        .validation_ctx = ctx,
        .queued_ms = ctx->queued_ms,
    };
    if (method == RECOGNITION_METHOD_TINYCLS) {
        fill_tinycls_pending(&job.meta.vision);
    } else if (method == RECOGNITION_METHOD_FISH31) {
        fill_fish31_pending(&job.meta.vision);
    } else {
        fill_yolo_pending(&job.meta.vision, method);
    }
    job.jpeg = alloc_psram_buffer(jpeg_size);
    if (!job.jpeg) {
        __atomic_sub_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
        validation_context_release(ctx);
        return validation_json_error(req, "500 Internal Server Error", "no validation jpeg buffer");
    }
    memcpy(job.jpeg, jpg_start, jpeg_size);

    validation_cache_set_state(ctx, VALIDATION_JOB_QUEUED, ESP_OK);
    validation_context_retain(ctx);
    if (!s_inference_queue || xQueueSend(s_inference_queue, &job, pdMS_TO_TICKS(100)) != pdTRUE) {
        free(job.jpeg);
        ctx->err = ESP_ERR_TIMEOUT;
        ctx->completed_ms = esp_timer_get_time() / 1000;
        validation_cache_set_state(ctx, VALIDATION_JOB_FAILED, ESP_ERR_TIMEOUT);
        __atomic_sub_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
        validation_context_release(ctx);
        validation_context_release(ctx);
        s_inference_queue_drops++;
        return validation_json_error(req, "409 Conflict",
                                     "inference queue is busy; wait for the current analysis and retry");
    }

    s_inference_jobs_queued++;
    /* HTTP 立即返回任务 ID；inference_task 持有剩余引用并异步发布结果。 */
    uint32_t result_id = ctx->id;
    validation_context_release(ctx);

    char status_uri[80];
    char overlay_uri[88];
    snprintf(status_uri, sizeof(status_uri), "/api/validate/status?id=%" PRIu32, result_id);
    snprintf(overlay_uri, sizeof(overlay_uri), "/api/validate/overlay.svg?id=%" PRIu32,
             result_id);
    char json[320];
    json_writer_t writer;
    json_writer_init(&writer, json, sizeof(json));
    json_writer_appendf(&writer,
                        "{\"ok\":true,\"id\":%" PRIu32
                        ",\"state\":\"queued\",\"status\":",
                        result_id);
    json_writer_append_escaped_string(&writer, status_uri);
    json_writer_appendf(&writer, ",\"overlay\":");
    json_writer_append_escaped_string(&writer, overlay_uri);
    json_writer_appendf(&writer, ",\"retry_after_ms\":500}");
    if (!json_writer_ok(&writer)) {
        return validation_json_error(req, "500 Internal Server Error",
                                     "could not serialize validation job response");
    }
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Location", status_uri);
    httpd_resp_set_hdr(req, "Retry-After", "1");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t validation_status_get_handler(httpd_req_t *req)
{
    record_http_poll(req);
    char query[64] = {0};
    uint32_t requested_id = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        !query_u32(query, "id", 1, UINT32_MAX, &requested_id)) {
        return validation_json_error(req, "400 Bad Request",
                                     "id is required and must be a positive integer");
    }
    if (!s_validation_lock) {
        return validation_json_error(req, "503 Service Unavailable",
                                     "validation result service is not ready");
    }

    validation_cache_t cache = {0};
    xSemaphoreTake(s_validation_lock, portMAX_DELAY);
    cache = s_validation_last;
    xSemaphoreGive(s_validation_lock);
    if (!cache.valid) {
        return validation_json_error(req, "404 Not Found", "validation job was not found");
    }
    if (cache.id != requested_id) {
        return validation_json_error(req, "410 Gone",
                                     "validation result was replaced by a newer job; run it again");
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t end_ms = cache.completed_ms > 0 ? cache.completed_ms : now_ms;
    int64_t elapsed_ms = end_ms >= cache.queued_ms ? end_ms - cache.queued_ms : 0;
    if (cache.state == VALIDATION_JOB_QUEUED || cache.state == VALIDATION_JOB_RUNNING) {
        char json[384];
        json_writer_t writer;
        json_writer_init(&writer, json, sizeof(json));
        json_writer_appendf(&writer,
                            "{\"ok\":true,\"id\":%" PRIu32 ",\"state\":\"%s\","
                            "\"done\":false,\"retry_after_ms\":500,"
                            "\"queued_ms\":%" PRId64 ",\"started_ms\":%" PRId64 ","
                            "\"elapsed_ms\":%" PRId64 "}",
                            cache.id,
                            cache.state == VALIDATION_JOB_QUEUED ? "queued" : "running",
                            cache.queued_ms, cache.started_ms, elapsed_ms);
        if (!json_writer_ok(&writer)) {
            return validation_json_error(req, "500 Internal Server Error",
                                         "could not serialize validation job status");
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return http_send_cstr_chunked(req, json);
    }

    if (cache.state == VALIDATION_JOB_FAILED || cache.err != ESP_OK) {
        char json[512];
        json_writer_t writer;
        json_writer_init(&writer, json, sizeof(json));
        json_writer_appendf(&writer,
                            "{\"ok\":false,\"id\":%" PRIu32
                            ",\"state\":\"failed\",\"done\":true,\"error_code\":%d,"
                            "\"error\":",
                            cache.id, (int)cache.err);
        json_writer_append_escaped_string(&writer, esp_err_to_name(cache.err));
        json_writer_appendf(&writer, ",\"message\":");
        json_writer_append_escaped_string(
            &writer, "board inference failed; the web server is still available and no reboot is required");
        json_writer_appendf(&writer,
                            ",\"queued_ms\":%" PRId64 ",\"started_ms\":%" PRId64
                            ",\"completed_ms\":%" PRId64 ",\"elapsed_ms\":%" PRId64 "}",
                            cache.queued_ms, cache.started_ms, cache.completed_ms, elapsed_ms);
        if (!json_writer_ok(&writer)) {
            return validation_json_error(req, "500 Internal Server Error",
                                         "could not serialize validation failure status");
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return http_send_cstr_chunked(req, json);
    }

    vision_result_t vision = cache.vision;
    const size_t json_cap = 13312;
    char *json = (char *)alloc_psram_buffer(json_cap);
    if (!json) {
        return validation_json_error(req, "503 Service Unavailable",
                                     "memory is temporarily low; close video streams and retry");
    }
    bool matched = validation_sample_matched(cache.sample, cache.method, &vision);
    char overlay_uri[88];
    snprintf(overlay_uri, sizeof(overlay_uri),
             "/api/validate/overlay.svg?id=%" PRIu32, cache.id);
    json_writer_t writer;
    json_writer_init(&writer, json, json_cap);
    json_writer_appendf(&writer,
                        "{\"ok\":true,\"id\":%" PRIu32
                        ",\"state\":\"done\",\"done\":true,\"sample\":",
                        cache.id);
    json_writer_append_escaped_string(&writer, validation_sample_name(cache.sample));
    json_writer_appendf(&writer, ",\"expected\":");
    json_writer_append_escaped_string(&writer, validation_sample_expected(cache.sample));
    json_writer_appendf(&writer, ",\"method\":");
    json_writer_append_escaped_string(&writer, recognition_method_name(cache.method));
    json_writer_appendf(&writer, ",\"matched\":%s,\"source_image\":",
                        matched ? "true" : "false");
    json_writer_append_escaped_string(&writer, validation_sample_image_uri(cache.sample));
    json_writer_appendf(&writer, ",\"overlay\":");
    json_writer_append_escaped_string(&writer, overlay_uri);
    json_writer_appendf(
        &writer,
        ",\"source_w\":%" PRIu32 ",\"source_h\":%" PRIu32
        ",\"jpeg_bytes\":%" PRIu32 ",\"model_bytes\":%" PRIu32
        ",\"model_input_size\":%" PRIu32 ",\"nms_threshold\":%" PRIu32
        ",\"raw_candidate_count\":%" PRIu32 ",\"detection_count\":%" PRIu32
        ",\"detections\":",
        cache.source_w, cache.source_h, cache.jpeg_size,
        model_bytes_for_method(cache.method), model_input_size_for_method(cache.method),
        (uint32_t)APP_YOLO_NMS_THRESHOLD_X100, vision.raw_candidate_count,
        vision.detection_count);
    json_writer_append_detections(&writer, &vision);
    json_writer_appendf(&writer, ",\"top_k\":");
    json_writer_append_top_k(&writer, &vision);
    json_writer_appendf(
        &writer,
        ",\"queued_ms\":%" PRId64
        ",\"started_ms\":%" PRId64 ",\"completed_ms\":%" PRId64
        ",\"elapsed_ms\":%" PRId64 ",\"error\":\"\",\"vision\":{\"label\":",
        cache.queued_ms, cache.started_ms, cache.completed_ms, elapsed_ms);
    json_writer_append_escaped_string(&writer, vision.label);
    json_writer_appendf(&writer, ",\"object\":");
    json_writer_append_escaped_string(&writer, vision.object);
    json_writer_appendf(&writer, ",\"model\":");
    json_writer_append_escaped_string(&writer, vision.model);
    json_writer_appendf(
        &writer,
        ",\"object_score\":%" PRIu32 ",\"candidate_score\":%" PRIu32
        ",\"box_min_score\":%" PRIu32 ",\"object_count\":%" PRIu32
        ",\"detection_count\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
        ",\"detections\":",
        vision.object_score, vision.candidate_score, vision.box_min_score,
        vision.object_count, vision.detection_count, vision.raw_candidate_count);
    json_writer_append_detections(&writer, &vision);
    json_writer_appendf(&writer, ",\"top_k\":");
    json_writer_append_top_k(&writer, &vision);
    json_writer_appendf(
        &writer,
        ",\"object_x\":%" PRIu32
        ",\"object_y\":%" PRIu32 ",\"object_w\":%" PRIu32
        ",\"object_h\":%" PRIu32 ",\"coke_score\":%" PRIu32
        ",\"sprite_score\":%" PRIu32 ",\"unknown_score\":%" PRIu32
        ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64 "}}",
        vision.object_x, vision.object_y,
        vision.object_w, vision.object_h, vision.coke_score, vision.sprite_score,
        vision.unknown_score, vision.inference_ms, vision.analysis_ms);
    if (!json_writer_ok(&writer)) {
        free(json);
        return validation_json_error(req, "500 Internal Server Error",
                                     "validation result is too large to serialize safely");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t validation_overlay_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    uint32_t requested_id = 0;
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        !query_u32(query, "id", 1, UINT32_MAX, &requested_id)) {
        return validation_json_error(req, "400 Bad Request",
                                     "id is required and must be a positive integer");
    }

    validation_cache_t cache = {0};
    if (s_validation_lock) {
        xSemaphoreTake(s_validation_lock, portMAX_DELAY);
        cache = s_validation_last;
        xSemaphoreGive(s_validation_lock);
    }
    if (!cache.valid) {
        return validation_json_error(req, "404 Not Found", "validation job was not found");
    }
    if (cache.id != requested_id) {
        return validation_json_error(req, "410 Gone",
                                     "validation result was replaced by a newer job; run it again");
    }
    if (cache.state == VALIDATION_JOB_QUEUED || cache.state == VALIDATION_JOB_RUNNING) {
        return validation_json_error(req, "409 Conflict",
                                     "validation result is not ready; poll the status endpoint");
    }
    if (cache.state != VALIDATION_JOB_SUCCEEDED || cache.err != ESP_OK ||
        cache.source_w == 0 || cache.source_h == 0) {
        return validation_json_error(req, "409 Conflict",
                                     "validation failed and has no overlay");
    }

    const uint8_t *jpg_start = NULL;
    const uint8_t *jpg_end = NULL;
    if (!validation_sample_image(cache.sample, &jpg_start, &jpg_end)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "validation image missing");
    }

    size_t jpeg_size = (size_t)(jpg_end - jpg_start);
    return send_overlay_svg_response(req, jpg_start, jpeg_size, cache.source_w, cache.source_h,
                                     &cache.vision);
}

static esp_err_t healthz_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"mode\":\"%s\",\"tf_mounted\":%s,\"storage\":\"%s\"}",
             app_mode_name(s_app_mode),
             s_sd_mounted ? "true" : "false", s_storage_backend);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static const char *field_idle_pause_reason_for_state(
    int64_t now_ms, bool acceptance_ok, uint32_t web_clients,
    uint32_t stream_clients, uint32_t validation_jobs,
    uint32_t download_clients, const dataset_run_status_t *dataset,
    bool inference_busy)
{
    if (s_app_mode != APP_MODE_SERVER || !s_network_active ||
        s_network_shutdown_for_idle) {
        return "当前模式不执行自动采集倒计时";
    }
    if (s_storage_quiescing || storage_transition_active() ||
        storage_any_request_pending()) {
        return "存储正在切换或重试";
    }
    if (!acceptance_ok) {
        return "TF 未通过写入验证";
    }
    if (web_clients > 0) {
        return "Web 客户端在线";
    }
    if (stream_clients > 0) {
        return "实时图传正在使用";
    }
    if (validation_jobs > 0 || (dataset && (dataset->queued || dataset->running))) {
        return "模型验证或数据集分析正在运行";
    }
    if (download_clients > 0) {
        return "文件下载正在运行";
    }
    if (inference_busy) {
        return "推理任务正在运行";
    }
    if (s_network_boot_window_until_ms > now_ms) {
        return "网络启动保护期";
    }
    return NULL;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    record_http_poll(req);
    frame_meta_t meta = {0};
    bool have_frame = copy_latest_meta(&meta);
    if (!have_frame) {
        fill_vision_disabled(&meta.vision);
    }
    char fourcc[5] = "----";
    /*
     * /api/status 的 JSON 字段较多，不能放在 HTTP 任务栈上。
     * ESP-IDF 默认 httpd 任务栈不大，栈上 8KB 缓冲会让服务端接受连接后卡死。
     */
    const size_t json_cap = 12288;
    char *json = (char *)alloc_psram_buffer(json_cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no status buffer");
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    uint64_t epoch_ms = wall_clock_epoch_ms();
    esp_reset_reason_t reset_reason = esp_reset_reason();
    int64_t age_ms = have_frame ? now_ms - meta.timestamp_ms : -1;
    int64_t last_network_activity_ms = __atomic_load_n(
        &s_last_network_activity_ms, __ATOMIC_ACQUIRE);
    int64_t network_idle_ms = last_network_activity_ms > 0 ?
                              now_ms - last_network_activity_ms : -1;
    uint32_t web_clients = web_client_count(now_ms);
    int64_t last_web_client_activity_ms = __atomic_load_n(
        &s_last_web_client_activity_ms, __ATOMIC_ACQUIRE);
    int64_t field_idle_anchor_ms = last_network_activity_ms;
    if (last_web_client_activity_ms > field_idle_anchor_ms) {
        field_idle_anchor_ms = last_web_client_activity_ms;
    }
    int64_t field_idle_ms = field_idle_anchor_ms > 0 ? now_ms - field_idle_anchor_ms : -1;
    int64_t network_boot_remaining_ms = s_network_boot_window_until_ms > now_ms ?
                                        s_network_boot_window_until_ms - now_ms : 0;
    power_state_t state = s_power_state;
    usb_msc_export_status_t usb_status = {0};
    usb_msc_export_get_status(&usb_status);
    bool usb_export_active = s_usb_storage_ready || s_app_mode == APP_MODE_USB_EXPORT;
    bool usb_host_visible = usb_status.host_connected && usb_export_active;
    uint32_t history_count = 0;
    uint32_t free_heap = 0;
    uint32_t min_free_heap = 0;
    uint32_t free_psram = 0;
    uint32_t min_free_psram = 0;
    uint32_t free_dma = (uint32_t)heap_caps_get_free_size(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    uint32_t min_free_dma = (uint32_t)heap_caps_get_minimum_free_size(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    uint32_t largest_dma = (uint32_t)heap_caps_get_largest_free_block(
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    int rssi = wifi_rssi();
    sample_memory_stats(&free_heap, &min_free_heap, &free_psram, &min_free_psram);
    const size_t detections_json_cap = 1280;
    const size_t top_k_json_cap = 512;
    const size_t dataset_labels_json_cap = 512;
    const size_t enrichment_json_cap = 768;
    const size_t resegment_json_cap = 384;
    char *detections_json = (char *)alloc_psram_buffer(detections_json_cap);
    char *top_k_json = (char *)alloc_psram_buffer(top_k_json_cap);
    char *dataset_labels_json = (char *)alloc_psram_buffer(dataset_labels_json_cap);
    char *enrichment_json = (char *)alloc_psram_buffer(enrichment_json_cap);
    char *resegment_json = (char *)alloc_psram_buffer(resegment_json_cap);
    if (!detections_json || !top_k_json || !dataset_labels_json || !enrichment_json || !resegment_json) {
        free(resegment_json);
        free(enrichment_json);
        free(dataset_labels_json);
        free(top_k_json);
        free(detections_json);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no status scratch buffer");
    }
    char ap_url[40] = {0};
    char sta_url[40] = {0};
    char eth_url[40] = {0};
    char mdns_url[64] = {0};
    char access_urls[256] = {0};
    char router_ssid_json[256] = {0};
    char cleanup_message_json[256] = {0};
    const char *primary_ip = s_ip_addr;
    dataset_run_status_t dataset_status;
    recording_enrichment_status_t enrichment_status = {0};
    recording_resegment_status_t resegment_status = {0};
    recording_cleanup_status_t cleanup_status = {0};
    bool status_arrays_ok =
        detections_to_json(detections_json, detections_json_cap, &meta.vision) &&
        top_k_to_json(top_k_json, top_k_json_cap, &meta.vision);
    dataset_status_copy(&dataset_status);
    recording_enrichment_get_status(&enrichment_status);
    resegment_status_copy(&resegment_status);
    recording_cleanup_status_copy(&cleanup_status);
    status_arrays_ok = status_arrays_ok &&
        label_counts_to_json(dataset_labels_json, dataset_labels_json_cap, dataset_status.labels);
    if (!status_arrays_ok) {
        free(resegment_json);
        free(enrichment_json);
        free(dataset_labels_json);
        free(top_k_json);
        free(detections_json);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status arrays exceed safe buffers");
    }
    bool tf_ready = storage_tf_ready();
    bool acceptance_ok = storage_acceptance_ok();
    uint64_t min_recording_free = recording_min_free_bytes();
    stream_stats_snapshot_t stream_stats = {0};
    stream_stats_get_snapshot(&stream_stats);
    uint32_t stream_clients = stream_stats.clients;
    uint32_t validation_jobs = __atomic_load_n(&s_validation_active_jobs, __ATOMIC_ACQUIRE);
    uint32_t download_clients = __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE);
    const char *field_idle_pause_reason = field_idle_pause_reason_for_state(
        now_ms, acceptance_ok, web_clients, stream_clients, validation_jobs,
        download_clients, &dataset_status, inference_worker_busy());
    bool field_idle_paused = field_idle_pause_reason != NULL;
    if (!field_idle_paused &&
        __atomic_load_n(&s_field_idle_pause_latched, __ATOMIC_ACQUIRE)) {
        field_idle_paused = true;
        field_idle_pause_reason = "正在重新开始完整倒计时";
    }
    if (!field_idle_pause_reason) {
        field_idle_pause_reason = "";
    }
    int64_t field_idle_remaining_ms = -1;
    if (s_field_auto_enable && s_field_idle_timeout_ms > 0 && field_idle_ms >= 0) {
        field_idle_remaining_ms = field_idle_paused ? (int64_t)s_field_idle_timeout_ms :
                                  (int64_t)s_field_idle_timeout_ms - field_idle_ms;
        if (field_idle_remaining_ms < 0) {
            field_idle_remaining_ms = 0;
        }
    }
    json_escape_string(router_ssid_json, sizeof(router_ssid_json), s_router_ssid);
    json_escape_string(cleanup_message_json, sizeof(cleanup_message_json),
                       cleanup_status.message);
    snprintf(ap_url, sizeof(ap_url), "http://%s/", s_ap_ip_addr);
    if (strcmp(s_sta_ip_addr, "0.0.0.0") != 0) {
        snprintf(sta_url, sizeof(sta_url), "http://%s/", s_sta_ip_addr);
    }
    if (strcmp(s_eth_ip_addr, "0.0.0.0") != 0) {
        snprintf(eth_url, sizeof(eth_url), "http://%s/", s_eth_ip_addr);
        primary_ip = s_eth_ip_addr;
    }
    if (CONFIG_APP_MDNS_ENABLE && s_mdns_started) {
        snprintf(mdns_url, sizeof(mdns_url), "http://%s.local/", CONFIG_APP_HOSTNAME);
    }
    json_writer_t access_writer;
    json_writer_init(&access_writer, access_urls, sizeof(access_urls));
    json_writer_appendf(&access_writer, "{\"mdns\":");
    json_writer_append_escaped_string(&access_writer, mdns_url);
    json_writer_appendf(&access_writer, ",\"ap\":");
    json_writer_append_escaped_string(&access_writer, ap_url);
    json_writer_appendf(&access_writer, ",\"sta\":");
    json_writer_append_escaped_string(&access_writer, sta_url);
    json_writer_appendf(&access_writer, ",\"eth\":");
    json_writer_append_escaped_string(&access_writer, eth_url);
    json_writer_appendf(&access_writer, "}");

    json_writer_t enrichment_writer;
    json_writer_init(&enrichment_writer, enrichment_json, enrichment_json_cap);
    json_writer_appendf(&enrichment_writer,
                        "{\"enabled\":%s,\"running\":%s,\"cancelled\":%s,\"raw_name\":",
                        enrichment_status.enabled ? "true" : "false",
                        enrichment_status.running ? "true" : "false",
                        enrichment_status.cancelled ? "true" : "false");
    json_writer_append_escaped_string(&enrichment_writer, enrichment_status.raw_name);
    json_writer_appendf(&enrichment_writer, ",\"output_name\":");
    json_writer_append_escaped_string(&enrichment_writer, enrichment_status.output_name);
    json_writer_appendf(&enrichment_writer, ",\"method\":");
    json_writer_append_escaped_string(&enrichment_writer, enrichment_status.method);
    json_writer_appendf(
        &enrichment_writer,
        ",\"pass_stride\":%" PRIu32 ",\"completed_stride\":%" PRIu32
        ",\"frame_index\":%" PRIu32 ",\"frame_count\":%" PRIu32
        ",\"inferred_frames\":%" PRIu32 ",\"output_frames\":%" PRIu32
        ",\"inference_coverage_x1000\":%" PRIu32
        ",\"passes_completed\":%" PRIu32 ",\"last_error\":",
        enrichment_status.pass_stride, enrichment_status.completed_stride,
        enrichment_status.frame_index, enrichment_status.frame_count,
        enrichment_status.inferred_frames, enrichment_status.output_frames,
        enrichment_status.inference_coverage_x1000,
        enrichment_status.passes_completed);
    json_writer_append_escaped_string(&enrichment_writer, enrichment_status.last_error);
    json_writer_appendf(&enrichment_writer, "}");

    json_writer_t resegment_writer;
    json_writer_init(&resegment_writer, resegment_json, resegment_json_cap);
    json_writer_appendf(
        &resegment_writer,
        "{\"requested\":%s,\"running\":%s,\"cancelled\":%s,"
        "\"target_ms\":%" PRIu32 ",\"input_segments\":%" PRIu32
        ",\"processed_segments\":%" PRIu32 ",\"output_segments\":%" PRIu32
        ",\"last_error\":",
        resegment_status.requested ? "true" : "false",
        resegment_status.running ? "true" : "false",
        resegment_status.cancelled ? "true" : "false",
        resegment_status.target_ms, resegment_status.input_segments,
        resegment_status.processed_segments, resegment_status.output_segments);
    json_writer_append_escaped_string(&resegment_writer, resegment_status.last_error);
    json_writer_appendf(&resegment_writer, "}");

    if (!json_writer_ok(&access_writer) || !json_writer_ok(&enrichment_writer) ||
        !json_writer_ok(&resegment_writer)) {
        free(resegment_json);
        free(enrichment_json);
        free(dataset_labels_json);
        free(top_k_json);
        free(detections_json);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status detail exceeds safe buffer");
    }

    if (s_history_lock) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        history_count = s_history_count;
        xSemaphoreGive(s_history_lock);
    }

    if (meta.pixel_format) {
        fourcc_to_str(meta.pixel_format, fourcc);
    }

    int json_len = snprintf(json, json_cap,
             "{"
             "\"ip\":\"%s\",\"target\":\"%s\",\"reset_reason\":\"%s\","
             "\"reset_reason_code\":%d,\"power_mode\":\"%s\","
             "\"app_mode\":\"%s\","
             "\"time_synced\":%s,\"time_source\":\"%s\",\"epoch_ms\":%" PRIu64 ","
             "\"network_mode\":\"%s\",\"recognition_method\":\"%s\",\"rescue_ap\":%s,"
             "\"sta_ip\":\"%s\",\"ap_ip\":\"%s\",\"eth_ip\":\"%s\",\"ap_clients\":%" PRIu32
             ",\"web_clients\":%" PRIu32 ",\"client_count\":%" PRIu32 ","
             "\"wifi_runtime_ready\":%s,\"wifi_started\":%s,\"wifi_init_failures\":%" PRIu32
             ",\"wifi_last_error\":\"%s\","
             "\"network_active\":%s,\"network_idle_ms\":%" PRId64 ","
             "\"eth_active\":%s,"
             "\"network_boot_window_remaining_ms\":%" PRId64 ","
             "\"network_shutdown_for_idle\":%s,\"network_reopen_requires_reboot\":%s,"
             "\"hostname\":\"%s\",\"mdns_url\":\"%s\","
             "\"ap_url\":\"%s\",\"sta_url\":\"%s\",\"eth_url\":\"%s\",\"access_urls\":%s,"
              "\"eth_enabled\":%s,\"eth_started\":%s,\"eth_link_up\":%s,"
              "\"eth_got_ip\":%s,\"eth_static_fallback\":%s,\"eth_last_error\":\"%s\","
              "\"usb_msc_enabled\":%s,\"usb_initialized\":%s,\"usb_host_connected\":%s,"
              "\"usb_bus_active\":%s,\"usb_sof_age_ms\":%" PRIu32 ","
              "\"usb_export_requested\":%s,\"usb_restore_requested\":%s,"
              "\"storage_retry_requested\":%s,"
              "\"usb_storage_owner\":\"%s\",\"usb_writable\":%s,"
              "\"storage_quiescing\":%s,\"file_download_clients\":%" PRIu32
              ",\"usb_last_error\":\"%s\",\"enrichment\":%s,\"resegment\":%s,"
              "\"recording_cleanup\":{\"state\":\"%s\",\"queued\":%s,"
              "\"running\":%s,\"done\":%s,\"ok\":%s,\"job_id\":%" PRIu32
              ",\"total_files\":%" PRIu32 ",\"deleted_files\":%" PRIu32
              ",\"remaining_files\":%" PRIu32 ",\"errors\":%" PRIu32
              ",\"errno\":%d,\"freed_bytes\":%" PRIu64
              ",\"queued_ms\":%" PRId64 ",\"started_ms\":%" PRId64
              ",\"finished_ms\":%" PRId64 ",\"message\":\"%s\"},"
             "\"config\":{\"router_ssid\":\"%s\",\"router_password_set\":%s,"
             "\"recording_segment_ms\":%" PRIu32 ",\"recording_segment_max_ms\":%" PRIu32
             ",\"field_idle_timeout_ms\":%" PRIu32
             ",\"field_auto_enable\":%s,\"field_idle_remaining_ms\":%" PRId64
             ",\"field_idle_paused\":%s,\"field_idle_pause_reason\":\"%s\","
             "\"box_min_score\":%" PRIu32 ",\"stream_max_fps\":%" PRIu32 ","
             "\"inference_interval_ms\":%" PRIu32 ",\"history_sample_interval_ms\":%" PRIu32 ","
             "\"jpeg_quality\":%" PRIu32 ","
             "\"yolo_input_size\":%" PRIu32 ",\"yolo_model_loaded\":%s,"
              "\"yolo26_available\":%s,\"yolo11_available\":%s,\"coco_available\":%s,"
              "\"tinycls_available\":%s,\"fish31_available\":%s},"
             "\"camera_ready\":%s,\"video_hw_ready\":%s,"
             "\"camera_error\":\"%s\",\"vision_enabled\":%s,\"history_enabled\":%s,"
             "\"width\":%" PRIu32 ",\"height\":%" PRIu32 ",\"pixel_format\":\"%s\","
             "\"sensor_frame_rate\":%" PRIu32 ",\"capture_fps_x100\":%" PRIu32 ",\"stream_fps_x100\":%" PRIu32 ","
             "\"last_capture_ms\":%" PRId64 ",\"last_encode_ms\":%" PRId64 ","
             "\"last_frame_age_ms\":%" PRId64 ",\"last_jpeg_bytes\":%" PRIu32 ",\"frame_seq\":%" PRIu32 ","
             "\"frames\":%" PRIu32 ",\"capture_errors\":%" PRIu32 ",\"frame_drops\":%" PRIu32 ","
             "\"stream_clients\":%" PRIu32 ",\"max_stream_clients\":%d,\"stream_errors\":%" PRIu32 ","
             "\"stream_frames\":%" PRIu64 ",\"stream_bytes\":%" PRIu64 ","
             "\"inference_frames\":%" PRIu32 ",\"inference_fps_x100\":%" PRIu32 ","
             "\"dropped_inference_frames\":%" PRIu32 ",\"inference_busy\":%s,"
             "\"validation_active_jobs\":%" PRIu32 ","
             "\"inference_queue_depth\":%" PRIu32 ",\"inference_jobs_queued\":%" PRIu32 ","
             "\"inference_jobs_completed\":%" PRIu32 ",\"inference_queue_drops\":%" PRIu32 ","
             "\"model_bytes\":%" PRIu32 ","
             "\"model_info\":{\"name\":\"%s\",\"bytes\":%" PRIu32 ",\"input_size\":%" PRIu32 ","
             "\"class_count\":%" PRIu32 ",\"max_detections\":%" PRIu32 ",\"nms_threshold\":%" PRIu32 "},"
             "\"requests\":%" PRIu32 ",\"standby_requests\":%" PRIu32 ",\"wake_requests\":%" PRIu32 ","
             "\"reconnect_count\":%" PRIu32 ",\"netmode_switches\":%" PRIu32 ","
             "\"history_count\":%" PRIu32 ",\"history_saved\":%" PRIu32 ",\"history_queued\":%" PRIu32 ","
             "\"history_dropped\":%" PRIu32 ",\"history_deleted\":%" PRIu32 ",\"history_sd_errors\":%" PRIu32 ","
             "\"recording_enabled\":%s,\"recording_segments\":%" PRIu32 ",\"recording_frames\":%" PRIu32 ","
             "\"recording_queued\":%" PRIu32 ",\"recording_dropped\":%" PRIu32 ","
             "\"recording_deleted\":%" PRIu32 ",\"recording_sd_errors\":%" PRIu32 ","
             "\"recording_zero_frame_archives\":%" PRIu32 ","
             "\"recording_summary_count\":%" PRIu32 ",\"recording_bytes\":%" PRIu64 ","
             "\"recording_current_uri\":\"%s\",\"recording_current_frames\":%" PRIu32 ","
             "\"recording_current_bytes\":%" PRIu64 ","
             "\"sd_mounted\":%s,\"file_storage_mounted\":%s,\"tf_card_mounted\":%s,"
             "\"tf_required\":true,\"tf_ready\":%s,\"storage_acceptance_ok\":%s,"
             "\"storage_write_verified\":%s,\"storage_write_verified_at_ms\":%" PRId64 ","
             "\"storage_io_latched\":%s,\"storage_last_errno\":%d,"
             "\"storage_index_version\":%" PRIu32 ",\"tf_min_accept_bytes\":%" PRIu64 ","
             "\"recording_min_free_bytes\":%" PRIu64 ","
             "\"storage_backend\":\"%s\",\"storage_status\":\"%s\",\"sd_mount_mode\":\"%s\","
             "\"sd_last_error\":\"%s\",\"sd_last_error_code\":%d,\"sd_attempts\":%" PRIu32
             ",\"sd_format_count\":%" PRIu32 ","
             "\"storage_service\":{\"mode\":\"%s\",\"status\":\"%s\",\"runs\":%" PRIu32
             ",\"last_mount_ok\":%s,\"last_mode\":\"%s\",\"last_error_code\":%d},"
             "\"sd_total_bytes\":%" PRIu64 ",\"sd_free_bytes\":%" PRIu64 ","
             "\"dataset_run\":{\"state\":\"%s\",\"queued\":%s,\"running\":%s,\"done\":%s,"
             "\"dataset\":\"%s\",\"method\":\"%s\",\"run_id\":\"%s\","
             "\"processed\":%" PRIu32 ",\"ok_frames\":%" PRIu32 ",\"failed_frames\":%" PRIu32
             ",\"detection_total\":%" PRIu32 ",\"avg_analysis_ms\":%" PRIu32
             ",\"p95_analysis_ms\":%" PRIu32 ",\"max_analysis_ms\":%" PRIu32
             ",\"last_frame_index\":%" PRIu32 ",\"last_overlay_uri\":\"%s\","
             "\"labels\":%s,\"error\":\"%s\"},"
             "\"uptime_ms\":%" PRId64 ",\"rssi_dbm\":%d,"
             "\"free_heap\":%" PRIu32 ",\"min_free_heap\":%" PRIu32 ","
             "\"free_psram\":%" PRIu32 ",\"min_free_psram\":%" PRIu32 ","
             "\"free_internal_dma\":%" PRIu32 ",\"min_free_internal_dma\":%" PRIu32
             ",\"largest_internal_dma_block\":%" PRIu32 ","
             "\"vision\":{\"label\":\"%s\",\"object\":\"%s\",\"scene\":\"%s\",\"color\":\"%s\",\"model\":\"%s\",\"motion\":%s,"
             "\"motion_score\":%" PRIu32 ",\"edge_score\":%" PRIu32 ",\"avg_luma\":%" PRIu32 ","
             "\"avg_r\":%" PRIu32 ",\"avg_g\":%" PRIu32 ",\"avg_b\":%" PRIu32 ","
             "\"object_score\":%" PRIu32 ",\"candidate_score\":%" PRIu32 ",\"box_min_score\":%" PRIu32 ","
             "\"object_count\":%" PRIu32 ",\"detection_count\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32 ","
             "\"detections\":%s,\"top_k\":%s,"
             "\"object_x\":%" PRIu32 ",\"object_y\":%" PRIu32 ",\"object_w\":%" PRIu32 ",\"object_h\":%" PRIu32 ","
             "\"coke_score\":%" PRIu32 ",\"sprite_score\":%" PRIu32 ","
             "\"unknown_score\":%" PRIu32 ","
             "\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64 "}"
             "}",
             primary_ip, CONFIG_IDF_TARGET, reset_reason_name(reset_reason),
             (int)reset_reason, power_state_name(state),
             app_mode_name(s_app_mode),
             epoch_ms > 0 && s_time_source != TIME_SOURCE_UNSYNCED ? "true" : "false",
             time_source_name(s_time_source), epoch_ms,
             network_mode_name(s_network_mode), recognition_method_name(s_recognition_method),
             s_rescue_ap_active ? "true" : "false",
             s_sta_ip_addr, s_ap_ip_addr, s_eth_ip_addr, s_ap_clients,
             web_clients, web_clients,
             s_wifi_runtime_ready ? "true" : "false",
             s_wifi_started ? "true" : "false",
             s_wifi_init_failures, s_wifi_last_error,
             s_network_active ? "true" : "false", network_idle_ms,
             s_eth_started && network_idle_ms >= 0 && network_idle_ms < 10000 ? "true" : "false",
             network_boot_remaining_ms,
             s_network_shutdown_for_idle ? "true" : "false",
             s_network_shutdown_for_idle ? "true" : "false",
             CONFIG_APP_HOSTNAME, mdns_url,
             ap_url, sta_url, eth_url, access_urls,
             CONFIG_APP_ETH_ENABLE ? "true" : "false",
             s_eth_started ? "true" : "false",
             s_eth_link_up ? "true" : "false",
             s_eth_got_ip ? "true" : "false",
              s_eth_static_fallback ? "true" : "false",
              s_eth_last_error,
              CONFIG_APP_USB_MSC_ENABLE ? "true" : "false",
              usb_status.initialized ? "true" : "false",
                usb_host_visible ? "true" : "false",
                (usb_status.bus_active && usb_export_active) ? "true" : "false",
                usb_status.last_sof_age_ms,
               storage_request_pending(&s_usb_export_requested) ? "true" : "false",
               storage_request_pending(&s_usb_restore_requested) ? "true" : "false",
               storage_request_pending(&s_storage_retry_requested) ? "true" : "false",
               s_usb_storage_ready ? "usb" : (s_sd_mounted ? "app" : "none"),
               usb_status.writable ? "true" : "false",
               s_storage_quiescing ? "true" : "false",
               __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE),
               s_usb_last_error, enrichment_json, resegment_json,
               recording_cleanup_state_name(cleanup_status.state),
               cleanup_status.state == RECORDING_CLEANUP_QUEUED ? "true" : "false",
               cleanup_status.state == RECORDING_CLEANUP_RUNNING ? "true" : "false",
               (cleanup_status.state == RECORDING_CLEANUP_SUCCEEDED ||
                cleanup_status.state == RECORDING_CLEANUP_FAILED) ? "true" : "false",
               cleanup_status.state == RECORDING_CLEANUP_SUCCEEDED ? "true" : "false",
               cleanup_status.job_id, cleanup_status.total_files,
               cleanup_status.deleted_files, cleanup_status.remaining_files,
               cleanup_status.error_count, cleanup_status.first_errno,
               cleanup_status.freed_bytes, cleanup_status.queued_ms,
               cleanup_status.started_ms, cleanup_status.finished_ms,
               cleanup_message_json,
              router_ssid_json, s_router_password[0] ? "true" : "false",
              s_recording_segment_ms, (uint32_t)APP_RECORDING_SEGMENT_MAX_MS,
              s_field_idle_timeout_ms,
             s_field_auto_enable ? "true" : "false", field_idle_remaining_ms,
             field_idle_paused ? "true" : "false", field_idle_pause_reason,
              s_box_min_score, s_stream_max_fps, (unsigned long)s_inference_interval_ms,
             s_history_sample_interval_ms, s_jpeg_quality,
             (unsigned long)active_yolo_input_size(), active_yolo_available() ? "true" : "false",
             yolo26_espdl_available() ? "true" : "false",
              yolo11_espdl_available() ? "true" : "false",
              coco_espdl_available() ? "true" : "false",
              tiny_cls_espdl_available() ? "true" : "false",
              fish31_espdl_available() ? "true" : "false",
              state == POWER_STATE_RUNNING ? "true" : "false",
             s_video_hw_ready ? "true" : "false",
             s_camera_error, s_vision_enabled ? "true" : "false", s_history_enabled ? "true" : "false",
             meta.width, meta.height, fourcc, meta.sensor_fps, s_capture_fps_x100, stream_stats.fps_x100,
             meta.capture_ms, meta.encode_ms, age_ms, meta.size, meta.seq,
             s_frames_total, s_capture_errors, s_frame_drops,
             stream_clients, CONFIG_APP_MAX_STREAM_CLIENTS, stream_stats.errors,
             stream_stats.frames_total, stream_stats.bytes_total,
             s_inference_frames_total, s_inference_fps_x100,
             s_inference_dropped_frames, s_inference_worker_busy ? "true" : "false",
             validation_jobs,
             (uint32_t)(s_inference_queue ? uxQueueMessagesWaiting(s_inference_queue) : 0),
             s_inference_jobs_queued, s_inference_jobs_completed, s_inference_queue_drops,
             active_model_bytes(),
             model_name_for_method(s_recognition_method), active_model_bytes(),
             model_input_size_for_method(s_recognition_method),
             model_class_count_for_method(s_recognition_method), (uint32_t)APP_MAX_DETECTIONS,
             (uint32_t)APP_YOLO_NMS_THRESHOLD_X100,
             s_requests, s_standby_requests, s_wake_requests,
             s_reconnect_count, s_netmode_switches,
             history_count, s_history_saved, s_history_queued,
             s_history_dropped, s_history_files_deleted, s_history_sd_errors,
             s_recording_enabled ? "true" : "false", s_recording_segments, s_recording_frames,
             s_recording_queued, s_recording_dropped,
             s_recording_files_deleted, s_recording_sd_errors,
             s_recording_zero_frame_archives,
             s_recording_summary_count, (uint64_t)s_recording_bytes,
             s_recording_current_uri, s_recording_current_frames,
             (uint64_t)s_recording_current_bytes,
             s_sd_mounted ? "true" : "false", s_sd_mounted ? "true" : "false",
             (s_sd_mounted && s_sd_card) ? "true" : "false",
             tf_ready ? "true" : "false", acceptance_ok ? "true" : "false",
             s_storage_write_verified ? "true" : "false", s_storage_write_verified_ms,
             s_storage_io_latched ? "true" : "false", s_storage_last_errno,
             (uint32_t)APP_JSONL_INDEX_VERSION, (uint64_t)0,
             min_recording_free,
             s_storage_backend, s_storage_status, s_sd_mount_mode,
             s_sd_last_error, s_sd_last_error_code, s_sd_attempts, s_sd_format_count,
             storage_service_mode_name(s_storage_service_mode), s_storage_service_status,
             s_storage_service_runs, s_storage_service_last_mount_ok ? "true" : "false",
             s_storage_service_last_mode, s_storage_service_last_error_code,
             s_sd_total_bytes, s_sd_free_bytes,
             dataset_status.queued ? "queued" :
                 (dataset_status.running ? "running" : (dataset_status.done ? "done" : "idle")),
             dataset_status.queued ? "true" : "false",
             dataset_status.running ? "true" : "false", dataset_status.done ? "true" : "false",
             dataset_status.dataset, recognition_method_name(dataset_status.method), dataset_status.run_id,
             dataset_status.processed, dataset_status.ok_frames, dataset_status.failed_frames,
             dataset_status.detection_total, dataset_status.avg_analysis_ms,
             dataset_status.p95_analysis_ms, dataset_status.max_analysis_ms,
             dataset_status.last_frame_index, dataset_status.last_overlay_uri,
             dataset_labels_json, dataset_status.last_error,
             now_ms, rssi,
             free_heap, min_free_heap, free_psram, min_free_psram,
             free_dma, min_free_dma, largest_dma,
             meta.vision.label, meta.vision.object, meta.vision.scene, meta.vision.color, meta.vision.model,
             meta.vision.motion ? "true" : "false",
             meta.vision.motion_score, meta.vision.edge_score, meta.vision.avg_luma,
             meta.vision.avg_r, meta.vision.avg_g, meta.vision.avg_b,
             meta.vision.object_score, meta.vision.candidate_score, meta.vision.box_min_score,
             meta.vision.object_count, meta.vision.detection_count, meta.vision.raw_candidate_count,
             detections_json, top_k_json,
             meta.vision.object_x, meta.vision.object_y, meta.vision.object_w, meta.vision.object_h,
             meta.vision.coke_score, meta.vision.sprite_score,
             meta.vision.unknown_score,
             meta.vision.inference_ms,
             meta.vision.analysis_ms);

    if (json_len < 0 || (size_t)json_len >= json_cap) {
        free(enrichment_json);
        free(resegment_json);
        free(dataset_labels_json);
        free(top_k_json);
        free(detections_json);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "status response exceeds safe buffer");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(enrichment_json);
    free(resegment_json);
    free(dataset_labels_json);
    free(top_k_json);
    free(detections_json);
    free(json);
    return ret;
}

static esp_err_t frame_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (export_mode_reject(req, "frame capture")) {
        return ESP_OK;
    }
    uint8_t *client_buf = alloc_psram_buffer(s_frame_capacity);
    if (!client_buf) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no frame buffer");
    }

    frame_meta_t meta = {0};
    if (!copy_latest_frame(client_buf, s_frame_capacity, &meta)) {
        free(client_buf);
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "no frame available");
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_buffer_chunked(req, (const char *)client_buf, meta.size);
    free(client_buf);
    return ret;
}

static esp_err_t vision_api_handler(httpd_req_t *req)
{
    if (req->method != HTTP_POST) {
        record_http_request(req);
        return reject_non_post_method(req);
    }
    return config_get_handler(req);
}

static esp_err_t recognition_api_handler(httpd_req_t *req)
{
    if (req->method != HTTP_POST) {
        record_http_request(req);
        return reject_non_post_method(req);
    }
    return config_get_handler(req);
}

static bool query_u32(const char *query, const char *key, uint32_t min_value,
                      uint32_t max_value, uint32_t *out_value)
{
    char text[24] = {0};
    if (!query || httpd_query_key_value(query, key, text, sizeof(text)) != ESP_OK) {
        return false;
    }
    if (!text[0] || text[0] == '-') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long value = strtoul(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0' || value > UINT32_MAX ||
        value < min_value || value > max_value) {
        return false;
    }
    *out_value = (uint32_t)value;
    return true;
}

static esp_err_t read_optional_url_query(httpd_req_t *req, char *query,
                                         size_t query_size, bool *present)
{
    if (!req || !query || query_size == 0 || !present) {
        return ESP_ERR_INVALID_ARG;
    }
    *present = false;
    query[0] = '\0';
    size_t length = httpd_req_get_url_query_len(req);
    if (length == 0) {
        return ESP_OK;
    }
    if (length >= query_size) {
        return ESP_ERR_HTTPD_RESULT_TRUNC;
    }
    esp_err_t ret = httpd_req_get_url_query_str(req, query, query_size);
    if (ret == ESP_OK) {
        *present = true;
    }
    return ret;
}

static bool query_contains_key(const char *query, const char *key)
{
    if (!query || !key || !key[0]) {
        return false;
    }
    size_t key_len = strlen(key);
    const char *cursor = query;
    while (*cursor) {
        const char *end = strchr(cursor, '&');
        if (!end) {
            end = cursor + strlen(cursor);
        }
        const char *equals = memchr(cursor, '=', (size_t)(end - cursor));
        if (equals && (size_t)(equals - cursor) == key_len &&
            memcmp(cursor, key, key_len) == 0) {
            return true;
        }
        cursor = *end ? end + 1 : end;
    }
    return false;
}

static int form_hex_value(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }
    if (ch >= 'a' && ch <= 'f') {
        return ch - 'a' + 10;
    }
    if (ch >= 'A' && ch <= 'F') {
        return ch - 'A' + 10;
    }
    return -1;
}

static esp_err_t form_url_decode(const char *encoded, char *decoded, size_t decoded_size)
{
    if (!encoded || !decoded || decoded_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t out = 0;
    for (size_t in = 0; encoded[in] != '\0'; in++) {
        unsigned char value = (unsigned char)encoded[in];
        if (value == '+') {
            value = ' ';
        } else if (value == '%') {
            if (encoded[in + 1] == '\0' || encoded[in + 2] == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
            int high = form_hex_value(encoded[in + 1]);
            int low = high >= 0 ? form_hex_value(encoded[in + 2]) : -1;
            if (high < 0 || low < 0) {
                return ESP_ERR_INVALID_ARG;
            }
            value = (unsigned char)((high << 4) | low);
            in += 2;
            if (value == '\0') {
                return ESP_ERR_INVALID_ARG;
            }
        }
        if (out + 1 >= decoded_size) {
            return ESP_ERR_HTTPD_RESULT_TRUNC;
        }
        decoded[out++] = (char)value;
    }
    decoded[out] = '\0';
    return ESP_OK;
}

static esp_err_t form_query_key_value(const char *query, const char *key,
                                      char *value, size_t value_size)
{
    char encoded[289] = {0};
    esp_err_t ret = httpd_query_key_value(query, key, encoded, sizeof(encoded));
    if (ret != ESP_OK) {
        return ret;
    }
    return form_url_decode(encoded, value, value_size);
}

static bool query_i64(const char *query, const char *key, int64_t min_value,
                      int64_t max_value, int64_t *out_value)
{
    char text[32] = {0};
    if (!query || !out_value ||
        httpd_query_key_value(query, key, text, sizeof(text)) != ESP_OK || !text[0]) {
        return false;
    }
    errno = 0;
    char *end = NULL;
    long long value = strtoll(text, &end, 10);
    if (errno == ERANGE || !end || *end != '\0' ||
        value < min_value || value > max_value) {
        return false;
    }
    *out_value = (int64_t)value;
    return true;
}

static esp_err_t reject_non_post_method(httpd_req_t *req)
{
    httpd_resp_set_status(req, "405 Method Not Allowed");
    httpd_resp_set_hdr(req, "Allow", "POST");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(
        req, "{\"ok\":false,\"error\":\"this operation requires POST\"}");
}

static esp_err_t reject_get_for_mutation_handler(httpd_req_t *req)
{
    record_http_request(req);
    const char *allow = req->user_ctx ? (const char *)req->user_ctx : "POST";
    httpd_resp_set_status(req, "405 Method Not Allowed");
    httpd_resp_set_hdr(req, "Allow", allow);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(
        req, "{\"ok\":false,\"error\":\"GET is not allowed for this operation\"}");
}

static bool config_bool_text_valid(const char *text)
{
    return text && (strcmp(text, "0") == 0 || strcmp(text, "1") == 0 ||
                    strcmp(text, "false") == 0 || strcmp(text, "true") == 0 ||
                    strcmp(text, "off") == 0 || strcmp(text, "on") == 0);
}

static bool config_bool_from_text(const char *text)
{
    return text && (strcmp(text, "1") == 0 || strcmp(text, "true") == 0 ||
                    strcmp(text, "on") == 0);
}

typedef struct {
    recognition_method_t recognition_method;
    network_mode_t network_mode;
    bool vision_enabled;
    bool history_enabled;
    bool recording_enabled;
    bool field_auto_enable;
    uint32_t box_min_score;
    uint32_t stream_max_fps;
    uint32_t inference_interval_ms;
    uint32_t history_sample_interval_ms;
    uint32_t jpeg_quality;
    uint32_t recording_segment_ms;
    uint32_t field_idle_timeout_ms;
    char router_ssid[33];
    char router_password[65];
} runtime_config_snapshot_t;

static void runtime_config_snapshot(runtime_config_snapshot_t *config)
{
    if (!config) {
        return;
    }
    *config = (runtime_config_snapshot_t) {
        .recognition_method = s_recognition_method,
        .network_mode = s_network_mode,
        .vision_enabled = s_vision_enabled,
        .history_enabled = s_history_enabled,
        .recording_enabled = s_recording_enabled,
        .field_auto_enable = s_field_auto_enable,
        .box_min_score = s_box_min_score,
        .stream_max_fps = s_stream_max_fps,
        .inference_interval_ms = s_inference_interval_ms,
        .history_sample_interval_ms = s_history_sample_interval_ms,
        .jpeg_quality = s_jpeg_quality,
        .recording_segment_ms = s_recording_segment_ms,
        .field_idle_timeout_ms = s_field_idle_timeout_ms,
    };
    strlcpy(config->router_ssid, s_router_ssid, sizeof(config->router_ssid));
    strlcpy(config->router_password, s_router_password,
            sizeof(config->router_password));
}

static esp_err_t persist_runtime_config(const runtime_config_snapshot_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    runtime_config_blob_t blob = {
        .magic = RUNTIME_CONFIG_BLOB_MAGIC,
        .format_version = RUNTIME_CONFIG_BLOB_VERSION,
        .blob_size = sizeof(runtime_config_blob_t),
        .recognition_method = (uint8_t)config->recognition_method,
        .network_mode = (uint8_t)config->network_mode,
        .flags = (config->vision_enabled ? RUNTIME_CONFIG_FLAG_VISION : 0) |
                 (config->history_enabled ? RUNTIME_CONFIG_FLAG_HISTORY : 0) |
                 (config->recording_enabled ? RUNTIME_CONFIG_FLAG_RECORDING : 0) |
                 (config->field_auto_enable ? RUNTIME_CONFIG_FLAG_FIELD_AUTO : 0),
        .box_min_score = config->box_min_score,
        .stream_max_fps = config->stream_max_fps,
        .inference_interval_ms = config->inference_interval_ms,
        .history_sample_interval_ms = config->history_sample_interval_ms,
        .jpeg_quality = config->jpeg_quality,
        .recording_segment_ms = config->recording_segment_ms,
        .field_idle_timeout_ms = config->field_idle_timeout_ms,
    };
    strlcpy(blob.router_ssid, config->router_ssid, sizeof(blob.router_ssid));
    strlcpy(blob.router_password, config->router_password,
            sizeof(blob.router_password));

    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = runtime_config_commit_blob(nvs, &blob);
    nvs_close(nvs);
    return ret;
}

static int config_key_index(const char *key, size_t key_len)
{
    static const struct {
        const char *key;
        uint8_t semantic_index;
    } keys[] = {
        {"method", 0},
        {"vision", 1}, {"enabled", 1},
        {"history", 2},
        {"recording", 3},
        {"field_auto_enable", 4},
        {"network_mode", 5}, {"netmode", 5}, {"mode", 5},
        {"router_ssid", 6}, {"wifi_ssid", 6},
        {"router_password", 7}, {"wifi_password", 7},
        {"box_min_score", 8}, {"box_min", 8},
        {"stream_max_fps", 9}, {"stream_fps", 9},
        {"inference_interval_ms", 10}, {"inference_ms", 10}, {"inf_ms", 10},
        {"history_sample_interval_ms", 11}, {"history_ms", 11},
        {"jpeg_quality", 12}, {"jpeg_q", 12},
        {"recording_segment_ms", 13}, {"segment_ms", 13},
        {"field_idle_timeout_ms", 14}, {"idle_ms", 14},
    };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        if (strlen(keys[i].key) == key_len &&
            memcmp(keys[i].key, key, key_len) == 0) {
            return (int)keys[i].semantic_index;
        }
    }
    return -1;
}

static esp_err_t validate_config_parameter_syntax(httpd_req_t *req, const char *query)
{
    const char *cursor = query;
    uint64_t seen_keys = 0;
    while (cursor && *cursor) {
        const char *end = strchr(cursor, '&');
        if (!end) {
            end = cursor + strlen(cursor);
        }
        const char *equals = memchr(cursor, '=', (size_t)(end - cursor));
        int key_index = equals && equals != cursor ?
                        config_key_index(cursor, (size_t)(equals - cursor)) : -1;
        if (key_index < 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "unknown or malformed config parameter");
            return ESP_ERR_INVALID_ARG;
        }
        uint64_t key_bit = 1ULL << (unsigned)key_index;
        if ((seen_keys & key_bit) != 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "duplicate config parameter is not allowed");
            return ESP_ERR_INVALID_ARG;
        }
        seen_keys |= key_bit;

        size_t encoded_len = (size_t)(end - equals - 1);
        size_t max_encoded_len = 95;
        size_t key_len = (size_t)(equals - cursor);
        if ((key_len == strlen("router_password") &&
             memcmp(cursor, "router_password", key_len) == 0) ||
            (key_len == strlen("wifi_password") &&
             memcmp(cursor, "wifi_password", key_len) == 0)) {
            max_encoded_len = 3U * 64U;
        } else if ((key_len == strlen("router_ssid") &&
                    memcmp(cursor, "router_ssid", key_len) == 0) ||
                   (key_len == strlen("wifi_ssid") &&
                    memcmp(cursor, "wifi_ssid", key_len) == 0)) {
            max_encoded_len = 3U * 32U;
        }
        if (encoded_len > max_encoded_len) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "config parameter is too long");
            return ESP_ERR_INVALID_ARG;
        }
        cursor = *end ? end + 1 : end;
    }
    return ESP_OK;
}

static esp_err_t validate_config_query(httpd_req_t *req, const char *query)
{
    if (!query || !query[0]) {
        return ESP_OK;
    }
    esp_err_t syntax_ret = validate_config_parameter_syntax(req, query);
    if (syntax_ret != ESP_OK) {
        return syntax_ret;
    }
    struct numeric_setting {
        const char *key;
        uint32_t min_value;
        uint32_t max_value;
    } numeric[] = {
        {"box_min_score", 50, 100}, {"box_min", 50, 100},
        {"stream_max_fps", 1, 30}, {"stream_fps", 1, 30},
        {"inference_interval_ms", 0, 600000}, {"inference_ms", 0, 600000},
        {"inf_ms", 0, 600000},
        {"history_sample_interval_ms", 250, 600000}, {"history_ms", 250, 600000},
        {"jpeg_quality", 1, 100}, {"jpeg_q", 1, 100},
        {"recording_segment_ms", APP_RECORDING_SEGMENT_MIN_MS, APP_RECORDING_SEGMENT_MAX_MS},
        {"segment_ms", APP_RECORDING_SEGMENT_MIN_MS, APP_RECORDING_SEGMENT_MAX_MS},
        {"field_idle_timeout_ms", APP_FIELD_IDLE_TIMEOUT_MIN_MS, APP_FIELD_IDLE_TIMEOUT_MAX_MS},
        {"idle_ms", APP_FIELD_IDLE_TIMEOUT_MIN_MS, APP_FIELD_IDLE_TIMEOUT_MAX_MS},
    };
    char value[96];
    for (size_t i = 0; i < sizeof(numeric) / sizeof(numeric[0]); i++) {
        if (httpd_query_key_value(query, numeric[i].key, value, sizeof(value)) == ESP_OK) {
            uint32_t parsed = 0;
            if (!query_u32(query, numeric[i].key, numeric[i].min_value,
                           numeric[i].max_value, &parsed)) {
                char message[128];
                snprintf(message, sizeof(message), "%s must be an integer in range %" PRIu32 "..%" PRIu32,
                         numeric[i].key, numeric[i].min_value, numeric[i].max_value);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, message);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    const char *bool_keys[] = {"vision", "enabled", "history", "recording", "field_auto_enable"};
    for (size_t i = 0; i < sizeof(bool_keys) / sizeof(bool_keys[0]); i++) {
        if (httpd_query_key_value(query, bool_keys[i], value, sizeof(value)) == ESP_OK &&
            !config_bool_text_valid(value)) {
            char message[96];
            snprintf(message, sizeof(message), "%s must be true/false or 1/0", bool_keys[i]);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, message);
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (httpd_query_key_value(query, "network_mode", value, sizeof(value)) == ESP_OK ||
        httpd_query_key_value(query, "netmode", value, sizeof(value)) == ESP_OK ||
        httpd_query_key_value(query, "mode", value, sizeof(value)) == ESP_OK) {
        if (strcmp(value, "sta") != 0 && strcmp(value, "softap") != 0 &&
            strcmp(value, "ap") != 0 && strcmp(value, "apsta") != 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "network_mode must be sta, softap, or apsta");
            return ESP_ERR_INVALID_ARG;
        }
    }
    esp_err_t ssid_ret = form_query_key_value(query, "router_ssid", value, sizeof(value));
    if (ssid_ret == ESP_ERR_NOT_FOUND) {
        ssid_ret = form_query_key_value(query, "wifi_ssid", value, sizeof(value));
    }
    if (ssid_ret != ESP_OK && ssid_ret != ESP_ERR_NOT_FOUND) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "router_ssid has invalid form encoding or is too long");
        return ESP_ERR_INVALID_ARG;
    }
    if (ssid_ret == ESP_OK && strlen(value) > 32) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "router_ssid must be 32 bytes or shorter");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t pass_ret = form_query_key_value(query, "router_password", value, sizeof(value));
    if (pass_ret == ESP_ERR_NOT_FOUND) {
        pass_ret = form_query_key_value(query, "wifi_password", value, sizeof(value));
    }
    if (pass_ret != ESP_OK && pass_ret != ESP_ERR_NOT_FOUND) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "router_password has invalid form encoding or is too long");
        return ESP_ERR_INVALID_ARG;
    }
    if (pass_ret == ESP_OK) {
        size_t pass_len = strlen(value);
        if (pass_len > 64 || (pass_len > 0 && pass_len < 8)) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "router_password must be empty or 8..64 bytes");
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[1024] = {0};
    char text[96] = {0};
    size_t url_query_len = httpd_req_get_url_query_len(req);
    if (url_query_len >= sizeof(query)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "config URL query is too long");
    }
    bool has_query = false;
    if (url_query_len > 0) {
        esp_err_t query_ret = httpd_req_get_url_query_str(req, query, sizeof(query));
        if (query_ret != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "config URL query could not be read");
        }
        has_query = true;
    }
    bool url_has_query = has_query;
    bool credentials_in_url = has_query &&
        (query_contains_key(query, "router_password") ||
         query_contains_key(query, "wifi_password"));
    if (req->method == HTTP_GET && has_query) {
        httpd_resp_set_status(req, "405 Method Not Allowed");
        httpd_resp_set_hdr(req, "Allow", "POST");
        return httpd_resp_sendstr(req, "GET /api/config is read-only; submit changes with POST");
    }
    if (req->method == HTTP_POST && url_has_query && credentials_in_url) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "send Wi-Fi credentials in the POST body, not the URL");
    }
    if (req->method == HTTP_POST &&
        (s_storage_quiescing || storage_transition_active())) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "settings cannot be changed while storage maintenance is starting or running",
            "wait for the page to reconnect and save the settings again");
    }

    size_t query_len = has_query ? strlen(query) : 0;
    if (req->method == HTTP_POST && req->content_len > 0) {
        size_t prefix_len = query_len + (query_len ? 1U : 0U);
        if (prefix_len >= sizeof(query) ||
            req->content_len >= sizeof(query) - prefix_len) {
            return httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE,
                                       "config request too large");
        }
        char *body = query + query_len;
        if (query_len > 0) {
            *body++ = '&';
            query_len++;
        }
        size_t received = 0;
        while (received < req->content_len) {
            int ret = httpd_req_recv(req, body + received,
                                     req->content_len - received);
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_set_status(req, "408 Request Timeout");
                return httpd_resp_sendstr(req, "config request body timed out");
            }
            if (ret <= 0) {
                return ESP_FAIL;
            }
            received += (size_t)ret;
        }
        body[received] = '\0';
        has_query = query[0] != '\0';
    }
    if (has_query) {
        esp_err_t validation_ret = validate_config_query(req, query);
        if (validation_ret != ESP_OK) {
            return validation_ret;
        }
    }
    runtime_config_snapshot_t current;
    runtime_config_snapshot_t candidate;
    runtime_config_snapshot(&current);
    candidate = current;

    if (has_query && httpd_query_key_value(query, "method", text, sizeof(text)) == ESP_OK) {
        if (strcmp(text, "off") == 0) {
            candidate.recognition_method = RECOGNITION_METHOD_OFF;
            candidate.vision_enabled = false;
        } else if (strcmp(text, "fish31") == 0 || strcmp(text, "fish") == 0) {
            if (!fish31_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "fish31 board backend is unavailable");
            }
            candidate.recognition_method = RECOGNITION_METHOD_FISH31;
            candidate.vision_enabled = true;
        } else if (strcmp(text, "tinycls") == 0 || strcmp(text, "tiny_cls") == 0) {
            if (!tiny_cls_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "tinycls board backend is unavailable");
            }
            candidate.recognition_method = RECOGNITION_METHOD_TINYCLS;
            candidate.vision_enabled = true;
        } else if (strcmp(text, "coco") == 0) {
            if (!coco_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "coco board backend is unavailable");
            }
            candidate.recognition_method = RECOGNITION_METHOD_COCO;
            candidate.vision_enabled = true;
        } else {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "supported method: off, fish31, tinycls, coco");
        }
    }
    if (has_query && (httpd_query_key_value(query, "vision", text, sizeof(text)) == ESP_OK ||
                      httpd_query_key_value(query, "enabled", text, sizeof(text)) == ESP_OK)) {
        candidate.vision_enabled = config_bool_from_text(text);
        if (!candidate.vision_enabled) {
            candidate.recognition_method = RECOGNITION_METHOD_OFF;
        } else if (candidate.recognition_method == RECOGNITION_METHOD_OFF) {
            candidate.recognition_method = preferred_recognition_method();
        }
    }
    if (has_query && httpd_query_key_value(query, "history", text, sizeof(text)) == ESP_OK) {
        candidate.history_enabled = config_bool_from_text(text);
    }
    if (has_query && httpd_query_key_value(query, "recording", text, sizeof(text)) == ESP_OK) {
        candidate.recording_enabled = config_bool_from_text(text);
    }
    if (has_query && httpd_query_key_value(query, "field_auto_enable", text, sizeof(text)) == ESP_OK) {
        candidate.field_auto_enable = config_bool_from_text(text);
    }

    if (has_query && (httpd_query_key_value(query, "netmode", text, sizeof(text)) == ESP_OK ||
                      httpd_query_key_value(query, "network_mode", text, sizeof(text)) == ESP_OK ||
                      httpd_query_key_value(query, "mode", text, sizeof(text)) == ESP_OK)) {
        candidate.network_mode = strcmp(text, "sta") == 0 ? NETWORK_MODE_STA :
                                 ((strcmp(text, "softap") == 0 || strcmp(text, "ap") == 0) ?
                                  NETWORK_MODE_SOFTAP : NETWORK_MODE_APSTA);
    }
    if (has_query && (form_query_key_value(query, "router_ssid", text, sizeof(text)) == ESP_OK ||
                      form_query_key_value(query, "wifi_ssid", text, sizeof(text)) == ESP_OK)) {
        strlcpy(candidate.router_ssid, text, sizeof(candidate.router_ssid));
    }
    if (has_query && (form_query_key_value(query, "router_password", text, sizeof(text)) == ESP_OK ||
                      form_query_key_value(query, "wifi_password", text, sizeof(text)) == ESP_OK)) {
        strlcpy(candidate.router_password, text, sizeof(candidate.router_password));
    }

    uint32_t value = 0;
    if (has_query && (query_u32(query, "box_min_score", 50, 100, &value) ||
                      query_u32(query, "box_min", 50, 100, &value))) {
        candidate.box_min_score = value;
    }
    if (has_query && (query_u32(query, "stream_max_fps", 1, 30, &value) ||
                      query_u32(query, "stream_fps", 1, 30, &value))) {
        candidate.stream_max_fps = value;
    }
    if (has_query && (query_u32(query, "inference_interval_ms", 0, 600000, &value) ||
                      query_u32(query, "inference_ms", 0, 600000, &value) ||
                      query_u32(query, "inf_ms", 0, 600000, &value))) {
        candidate.inference_interval_ms = value;
    }
    if (has_query && (query_u32(query, "history_sample_interval_ms", 250, 600000, &value) ||
                      query_u32(query, "history_ms", 250, 600000, &value))) {
        candidate.history_sample_interval_ms = value;
    }
    if (has_query && (query_u32(query, "jpeg_quality", 1, 100, &value) ||
                      query_u32(query, "jpeg_q", 1, 100, &value))) {
        candidate.jpeg_quality = value;
    }
    if (has_query && (query_u32(query, "recording_segment_ms",
                                APP_RECORDING_SEGMENT_MIN_MS,
                                APP_RECORDING_SEGMENT_MAX_MS, &value) ||
                      query_u32(query, "segment_ms",
                                APP_RECORDING_SEGMENT_MIN_MS,
                                APP_RECORDING_SEGMENT_MAX_MS, &value))) {
        candidate.recording_segment_ms = value;
    }
    if (has_query && (query_u32(query, "field_idle_timeout_ms",
                                APP_FIELD_IDLE_TIMEOUT_MIN_MS,
                                APP_FIELD_IDLE_TIMEOUT_MAX_MS, &value) ||
                      query_u32(query, "idle_ms",
                                APP_FIELD_IDLE_TIMEOUT_MIN_MS,
                                APP_FIELD_IDLE_TIMEOUT_MAX_MS, &value))) {
        candidate.field_idle_timeout_ms = value;
    }

    bool network_mode_changed = candidate.network_mode != current.network_mode;
    bool credentials_changed = strcmp(candidate.router_ssid, current.router_ssid) != 0 ||
                               strcmp(candidate.router_password, current.router_password) != 0;
    bool network_reconfigure = network_mode_changed ||
                               (credentials_changed && network_mode_has_sta(candidate.network_mode));
    if (network_reconfigure && !s_netmode_queue) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req,
                                  "network service is not ready; no settings were changed");
    }

    if (has_query) {
        esp_err_t persist_ret = persist_runtime_config(&candidate);
        if (persist_ret != ESP_OK) {
            ESP_LOGE(TAG, "config transaction commit failed: %s", esp_err_to_name(persist_ret));
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "settings could not be saved; runtime state is unchanged");
        }

        bool inference_config_changed =
            candidate.recognition_method != current.recognition_method ||
            candidate.vision_enabled != current.vision_enabled;
        s_recognition_method = candidate.recognition_method;
        s_vision_enabled = candidate.vision_enabled;
        s_history_enabled = candidate.history_enabled;
        s_recording_enabled = candidate.recording_enabled;
        s_field_auto_enable = candidate.field_auto_enable;
        s_box_min_score = candidate.box_min_score;
        s_stream_max_fps = candidate.stream_max_fps;
        s_inference_interval_ms = candidate.inference_interval_ms;
        s_history_sample_interval_ms = candidate.history_sample_interval_ms;
        s_jpeg_quality = candidate.jpeg_quality;
        s_recording_segment_ms = candidate.recording_segment_ms;
        s_field_idle_timeout_ms = candidate.field_idle_timeout_ms;
        strlcpy(s_router_ssid, candidate.router_ssid, sizeof(s_router_ssid));
        strlcpy(s_router_password, candidate.router_password, sizeof(s_router_password));

        if (inference_config_changed) {
            s_last_inference_ms = 0;
            s_prev_luma_valid = false;
        }
        if (candidate.jpeg_quality != current.jpeg_quality && s_camera.valid) {
            set_camera_jpeg_quality(&s_camera, (int)candidate.jpeg_quality);
        }
        if (candidate.field_idle_timeout_ms != current.field_idle_timeout_ms ||
            candidate.field_auto_enable != current.field_auto_enable) {
            open_network_access_window("field collection settings changed");
        }
    }

    char router_ssid_json[256];
    json_escape_string(router_ssid_json, sizeof(router_ssid_json), s_router_ssid);
    char json[1800];
    int json_len = snprintf(json, sizeof(json),
             "{\"ok\":true,\"recognition_method\":\"%s\",\"network_mode\":\"%s\","
             "\"rescue_ap\":%s,\"vision_enabled\":%s,\"history_enabled\":%s,\"recording_enabled\":%s,"
             "\"router_ssid\":\"%s\",\"router_password_set\":%s,"
             "\"recording_segment_ms\":%" PRIu32 ",\"recording_segment_max_ms\":%" PRIu32
             ",\"field_idle_timeout_ms\":%" PRIu32
             ",\"field_auto_enable\":%s,"
             "\"box_min_score\":%" PRIu32 ",\"stream_max_fps\":%" PRIu32 ","
             "\"inference_interval_ms\":%" PRIu32 ",\"history_sample_interval_ms\":%" PRIu32 ","
             "\"jpeg_quality\":%" PRIu32 ","
             "\"yolo_input_size\":%" PRIu32 ",\"yolo_model_loaded\":%s,"
              "\"yolo26_available\":%s,\"yolo11_available\":%s,\"coco_available\":%s,"
              "\"tinycls_available\":%s,\"fish31_available\":%s,"
             "\"model_info\":{\"name\":\"%s\",\"bytes\":%" PRIu32 ",\"input_size\":%" PRIu32 ","
             "\"class_count\":%" PRIu32 ",\"max_detections\":%" PRIu32 ",\"nms_threshold\":%" PRIu32 "}}",
             recognition_method_name(candidate.recognition_method),
             network_mode_name(candidate.network_mode),
             s_rescue_ap_active ? "true" : "false",
             s_vision_enabled ? "true" : "false", s_history_enabled ? "true" : "false",
             s_recording_enabled ? "true" : "false",
             router_ssid_json, s_router_password[0] ? "true" : "false",
             s_recording_segment_ms, (uint32_t)APP_RECORDING_SEGMENT_MAX_MS,
             s_field_idle_timeout_ms,
             s_field_auto_enable ? "true" : "false",
             s_box_min_score, s_stream_max_fps, (unsigned long)s_inference_interval_ms,
             s_history_sample_interval_ms, s_jpeg_quality,
             (unsigned long)active_yolo_input_size(), active_yolo_available() ? "true" : "false",
             yolo26_espdl_available() ? "true" : "false",
              yolo11_espdl_available() ? "true" : "false",
              coco_espdl_available() ? "true" : "false",
              tiny_cls_espdl_available() ? "true" : "false",
              fish31_espdl_available() ? "true" : "false",
              model_name_for_method(s_recognition_method), active_model_bytes(),
              model_input_size_for_method(s_recognition_method),
             model_class_count_for_method(candidate.recognition_method), (uint32_t)APP_MAX_DETECTIONS,
             (uint32_t)APP_YOLO_NMS_THRESHOLD_X100);
    if (json_len < 0 || (size_t)json_len >= sizeof(json)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "config response exceeds safe buffer");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t response_ret = http_send_cstr_chunked(req, json);
    if (has_query && network_reconfigure) {
        s_wifi_reconfigure_requested = credentials_changed;
        xQueueOverwrite(s_netmode_queue, &candidate.network_mode);
    }
    return response_ret;
}

static esp_err_t history_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 20;
    bool has_query = false;
    esp_err_t query_ret = read_optional_url_query(req, query, sizeof(query), &has_query);
    if (query_ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "history query is too long or unreadable");
    }
    esp_err_t limit_ret = has_query ?
                          httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) :
                          ESP_ERR_NOT_FOUND;
    if (limit_ret != ESP_OK && limit_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "limit parameter is too long or malformed");
    }
    if (limit_ret == ESP_OK) {
        if (!query_u32(query, "limit", 1, 64, &limit)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit must be an integer in range 1..64");
        }
    }

    size_t cap = 768 + (size_t)limit * 2300;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no history buffer");
    }

    uint32_t count = 0;
    char *item = (char *)alloc_psram_buffer(4096);
    if (!item) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no history item buffer");
    }

    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    json_writer_appendf(&writer,
                        "{\"count\":%" PRIu32 ",\"saved\":%" PRIu32
                        ",\"queued\":%" PRIu32 ",\"dropped\":%" PRIu32
                        ",\"deleted\":%" PRIu32 ",\"sd_errors\":%" PRIu32
                        ",\"sd_mounted\":%s,\"storage_status\":",
                        s_history_count, s_history_saved, s_history_queued, s_history_dropped,
                        s_history_files_deleted, s_history_sd_errors,
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, s_storage_status);
    json_writer_appendf(&writer, ",\"records\":[");

    if (s_history_lock && s_history_records) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        count = s_history_count < limit ? s_history_count : limit;
        for (uint32_t i = 0; i < count && json_writer_ok(&writer); i++) {
            uint32_t idx = (s_history_head + CONFIG_APP_HISTORY_MAX_RECORDS - 1 - i) %
                           CONFIG_APP_HISTORY_MAX_RECORDS;
            history_record_to_json(item, 4096, &s_history_records[idx]);
            json_writer_appendf(&writer, "%s%s", i ? "," : "", item);
        }
        xSemaphoreGive(s_history_lock);
    }

    json_writer_appendf(&writer, "]}");
    if (!json_writer_ok(&writer)) {
        free(item);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "history response exceeds safe buffer");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(item);
    free(json);
    return ret;
}

static void append_recent_jsonl_array(json_writer_t *writer, const char *path, uint32_t limit)
{
    if (!writer || !json_writer_ok(writer)) {
        return;
    }
    if (!path || limit == 0) {
        json_writer_appendf(writer, "[]");
        return;
    }

    FILE *file = fopen(path, "r");
    if (!file) {
        json_writer_appendf(writer, "[]");
        return;
    }

    const size_t line_bytes = JSONL_TAIL_LINE_BYTES;
    char *lines = (char *)alloc_psram_buffer((size_t)limit * line_bytes);
    if (!lines) {
        fclose(file);
        json_writer_appendf(writer, "[]");
        return;
    }
    memset(lines, 0, (size_t)limit * line_bytes);

    char *line = (char *)alloc_psram_buffer(line_bytes);
    if (!line) {
        free(lines);
        fclose(file);
        json_writer_appendf(writer, "[]");
        return;
    }

    uint32_t total = 0;
    while (fgets(line, (int)line_bytes, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (!json_validate_object(line, len)) {
            continue;
        }
        char *slot = lines + (size_t)(total % limit) * line_bytes;
        strlcpy(slot, line, line_bytes);
        total++;
    }

    json_writer_appendf(writer, "[");
    uint32_t count = total < limit ? total : limit;
    uint32_t start = total > count ? total - count : 0;
    for (uint32_t i = 0; i < count && json_writer_ok(writer); i++) {
        uint32_t idx = (start + i) % limit;
        const char *slot = lines + (size_t)idx * line_bytes;
        json_writer_appendf(writer, "%s%s", i ? "," : "", slot);
    }
    json_writer_appendf(writer, "]");

    free(line);
    free(lines);
    fclose(file);
}

typedef struct {
    char kind[16];
    char name[96];
    char uri[128];
    char meta_uri[128];
    char method[16];
    char model[48];
    char labels[512];
    int64_t start_ms;
    int64_t end_ms;
    uint64_t start_epoch_ms;
    uint64_t end_epoch_ms;
    uint64_t bytes;
    int64_t duration_ms;
    uint32_t frames;
    uint32_t hit_frames;
    uint32_t detection_total;
} recording_index_entry_t;

static uint64_t json_u64_field_or_zero(const char *line, const char *key)
{
    int64_t value = 0;
    return json_get_int64_field(line, key, &value) && value > 0 ? (uint64_t)value : 0;
}

static bool json_copy_array_field(const char *line, const char *key, char *out, size_t out_size)
{
    if (!line || !key || !out || out_size == 0) {
        return false;
    }
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(line, pattern);
    if (!p) {
        return false;
    }
    p += strlen(pattern);
    if (*p != '[') {
        return false;
    }
    const char *start = p;
    int depth = 0;
    while (*p) {
        if (*p == '[') {
            depth++;
        } else if (*p == ']') {
            depth--;
            if (depth == 0) {
                p++;
                break;
            }
        }
        p++;
    }
    if (depth != 0) {
        return false;
    }
    size_t n = (size_t)(p - start);
    if (n >= out_size) {
        return false;
    }
    memcpy(out, start, n);
    out[n] = '\0';
    return true;
}

static void parse_recording_index_line(const char *line, recording_index_entry_t *entry)
{
    memset(entry, 0, sizeof(*entry));
    strlcpy(entry->labels, "[]", sizeof(entry->labels));
    json_get_string_field(line, "kind", entry->kind, sizeof(entry->kind));
    json_get_string_field(line, "name", entry->name, sizeof(entry->name));
    if (!entry->name[0]) {
        json_get_string_field(line, "segment", entry->name, sizeof(entry->name));
    }
    json_get_string_field(line, "uri", entry->uri, sizeof(entry->uri));
    json_get_string_field(line, "meta_uri", entry->meta_uri, sizeof(entry->meta_uri));
    json_get_string_field(line, "method", entry->method, sizeof(entry->method));
    json_get_string_field(line, "model", entry->model, sizeof(entry->model));
    json_get_int64_field(line, "start_ms", &entry->start_ms);
    json_get_int64_field(line, "end_ms", &entry->end_ms);
    entry->start_epoch_ms = json_u64_field_or_zero(line, "start_epoch_ms");
    entry->end_epoch_ms = json_u64_field_or_zero(line, "end_epoch_ms");
    entry->bytes = json_u64_field_or_zero(line, "bytes");
    json_get_int64_field(line, "duration_ms", &entry->duration_ms);
    json_get_u32_field(line, "frames", &entry->frames);
    json_get_u32_field(line, "hit_frames", &entry->hit_frames);
    json_get_u32_field(line, "detection_total", &entry->detection_total);
    json_copy_array_field(line, "labels", entry->labels, sizeof(entry->labels));
    if (is_annotated_recording_name(entry->name)) {
        strlcpy(entry->kind, "annotated", sizeof(entry->kind));
    } else if (strncmp(entry->name, "raw_", 4) == 0) {
        strlcpy(entry->kind, "raw", sizeof(entry->kind));
    } else if (!entry->kind[0]) {
        strlcpy(entry->kind, "raw", sizeof(entry->kind));
    }
    if (!entry->uri[0] && entry->name[0]) {
        snprintf(entry->uri, sizeof(entry->uri), RECORDING_URI_PREFIX "%s", entry->name);
    }
    if (!entry->meta_uri[0] && entry->name[0]) {
        char meta_name[96];
        meta_name_for_recording(entry->name, meta_name, sizeof(meta_name));
        snprintf(entry->meta_uri, sizeof(entry->meta_uri), RECORDING_META_URI_PREFIX "%s", meta_name);
    }
    recognition_method_t method = recognition_method_from_text_hint(entry->method);
    if (method == RECOGNITION_METHOD_OFF) {
        method = recognition_method_from_text_hint(entry->model);
    }
    if (method == RECOGNITION_METHOD_OFF) {
        method = recognition_method_from_text_hint(entry->name);
    }
    if (method == RECOGNITION_METHOD_OFF) {
        method = RECOGNITION_METHOD_FISH31;
    }
    strlcpy(entry->method, recognition_method_name(method), sizeof(entry->method));
    if (!entry->model[0]) {
        strlcpy(entry->model, model_name_for_method(method), sizeof(entry->model));
    }
    if (entry->duration_ms <= 0 && entry->end_ms > entry->start_ms) {
        entry->duration_ms = entry->end_ms - entry->start_ms;
    }
}

static uint32_t load_recent_recording_entries(recording_index_entry_t *entries,
                                              uint32_t max_entries)
{
    if (!entries || max_entries == 0) {
        return 0;
    }
    FILE *file = fopen(RECORDING_INDEX_PATH, "r");
    if (!file) {
        return 0;
    }
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        fclose(file);
        return 0;
    }
    uint32_t total = 0;
    while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (!json_validate_object(line, len)) {
            continue;
        }
        parse_recording_index_line(line, &entries[total % max_entries]);
        total++;
    }
    free(line);
    fclose(file);
    uint32_t count = total < max_entries ? total : max_entries;
    if (total > max_entries) {
        recording_index_entry_t *tmp = (recording_index_entry_t *)calloc(count, sizeof(*tmp));
        if (tmp) {
            uint32_t start = total - count;
            for (uint32_t i = 0; i < count; i++) {
                tmp[i] = entries[(start + i) % max_entries];
            }
            memcpy(entries, tmp, (size_t)count * sizeof(*entries));
            free(tmp);
        }
    }
    return count;
}

static void append_recording_entry_json(json_writer_t *writer,
                                        const recording_index_entry_t *entry)
{
    if (!writer || !json_writer_ok(writer) || !entry) {
        return;
    }

    json_writer_appendf(writer, "{\"kind\":");
    json_writer_append_escaped_string(writer, entry->kind);
    json_writer_appendf(writer, ",\"name\":");
    json_writer_append_escaped_string(writer, entry->name);
    json_writer_appendf(writer, ",\"uri\":");
    json_writer_append_escaped_string(writer, entry->uri);
    json_writer_appendf(writer, ",\"meta_uri\":");
    json_writer_append_escaped_string(writer, entry->meta_uri);
    json_writer_appendf(writer, ",\"method\":");
    json_writer_append_escaped_string(writer, entry->method);
    json_writer_appendf(writer, ",\"model\":");
    json_writer_append_escaped_string(writer, entry->model);
    json_writer_appendf(writer,
                        ",\"start_ms\":%" PRId64 ",\"end_ms\":%" PRId64
                        ",\"start_epoch_ms\":%" PRIu64 ",\"end_epoch_ms\":%" PRIu64
                        ",\"duration_ms\":%" PRId64 ",\"frames\":%" PRIu32
                        ",\"bytes\":%" PRIu64 ",\"hit_frames\":%" PRIu32
                        ",\"detection_total\":%" PRIu32 ",\"labels\":%s}",
                        entry->start_ms, entry->end_ms,
                        entry->start_epoch_ms, entry->end_epoch_ms, entry->duration_ms,
                        entry->frames, entry->bytes, entry->hit_frames,
                        entry->detection_total, entry->labels[0] ? entry->labels : "[]");
}

static void append_recording_groups_json(json_writer_t *writer, uint32_t limit)
{
    if (!writer || !json_writer_ok(writer)) {
        return;
    }
    uint32_t max_entries = limit * 3U + 12U;
    if (max_entries < 24U) {
        max_entries = 24U;
    } else if (max_entries > 180U) {
        max_entries = 180U;
    }
    recording_index_entry_t *entries =
        (recording_index_entry_t *)calloc(max_entries, sizeof(*entries));
    bool *used = (bool *)calloc(max_entries, sizeof(bool));
    if (!entries || !used) {
        free(entries);
        free(used);
        json_writer_appendf(writer, "[]");
        return;
    }
    uint32_t count = load_recent_recording_entries(entries, max_entries);
    json_writer_appendf(writer, "[");
    uint32_t groups = 0;
    bool first = true;
    for (int32_t i = (int32_t)count - 1;
         i >= 0 && groups < limit && json_writer_ok(writer); i--) {
        if (used[i] || strcmp(entries[i].kind, "raw") != 0) {
            continue;
        }
        char expected[96] = {0};
        char legacy_expected[96] = {0};
        annotated_name_for_raw(entries[i].name, expected, sizeof(expected));
        legacy_annotated_name_for_raw(entries[i].name, legacy_expected, sizeof(legacy_expected));
        int32_t ann = -1;
        for (uint32_t j = 0; j < count; j++) {
            if (!used[j] && strcmp(entries[j].kind, "annotated") == 0 &&
                ((expected[0] && strcmp(entries[j].name, expected) == 0) ||
                 (legacy_expected[0] && strcmp(entries[j].name, legacy_expected) == 0))) {
                ann = (int32_t)j;
                break;
            }
        }
        used[i] = true;
        if (ann >= 0) {
            used[ann] = true;
        }
        const recording_index_entry_t *raw = &entries[i];
        const recording_index_entry_t *annotated = ann >= 0 ? &entries[ann] : NULL;
        bool method_mismatch = annotated && strcmp(raw->method, annotated->method) != 0;
        const char *fill_state = !annotated ? "missing" :
                                 (method_mismatch ? "rebuild" : "ready");
        json_writer_appendf(writer, "%s{\"time_ms\":%" PRIu64 ",\"method\":",
                            first ? "" : ",",
                            raw->start_epoch_ms ? raw->start_epoch_ms : (uint64_t)raw->start_ms);
        json_writer_append_escaped_string(writer, raw->method);
        json_writer_appendf(writer, ",\"model\":");
        json_writer_append_escaped_string(writer, raw->model);
        json_writer_appendf(writer, ",\"fill_state\":");
        json_writer_append_escaped_string(writer, fill_state);
        json_writer_appendf(writer, ",\"needs_rebuild\":%s,\"raw\":",
                            (!annotated || method_mismatch) ? "true" : "false");
        append_recording_entry_json(writer, raw);
        json_writer_appendf(writer, ",\"annotated\":");
        if (annotated) {
            append_recording_entry_json(writer, annotated);
        } else {
            json_writer_appendf(writer, "null");
        }
        json_writer_appendf(writer, "}");
        first = false;
        groups++;
    }
    json_writer_appendf(writer, "]");
    free(used);
    free(entries);
}

typedef struct {
    avi_mjpeg_writer_t *raw_writer;
    avi_mjpeg_writer_t *annotated_writer;
    FILE *raw_meta;
    FILE *annotated_meta;
    char raw_name[96];
    char annotated_name[96];
    char raw_final_path[384];
    char raw_part_path[384];
    char raw_meta_path[384];
    char annotated_final_path[384];
    char annotated_part_path[384];
    char annotated_meta_path[384];
    recognition_method_t method;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t frames;
    uint64_t raw_bytes;
    uint64_t annotated_bytes;
    int64_t start_ms;
    int64_t last_ms;
    uint64_t start_epoch_ms;
} resegment_output_t;

static bool resegment_should_pause(void)
{
    if (s_storage_quiescing || storage_mode_request_pending() ||
        s_app_mode != APP_MODE_SERVER || !s_sd_mounted ||
        s_power_state != POWER_STATE_STANDBY ||
        stream_stats_client_count() > 0 ||
        __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) > 0 ||
        inference_worker_busy() ||
        recording_enrichment_has_request() ||
        (s_history_queue && uxQueueMessagesWaiting(s_history_queue) > 0) ||
        (s_recording_queue && uxQueueMessagesWaiting(s_recording_queue) > 0)) {
        return true;
    }
    recording_enrichment_status_t enrichment = {0};
    recording_enrichment_get_status(&enrichment);
    if (enrichment.running) {
        return true;
    }
    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    return dataset.queued || dataset.running;
}

static esp_err_t resegment_close_output(resegment_output_t *out,
                                        char generated[][96],
                                        uint32_t max_generated,
                                        uint32_t *generated_count)
{
    if (!out || (!out->raw_writer && !out->annotated_writer)) {
        return ESP_OK;
    }
    uint64_t duration_ms = out->last_ms > out->start_ms ?
        (uint64_t)(out->last_ms - out->start_ms) : 0;
    esp_err_t ret = ESP_OK;
    if (out->annotated_writer) {
        avi_mjpeg_writer_set_duration_ms(out->annotated_writer, duration_ms);
        esp_err_t ann_ret = avi_mjpeg_writer_close(out->annotated_writer);
        if (ann_ret != ESP_OK) {
            ret = ann_ret;
        }
        out->annotated_writer = NULL;
    }
    if (out->raw_writer) {
        avi_mjpeg_writer_set_duration_ms(out->raw_writer, duration_ms);
        esp_err_t raw_ret = avi_mjpeg_writer_close(out->raw_writer);
        if (raw_ret != ESP_OK && ret == ESP_OK) {
            ret = raw_ret;
        }
        out->raw_writer = NULL;
    }
    if (out->annotated_meta) {
        if (fflush(out->annotated_meta) != 0 || fsync(fileno(out->annotated_meta)) != 0) {
            ret = ESP_FAIL;
        }
        if (fclose(out->annotated_meta) != 0 && ret == ESP_OK) {
            ret = ESP_FAIL;
        }
        out->annotated_meta = NULL;
    }
    if (out->raw_meta) {
        if (fflush(out->raw_meta) != 0 || fsync(fileno(out->raw_meta)) != 0) {
            ret = ESP_FAIL;
        }
        if (fclose(out->raw_meta) != 0 && ret == ESP_OK) {
            ret = ESP_FAIL;
        }
        out->raw_meta = NULL;
    }
    if (ret == ESP_OK && generated && generated_count && *generated_count < max_generated) {
        strlcpy(generated[*generated_count], out->raw_name, 96);
        (*generated_count)++;
    }
    memset(out, 0, sizeof(*out));
    return ret;
}

static esp_err_t resegment_open_output(resegment_output_t *out,
                                       recognition_method_t method,
                                       uint32_t width,
                                       uint32_t height,
                                       uint32_t fps,
                                       int64_t start_ms,
                                       uint64_t start_epoch_ms,
                                       uint32_t sequence)
{
    memset(out, 0, sizeof(*out));
    out->method = method;
    out->width = width;
    out->height = height;
    out->fps = fps ? fps : 1U;
    out->start_ms = start_ms;
    out->last_ms = start_ms;
    out->start_epoch_ms = start_epoch_ms;

    char slug[40];
    recording_time_slug(start_epoch_ms, start_ms, slug, sizeof(slug));
    for (uint32_t attempt = 0; attempt < 100; attempt++) {
        char suffix[12] = {0};
        if (attempt) {
            snprintf(suffix, sizeof(suffix), "_r%02" PRIu32, attempt);
        }
        snprintf(out->raw_name, sizeof(out->raw_name),
                 "raw_%s_%s_seg%03" PRIu32 "%s.avi",
                 slug, recognition_method_name(method), sequence, suffix);
        if (!annotated_name_for_raw(out->raw_name, out->annotated_name,
                                    sizeof(out->annotated_name))) {
            return ESP_FAIL;
        }
        snprintf(out->raw_final_path, sizeof(out->raw_final_path),
                 "%s/%s", RECORDING_DIR, out->raw_name);
        snprintf(out->annotated_final_path, sizeof(out->annotated_final_path),
                 "%s/%s", RECORDING_DIR, out->annotated_name);
        if (access(out->raw_final_path, F_OK) != 0 &&
            access(out->annotated_final_path, F_OK) != 0) {
            break;
        }
    }
    snprintf(out->raw_part_path, sizeof(out->raw_part_path),
             "%s/%s.part", RECORDING_DIR, out->raw_name);
    snprintf(out->annotated_part_path, sizeof(out->annotated_part_path),
             "%s/%s.part", RECORDING_DIR, out->annotated_name);
    char raw_meta_name[96];
    char annotated_meta_name[96];
    meta_name_for_recording(out->raw_name, raw_meta_name, sizeof(raw_meta_name));
    meta_name_for_recording(out->annotated_name, annotated_meta_name, sizeof(annotated_meta_name));
    snprintf(out->raw_meta_path, sizeof(out->raw_meta_path),
             "%s/%s", RECORDING_DIR, raw_meta_name);
    snprintf(out->annotated_meta_path, sizeof(out->annotated_meta_path),
             "%s/%s", RECORDING_DIR, annotated_meta_name);

    esp_err_t ret = avi_mjpeg_writer_open(&out->raw_writer, out->raw_part_path,
                                          out->raw_final_path, width, height, out->fps);
    if (ret != ESP_OK) {
        memset(out, 0, sizeof(*out));
        return ret;
    }
    ret = avi_mjpeg_writer_open(&out->annotated_writer, out->annotated_part_path,
                                out->annotated_final_path, width, height, out->fps);
    if (ret != ESP_OK) {
        avi_mjpeg_writer_abort(out->raw_writer);
        unlink(out->raw_part_path);
        memset(out, 0, sizeof(*out));
        return ret;
    }
    out->raw_meta = fopen(out->raw_meta_path, "w");
    out->annotated_meta = fopen(out->annotated_meta_path, "w");
    if (!out->raw_meta || !out->annotated_meta) {
        if (out->raw_meta) {
            fclose(out->raw_meta);
        }
        if (out->annotated_meta) {
            fclose(out->annotated_meta);
        }
        avi_mjpeg_writer_abort(out->annotated_writer);
        avi_mjpeg_writer_abort(out->raw_writer);
        unlink(out->annotated_part_path);
        unlink(out->raw_part_path);
        memset(out, 0, sizeof(*out));
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t resegment_write_frame(resegment_output_t *out,
                                       const uint8_t *raw_jpeg,
                                       size_t raw_jpeg_size,
                                       const uint8_t *annotated_jpeg,
                                       size_t annotated_jpeg_size,
                                       int64_t frame_ms,
                                       uint64_t frame_epoch_ms,
                                       bool copied_annotation)
{
    if (!out || !out->raw_writer || !out->annotated_writer ||
        !raw_jpeg || raw_jpeg_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t *ann_jpeg = annotated_jpeg && annotated_jpeg_size > 0 ?
        annotated_jpeg : raw_jpeg;
    size_t ann_size = annotated_jpeg && annotated_jpeg_size > 0 ?
        annotated_jpeg_size : raw_jpeg_size;

    esp_err_t ret = avi_mjpeg_writer_add_frame(out->raw_writer, raw_jpeg, raw_jpeg_size);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = avi_mjpeg_writer_add_frame(out->annotated_writer, ann_jpeg, ann_size);
    if (ret != ESP_OK) {
        return ret;
    }
    out->frames++;
    out->raw_bytes += raw_jpeg_size;
    out->annotated_bytes += ann_size;
    out->last_ms = frame_ms;
    if (fprintf(out->raw_meta,
                "{\"index_version\":%" PRIu32 ",\"segment\":\"%s\","
                "\"segment_uri\":\"%s%s\",\"kind\":\"raw\","
                "\"meta_uri\":\"%s%s\",\"storage_backend\":\"%s\","
                "\"frame_index\":%" PRIu32 ",\"seq\":%" PRIu32
                ",\"time_ms\":%" PRId64 ",\"epoch_ms\":%" PRIu64
                ",\"jpeg_bytes\":%u,\"width\":%" PRIu32 ",\"height\":%" PRIu32
                ",\"method\":\"%s\",\"model\":\"%s\",\"object_count\":0,"
                "\"top_k\":[],\"box_min_score\":%" PRIu32 ",\"best_score\":0,"
                "\"candidate_score\":0,\"raw_candidate_count\":0,"
                "\"inference_ms\":0,\"analysis_ms\":0,"
                "\"detection_count\":0,\"detections\":[],\"resegmented\":true}\n",
                (uint32_t)APP_JSONL_INDEX_VERSION, out->raw_name,
                RECORDING_URI_PREFIX, out->raw_name,
                RECORDING_META_URI_PREFIX, strrchr(out->raw_meta_path, '/') ?
                    strrchr(out->raw_meta_path, '/') + 1 : out->raw_meta_path,
                s_storage_backend, out->frames, out->frames,
                frame_ms, frame_epoch_ms, (unsigned)raw_jpeg_size,
                out->width, out->height,
                recognition_method_name(out->method), model_name_for_method(out->method),
                s_box_min_score) < 0) {
        return ESP_FAIL;
    }
    if (fprintf(out->annotated_meta,
                "{\"index_version\":%" PRIu32 ",\"segment\":\"%s\","
                "\"segment_uri\":\"%s%s\",\"kind\":\"annotated\","
                "\"meta_uri\":\"%s%s\",\"storage_backend\":\"%s\","
                "\"frame_index\":%" PRIu32 ",\"seq\":%" PRIu32
                ",\"time_ms\":%" PRId64 ",\"epoch_ms\":%" PRIu64
                ",\"jpeg_bytes\":%u,\"width\":%" PRIu32 ",\"height\":%" PRIu32
                ",\"method\":\"%s\",\"model\":\"%s\",\"object_count\":0,"
                "\"top_k\":[],\"box_min_score\":%" PRIu32 ",\"best_score\":0,"
                "\"candidate_score\":0,\"raw_candidate_count\":0,"
                "\"inference_ms\":0,\"analysis_ms\":0,"
                "\"detection_count\":0,\"detections\":[],\"resegmented\":true,"
                "\"result_source\":\"%s\",\"source_frame_index\":%" PRIu32 "}\n",
                (uint32_t)APP_JSONL_INDEX_VERSION, out->annotated_name,
                RECORDING_ANNOTATED_URI_PREFIX, out->annotated_name,
                RECORDING_META_URI_PREFIX, strrchr(out->annotated_meta_path, '/') ?
                    strrchr(out->annotated_meta_path, '/') + 1 : out->annotated_meta_path,
                s_storage_backend, out->frames, out->frames,
                frame_ms, frame_epoch_ms, (unsigned)ann_size,
                out->width, out->height,
                recognition_method_name(out->method), model_name_for_method(out->method),
                s_box_min_score,
                copied_annotation ? "copied-annotated" : "raw-fallback",
                out->frames) < 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void resegment_delete_generated(char generated[][96], uint32_t generated_count)
{
    uint64_t freed = 0;
    for (uint32_t i = 0; i < generated_count; i++) {
        bool failed = false;
        delete_recording_files_by_name(generated[i], &freed, &failed);
        char annotated_name[96] = {0};
        if (annotated_name_for_raw(generated[i], annotated_name, sizeof(annotated_name))) {
            delete_recording_files_by_name(annotated_name, &freed, &failed);
        }
        bool index_failed = false;
        remove_recording_index_rows(generated[i], &index_failed);
        if (annotated_name[0]) {
            remove_recording_index_rows(annotated_name, &index_failed);
        }
    }
}

static esp_err_t resegment_recordings_to_target(uint32_t target_ms)
{
    enum { MAX_RESEG_INPUTS = 180, MAX_RESEG_OUTPUTS = 256 };
    recording_index_entry_t *entries =
        (recording_index_entry_t *)calloc(MAX_RESEG_INPUTS, sizeof(*entries));
    char (*generated)[96] = (char (*)[96])calloc(MAX_RESEG_OUTPUTS, 96);
    if (!entries || !generated) {
        free(entries);
        free(generated);
        return ESP_ERR_NO_MEM;
    }
    uint32_t count = load_recent_recording_entries(entries, MAX_RESEG_INPUTS);
    uint32_t raw_count = 0;
    bool need_resegment = false;
    for (uint32_t i = 0; i < count; i++) {
        if (strcmp(entries[i].kind, "raw") != 0) {
            continue;
        }
        raw_count++;
        uint32_t duration = entries[i].duration_ms > 0 ? (uint32_t)entries[i].duration_ms : 0;
        uint32_t delta = duration > target_ms ? duration - target_ms : target_ms - duration;
        if (duration > 0 && delta > 2000U) {
            need_resegment = true;
        }
    }
    if (raw_count == 0 || !need_resegment) {
        resegment_status_update(false, false, false, raw_count, raw_count, 0,
                                raw_count == 0 ? "no raw recordings" : "already at target");
        free(entries);
        free(generated);
        return ESP_ERR_NOT_FOUND;
    }

    resegment_status_update(true, true, false, raw_count, 0, 0, "running");
    resegment_output_t out = {0};
    uint32_t processed = 0;
    uint32_t generated_count = 0;
    esp_err_t ret = ESP_OK;
    uint32_t sequence = 1;

    for (uint32_t i = 0; i < count && ret == ESP_OK; i++) {
        if (strcmp(entries[i].kind, "raw") != 0) {
            continue;
        }
        if (resegment_should_pause()) {
            ret = ESP_ERR_TIMEOUT;
            break;
        }
        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entries[i].name);
        avi_mjpeg_reader_t *reader = NULL;
        avi_mjpeg_reader_t *annotated_reader = NULL;
        avi_mjpeg_info_t info = {0};
        ret = avi_mjpeg_reader_open(&reader, path, &info);
        if (ret != ESP_OK) {
            break;
        }
        avi_mjpeg_info_t annotated_info = {0};
        char annotated_path[384] = {0};
        char annotated_name[96] = {0};
        if (annotated_name_for_raw(entries[i].name, annotated_name, sizeof(annotated_name))) {
            snprintf(annotated_path, sizeof(annotated_path), "%s/%s",
                     RECORDING_DIR, annotated_name);
            if (avi_mjpeg_reader_open(&annotated_reader, annotated_path,
                                      &annotated_info) == ESP_OK) {
                if (annotated_info.width != info.width ||
                    annotated_info.height != info.height) {
                    avi_mjpeg_reader_close(annotated_reader);
                    annotated_reader = NULL;
                }
            }
        }
        if (!annotated_reader &&
            legacy_annotated_name_for_raw(entries[i].name, annotated_name,
                                          sizeof(annotated_name))) {
            snprintf(annotated_path, sizeof(annotated_path), "%s/%s",
                     RECORDING_DIR, annotated_name);
            if (avi_mjpeg_reader_open(&annotated_reader, annotated_path,
                                      &annotated_info) == ESP_OK) {
                if (annotated_info.width != info.width ||
                    annotated_info.height != info.height) {
                    avi_mjpeg_reader_close(annotated_reader);
                    annotated_reader = NULL;
                }
            }
        }
        uint32_t fps = info.scale ? (info.rate + info.scale / 2U) / info.scale :
                       CONFIG_APP_FIELD_RECORDING_MAX_FPS;
        if (fps == 0) {
            fps = 1;
        }
        recognition_method_t method = recognition_method_from_text_hint(entries[i].method);
        if (method == RECOGNITION_METHOD_OFF) {
            method = recognition_method_from_text_hint(entries[i].model);
        }
        if (method == RECOGNITION_METHOD_OFF) {
            method = RECOGNITION_METHOD_FISH31;
        }
        while (ret == ESP_OK) {
            if (resegment_should_pause()) {
                ret = ESP_ERR_TIMEOUT;
                break;
            }
            const uint8_t *jpeg = NULL;
            size_t jpeg_size = 0;
            uint32_t frame_index = 0;
            ret = avi_mjpeg_reader_next(reader, &jpeg, &jpeg_size, &frame_index);
            if (ret != ESP_OK) {
                if (ret == ESP_ERR_NOT_FOUND || ret == ESP_ERR_INVALID_SIZE) {
                    ret = ESP_OK;
                }
                break;
            }
            uint64_t offset_ms = info.frame_count > 1 ?
                ((uint64_t)(frame_index - 1U) * info.duration_ms) /
                    (info.frame_count - 1U) : 0;
            int64_t frame_ms = entries[i].start_ms + (int64_t)offset_ms;
            uint64_t frame_epoch_ms = entries[i].start_epoch_ms ?
                entries[i].start_epoch_ms + offset_ms : 0;
            bool open_new = !out.raw_writer ||
                            out.width != info.width || out.height != info.height ||
                            out.fps != fps || out.method != method ||
                            (target_ms > 0 && frame_ms - out.start_ms >= (int64_t)target_ms);
            if (open_new) {
                ret = resegment_close_output(&out, generated, MAX_RESEG_OUTPUTS, &generated_count);
                if (ret != ESP_OK) {
                    break;
                }
                if (generated_count >= MAX_RESEG_OUTPUTS) {
                    ret = ESP_ERR_NO_MEM;
                    break;
                }
                ret = resegment_open_output(&out, method, info.width, info.height, fps,
                                            frame_ms, frame_epoch_ms, sequence++);
                if (ret != ESP_OK) {
                    break;
                }
            }
            const uint8_t *annotated_jpeg = NULL;
            size_t annotated_size = 0;
            bool copied_annotation = false;
            if (annotated_reader) {
                uint32_t annotated_frame_index = 0;
                esp_err_t ann_ret = avi_mjpeg_reader_next(annotated_reader,
                                                          &annotated_jpeg,
                                                          &annotated_size,
                                                          &annotated_frame_index);
                if (ann_ret == ESP_OK && annotated_jpeg && annotated_size > 0) {
                    copied_annotation = true;
                } else {
                    ESP_LOGW(TAG,
                             "resegment annotation fallback: raw=%s frame=%" PRIu32
                             " annotated=%s err=%s",
                             entries[i].name, frame_index, annotated_name,
                             esp_err_to_name(ann_ret));
                    avi_mjpeg_reader_close(annotated_reader);
                    annotated_reader = NULL;
                    annotated_jpeg = NULL;
                    annotated_size = 0;
                }
            }
            ret = resegment_write_frame(&out, jpeg, jpeg_size,
                                        annotated_jpeg, annotated_size,
                                        frame_ms, frame_epoch_ms,
                                        copied_annotation);
        }
        if (annotated_reader) {
            avi_mjpeg_reader_close(annotated_reader);
        }
        avi_mjpeg_reader_close(reader);
        processed++;
        resegment_status_update(true, true, false, raw_count, processed,
                                generated_count, "running");
    }
    if (ret == ESP_OK) {
        ret = resegment_close_output(&out, generated, MAX_RESEG_OUTPUTS, &generated_count);
    } else if (out.raw_writer || out.annotated_writer) {
        if (out.annotated_writer) {
            avi_mjpeg_writer_abort(out.annotated_writer);
        }
        if (out.raw_writer) {
            avi_mjpeg_writer_abort(out.raw_writer);
        }
        if (out.annotated_meta) {
            fclose(out.annotated_meta);
        }
        if (out.raw_meta) {
            fclose(out.raw_meta);
        }
        unlink(out.annotated_part_path);
        unlink(out.raw_part_path);
        unlink(out.annotated_meta_path);
        unlink(out.raw_meta_path);
    }

    if (ret == ESP_OK) {
        uint64_t freed = 0;
        for (uint32_t i = 0; i < count; i++) {
            if (strcmp(entries[i].kind, "raw") != 0) {
                continue;
            }
            bool failed = false;
            delete_recording_files_by_name(entries[i].name, &freed, &failed);
            bool index_failed = false;
            remove_recording_index_rows(entries[i].name, &index_failed);
            char annotated_name[96] = {0};
            if (annotated_name_for_raw(entries[i].name, annotated_name, sizeof(annotated_name))) {
                delete_recording_files_by_name(annotated_name, &freed, &failed);
                remove_recording_index_rows(annotated_name, &index_failed);
            }
            char legacy_annotated_name[96] = {0};
            if (legacy_annotated_name_for_raw(entries[i].name, legacy_annotated_name,
                                              sizeof(legacy_annotated_name))) {
                delete_recording_files_by_name(legacy_annotated_name, &freed, &failed);
                remove_recording_index_rows(legacy_annotated_name, &index_failed);
            }
        }
        reconcile_recording_indexes();
        update_sd_info();
        resegment_status_update(false, false, false, raw_count, processed,
                                generated_count, "ok");
    } else {
        resegment_delete_generated(generated, generated_count);
        resegment_status_update(false, false, ret == ESP_ERR_TIMEOUT, raw_count, processed,
                                generated_count,
                                ret == ESP_ERR_TIMEOUT ? "cancelled by foreground activity" :
                                esp_err_to_name(ret));
    }
    free(entries);
    free(generated);
    return ret;
}

static void append_recent_jsonl_wrapped_array(json_writer_t *writer,
                                              const char *path, uint32_t limit,
                                              const char *type, bool *need_comma)
{
    if (!writer || !json_writer_ok(writer) || !path || !type ||
        !need_comma || limit == 0) {
        return;
    }

    FILE *file = fopen(path, "r");
    if (!file) {
        return;
    }

    const size_t line_bytes = JSONL_TAIL_LINE_BYTES;
    char *lines = (char *)alloc_psram_buffer((size_t)limit * line_bytes);
    char *line = (char *)alloc_psram_buffer(line_bytes);
    if (!lines || !line) {
        free(lines);
        free(line);
        fclose(file);
        return;
    }
    memset(lines, 0, (size_t)limit * line_bytes);

    uint32_t total = 0;
    while (fgets(line, (int)line_bytes, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (!json_validate_object(line, len)) {
            continue;
        }
        char *slot = lines + (size_t)(total % limit) * line_bytes;
        strlcpy(slot, line, line_bytes);
        total++;
    }
    fclose(file);

    uint32_t count = total < limit ? total : limit;
    uint32_t start = total > count ? total - count : 0;
    for (uint32_t i = 0; i < count && json_writer_ok(writer); i++) {
        uint32_t idx = (start + i) % limit;
        const char *slot = lines + (size_t)idx * line_bytes;
        json_writer_appendf(writer, "%s{\"type\":", *need_comma ? "," : "");
        json_writer_append_escaped_string(writer, type);
        json_writer_appendf(writer, ",\"data\":%s}", slot);
        *need_comma = true;
    }

    free(line);
    free(lines);
}

static esp_err_t timeline_get_handler_internal(httpd_req_t *req)
{
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 50;
    bool has_query = false;
    esp_err_t query_ret = read_optional_url_query(req, query, sizeof(query), &has_query);
    if (query_ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "timeline query is too long or unreadable");
    }
    esp_err_t limit_ret = has_query ?
                          httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) :
                          ESP_ERR_NOT_FOUND;
    if (limit_ret != ESP_OK && limit_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "limit parameter is too long or malformed");
    }
    if (limit_ret == ESP_OK) {
        if (!query_u32(query, "limit", 1, 100, &limit)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit must be an integer in range 1..100");
        }
    }

    size_t cap = 2048 + (size_t)limit * 4096;
    char *json = (char *)alloc_psram_buffer(cap);
    char *item = (char *)alloc_psram_buffer(4096);
    if (!json || !item) {
        free(json);
        free(item);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no timeline buffer");
    }

    bool need_comma = false;
    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    json_writer_appendf(&writer, "{\"sd_mounted\":%s,\"storage_backend\":",
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, s_storage_backend);
    json_writer_appendf(&writer, ",\"storage_status\":");
    json_writer_append_escaped_string(&writer, s_storage_status);
    json_writer_appendf(&writer,
                        ",\"tf_ready\":%s,\"storage_acceptance_ok\":%s"
                        ",\"index_version\":%" PRIu32 ",\"recording_enabled\":%s"
                        ",\"history_saved\":%" PRIu32 ",\"recording_segments\":%" PRIu32
                        ",\"summary_count\":%" PRIu32 ",\"timeline\":[",
                        storage_tf_ready() ? "true" : "false",
                        storage_acceptance_ok() ? "true" : "false",
                        (uint32_t)APP_JSONL_INDEX_VERSION,
                        s_recording_enabled ? "true" : "false",
                        s_history_saved, s_recording_segments, s_recording_summary_count);

    if (s_history_lock && s_history_records) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        uint32_t count = s_history_count < limit ? s_history_count : limit;
        for (uint32_t i = 0; i < count && json_writer_ok(&writer); i++) {
            uint32_t idx = (s_history_head + CONFIG_APP_HISTORY_MAX_RECORDS - 1 - i) %
                           CONFIG_APP_HISTORY_MAX_RECORDS;
            history_record_to_json(item, 4096, &s_history_records[idx]);
            json_writer_appendf(&writer, "%s{\"type\":\"history\",\"data\":%s}",
                                need_comma ? "," : "", item);
            need_comma = true;
        }
        xSemaphoreGive(s_history_lock);
    }
    append_recent_jsonl_wrapped_array(&writer, RECORDING_INDEX_PATH, limit,
                                      "recording", &need_comma);
    append_recent_jsonl_wrapped_array(&writer, RECORDING_SUMMARY_PATH, limit,
                                      "summary", &need_comma);
    json_writer_appendf(&writer, "]}");

    if (!json_writer_ok(&writer)) {
        free(item);
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "timeline response exceeds safe buffer");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(item);
    free(json);
    return ret;
}

static bool storage_usb_owned(void)
{
    return s_usb_storage_ready || s_app_mode == APP_MODE_USB_EXPORT;
}

static bool usb_auto_export_allowed_mode(app_mode_t mode)
{
    return mode == APP_MODE_SERVER ||
           mode == APP_MODE_FIELD ||
           mode == APP_MODE_EXPORT;
}

static esp_err_t send_customer_action_json(httpd_req_t *req, const char *status,
                                           const char *reason, const char *message,
                                           const char *action)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char json[512];
    snprintf(json, sizeof(json),
             "{\"ok\":false,\"reason\":\"%s\",\"app_mode\":\"%s\","
             "\"usb_storage_owner\":\"%s\",\"storage_quiescing\":%s,"
             "\"sd_mounted\":%s,\"file_storage_mounted\":%s,"
             "\"message\":\"%s\",\"action\":\"%s\"}",
             reason ? reason : "error", app_mode_name(s_app_mode),
             s_usb_storage_ready ? "usb" : (s_sd_mounted ? "app" : "none"),
             s_storage_quiescing ? "true" : "false",
             s_sd_mounted ? "true" : "false",
             storage_tf_ready() ? "true" : "false",
             message ? message : "operation unavailable",
             action ? action : "check device status");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t send_usb_storage_json(httpd_req_t *req)
{
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":false,\"sd_mounted\":false,\"usb_storage_owner\":\"usb\","
        "\"error\":\"TF card is exported to the computer over USB; safely eject it before restoring TF to the device\"}");
}

static esp_err_t send_storage_unavailable_text(httpd_req_t *req)
{
    if (s_storage_quiescing) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage is switching to USB ownership");
    }
    if (storage_usb_owned()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(
            req,
            "TF card is exported to the computer over USB; safely eject it before restoring TF to the device");
    }
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_sendstr(req, "TF card is not mounted");
}

static esp_err_t recordings_get_handler(httpd_req_t *req)
{
    record_http_poll(req);
    if (recording_cleanup_active() ||
        storage_transition_owner() == STORAGE_TRANSITION_RECORDING_CLEANUP) {
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return send_customer_action_json(
            req, "423 Locked", "recording_cleanup_running",
            "recording cleanup is running in the background",
            "wait for the cleanup status on the Web page before refreshing recordings");
    }
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 20;
    bool has_query = false;
    esp_err_t query_ret = read_optional_url_query(req, query, sizeof(query), &has_query);
    if (query_ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "recordings query is too long or unreadable");
    }
    esp_err_t limit_ret = has_query ?
                          httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) :
                          ESP_ERR_NOT_FOUND;
    if (limit_ret != ESP_OK && limit_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "limit parameter is too long or malformed");
    }
    if (limit_ret == ESP_OK) {
        if (!query_u32(query, "limit", 1, 100, &limit)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit must be an integer in range 1..100");
        }
    }
    if (s_storage_quiescing || storage_usb_owned()) {
        return send_usb_storage_json(req);
    }

    size_t cap = 1536 + (size_t)limit * JSONL_TAIL_LINE_BYTES * 2U;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no recordings buffer");
    }

    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    json_writer_appendf(&writer, "{\"sd_mounted\":%s,\"storage_backend\":",
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, s_storage_backend);
    json_writer_appendf(&writer, ",\"storage_status\":");
    json_writer_append_escaped_string(&writer, s_storage_status);
    json_writer_appendf(&writer,
                        ",\"tf_ready\":%s,\"storage_acceptance_ok\":%s"
                        ",\"index_version\":%" PRIu32 ",\"recording_enabled\":%s"
                        ",\"segments\":%" PRIu32 ",\"frames\":%" PRIu32
                        ",\"queued\":%" PRIu32 ",\"dropped\":%" PRIu32
                        ",\"deleted\":%" PRIu32 ",\"sd_errors\":%" PRIu32
                        ",\"summary_count\":%" PRIu32 ",\"bytes\":%" PRIu64
                        ",\"current_uri\":",
                        storage_tf_ready() ? "true" : "false",
                        storage_acceptance_ok() ? "true" : "false",
                        (uint32_t)APP_JSONL_INDEX_VERSION,
                        s_recording_enabled ? "true" : "false",
                        s_recording_segments, s_recording_frames,
                        s_recording_queued, s_recording_dropped, s_recording_files_deleted,
                        s_recording_sd_errors, s_recording_summary_count,
                        (uint64_t)s_recording_bytes);
    json_writer_append_escaped_string(&writer, s_recording_current_uri);
    json_writer_appendf(&writer,
                        ",\"current_frames\":%" PRIu32 ",\"current_bytes\":%" PRIu64
                        ",\"recordings\":",
                        s_recording_current_frames, (uint64_t)s_recording_current_bytes);
    append_recent_jsonl_array(&writer, RECORDING_INDEX_PATH, limit);
    json_writer_appendf(&writer, ",\"recording_groups\":");
    append_recording_groups_json(&writer, limit);
    json_writer_appendf(&writer, ",\"summaries\":");
    append_recent_jsonl_array(&writer, RECORDING_SUMMARY_PATH, limit);
    json_writer_appendf(&writer, "}");

    if (!json_writer_ok(&writer)) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "recordings response exceeds safe buffer");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t timeline_async_handler(httpd_req_t *req)
{
    esp_err_t ret = timeline_get_handler_internal(req);
    file_download_reader_end();
    return ret;
}

static esp_err_t timeline_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, timeline_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no storage reader worker available");
    }
    return ESP_OK;
}

static bool recording_storage_jobs_busy_for_mutation(void)
{
    recording_enrichment_status_t enrichment = {0};
    recording_enrichment_get_status(&enrichment);
    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    return recording_enrichment_has_request() || enrichment.running ||
           dataset.queued || dataset.running ||
           __atomic_load_n(&s_history_worker_busy, __ATOMIC_ACQUIRE) ||
           (s_history_queue && uxQueueMessagesWaiting(s_history_queue) > 0) ||
           (s_recording_queue && uxQueueMessagesWaiting(s_recording_queue) > 0) ||
           s_recording_current_uri[0] != '\0';
}

static esp_err_t queue_recording_cleanup_response(httpd_req_t *req)
{
    recording_cleanup_status_t current = {0};
    recording_cleanup_status_copy(&current);
    if (current.state == RECORDING_CLEANUP_QUEUED ||
        current.state == RECORDING_CLEANUP_RUNNING) {
        char json[320];
        snprintf(json, sizeof(json),
                 "{\"ok\":true,\"queued\":true,\"coalesced\":true,"
                 "\"job_id\":%" PRIu32 ",\"state\":\"%s\","
                 "\"status_uri\":\"/api/status\","
                 "\"message\":\"recording cleanup is already in progress\"}",
                 current.job_id, recording_cleanup_state_name(current.state));
        httpd_resp_set_status(req, "202 Accepted");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return http_send_cstr_chunked(req, json);
    }
    if (!s_recording_cleanup_queue || !s_storage_lock) {
        return send_customer_action_json(
            req, "503 Service Unavailable", "cleanup_unavailable",
            "recording cleanup service is not available",
            "retry after the device finishes starting; reboot only if the service remains unavailable");
    }
    if (s_storage_quiescing || storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_or_storage_transition",
            "recordings cannot be cleaned while TF ownership is changing or exported over USB",
            "finish USB export or storage recovery, then retry cleanup from Web");
    }
    if (s_app_mode != APP_MODE_SERVER || s_network_shutdown_for_idle) {
        return send_customer_action_json(
            req, "409 Conflict", "mode_busy",
            "recordings can only be cleaned in Web server mode",
            "return to server mode and retry cleanup");
    }
    if (!s_sd_mounted) {
        return send_customer_action_json(
            req, "503 Service Unavailable", "tf_not_mounted",
            "TF card is not mounted",
            "check or replace the TF card, then use Check and retry TF before cleanup");
    }
    if (recording_storage_jobs_busy_for_mutation()) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "409 Conflict", "storage_job_active",
            "a recording, fill-frame, dataset, or history storage job is still active",
            "wait for the active job to finish, then retry cleanup");
    }

    uint32_t downloads = __atomic_load_n(&s_file_download_clients,
                                         __ATOMIC_ACQUIRE);
    if (downloads > 0) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "409 Conflict", "file_transfer_active",
            "a recording download or upload is still active",
            "wait for the transfer to finish or cancel it, then retry cleanup");
    }
    if (!storage_transition_try_acquire(
            STORAGE_TRANSITION_RECORDING_CLEANUP)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage operation is starting or running",
            "wait for the current operation to finish and retry cleanup");
    }

    downloads = __atomic_load_n(&s_file_download_clients,
                                __ATOMIC_ACQUIRE);
    if (downloads > 0 || recording_storage_jobs_busy_for_mutation()) {
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "409 Conflict", "storage_in_use",
            "a file transfer or storage job started before cleanup",
            "wait for it to finish, then retry cleanup");
    }
    if (xSemaphoreTake(s_storage_lock, 0) != pdTRUE) {
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "409 Conflict", "storage_in_use",
            "storage became busy before cleanup could start",
            "wait for the current download or storage task to finish and retry cleanup");
    }

    uint32_t job_id = __atomic_add_fetch(
        &s_recording_cleanup_job_sequence, 1, __ATOMIC_ACQ_REL);
    if (job_id == 0) {
        job_id = __atomic_add_fetch(
            &s_recording_cleanup_job_sequence, 1, __ATOMIC_ACQ_REL);
    }
    recording_cleanup_request_t request = {.job_id = job_id};
    recording_cleanup_status_set_queued(job_id);
    BaseType_t queued = xQueueSend(s_recording_cleanup_queue, &request, 0);
    xSemaphoreGive(s_storage_lock);
    if (queued != pdTRUE) {
        recording_cleanup_status_finish(
            job_id, false, 0, 0, 0, 1, EBUSY,
            "recording cleanup queue is busy; retry from Web");
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        return send_customer_action_json(
            req, "503 Service Unavailable", "cleanup_queue_busy",
            "recording cleanup could not be queued",
            "wait briefly and retry; no recording was deleted");
    }

    field_idle_pause_latch();
    open_network_access_window("recording cleanup queued from Web");
    ESP_LOGI(TAG, "queued recording cleanup job=%" PRIu32, job_id);
    char json[320];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":true,\"coalesced\":false,"
             "\"job_id\":%" PRIu32 ",\"state\":\"queued\","
             "\"status_uri\":\"/api/status\","
             "\"message\":\"recording cleanup queued; Web remains available\"}",
             job_id);
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Retry-After", "1");
    return http_send_cstr_chunked(req, json);
}

static bool recording_mutation_try_begin(httpd_req_t *req)
{
    if (recording_storage_jobs_busy_for_mutation()) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        send_customer_action_json(
            req, "409 Conflict", "storage_job_active",
            "a recording, fill-frame, dataset, or history storage job is still active",
            "wait for the active job to finish, then retry the delete operation");
        return false;
    }
    if (__atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) > 0) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        send_customer_action_json(
            req, "409 Conflict", "file_transfer_active",
            "a file download or upload is still active",
            "wait for the transfer to finish or cancel it, then retry the delete operation");
        return false;
    }
    if (!storage_transition_try_acquire(
            STORAGE_TRANSITION_RECORDING_CLEANUP)) {
        send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage operation is starting or running",
            "wait for the current operation to finish and retry the delete operation");
        return false;
    }
    if (__atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) > 0 ||
        recording_storage_jobs_busy_for_mutation()) {
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        httpd_resp_set_hdr(req, "Retry-After", "2");
        send_customer_action_json(
            req, "409 Conflict", "storage_in_use",
            "a file transfer or storage job started before deletion",
            "wait for it to finish, then retry the delete operation");
        return false;
    }
    if (!s_storage_lock || xSemaphoreTake(s_storage_lock, 0) != pdTRUE) {
        storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
        httpd_resp_set_hdr(req, "Retry-After", "2");
        send_customer_action_json(
            req, "409 Conflict", "storage_in_use",
            "storage became busy before deletion could start",
            "wait for the current transfer or storage task to finish and retry");
        return false;
    }
    field_idle_pause_latch();
    return true;
}

static void recording_mutation_end(void)
{
    xSemaphoreGive(s_storage_lock);
    storage_transition_release(STORAGE_TRANSITION_RECORDING_CLEANUP);
    field_idle_pause_latch();
}

static esp_err_t cleanup_recordings_post_handler(httpd_req_t *req)
{
    record_http_request(req);
    return queue_recording_cleanup_response(req);
}

static esp_err_t recordings_delete_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[64] = {0};
    if (!query_confirm_delete(req, query, sizeof(query))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "confirm=DELETE required");
    }
    return queue_recording_cleanup_response(req);
}

static esp_err_t recording_enrich_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
    if (recording_cleanup_active() ||
        storage_transition_owner() == STORAGE_TRANSITION_RECORDING_CLEANUP) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "423 Locked", "recording_cleanup_running",
            "recording cleanup is running in the background",
            "wait for cleanup to finish before starting fill-frame processing");
    }
    if (storage_transition_active()) {
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage operation is starting or running",
            "wait for the current storage operation to finish, then retry fill-frame processing");
    }
    char query[160] = {0};
    char name[96] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name) ||
        strncmp(name, "raw_", 4) != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "raw recording name required");
    }
    if (s_storage_quiescing || storage_usb_owned()) {
        return send_usb_storage_json(req);
    }
    if (!s_sd_mounted) {
        return send_storage_unavailable_text(req);
    }
    if (!s_storage_lock ||
        xSemaphoreTake(s_storage_lock,
                       pdMS_TO_TICKS(RECORDING_CLEANUP_LOCK_TIMEOUT_MS)) !=
            pdTRUE) {
        httpd_resp_set_hdr(req, "Retry-After", "1");
        return send_customer_action_json(
            req, "409 Conflict", "storage_in_use",
            "storage is finishing another operation",
            "wait briefly, then retry fill-frame processing");
    }
    if (storage_transition_active()) {
        bool cleanup_active = recording_cleanup_active() ||
            storage_transition_owner() == STORAGE_TRANSITION_RECORDING_CLEANUP;
        xSemaphoreGive(s_storage_lock);
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return send_customer_action_json(
            req, cleanup_active ? "423 Locked" : "409 Conflict",
            cleanup_active ? "recording_cleanup_running" : "storage_busy",
            cleanup_active ? "recording cleanup started before fill-frame processing" :
                             "another storage operation started before fill-frame processing",
            "wait for the current storage operation to finish, then retry");
    }
    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    struct stat st = {0};
    if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
        xSemaphoreGive(s_storage_lock);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "raw recording not found");
    }
    esp_err_t err = recording_enrichment_request(name);
    xSemaphoreGive(s_storage_lock);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req,
                                  "fill-frame request is already active; wait for it to finish and retry");
    }
    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(50));

    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":\"%s\",\"mode\":\"fill_frames\"}",
             name);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_files_get_handler_internal(httpd_req_t *req)
{
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 200;
    bool has_query = false;
    esp_err_t query_ret = read_optional_url_query(req, query, sizeof(query), &has_query);
    if (query_ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "storage files query is too long or unreadable");
    }
    esp_err_t limit_ret = has_query ?
                          httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) :
                          ESP_ERR_NOT_FOUND;
    if (limit_ret != ESP_OK && limit_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "limit parameter is too long or malformed");
    }
    if (limit_ret == ESP_OK) {
        if (!query_u32(query, "limit", 1, 1000, &limit)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit must be an integer in range 1..1000");
        }
    }
    if (s_storage_quiescing || storage_usb_owned()) {
        return send_usb_storage_json(req);
    }

    size_t cap = 256U + (size_t)limit * 192U;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no storage files buffer");
    }

    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    json_writer_appendf(&writer, "{\"ok\":true,\"sd_mounted\":%s,\"backend\":",
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, s_storage_backend);
    json_writer_appendf(&writer, ",\"files\":[");
    uint32_t returned = 0;
    bool first = true;
    if (s_sd_mounted) {
        DIR *dir = opendir(RECORDING_DIR);
        if (dir) {
            struct dirent *entry;
            while (returned < limit && json_writer_ok(&writer) &&
                   (entry = readdir(dir)) != NULL) {
                const char *name = entry->d_name;
                if (!is_safe_snapshot_name(name) ||
                    !(has_suffix(name, ".avi") || has_suffix(name, ".avi.part") ||
                      has_suffix(name, ".mjpg") || has_suffix(name, ".jsonl"))) {
                    continue;
                }
                char path[384];
                struct stat st = {0};
                snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
                if (stat(path, &st) != 0 || !S_ISREG(st.st_mode)) {
                    continue;
                }
                json_writer_appendf(&writer, "%s{\"name\":", first ? "" : ",");
                json_writer_append_escaped_string(&writer, name);
                json_writer_appendf(&writer, ",\"bytes\":%lld,\"part\":%s}",
                                    (long long)st.st_size,
                                    has_suffix(name, ".part") ? "true" : "false");
                if (!json_writer_ok(&writer)) {
                    break;
                }
                first = false;
                returned++;
            }
            closedir(dir);
        }
    }
    json_writer_appendf(&writer, "],\"returned\":%" PRIu32 "}", returned);
    if (!json_writer_ok(&writer)) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "storage files response exceeds safe buffer");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t storage_files_async_handler(httpd_req_t *req)
{
    esp_err_t ret = storage_files_get_handler_internal(req);
    file_download_reader_end();
    return ret;
}

static esp_err_t storage_files_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, storage_files_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no storage reader worker available");
    }
    return ESP_OK;
}

static esp_err_t http_socket_send_all(httpd_req_t *req, const char *buf, size_t len)
{
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd < 0 || !buf) {
        return ESP_FAIL;
    }

    size_t off = 0;
    while (off < len) {
        if (http_server_is_stopping()) {
            return ESP_ERR_INVALID_STATE;
        }
        int sent = httpd_socket_send(req->handle, sockfd, buf + off, len - off, 0);
        if (sent == HTTPD_SOCK_ERR_TIMEOUT) {
            ESP_LOGW(TAG,
                     "file transfer stopped after %u seconds without socket progress",
                     (unsigned)HTTP_FILE_SEND_WAIT_TIMEOUT_SEC);
            return ESP_ERR_TIMEOUT;
        }
        if (sent <= 0) {
            return ESP_FAIL;
        }
        off += (size_t)sent;
    }
    return ESP_OK;
}

static bool parse_u64_decimal_span(const char *text, size_t length, uint64_t *value)
{
    if (!text || length == 0 || !value) {
        return false;
    }
    uint64_t parsed = 0;
    for (size_t i = 0; i < length; i++) {
        if (text[i] < '0' || text[i] > '9') {
            return false;
        }
        uint32_t digit = (uint32_t)(text[i] - '0');
        if (parsed > (UINT64_MAX - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return true;
}

static esp_err_t send_invalid_range(httpd_req_t *req, uint64_t file_size,
                                    const char *message)
{
    char content_range[64];
    snprintf(content_range, sizeof(content_range), "bytes */%" PRIu64, file_size);
    httpd_resp_set_status(req, "416 Range Not Satisfiable");
    httpd_resp_set_hdr(req, "Content-Range", content_range);
    httpd_resp_set_hdr(req, "Accept-Ranges", "bytes");
    return httpd_resp_sendstr(req, message ? message : "invalid byte range");
}

static bool file_download_should_abort(void)
{
    return http_server_is_stopping() || s_storage_quiescing ||
           storage_usb_owned() || storage_transition_active();
}

static void file_download_reader_end(void)
{
    uint32_t current = __atomic_load_n(&s_file_download_clients,
                                       __ATOMIC_ACQUIRE);
    while (current > 0) {
        uint32_t desired = current - 1U;
        if (__atomic_compare_exchange_n(&s_file_download_clients, &current,
                                        desired, false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE)) {
            return;
        }
    }
    ESP_LOGE(TAG, "file transfer reader counter underflow prevented");
}

static bool file_download_reader_try_begin(void)
{
    if (file_download_should_abort()) {
        return false;
    }
    __atomic_add_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);
    if (file_download_should_abort()) {
        file_download_reader_end();
        return false;
    }
    return true;
}

static esp_err_t send_file_download_unavailable(httpd_req_t *req)
{
    if (recording_cleanup_active() ||
        storage_transition_owner() == STORAGE_TRANSITION_RECORDING_CLEANUP) {
        httpd_resp_set_status(req, "423 Locked");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(
            req, "recording cleanup is running; retry the download when the Web page shows completion");
    }
    if (http_server_is_stopping() || s_storage_quiescing ||
        storage_transition_active()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(
            req, "file transfer is temporarily paused for storage maintenance; retry shortly");
    }
    return send_storage_unavailable_text(req);
}

static esp_err_t send_file_storage_busy(httpd_req_t *req)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_hdr(req, "Retry-After", "1");
    return httpd_resp_sendstr(
        req, "storage is finishing another operation; retry the file transfer shortly");
}

static esp_err_t send_file_response_internal(httpd_req_t *req, const char *path,
                                             const char *type,
                                             bool reader_reserved)
{
    if (!reader_reserved && !file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (reader_reserved && file_download_should_abort()) {
        file_download_reader_end();
        return send_file_download_unavailable(req);
    }
    if (!s_storage_lock ||
        xSemaphoreTake(s_storage_lock,
                       pdMS_TO_TICKS(RECORDING_CLEANUP_LOCK_TIMEOUT_MS)) !=
            pdTRUE) {
        file_download_reader_end();
        return send_file_storage_busy(req);
    }
    if (file_download_should_abort()) {
        xSemaphoreGive(s_storage_lock);
        file_download_reader_end();
        return send_file_download_unavailable(req);
    }
    struct stat st = {0};
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        xSemaphoreGive(s_storage_lock);
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
    }
    if (fstat(fd, &st) != 0 || st.st_size < 0 || !S_ISREG(st.st_mode)) {
        close(fd);
        xSemaphoreGive(s_storage_lock);
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "file metadata is unavailable");
    }
    xSemaphoreGive(s_storage_lock);

    uint64_t start = 0;
    uint64_t end = st.st_size > 0 ? (uint64_t)st.st_size - 1U : 0;
    bool partial = false;
    char range[96] = {0};
    size_t range_len = httpd_req_get_hdr_value_len(req, "Range");
    if (range_len > 0) {
        if (range_len >= sizeof(range) ||
            httpd_req_get_hdr_value_str(req, "Range", range, sizeof(range)) != ESP_OK) {
            close(fd);
            file_download_reader_end();
            return send_invalid_range(req, (uint64_t)st.st_size,
                                      "Range header is too long or unreadable");
        }
        if (st.st_size == 0 || strncmp(range, "bytes=", 6) != 0) {
            close(fd);
            file_download_reader_end();
            return send_invalid_range(req, (uint64_t)st.st_size,
                                      "only a single bytes range is supported");
        }
        char *spec = range + 6;
        char *dash = strchr(spec, '-');
        if (!dash || strchr(spec, ',') || strchr(dash + 1, '-') ||
            (dash == spec && dash[1] == '\0')) {
            close(fd);
            file_download_reader_end();
            return send_invalid_range(req, (uint64_t)st.st_size,
                                      "invalid byte range syntax");
        }
        size_t start_len = (size_t)(dash - spec);
        size_t end_len = strlen(dash + 1);
        *dash = '\0';
        if (start_len > 0) {
            if (!parse_u64_decimal_span(spec, start_len, &start) ||
                (end_len > 0 &&
                 !parse_u64_decimal_span(dash + 1, end_len, &end))) {
                close(fd);
                file_download_reader_end();
                return send_invalid_range(req, (uint64_t)st.st_size,
                                          "byte range must contain decimal integers only");
            }
        } else {
            uint64_t suffix = 0;
            if (!parse_u64_decimal_span(dash + 1, end_len, &suffix) || suffix == 0) {
                close(fd);
                file_download_reader_end();
                return send_invalid_range(req, (uint64_t)st.st_size,
                                          "suffix byte range must be greater than zero");
            }
            if (suffix > (uint64_t)st.st_size) {
                suffix = st.st_size;
            }
            start = (uint64_t)st.st_size - suffix;
        }
        if (start >= (uint64_t)st.st_size || end < start) {
            close(fd);
            file_download_reader_end();
            return send_invalid_range(req, (uint64_t)st.st_size,
                                      "byte range is outside the file");
        }
        if (end >= (uint64_t)st.st_size) {
            end = (uint64_t)st.st_size - 1U;
        }
        partial = true;
    }

    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Accept-Ranges", "bytes");
    if (partial) {
        char content_range[96];
        snprintf(content_range, sizeof(content_range), "bytes %" PRIu64 "-%" PRIu64 "/%lld",
                 start, end, (long long)st.st_size);
        httpd_resp_set_status(req, "206 Partial Content");
        httpd_resp_set_hdr(req, "Content-Range", content_range);
    }
    if (start > 0 && lseek(fd, (off_t)start, SEEK_SET) < 0) {
        close(fd);
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "seek failed");
    }

    uint8_t *buf = malloc(HTTP_SAFE_CHUNK_BYTES);
    if (!buf) {
        close(fd);
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    esp_err_t ret = ESP_OK;
    uint64_t remaining = st.st_size > 0 ? end - start + 1U : 0;
    bool fixed_length = remaining <= (uint64_t)INT_MAX;
    if (fixed_length) {
        ret = httpd_resp_send(req, NULL, (ssize_t)remaining);
        if (ret == ESP_OK) {
            while (remaining > 0) {
                if (file_download_should_abort()) {
                    ret = ESP_ERR_INVALID_STATE;
                    break;
                }
                size_t want = remaining > HTTP_SAFE_CHUNK_BYTES ? HTTP_SAFE_CHUNK_BYTES : (size_t)remaining;
                ssize_t n = read(fd, buf, want);
                if (n <= 0) {
                    ret = ESP_FAIL;
                    break;
                }
                ret = http_socket_send_all(req, (const char *)buf, (size_t)n);
                if (ret != ESP_OK) {
                    break;
                }
                remaining -= (uint64_t)n;
            }
        }
    } else {
        while (remaining > 0) {
            if (file_download_should_abort()) {
                ret = ESP_ERR_INVALID_STATE;
                break;
            }
            size_t want = remaining > HTTP_SAFE_CHUNK_BYTES ? HTTP_SAFE_CHUNK_BYTES : (size_t)remaining;
            ssize_t n = read(fd, buf, want);
            if (n > 0) {
                ret = http_send_chunk_part(req, (const char *)buf, (size_t)n);
                if (ret != ESP_OK) {
                    break;
                }
                remaining -= (uint64_t)n;
            } else {
                ret = ESP_FAIL;
                break;
            }
        }
    }
    free(buf);
    close(fd);
    file_download_reader_end();

    if (ret == ESP_OK && !fixed_length && !file_download_should_abort()) {
        ret = httpd_resp_send_chunk(req, NULL, 0);
    }
    return ret;
}

static esp_err_t history_file_async_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        file_download_reader_end();
        return send_storage_unavailable_text(req);
    }
    return send_file_response_internal(req, HISTORY_JSONL_PATH,
                                       "application/x-ndjson", true);
}

static esp_err_t history_file_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!s_sd_mounted) {
        return send_storage_unavailable_text(req);
    }
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, history_file_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static esp_err_t snapshot_async_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        file_download_reader_end();
        return send_storage_unavailable_text(req);
    }

    const char *name = req->uri + strlen(SNAPSHOT_URI_PREFIX);
    if (!is_safe_snapshot_name(name)) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad snapshot name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, name);
    return send_file_response_internal(req, path, "image/jpeg", true);
}

static esp_err_t snapshot_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!s_sd_mounted) {
        return send_storage_unavailable_text(req);
    }
    const char *name = req->uri + strlen(SNAPSHOT_URI_PREFIX);
    if (!is_safe_snapshot_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad snapshot name");
    }
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, snapshot_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_async_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        file_download_reader_end();
        return send_storage_unavailable_text(req);
    }

    const char *name = req->uri + strlen(RECORDING_URI_PREFIX);
    if (!is_safe_recording_name(name)) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    return send_file_response_internal(req, path,
                                       has_suffix(name, ".avi") ?
                                       "video/x-msvideo" : STREAM_CONTENT_TYPE,
                                       true);
}

static esp_err_t recording_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!s_sd_mounted) {
        return send_storage_unavailable_text(req);
    }
    const char *name = req->uri + strlen(RECORDING_URI_PREFIX);
    if (!is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording name");
    }
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, recording_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_meta_async_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        file_download_reader_end();
        return send_storage_unavailable_text(req);
    }

    const char *name = req->uri + strlen(RECORDING_META_URI_PREFIX);
    if (!is_safe_recording_meta_name(name)) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording meta name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    return send_file_response_internal(req, path, "application/x-ndjson", true);
}

static esp_err_t recording_meta_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!s_sd_mounted) {
        return send_storage_unavailable_text(req);
    }
    const char *name = req->uri + strlen(RECORDING_META_URI_PREFIX);
    if (!is_safe_recording_meta_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "bad recording meta name");
    }
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, recording_meta_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static bool query_confirm_delete(httpd_req_t *req, char *query, size_t query_size)
{
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, query_size) != ESP_OK) {
        return false;
    }
    if (httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK) {
        return false;
    }
    return strcmp(confirm, "DELETE") == 0;
}

static bool recording_time_from_name(const char *name, int64_t *time_ms)
{
    return recording_time_mark_from_name(name, time_ms);
}

static int delete_path_if_file(const char *path, uint64_t *freed_bytes)
{
    struct stat st = {0};
    if (!path) {
        return -1;
    }
    if (stat(path, &st) != 0) {
        return errno == ENOENT ? 0 : -1;
    }
    if (S_ISDIR(st.st_mode)) {
        return 0;
    }
    if (unlink(path) == 0) {
        if (freed_bytes) {
            *freed_bytes += (uint64_t)st.st_size;
        }
        return 1;
    }
    return -1;
}

static int delete_recording_files_by_name(const char *name, uint64_t *freed_bytes,
                                          bool *failed)
{
    if (failed) {
        *failed = false;
    }
    if (!is_safe_recording_name(name)) {
        if (failed) {
            *failed = true;
        }
        return 0;
    }

    int deleted = 0;
    int first_errno = 0;
    char path[384];
    const char *media_suffixes[] = {"", ".part", ".part.corrupt"};
    for (size_t i = 0; i < sizeof(media_suffixes) / sizeof(media_suffixes[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s%s", RECORDING_DIR, name, media_suffixes[i]);
        errno = 0;
        int ret = delete_path_if_file(path, freed_bytes);
        if (ret > 0) {
            deleted += ret;
        } else if (ret < 0 && failed) {
            *failed = true;
            if (first_errno == 0) {
                first_errno = errno ? errno : EIO;
            }
        }
    }

    char meta_name[96];
    meta_name_for_recording(name, meta_name, sizeof(meta_name));
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, meta_name);
    errno = 0;
    int ret = delete_path_if_file(path, freed_bytes);
    if (ret > 0) {
        deleted += ret;
    } else if (ret < 0 && failed) {
        *failed = true;
        if (first_errno == 0) {
            first_errno = errno ? errno : EIO;
        }
    }

    /* 删除同名 .idx 附属文件 */
    char idx_name[96];
    strlcpy(idx_name, name, sizeof(idx_name));
    size_t name_len = strlen(idx_name);
    if (name_len > 4 && strcmp(idx_name + name_len - 4, ".avi") == 0) {
        strcpy(idx_name + name_len - 4, ".idx");
    } else {
        strlcat(idx_name, ".idx", sizeof(idx_name));
    }
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, idx_name);
    errno = 0;
    ret = delete_path_if_file(path, freed_bytes);
    if (ret > 0) {
        deleted += ret;
    } else if (ret < 0 && failed) {
        *failed = true;
        if (first_errno == 0) {
            first_errno = errno ? errno : EIO;
        }
    }
    errno = first_errno;
    return deleted;
}

typedef struct {
    uint32_t removed;
    bool failed;
} jsonl_filter_result_t;

static bool json_line_string_equals(const char *line, const char *key, const char *value)
{
    if (!line || !key || !value) {
        return false;
    }
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":\"", key);
    const char *start = strstr(line, needle);
    if (!start) {
        return false;
    }
    start += strlen(needle);
    size_t value_len = strlen(value);
    return strncmp(start, value, value_len) == 0 && start[value_len] == '"';
}

static bool replace_jsonl_file(const char *path, const char *tmp_path)
{
    char backup_path[384];
    snprintf(backup_path, sizeof(backup_path), "%s.bak", path);
    errno = 0;
    if (unlink(backup_path) != 0 && errno != ENOENT) {
        unlink(tmp_path);
        return false;
    }
    if (rename(path, backup_path) != 0) {
        unlink(tmp_path);
        return false;
    }
    if (rename(tmp_path, path) != 0) {
        rename(backup_path, path);
        unlink(tmp_path);
        return false;
    }
    errno = 0;
    return unlink(backup_path) == 0 || errno == ENOENT;
}

static jsonl_filter_result_t filter_jsonl_string_field(const char *path, const char *tmp_path,
                                                        const char *key, const char *value)
{
    jsonl_filter_result_t result = {0};
    FILE *in = fopen(path, "r");
    if (!in) {
        result.failed = errno != ENOENT;
        return result;
    }
    FILE *out = fopen(tmp_path, "w");
    if (!out) {
        fclose(in);
        result.failed = true;
        return result;
    }

    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        fclose(in);
        fclose(out);
        unlink(tmp_path);
        result.failed = true;
        return result;
    }

    while (fgets(line, JSONL_TAIL_LINE_BYTES, in)) {
        if (json_line_string_equals(line, key, value)) {
            result.removed++;
        } else if (fputs(line, out) == EOF) {
            result.failed = true;
            break;
        }
    }
    if (ferror(in)) {
        result.failed = true;
    }
    free(line);
    fclose(in);
    if (fflush(out) != 0) {
        result.failed = true;
    }
    if (fsync(fileno(out)) != 0) {
        result.failed = true;
    }
    if (fclose(out) != 0) {
        result.failed = true;
    }

    if (result.failed) {
        unlink(tmp_path);
        return result;
    }
    if (result.removed == 0) {
        unlink(tmp_path);
        return result;
    }
    if (!replace_jsonl_file(path, tmp_path)) {
        result.failed = true;
        result.removed = 0;
    }
    return result;
}

static uint32_t remove_recording_index_rows(const char *name, bool *failed)
{
    const struct {
        const char *path;
        const char *tmp_path;
        const char *key;
    } indexes[] = {
        {RECORDING_INDEX_PATH, RECORDING_INDEX_TMP_PATH, "name"},
        {RECORDING_INDEX_OLD_PATH, RECORDING_INDEX_OLD_TMP_PATH, "name"},
        {RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_TMP_PATH, "segment"},
        {RECORDING_SUMMARY_OLD_PATH, RECORDING_SUMMARY_OLD_TMP_PATH, "segment"},
        {EVENT_INDEX_PATH, EVENT_INDEX_TMP_PATH, "segment"},
    };
    uint32_t removed = 0;
    bool any_failed = false;
    for (size_t i = 0; i < sizeof(indexes) / sizeof(indexes[0]); i++) {
        jsonl_filter_result_t result = filter_jsonl_string_field(
            indexes[i].path, indexes[i].tmp_path, indexes[i].key, name);
        removed += result.removed;
        any_failed |= result.failed;
    }
    if (failed) {
        *failed = any_failed;
    }
    return removed;
}

static int64_t json_line_i64(const char *line, const char *key)
{
    int64_t value = -1;
    return json_get_int64_field(line, key, &value) ? value : -1;
}

static uint32_t filter_jsonl_time_range(const char *path, const char *tmp_path,
                                        const char *key, int64_t from_ms,
                                        int64_t to_ms, bool *operation_failed)
{
    FILE *in = fopen(path, "r");
    if (!in) {
        if (errno != ENOENT && operation_failed) {
            *operation_failed = true;
        }
        return 0;
    }
    FILE *out = fopen(tmp_path, "w");
    if (!out) {
        fclose(in);
        if (operation_failed) {
            *operation_failed = true;
        }
        return 0;
    }

    uint32_t removed = 0;
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        fclose(in);
        fclose(out);
        unlink(tmp_path);
        if (operation_failed) {
            *operation_failed = true;
        }
        return 0;
    }

    bool failed = false;
    while (fgets(line, JSONL_TAIL_LINE_BYTES, in)) {
        int64_t t = json_line_i64(line, key);
        bool in_range = t >= from_ms && t <= to_ms;
        if (in_range) {
            removed++;
        } else if (fputs(line, out) == EOF) {
            failed = true;
            break;
        }
    }
    free(line);
    failed |= ferror(in);
    fclose(in);
    if (fflush(out) != 0) {
        failed = true;
    }
    if (fsync(fileno(out)) != 0) {
        failed = true;
    }
    if (fclose(out) != 0) {
        failed = true;
    }
    if (failed) {
        unlink(tmp_path);
        if (operation_failed) {
            *operation_failed = true;
        }
        return 0;
    }
    if (removed == 0) {
        unlink(tmp_path);
        return 0;
    }
    if (!replace_jsonl_file(path, tmp_path)) {
        if (operation_failed) {
            *operation_failed = true;
        }
        return 0;
    }
    return removed;
}

static uint32_t delete_recordings_in_range(int64_t from_ms, int64_t to_ms,
                                           uint64_t *freed_bytes, bool *failed)
{
    const size_t batch_size = RECORDING_CLEANUP_BATCH_FALLBACK;
    char (*names)[RECORDING_CLEANUP_NAME_SIZE] =
        malloc(batch_size * RECORDING_CLEANUP_NAME_SIZE);
    if (!names) {
        if (failed) {
            *failed = true;
        }
        return 0;
    }
    uint32_t deleted = 0;
    bool any_failed = false;
    for (;;) {
        size_t count = 0;
        bool scan_failed = false;
        errno = 0;
        DIR *dir = opendir(RECORDING_DIR);
        if (!dir) {
            if (errno != ENOENT) {
                any_failed = true;
            }
            break;
        }
        struct dirent *entry;
        errno = 0;
        while (count < batch_size && (entry = readdir(dir)) != NULL) {
            int64_t t = 0;
            if (!is_safe_recording_name(entry->d_name) ||
                !recording_time_from_name(entry->d_name, &t) ||
                t < from_ms || t > to_ms) {
                continue;
            }
            if (strlcpy(names[count], entry->d_name,
                        RECORDING_CLEANUP_NAME_SIZE) >=
                RECORDING_CLEANUP_NAME_SIZE) {
                any_failed = true;
                continue;
            }
            count++;
        }
        if (count < batch_size && errno != 0) {
            any_failed = true;
            scan_failed = true;
        }
        if (closedir(dir) != 0) {
            any_failed = true;
            scan_failed = true;
        }
        if (count == 0) {
            break;
        }
        uint32_t deleted_this_pass = 0;
        for (size_t i = 0; i < count; i++) {
            bool delete_failed = false;
            int ret = delete_recording_files_by_name(
                names[i], freed_bytes, &delete_failed);
            if (ret > 0) {
                deleted += (uint32_t)ret;
                deleted_this_pass += (uint32_t)ret;
            }
            any_failed |= delete_failed;
        }
        if (scan_failed || deleted_this_pass == 0) {
            break;
        }
    }
    free(names);
    if (failed) {
        *failed |= any_failed;
    }
    return deleted;
}

static uint32_t delete_dir_whitelist(const char *dir_path, const char *suffix_a,
                                     const char *suffix_b, const char *suffix_c,
                                     uint64_t *freed_bytes, bool *failed)
{
    const size_t batch_size = RECORDING_CLEANUP_BATCH_FALLBACK;
    char (*names)[RECORDING_CLEANUP_NAME_SIZE] =
        malloc(batch_size * RECORDING_CLEANUP_NAME_SIZE);
    if (!names) {
        if (failed) {
            *failed = true;
        }
        return 0;
    }
    uint32_t deleted = 0;
    bool any_failed = false;
    for (;;) {
        size_t count = 0;
        bool scan_failed = false;
        errno = 0;
        DIR *dir = opendir(dir_path);
        if (!dir) {
            if (errno != ENOENT) {
                any_failed = true;
            }
            break;
        }
        struct dirent *entry;
        errno = 0;
        while (count < batch_size && (entry = readdir(dir)) != NULL) {
            bool suffix_ok = has_suffix(entry->d_name, suffix_a) ||
                             (suffix_b && has_suffix(entry->d_name, suffix_b)) ||
                             (suffix_c && has_suffix(entry->d_name, suffix_c));
            if (!suffix_ok || strcmp(entry->d_name, ".") == 0 ||
                strcmp(entry->d_name, "..") == 0) {
                continue;
            }
            if (strlcpy(names[count], entry->d_name,
                        RECORDING_CLEANUP_NAME_SIZE) >=
                RECORDING_CLEANUP_NAME_SIZE) {
                any_failed = true;
                continue;
            }
            count++;
        }
        if (count < batch_size && errno != 0) {
            any_failed = true;
            scan_failed = true;
        }
        if (closedir(dir) != 0) {
            any_failed = true;
            scan_failed = true;
        }
        if (count == 0) {
            break;
        }
        uint32_t deleted_this_pass = 0;
        for (size_t i = 0; i < count; i++) {
            char path[384];
            int path_len = snprintf(path, sizeof(path), "%s/%s",
                                    dir_path, names[i]);
            if (path_len < 0 || (size_t)path_len >= sizeof(path)) {
                any_failed = true;
                continue;
            }
            int ret = delete_path_if_file(path, freed_bytes);
            if (ret > 0) {
                deleted += (uint32_t)ret;
                deleted_this_pass += (uint32_t)ret;
            } else if (ret < 0) {
                any_failed = true;
            }
        }
        if (scan_failed || deleted_this_pass == 0) {
            break;
        }
    }
    free(names);
    if (failed) {
        *failed |= any_failed;
    }
    return deleted;
}

static esp_err_t recording_delete_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[320] = {0};
    char name[96] = {0};
    char paired_name[96] = {0};
    if (!query_confirm_delete(req, query, sizeof(query)) ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name and confirm=DELETE required");
    }
    esp_err_t paired_ret =
        httpd_query_key_value(query, "paired_name", paired_name, sizeof(paired_name));
    if (paired_ret != ESP_OK && paired_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "paired_name is too long or malformed");
    }
    if (paired_ret == ESP_OK) {
        if (!is_safe_recording_name(paired_name)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "paired_name is invalid");
        }
        if (strcmp(name, paired_name) == 0) {
            paired_name[0] = '\0';
        }
    }

    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    if (!recording_mutation_try_begin(req)) {
        return ESP_OK;
    }
    uint64_t freed = 0;
    bool file_failed = false;
    int deleted = delete_recording_files_by_name(name, &freed, &file_failed);
    bool index_failed = false;
    uint32_t removed_rows = !file_failed ?
                            remove_recording_index_rows(name, &index_failed) : 0;
    if (paired_name[0]) {
        bool paired_file_failed = false;
        int paired_deleted = delete_recording_files_by_name(
            paired_name, &freed, &paired_file_failed);
        bool paired_index_failed = false;
        uint32_t paired_removed_rows = !paired_file_failed ?
                                       remove_recording_index_rows(
                                           paired_name, &paired_index_failed) : 0;
        deleted += paired_deleted;
        removed_rows += paired_removed_rows;
        file_failed |= paired_file_failed;
        index_failed |= paired_index_failed;
    }
    if (deleted > 0) {
        s_recording_files_deleted += (uint32_t)deleted;
    }
    bool failed = file_failed || index_failed || update_sd_info() != ESP_OK;
    recording_mutation_end();
    bool not_found = !failed && deleted == 0 && removed_rows == 0;

    char json[384];
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"name\":\"%s\",\"paired_name\":\"%s\",\"deleted_files\":%d,"
             "\"removed_index_rows\":%" PRIu32 ",\"freed_bytes\":%" PRIu64
             ",\"error\":\"%s\"}",
             (failed || not_found) ? "false" : "true", name, paired_name, deleted,
             removed_rows, freed, failed ? "partial delete failure" :
             (not_found ? "not found" : ""));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (failed) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    } else if (not_found) {
        httpd_resp_set_status(req, "404 Not Found");
    }
    return http_send_cstr_chunked(req, json);
}

static esp_err_t timeline_delete_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[192] = {0};
    int64_t from_ms = 0;
    int64_t to_ms = 0;
    if (!query_confirm_delete(req, query, sizeof(query)) ||
        !query_i64(query, "from_ms", 1, INT64_MAX, &from_ms) ||
        !query_i64(query, "to_ms", from_ms, INT64_MAX, &to_ms)) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "from_ms and to_ms must be valid integer timestamps with from_ms <= to_ms; confirm=DELETE required");
    }
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    if (!recording_mutation_try_begin(req)) {
        return ESP_OK;
    }
    uint64_t freed = 0;
    bool failed = false;
    uint32_t deleted = delete_recordings_in_range(
        from_ms, to_ms, &freed, &failed);
    uint32_t filtered = 0;
    filtered += filter_jsonl_time_range(RECORDING_INDEX_PATH, RECORDING_INDEX_TMP_PATH, "start_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(RECORDING_INDEX_OLD_PATH, RECORDING_INDEX_OLD_TMP_PATH, "start_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_TMP_PATH, "start_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(RECORDING_SUMMARY_OLD_PATH, RECORDING_SUMMARY_OLD_TMP_PATH, "start_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(HISTORY_JSONL_PATH, HISTORY_JSONL_TMP_PATH, "time_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(HISTORY_JSONL_OLD_PATH, HISTORY_JSONL_OLD_TMP_PATH, "time_ms", from_ms, to_ms, &failed);
    filtered += filter_jsonl_time_range(EVENT_INDEX_PATH, EVENT_INDEX_TMP_PATH, "time_ms", from_ms, to_ms, &failed);
    s_recording_files_deleted += deleted;
    if (update_sd_info() != ESP_OK) {
        failed = true;
    }
    recording_mutation_end();

    char json[256];
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"deleted_files\":%" PRIu32 ",\"filtered_records\":%" PRIu32
             ",\"freed_bytes\":%" PRIu64 ",\"error\":\"%s\"}",
             failed ? "false" : "true", deleted, filtered, freed,
             failed ? "timeline cleanup was incomplete; inspect TF health and retry" : "");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (failed) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    }
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_records_delete_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[96] = {0};
    char scope[16] = {0};
    if (!query_confirm_delete(req, query, sizeof(query)) ||
        httpd_query_key_value(query, "scope", scope, sizeof(scope)) != ESP_OK ||
        strcmp(scope, "all") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "scope=all and confirm=DELETE required");
    }
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    if (!recording_mutation_try_begin(req)) {
        return ESP_OK;
    }
    uint64_t freed = 0;
    uint32_t deleted = 0;
    bool failed = false;
    deleted += delete_dir_whitelist(RECORDING_DIR, ".avi", ".mjpg", ".jsonl", &freed, &failed);
    deleted += delete_dir_whitelist(RECORDING_DIR, ".part", ".idx", ".new", &freed, &failed);
    deleted += delete_dir_whitelist(RECORDING_DIR, ".prev", ".corrupt", NULL, &freed, &failed);
    deleted += delete_dir_whitelist(RECORDING_RECOVERY_DIR, ".avi", ".jsonl", ".part", &freed, &failed);
    deleted += delete_dir_whitelist(RECORDING_RECOVERY_DIR, ".idx", ".new", ".prev", &freed, &failed);
    deleted += delete_dir_whitelist(RECORDING_RECOVERY_DIR, ".corrupt", ".zero_frame", NULL, &freed, &failed);
    deleted += delete_dir_whitelist(HISTORY_SNAPSHOT_DIR, ".jpg", ".jpeg", NULL, &freed, &failed);
    deleted += delete_dir_whitelist(DATASET_RUN_DIR, ".jsonl", ".json", NULL, &freed, &failed);
    const char *indexes[] = {
        HISTORY_JSONL_PATH, HISTORY_JSONL_OLD_PATH,
        HISTORY_JSONL_TMP_PATH, HISTORY_JSONL_OLD_TMP_PATH,
        RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH,
        RECORDING_INDEX_TMP_PATH, RECORDING_INDEX_OLD_TMP_PATH,
        RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_OLD_PATH,
        RECORDING_SUMMARY_TMP_PATH, RECORDING_SUMMARY_OLD_TMP_PATH,
        EVENT_INDEX_PATH, EVENT_INDEX_TMP_PATH, SESSION_INDEX_PATH,
        HISTORY_JSONL_PATH ".bak", HISTORY_JSONL_OLD_PATH ".bak",
        RECORDING_INDEX_PATH ".bak", RECORDING_INDEX_OLD_PATH ".bak",
        RECORDING_SUMMARY_PATH ".bak", RECORDING_SUMMARY_OLD_PATH ".bak",
        EVENT_INDEX_PATH ".bak",
    };
    for (size_t i = 0; i < sizeof(indexes) / sizeof(indexes[0]); i++) {
        int ret = delete_path_if_file(indexes[i], &freed);
        if (ret > 0) {
            deleted += (uint32_t)ret;
        } else if (ret < 0) {
            failed = true;
        }
    }
    s_recording_files_deleted += deleted;
    if (update_sd_info() != ESP_OK) {
        failed = true;
    }
    if (!failed && s_history_lock && s_history_records) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        memset(s_history_records, 0,
               CONFIG_APP_HISTORY_MAX_RECORDS * sizeof(*s_history_records));
        s_history_head = 0;
        s_history_count = 0;
        xSemaphoreGive(s_history_lock);
        s_history_saved = 0;
        s_history_queued = 0;
        s_history_dropped = 0;
        s_history_files_deleted = 0;
    }
    recording_mutation_end();

    char json[256];
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"deleted_files\":%" PRIu32 ",\"freed_bytes\":%" PRIu64
             ",\"error\":\"%s\"}",
             failed ? "false" : "true", deleted, freed,
             failed ? "record cleanup was incomplete; inspect TF health and retry" : "");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (failed) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    }
    return http_send_cstr_chunked(req, json);
}

static bool queue_storage_service_request(const storage_service_request_t *request)
{
    if (!request || !s_storage_service_queue) {
        return false;
    }

    if (!storage_transition_try_acquire(STORAGE_TRANSITION_MAINTENANCE)) {
        return false;
    }
    if (xQueueSend(s_storage_service_queue, request, 0) != pdTRUE) {
        storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
        return false;
    }
    return true;
}

static esp_err_t storage_format_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[96] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "FORMAT") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "confirm=FORMAT required");
    }
    if (!CONFIG_APP_SD_ENABLE) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card support disabled");
    }

    if (!s_storage_service_queue) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage service unavailable");
    }
    if (s_app_mode != APP_MODE_SERVER || s_network_shutdown_for_idle) {
        return send_customer_action_json(
            req, "409 Conflict", "mode_busy",
            "TF can only be formatted in Web server mode",
            "finish field, export, or USB storage mode before formatting the card");
    }
    if (storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_export",
            "TF card is currently owned by the USB host",
            "safely eject P4_BUOY and restore storage before formatting");
    }
    if (s_storage_quiescing || storage_transition_active() ||
        storage_any_request_pending()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage maintenance operation is already running",
            "wait for the current operation to finish and retry from the Web page");
    }

    storage_service_request_t svc = {
        .hold_ms = 500,
        .format_requested = true,
    };
    if (!queue_storage_service_request(&svc)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage maintenance operation was queued first",
            "wait for the current operation to finish and retry from the Web page");
    }

    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":true,\"queued\":true,\"format_requested\":true,"
        "\"temporary_disconnect\":true,\"reboot_required\":false,"
        "\"note\":\"Capture and storage users will pause safely; Web access restores automatically without reboot\"}");
}

static esp_err_t storage_remount_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[128] = {0};
    char confirm[16] = {0};
    char hold_text[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "REMOUNT") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "confirm=REMOUNT required");
    }
    if (!CONFIG_APP_SD_ENABLE || !s_storage_service_queue) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage service unavailable");
    }
    if (s_app_mode != APP_MODE_SERVER || s_network_shutdown_for_idle) {
        return send_customer_action_json(
            req, "409 Conflict", "mode_busy",
            "TF can only be remounted in Web server mode",
            "finish field, export, or USB storage mode before remounting the card");
    }
    if (storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_export",
            "TF card is currently owned by the USB host",
            "safely eject P4_BUOY and restore storage before remounting");
    }
    if (s_storage_quiescing || storage_transition_active() ||
        storage_any_request_pending()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage maintenance operation is already running",
            "wait for the current operation to finish and retry from the Web page");
    }

    storage_service_request_t svc = {
        .hold_ms = 2000,
        .format_requested = false,
    };
    esp_err_t hold_ret =
        httpd_query_key_value(query, "hold_ms", hold_text, sizeof(hold_text));
    if (hold_ret != ESP_OK && hold_ret != ESP_ERR_NOT_FOUND) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "hold_ms is too long or malformed");
    }
    if (hold_ret == ESP_OK) {
        if (!query_u32(query, "hold_ms", 100, 30000, &svc.hold_ms)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "hold_ms must be an integer in range 100..30000");
        }
    }

    if (!queue_storage_service_request(&svc)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage maintenance operation was queued first",
            "wait for the current operation to finish and retry from the Web page");
    }

    char json[320];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":true,\"hold_ms\":%" PRIu32
             ",\"reboot_required\":false,"
             "\"note\":\"Wi-Fi/HTTP will pause for TF probing, then restore automatically without reboot\"}",
             svc.hold_ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_retry_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "RETRY") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=RETRY required");
    }
    if (s_app_mode != APP_MODE_SERVER || s_network_shutdown_for_idle) {
        return send_customer_action_json(
            req, "409 Conflict", "mode_busy",
            "TF can only be retried in Web server mode",
            "finish field, export, or USB storage mode before retrying the card");
    }
    if (storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_export",
            "TF card is currently owned by the USB host",
            "safely eject P4_BUOY and restore storage before retrying");
    }
    if (s_storage_quiescing || storage_transition_active() ||
        storage_any_request_pending()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "a storage recovery operation is already running",
            "wait for the current operation to finish; the Web page will update automatically");
    }
    if (storage_acceptance_ok()) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return http_send_cstr_chunked(
            req, "{\"ok\":true,\"queued\":false,\"already_ready\":true}");
    }

    if (!storage_transition_try_acquire(STORAGE_TRANSITION_RETRY)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage maintenance operation was requested first",
            "wait for the current operation to finish; the Web page will update automatically");
    }
    storage_request_set(&s_storage_retry_requested);
    open_network_access_window("TF retry requested from Web");
    httpd_resp_set_status(req, "202 Accepted");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":true,\"queued\":true,\"temporary_http_pause\":true,"
        "\"reboot_required\":false,\"note\":\"Wi-Fi stays connected; the page reconnects automatically after TF write verification\"}");
}

static esp_err_t field_mode_start_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "FIELD") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=FIELD required");
    }
    if (s_app_mode == APP_MODE_FIELD || s_network_shutdown_for_idle) {
        return send_customer_action_json(
            req, "409 Conflict", "field_active",
            "field recording is already active or pending",
            "reboot the device to return to Web mode before starting again");
    }
    if (s_storage_quiescing || storage_transition_active()) {
        return send_customer_action_json(
            req, "503 Service Unavailable", "storage_busy",
            "storage is switching ownership",
            "wait until the storage status becomes ready, then retry field recording");
    }
    if (storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_export",
            "TF card is exported to the computer as a USB drive",
            "click USB restore in Web or unplug USB before field recording");
    }
    if (s_app_mode != APP_MODE_SERVER && s_app_mode != APP_MODE_EXPORT) {
        return send_customer_action_json(
            req, "409 Conflict", "wrong_mode",
            "current application mode cannot enter field recording",
            "return the device to Web server mode, then retry field recording");
    }
    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    if (__atomic_load_n(&s_validation_active_jobs, __ATOMIC_ACQUIRE) > 0 ||
        dataset.queued || dataset.running || inference_worker_busy()) {
        return send_customer_action_json(
            req, "409 Conflict", "validation_busy",
            "a model validation or dataset analysis is still running",
            "wait for validation to finish; the Web page will keep the field countdown paused");
    }
    if (!s_sd_mounted || !storage_acceptance_ok()) {
        return send_customer_action_json(
            req, "503 Service Unavailable", "tf_unavailable",
            "TF card is not ready for recording",
            "check or replace the TF card, then use storage retry in Web; reboot is not required");
    }
    if (!storage_transition_try_acquire(STORAGE_TRANSITION_FIELD)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage ownership transition was requested first",
            "wait for the current operation to finish; the Web page will update automatically");
    }
    storage_request_set(&s_field_mode_requested);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req, "{\"ok\":true,\"mode\":\"field_pending\",\"reboot_to_return\":\"server\"}");
}

static esp_err_t export_mode_start_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "EXPORT") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=EXPORT required");
    }
    if (s_app_mode == APP_MODE_FIELD || s_network_shutdown_for_idle) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "field capture active; reboot before export");
    }
    if (s_app_mode == APP_MODE_USB_EXPORT || storage_usb_owned()) {
        return send_customer_action_json(
            req, "409 Conflict", "usb_export",
            "TF card is currently owned by the USB host",
            "safely eject P4_BUOY and restore storage before entering Ethernet export mode");
    }
    if (s_app_mode == APP_MODE_EXPORT) {
        return send_customer_action_json(
            req, "409 Conflict", "export_active",
            "Ethernet export mode is already active",
            "continue downloading over Ethernet; no additional mode switch is needed");
    }
    if (s_storage_quiescing || storage_transition_active()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "export mode cannot start during storage maintenance",
            "wait for the page to reconnect and try again");
    }
    if (!storage_transition_try_acquire(STORAGE_TRANSITION_EXPORT)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage ownership transition was requested first",
            "wait for the current operation to finish and retry");
    }
    if (!s_eth_started && eth_init_runtime() != ESP_OK) {
        storage_transition_release(STORAGE_TRANSITION_EXPORT);
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Ethernet unavailable");
    }
    storage_request_set(&s_export_mode_requested);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req, "{\"ok\":true,\"mode\":\"export_pending\",\"note\":\"camera and Wi-Fi will stop; Ethernet HTTP remains active\"}");
}

static int compare_u32_for_qsort(const void *a, const void *b)
{
    uint32_t av = *(const uint32_t *)a;
    uint32_t bv = *(const uint32_t *)b;
    return (av > bv) - (av < bv);
}

static void dataset_status_copy(dataset_run_status_t *out)
{
    if (!out) {
        return;
    }
    if (s_dataset_lock) {
        xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
        *out = s_dataset_status;
        xSemaphoreGive(s_dataset_lock);
    } else {
        *out = s_dataset_status;
    }
}

static void dataset_status_update(const dataset_run_status_t *status)
{
    if (!status) {
        return;
    }
    if (s_dataset_lock) {
        xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
        s_dataset_status = *status;
        xSemaphoreGive(s_dataset_lock);
    } else {
        s_dataset_status = *status;
    }
}

static bool dataset_status_try_queue(const dataset_run_status_t *queued)
{
    if (!queued) {
        return false;
    }
    if (s_dataset_lock) {
        xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
        if (s_dataset_status.queued || s_dataset_status.running) {
            xSemaphoreGive(s_dataset_lock);
            return false;
        }
        if (s_dataset_frame_cache) {
            memset(s_dataset_frame_cache, 0,
                   sizeof(*s_dataset_frame_cache) * CONFIG_APP_DATASET_RUN_MAX_FRAMES);
        }
        s_dataset_status = *queued;
        xSemaphoreGive(s_dataset_lock);
        return true;
    }
    if (s_dataset_status.queued || s_dataset_status.running) {
        return false;
    }
    s_dataset_status = *queued;
    return true;
}

static bool is_builtin_coco_video_dataset(const char *dataset)
{
    return dataset && strcmp(dataset, BUILTIN_COCO_VIDEO_DATASET) == 0;
}

static bool is_builtin_tinycls_video_dataset(const char *dataset)
{
    return dataset && strcmp(dataset, BUILTIN_TINYCLS_VIDEO_DATASET) == 0;
}

static bool is_builtin_fish31_video_dataset(const char *dataset)
{
    return dataset && strcmp(dataset, BUILTIN_FISH31_VIDEO_DATASET) == 0;
}

static bool is_builtin_video_dataset(const char *dataset)
{
    return is_builtin_coco_video_dataset(dataset) ||
           is_builtin_tinycls_video_dataset(dataset) ||
           is_builtin_fish31_video_dataset(dataset);
}

static bool builtin_coco_video_frame_image(uint32_t frame_index,
                                           const uint8_t **start,
                                           const uint8_t **end)
{
    static const uint8_t *const starts[BUILTIN_COCO_VIDEO_FRAMES] = {
        coco_video_frame_00001_jpg_start, coco_video_frame_00002_jpg_start,
        coco_video_frame_00003_jpg_start, coco_video_frame_00004_jpg_start,
        coco_video_frame_00005_jpg_start, coco_video_frame_00006_jpg_start,
        coco_video_frame_00007_jpg_start, coco_video_frame_00008_jpg_start,
        coco_video_frame_00009_jpg_start, coco_video_frame_00010_jpg_start,
        coco_video_frame_00011_jpg_start, coco_video_frame_00012_jpg_start,
        coco_video_frame_00013_jpg_start, coco_video_frame_00014_jpg_start,
        coco_video_frame_00015_jpg_start, coco_video_frame_00016_jpg_start,
    };
    static const uint8_t *const ends[BUILTIN_COCO_VIDEO_FRAMES] = {
        coco_video_frame_00001_jpg_end, coco_video_frame_00002_jpg_end,
        coco_video_frame_00003_jpg_end, coco_video_frame_00004_jpg_end,
        coco_video_frame_00005_jpg_end, coco_video_frame_00006_jpg_end,
        coco_video_frame_00007_jpg_end, coco_video_frame_00008_jpg_end,
        coco_video_frame_00009_jpg_end, coco_video_frame_00010_jpg_end,
        coco_video_frame_00011_jpg_end, coco_video_frame_00012_jpg_end,
        coco_video_frame_00013_jpg_end, coco_video_frame_00014_jpg_end,
        coco_video_frame_00015_jpg_end, coco_video_frame_00016_jpg_end,
    };
    if (!start || !end || frame_index == 0 || frame_index > BUILTIN_COCO_VIDEO_FRAMES) {
        return false;
    }
    *start = starts[frame_index - 1U];
    *end = ends[frame_index - 1U];
    return *end > *start;
}

static bool builtin_tinycls_video_frame_image(uint32_t frame_index,
                                              const uint8_t **start,
                                              const uint8_t **end)
{
    static const uint8_t *const starts[BUILTIN_TINYCLS_VIDEO_FRAMES] = {
        tinycls_video_frame_00001_jpg_start, tinycls_video_frame_00002_jpg_start,
        tinycls_video_frame_00003_jpg_start, tinycls_video_frame_00004_jpg_start,
        tinycls_video_frame_00005_jpg_start, tinycls_video_frame_00006_jpg_start,
        tinycls_video_frame_00007_jpg_start, tinycls_video_frame_00008_jpg_start,
        tinycls_video_frame_00009_jpg_start, tinycls_video_frame_00010_jpg_start,
        tinycls_video_frame_00011_jpg_start, tinycls_video_frame_00012_jpg_start,
        tinycls_video_frame_00013_jpg_start, tinycls_video_frame_00014_jpg_start,
        tinycls_video_frame_00015_jpg_start, tinycls_video_frame_00016_jpg_start,
    };
    static const uint8_t *const ends[BUILTIN_TINYCLS_VIDEO_FRAMES] = {
        tinycls_video_frame_00001_jpg_end, tinycls_video_frame_00002_jpg_end,
        tinycls_video_frame_00003_jpg_end, tinycls_video_frame_00004_jpg_end,
        tinycls_video_frame_00005_jpg_end, tinycls_video_frame_00006_jpg_end,
        tinycls_video_frame_00007_jpg_end, tinycls_video_frame_00008_jpg_end,
        tinycls_video_frame_00009_jpg_end, tinycls_video_frame_00010_jpg_end,
        tinycls_video_frame_00011_jpg_end, tinycls_video_frame_00012_jpg_end,
        tinycls_video_frame_00013_jpg_end, tinycls_video_frame_00014_jpg_end,
        tinycls_video_frame_00015_jpg_end, tinycls_video_frame_00016_jpg_end,
    };
    if (!start || !end || frame_index == 0 || frame_index > BUILTIN_TINYCLS_VIDEO_FRAMES) {
        return false;
    }
    *start = starts[frame_index - 1U];
    *end = ends[frame_index - 1U];
    return *end > *start;
}

static bool builtin_video_frame_image(const char *dataset,
                                      uint32_t frame_index,
                                      const uint8_t **start,
                                      const uint8_t **end)
{
    if (is_builtin_coco_video_dataset(dataset)) {
        return builtin_coco_video_frame_image(frame_index, start, end);
    }
    if (is_builtin_tinycls_video_dataset(dataset)) {
        return builtin_tinycls_video_frame_image(frame_index, start, end);
    }
    if (is_builtin_fish31_video_dataset(dataset)) {
        static const uint8_t *const starts[BUILTIN_FISH31_VIDEO_FRAMES] = {
            fish31_video_frame_00001_jpg_start, fish31_video_frame_00002_jpg_start,
            fish31_video_frame_00003_jpg_start, fish31_video_frame_00004_jpg_start,
            fish31_video_frame_00005_jpg_start, fish31_video_frame_00006_jpg_start,
            fish31_video_frame_00007_jpg_start, fish31_video_frame_00008_jpg_start,
            fish31_video_frame_00009_jpg_start, fish31_video_frame_00010_jpg_start,
            fish31_video_frame_00011_jpg_start, fish31_video_frame_00012_jpg_start,
            fish31_video_frame_00013_jpg_start, fish31_video_frame_00014_jpg_start,
            fish31_video_frame_00015_jpg_start, fish31_video_frame_00016_jpg_start,
        };
        static const uint8_t *const ends[BUILTIN_FISH31_VIDEO_FRAMES] = {
            fish31_video_frame_00001_jpg_end, fish31_video_frame_00002_jpg_end,
            fish31_video_frame_00003_jpg_end, fish31_video_frame_00004_jpg_end,
            fish31_video_frame_00005_jpg_end, fish31_video_frame_00006_jpg_end,
            fish31_video_frame_00007_jpg_end, fish31_video_frame_00008_jpg_end,
            fish31_video_frame_00009_jpg_end, fish31_video_frame_00010_jpg_end,
            fish31_video_frame_00011_jpg_end, fish31_video_frame_00012_jpg_end,
            fish31_video_frame_00013_jpg_end, fish31_video_frame_00014_jpg_end,
            fish31_video_frame_00015_jpg_end, fish31_video_frame_00016_jpg_end,
        };
        if (!start || !end || frame_index == 0 || frame_index > BUILTIN_FISH31_VIDEO_FRAMES) {
            return false;
        }
        *start = starts[frame_index - 1U];
        *end = ends[frame_index - 1U];
        return *end > *start;
    }
    return false;
}

static uint32_t builtin_video_frame_count(const char *dataset)
{
    if (is_builtin_coco_video_dataset(dataset)) {
        return BUILTIN_COCO_VIDEO_FRAMES;
    }
    if (is_builtin_tinycls_video_dataset(dataset)) {
        return BUILTIN_TINYCLS_VIDEO_FRAMES;
    }
    if (is_builtin_fish31_video_dataset(dataset)) {
        return BUILTIN_FISH31_VIDEO_FRAMES;
    }
    return 0;
}

static void dataset_frame_cache_clear(void)
{
    if (!s_dataset_frame_cache) {
        return;
    }
    xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
    memset(s_dataset_frame_cache, 0,
           sizeof(*s_dataset_frame_cache) * CONFIG_APP_DATASET_RUN_MAX_FRAMES);
    xSemaphoreGive(s_dataset_lock);
}

static void dataset_frame_cache_store(const dataset_run_status_t *status,
                                      uint32_t frame_index,
                                      const vision_result_t *vision,
                                      uint32_t source_w, uint32_t source_h)
{
    if (!s_dataset_frame_cache || !status || !vision || frame_index == 0) {
        return;
    }
    xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
    dataset_frame_cache_t *cache = NULL;
    for (uint32_t i = 0; i < CONFIG_APP_DATASET_RUN_MAX_FRAMES; i++) {
        dataset_frame_cache_t *candidate = &s_dataset_frame_cache[i];
        if (!candidate->valid) {
            cache = candidate;
            break;
        }
        if (candidate->frame_index == frame_index &&
            strcmp(candidate->run_id, status->run_id) == 0 &&
            strcmp(candidate->dataset, status->dataset) == 0) {
            cache = candidate;
            break;
        }
    }
    if (!cache) {
        xSemaphoreGive(s_dataset_lock);
        return;
    }
    memset(cache, 0, sizeof(*cache));
    cache->valid = true;
    strlcpy(cache->run_id, status->run_id, sizeof(cache->run_id));
    strlcpy(cache->dataset, status->dataset, sizeof(cache->dataset));
    cache->frame_index = frame_index;
    cache->vision = *vision;
    cache->source_w = source_w;
    cache->source_h = source_h;
    xSemaphoreGive(s_dataset_lock);
}

static bool dataset_frame_cache_copy(const char *run_id, const char *dataset,
                                     uint32_t frame_index,
                                     dataset_frame_cache_t *out)
{
    if (!s_dataset_frame_cache || !run_id || !dataset || !out || frame_index == 0) {
        return false;
    }
    xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
    bool found = false;
    for (uint32_t i = 0; i < CONFIG_APP_DATASET_RUN_MAX_FRAMES; i++) {
        const dataset_frame_cache_t *cache = &s_dataset_frame_cache[i];
        if (cache->valid && cache->frame_index == frame_index &&
            strcmp(cache->run_id, run_id) == 0 &&
            strcmp(cache->dataset, dataset) == 0) {
            *out = *cache;
            found = true;
            break;
        }
    }
    xSemaphoreGive(s_dataset_lock);
    return found;
}

static esp_err_t run_jpeg_on_inference_queue(uint8_t *jpeg, uint32_t jpeg_size,
                                             recognition_method_t method,
                                             vision_result_t *vision,
                                             uint32_t *source_w, uint32_t *source_h)
{
    if (!jpeg || !jpeg_size || !vision || !source_w || !source_h ||
        !recognition_method_uses_jpeg_inference(method) ||
        !validation_selftest_method_available(method)) {
        free(jpeg);
        return ESP_ERR_INVALID_ARG;
    }

    validation_context_t *ctx = validation_context_create();
    if (!ctx) {
        free(jpeg);
        return ESP_ERR_NO_MEM;
    }
    ctx->sample = VALIDATION_SAMPLE_NONE;
    ctx->method = method;
    ctx->box_min_score = 50;
    ctx->jpeg_size = jpeg_size;
    ctx->queued_ms = esp_timer_get_time() / 1000;

    inference_job_t job = {
        .jpeg = jpeg,
        .jpeg_size = jpeg_size,
        .method = method,
        .box_min_score = 50,
        .validation = true,
        .validation_sample = VALIDATION_SAMPLE_NONE,
        .validation_ctx = ctx,
        .queued_ms = ctx->queued_ms,
    };
    if (method == RECOGNITION_METHOD_TINYCLS) {
        fill_tinycls_pending(&job.meta.vision);
    } else if (method == RECOGNITION_METHOD_FISH31) {
        fill_fish31_pending(&job.meta.vision);
    } else {
        fill_yolo_pending(&job.meta.vision, method);
    }

    __atomic_add_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
    validation_context_retain(ctx);
    if (!s_inference_queue ||
        xQueueSend(s_inference_queue, &job, pdMS_TO_TICKS(5000)) != pdTRUE) {
        free(jpeg);
        __atomic_sub_fetch(&s_validation_active_jobs, 1, __ATOMIC_ACQ_REL);
        validation_context_release(ctx);
        validation_context_release(ctx);
        s_inference_queue_drops++;
        return ESP_ERR_TIMEOUT;
    }

    s_inference_jobs_queued++;
    if (xSemaphoreTake(ctx->done, pdMS_TO_TICKS(45000)) != pdTRUE) {
        validation_context_release(ctx);
        return ESP_ERR_TIMEOUT;
    }

    *vision = ctx->vision;
    *source_w = ctx->source_w;
    *source_h = ctx->source_h;
    esp_err_t err = ctx->err;
    validation_context_release(ctx);
    return err;
}

static esp_err_t generate_annotated_jpeg_from_vision(
    const uint8_t *jpeg,
    uint32_t jpeg_size,
    recognition_method_t method,
    const vision_result_t *vision,
    uint32_t source_w,
    uint32_t source_h,
    uint8_t **annotated_jpeg,
    size_t *annotated_size)
{
    if (!jpeg || !jpeg_size || !vision || !annotated_jpeg || !annotated_size) {
        return ESP_ERR_INVALID_ARG;
    }
    *annotated_jpeg = NULL;
    *annotated_size = 0;

    if (method == RECOGNITION_METHOD_COCO) {
        coco_espdl_result_t coco = {0};
        coco.source_w = source_w;
        coco.source_h = source_h;
        coco.raw_candidate_count = vision->raw_candidate_count;
        coco.score = vision->object_score;
        strlcpy(coco.label, vision->label, sizeof(coco.label));
        uint32_t count = vision->detection_count > COCO_ESPDL_MAX_DETECTIONS ?
                         COCO_ESPDL_MAX_DETECTIONS : vision->detection_count;
        coco.detection_count = count;
        coco.has_candidate = count > 0;
        for (uint32_t i = 0; i < count; i++) {
            coco.detections[i].class_id = vision->detections[i].class_id;
            coco.detections[i].score = vision->detections[i].score;
            coco.detections[i].x = (int32_t)vision->detections[i].x;
            coco.detections[i].y = (int32_t)vision->detections[i].y;
            coco.detections[i].w = (int32_t)vision->detections[i].w;
            coco.detections[i].h = (int32_t)vision->detections[i].h;
            strlcpy(coco.detections[i].label, vision->detections[i].label,
                    sizeof(coco.detections[i].label));
        }
        return coco_espdl_annotate_jpeg(jpeg, jpeg_size, &coco, s_box_min_score, s_jpeg_quality,
                                        annotated_jpeg, annotated_size);
    }

    char title[64];
    char subtitle[96] = {0};
    snprintf(title, sizeof(title), "%s %" PRIu32 "%%",
             vision->label[0] ? vision->label : recognition_method_name(method),
             vision->object_score);
    size_t off = 0;
    for (uint32_t i = 0; i < vision->top_k_count && i < 3 && off < sizeof(subtitle); i++) {
        size_t remaining = sizeof(subtitle) - off;
        int written = snprintf(subtitle + off, remaining, "%s%s %" PRIu32 "%%",
                               i ? " / " : "", vision->top_k[i].label,
                               vision->top_k[i].score);
        if (written < 0 || (size_t)written >= remaining) {
            off = sizeof(subtitle) - 1U;
            break;
        }
        off += (size_t)written;
    }
    return coco_espdl_annotate_label_jpeg(jpeg, jpeg_size, title, subtitle,
                                          s_jpeg_quality,
                                          annotated_jpeg, annotated_size);
}

static esp_err_t enrichment_infer_annotate_cb(
    const uint8_t *jpeg,
    size_t jpeg_size,
    const char *method_text,
    uint32_t frame_index,
    uint32_t min_score,
    uint8_t jpeg_quality,
    char *meta_json,
    size_t meta_json_size,
    uint8_t **annotated_jpeg,
    size_t *annotated_jpeg_size,
    void *arg)
{
    (void)frame_index;
    (void)arg;
    if (!jpeg || jpeg_size == 0 || jpeg_size > UINT32_MAX || !meta_json ||
        meta_json_size == 0 || !annotated_jpeg || !annotated_jpeg_size) {
        return ESP_ERR_INVALID_ARG;
    }
    *annotated_jpeg = NULL;
    *annotated_jpeg_size = 0;
    meta_json[0] = '\0';

    recognition_method_t method = parse_validation_method(method_text);
    if (method != RECOGNITION_METHOD_FISH31 &&
        method != RECOGNITION_METHOD_TINYCLS &&
        method != RECOGNITION_METHOD_COCO) {
        method = RECOGNITION_METHOD_FISH31;
    }
    uint8_t *copy = alloc_psram_buffer((uint32_t)jpeg_size);
    if (!copy) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, jpeg, jpeg_size);

    vision_result_t vision = {0};
    uint32_t source_w = 0;
    uint32_t source_h = 0;
    esp_err_t ret = run_jpeg_on_inference_queue(copy, (uint32_t)jpeg_size,
                                                method, &vision,
                                                &source_w, &source_h);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "enrichment inference failed: method=%s frame=%" PRIu32 " err=%s",
                 recognition_method_name(method), frame_index, esp_err_to_name(ret));
        return ret;
    }

    char detections_json[1280];
    char top_k_json[512];
    char label[64];
    char object[64];
    char model[64];
    if (!detections_to_json(detections_json, sizeof(detections_json), &vision) ||
        !top_k_to_json(top_k_json, sizeof(top_k_json), &vision)) {
        return ESP_ERR_INVALID_SIZE;
    }
    json_escape_string(label, sizeof(label), vision.label);
    json_escape_string(object, sizeof(object), vision.object);
    json_escape_string(model, sizeof(model), vision.model);
    int meta_len = snprintf(meta_json, meta_json_size,
             "\"model\":\"%s\",\"label\":\"%s\",\"object\":\"%s\","
             "\"object_count\":%" PRIu32 ",\"top_k\":%s,"
             "\"box_min_score\":%" PRIu32 ",\"best_score\":%" PRIu32
             ",\"candidate_score\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
             ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64
             ",\"detection_count\":%" PRIu32 ",\"detections\":%s",
             model, label, object,
             vision.object_count, top_k_json,
             min_score, vision.object_score,
             vision.candidate_score, vision.raw_candidate_count,
             vision.inference_ms, vision.analysis_ms,
             vision.detection_count, detections_json);
    if (meta_len < 0 || (size_t)meta_len >= meta_json_size) {
        meta_json[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }

    if (method == RECOGNITION_METHOD_COCO) {
        coco_espdl_result_t coco = {0};
        coco.source_w = source_w;
        coco.source_h = source_h;
        coco.raw_candidate_count = vision.raw_candidate_count;
        coco.score = vision.object_score;
        strlcpy(coco.label, vision.label, sizeof(coco.label));
        uint32_t count = vision.detection_count > COCO_ESPDL_MAX_DETECTIONS ?
                         COCO_ESPDL_MAX_DETECTIONS : vision.detection_count;
        coco.detection_count = count;
        coco.has_candidate = count > 0;
        for (uint32_t i = 0; i < count; i++) {
            coco.detections[i].class_id = vision.detections[i].class_id;
            coco.detections[i].score = vision.detections[i].score;
            coco.detections[i].x = (int32_t)vision.detections[i].x;
            coco.detections[i].y = (int32_t)vision.detections[i].y;
            coco.detections[i].w = (int32_t)vision.detections[i].w;
            coco.detections[i].h = (int32_t)vision.detections[i].h;
            strlcpy(coco.detections[i].label, vision.detections[i].label,
                    sizeof(coco.detections[i].label));
        }
        ret = coco_espdl_annotate_jpeg(jpeg, jpeg_size, &coco, min_score, jpeg_quality,
                                       annotated_jpeg, annotated_jpeg_size);
        if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "enrichment COCO annotation fallback to raw frame: frame=%" PRIu32
                     " err=%s", frame_index, esp_err_to_name(ret));
        }
        return ESP_OK;
    }

    char title[64];
    char subtitle[96] = {0};
    snprintf(title, sizeof(title), "%s %" PRIu32 "%%",
             vision.label[0] ? vision.label : recognition_method_name(method),
             vision.object_score);
    size_t off = 0;
    for (uint32_t i = 0; i < vision.top_k_count && i < 3 && off < sizeof(subtitle); i++) {
        size_t remaining = sizeof(subtitle) - off;
        int written = snprintf(subtitle + off, remaining, "%s%s %" PRIu32 "%%",
                               i ? " / " : "", vision.top_k[i].label,
                               vision.top_k[i].score);
        if (written < 0 || (size_t)written >= remaining) {
            off = sizeof(subtitle) - 1U;
            break;
        }
        off += (size_t)written;
    }
    ret = coco_espdl_annotate_label_jpeg(jpeg, jpeg_size, title, subtitle,
                                         jpeg_quality,
                                         annotated_jpeg, annotated_jpeg_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "enrichment label annotation fallback to raw frame: frame=%" PRIu32
                 " err=%s", frame_index, esp_err_to_name(ret));
        return ESP_OK;
    }
    return ESP_OK;
}

static uint8_t *read_file_to_psram(const char *path, uint32_t *out_size)
{
    struct stat st = {0};
    if (!path || !out_size || stat(path, &st) != 0 || st.st_size <= 0 || st.st_size > 1024 * 1024) {
        return NULL;
    }
    FILE *file = fopen(path, "rb");
    if (!file) {
        return NULL;
    }
    uint8_t *buf = alloc_psram_buffer((uint32_t)st.st_size);
    if (!buf) {
        fclose(file);
        return NULL;
    }
    size_t n = fread(buf, 1, (size_t)st.st_size, file);
    fclose(file);
    if (n != (size_t)st.st_size) {
        free(buf);
        return NULL;
    }
    *out_size = (uint32_t)st.st_size;
    return buf;
}

static const char *json_find_number_value(const char *line, const char *key)
{
    if (!line || !key) {
        return NULL;
    }
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(line, pattern);
    if (!p) {
        return NULL;
    }
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        p++;
    }
    return p;
}

static bool json_integer_token_terminated(const char *end)
{
    if (!end) {
        return false;
    }
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') {
        end++;
    }
    return *end == '\0' || *end == ',' || *end == '}' || *end == ']';
}

static bool json_parse_i64_value(const char *text, int64_t *out)
{
    if (!text || !out || text[0] == '+' ||
        (text[0] != '-' && (text[0] < '0' || text[0] > '9')) ||
        (text[0] == '-' && (text[1] < '0' || text[1] > '9'))) {
        return false;
    }
    const char *digits = text[0] == '-' ? text + 1 : text;
    if (digits[0] == '0' && digits[1] >= '0' && digits[1] <= '9') {
        return false;
    }
    errno = 0;
    char *end = NULL;
    long long value = strtoll(text, &end, 10);
    if (end == text || errno == ERANGE || !json_integer_token_terminated(end)) {
        return false;
    }
    *out = (int64_t)value;
    return true;
}

static bool json_parse_u32_value(const char *text, uint32_t *out)
{
    if (!text || !out || text[0] < '0' || text[0] > '9' ||
        (text[0] == '0' && text[1] >= '0' && text[1] <= '9')) {
        return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long long value = strtoull(text, &end, 10);
    if (end == text || errno == ERANGE || value > UINT32_MAX ||
        !json_integer_token_terminated(end)) {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

static bool json_get_int64_field(const char *line, const char *key, int64_t *out)
{
    const char *p = json_find_number_value(line, key);
    if (!p || !out) {
        return false;
    }
    return json_parse_i64_value(p, out);
}

static bool json_get_u32_field(const char *line, const char *key, uint32_t *out)
{
    const char *p = json_find_number_value(line, key);
    if (!p || !out) {
        return false;
    }
    return json_parse_u32_value(p, out);
}

static bool json_get_string_field(const char *line, const char *key, char *out, size_t out_size)
{
    if (!line || !key || !out || out_size == 0) {
        return false;
    }
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(line, pattern);
    if (!p) {
        return false;
    }
    p += strlen(pattern);
    const char *end = strchr(p, '"');
    if (!end) {
        return false;
    }
    size_t n = (size_t)(end - p);
    if (n >= out_size) {
        n = out_size - 1;
    }
    memcpy(out, p, n);
    out[n] = '\0';
    return true;
}

static uint32_t json_max_score_in_line(const char *line)
{
    uint32_t max_score = 0;
    const char *p = line;
    while (p && (p = strstr(p, "\"score\":")) != NULL) {
        uint32_t v = 0;
        if (json_parse_u32_value(p + strlen("\"score\":"), &v) && v > max_score) {
            max_score = v;
        }
        p += strlen("\"score\":");
    }
    const char *keys[] = {"best_score", "object_score", "candidate_score"};
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        uint32_t v = 0;
        if (json_get_u32_field(line, keys[i], &v) && v > max_score) {
            max_score = v;
        }
    }
    return max_score;
}

static bool parse_vision_from_json_line(const char *line, vision_result_t *vision)
{
    if (!line || !vision) {
        return false;
    }
    memset(vision, 0, sizeof(*vision));
    strlcpy(vision->label, "unknown", sizeof(vision->label));
    strlcpy(vision->object, "unknown", sizeof(vision->object));
    strlcpy(vision->model, COCO_MODEL_NAME, sizeof(vision->model));
    json_get_string_field(line, "model", vision->model, sizeof(vision->model));
    json_get_u32_field(line, "box_min_score", &vision->box_min_score);
    json_get_u32_field(line, "candidate_score", &vision->candidate_score);
    json_get_u32_field(line, "best_score", &vision->object_score);
    json_get_u32_field(line, "raw_candidate_count", &vision->raw_candidate_count);
    json_get_int64_field(line, "inference_ms", &vision->inference_ms);
    json_get_int64_field(line, "analysis_ms", &vision->analysis_ms);
    json_get_string_field(line, "label", vision->label, sizeof(vision->label));
    json_get_string_field(line, "object", vision->object, sizeof(vision->object));

    const char *top = strstr(line, "\"top_k\":[");
    while (top && vision->top_k_count < TINY_CLS_TOP_K &&
           (top = strstr(top, "{\"label\":\"")) != NULL) {
        tiny_cls_topk_t *item = &vision->top_k[vision->top_k_count];
        if (!json_get_string_field(top, "label", item->label, sizeof(item->label))) {
            break;
        }
        json_get_u32_field(top, "class_id", &item->class_id);
        json_get_u32_field(top, "score", &item->score);
        vision->top_k_count++;
        top += strlen("{\"label\":\"");
    }
    if (vision->top_k_count > 0 && vision->object_score == 0) {
        const tiny_cls_topk_t *best = &vision->top_k[0];
        vision->object_score = best->score;
        vision->candidate_score = best->score;
        strlcpy(vision->label, best->label, sizeof(vision->label));
        strlcpy(vision->object, best->label, sizeof(vision->object));
        vision->object_count = 1;
    }

    const char *p = strstr(line, "\"detections\":[");
    if (!p) {
        json_get_u32_field(line, "detection_count", &vision->detection_count);
        return true;
    }
    while (vision->detection_count < APP_MAX_DETECTIONS &&
           (p = strstr(p, "{\"label\":\"")) != NULL) {
        vision_detection_t *d = &vision->detections[vision->detection_count];
        if (!json_get_string_field(p, "label", d->label, sizeof(d->label))) {
            break;
        }
        json_get_u32_field(p, "class_id", &d->class_id);
        json_get_u32_field(p, "score", &d->score);
        json_get_u32_field(p, "x", &d->x);
        json_get_u32_field(p, "y", &d->y);
        json_get_u32_field(p, "w", &d->w);
        json_get_u32_field(p, "h", &d->h);
        if (d->score > vision->object_score) {
            vision->object_score = d->score;
            strlcpy(vision->object, d->label, sizeof(vision->object));
            strlcpy(vision->label, d->label, sizeof(vision->label));
            vision->object_x = d->x;
            vision->object_y = d->y;
            vision->object_w = d->w;
            vision->object_h = d->h;
        }
        vision->detection_count++;
        vision->object_count = vision->detection_count;
        p += strlen("{\"label\":\"");
    }
    return true;
}

static esp_err_t read_mjpeg_frame_to_psram(const char *path, uint32_t target_frame,
                                           uint8_t **out_jpeg, uint32_t *out_size)
{
    if (!path || target_frame == 0 || !out_jpeg || !out_size) {
        return ESP_ERR_INVALID_ARG;
    }
    FILE *file = fopen(path, "rb");
    if (!file) {
        return ESP_ERR_NOT_FOUND;
    }
    uint32_t cap = s_frame_capacity ? s_frame_capacity : CONFIG_APP_FRAME_BUFFER_BYTES;
    uint8_t *buf = alloc_psram_buffer(cap);
    if (!buf) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }

    uint32_t frame = 0;
    uint32_t size = 0;
    bool copying = false;
    int prev = -1;
    int c = 0;
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    while ((c = fgetc(file)) != EOF) {
        if (!copying) {
            if (prev == 0xff && c == 0xd8) {
                frame++;
                if (frame == target_frame) {
                    copying = true;
                    size = 0;
                    buf[size++] = 0xff;
                    buf[size++] = 0xd8;
                }
            }
            prev = c;
            continue;
        }

        if (size >= cap) {
            ret = ESP_ERR_NO_MEM;
            break;
        }
        buf[size++] = (uint8_t)c;
        if (prev == 0xff && c == 0xd9) {
            *out_jpeg = buf;
            *out_size = size;
            ret = ESP_OK;
            break;
        }
        prev = c;
    }
    fclose(file);
    if (ret != ESP_OK) {
        free(buf);
    }
    return ret;
}

static bool read_recording_meta_line(const char *recording_name, uint32_t frame_index,
                                     char *line, size_t line_size)
{
    if (!recording_name || !line || line_size == 0 || frame_index == 0 ||
        !is_safe_recording_name(recording_name)) {
        return false;
    }
    char meta_name[96];
    char path[384];
    meta_name_for_recording(recording_name, meta_name, sizeof(meta_name));
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, meta_name);
    FILE *file = fopen(path, "r");
    if (!file) {
        return false;
    }
    bool found = false;
    while (fgets(line, (int)line_size, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (!json_validate_object(line, len)) {
            continue;
        }
        uint32_t idx = 0;
        if (json_get_u32_field(line, "frame_index", &idx) && idx == frame_index) {
            found = true;
            break;
        }
    }
    fclose(file);
    return found;
}

static bool read_dataset_result_line(const char *run_id, uint32_t frame_index,
                                     char *line, size_t line_size)
{
    if (!run_id || !line || line_size == 0 || frame_index == 0 || !is_safe_snapshot_name(run_id)) {
        return false;
    }
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.jsonl", DATASET_RUN_DIR, run_id);
    FILE *file = fopen(path, "r");
    if (!file) {
        return false;
    }
    bool found = false;
    while (fgets(line, (int)line_size, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (!json_validate_object(line, len)) {
            continue;
        }
        uint32_t idx = 0;
        if (json_get_u32_field(line, "index", &idx) && idx == frame_index) {
            found = true;
            break;
        }
    }
    fclose(file);
    return found;
}

typedef struct {
    json_writer_t *writer;
    const char *label;
    const char *type_filter;
    int64_t from_ms;
    int64_t to_ms;
    uint32_t min_score;
    uint32_t limit;
    uint32_t cursor;
    uint32_t matched;
    uint32_t returned;
    bool need_comma;
    bool has_more;
} search_ctx_t;

static bool search_type_matches(const search_ctx_t *ctx, const char *type)
{
    if (!ctx || !type || !ctx->type_filter || !ctx->type_filter[0] ||
        strcmp(ctx->type_filter, "all") == 0) {
        return true;
    }
    if (strcmp(ctx->type_filter, type) == 0) {
        return true;
    }
    return strcmp(ctx->type_filter, "event") == 0 && strcmp(type, "history") == 0;
}

static bool search_type_filter_valid(const char *type)
{
    return type &&
           (strcmp(type, "all") == 0 || strcmp(type, "history") == 0 ||
            strcmp(type, "event") == 0 || strcmp(type, "recording") == 0 ||
            strcmp(type, "summary") == 0 || strcmp(type, "frame") == 0);
}

static bool search_label_matches(const char *line, const char *label)
{
    if (!label || !label[0]) {
        return true;
    }
    char pattern[80];
    snprintf(pattern, sizeof(pattern), "\"label\":\"%s\"", label);
    if (strstr(line, pattern)) {
        return true;
    }
    snprintf(pattern, sizeof(pattern), "\"object\":\"%s\"", label);
    if (strstr(line, pattern)) {
        return true;
    }
    return strstr(line, label) != NULL;
}

static bool search_time_matches(const char *line, int64_t from_ms, int64_t to_ms)
{
    if (from_ms <= 0 && to_ms <= 0) {
        return true;
    }
    int64_t start_ms = 0;
    int64_t end_ms = 0;
    bool have_start = json_get_int64_field(line, "start_ms", &start_ms);
    bool have_end = json_get_int64_field(line, "end_ms", &end_ms);
    if (!have_start) {
        have_start = json_get_int64_field(line, "time_ms", &start_ms);
    }
    if (!have_start) {
        have_start = json_get_int64_field(line, "stored_ms", &start_ms);
    }
    if (!have_end) {
        end_ms = start_ms;
        have_end = have_start;
    }
    if (!have_start || !have_end) {
        return false;
    }
    if (from_ms > 0 && end_ms < from_ms) {
        return false;
    }
    if (to_ms > 0 && start_ms > to_ms) {
        return false;
    }
    return true;
}

static bool search_score_matches(const char *line, uint32_t min_score)
{
    if (min_score == 0) {
        return true;
    }
    return json_max_score_in_line(line) >= min_score;
}

static bool search_consider_line(search_ctx_t *ctx, const char *type, const char *line)
{
    size_t line_length = line ? strlen(line) : 0;
    if (!ctx || !ctx->writer || !type ||
        !json_validate_object(line, line_length)) {
        return false;
    }
    if (!search_type_matches(ctx, type) ||
        !search_label_matches(line, ctx->label) ||
        !search_time_matches(line, ctx->from_ms, ctx->to_ms) ||
        !search_score_matches(line, ctx->min_score)) {
        return false;
    }

    ctx->matched++;
    if (ctx->matched <= ctx->cursor) {
        return false;
    }
    if (ctx->returned >= ctx->limit) {
        ctx->has_more = true;
        return true;
    }

    json_writer_appendf(ctx->writer, "%s{\"type\":", ctx->need_comma ? "," : "");
    json_writer_append_escaped_string(ctx->writer, type);
    json_writer_appendf(ctx->writer, ",\"data\":%s}", line);
    if (!json_writer_ok(ctx->writer)) {
        return true;
    }
    ctx->need_comma = true;
    ctx->returned++;
    return false;
}

static bool search_scan_jsonl_file(search_ctx_t *ctx, const char *path, const char *type)
{
    FILE *file = fopen(path, "r");
    if (!file) {
        return false;
    }
    char *line = (char *)alloc_psram_buffer(4096);
    if (!line) {
        fclose(file);
        return false;
    }
    bool stop = false;
    while (!stop && fgets(line, 4096, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        stop = search_consider_line(ctx, type, line);
    }
    free(line);
    fclose(file);
    return stop;
}

static bool search_scan_recording_sidecars(search_ctx_t *ctx)
{
    if (!ctx || !search_type_matches(ctx, "frame")) {
        return false;
    }
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return false;
    }
    bool stop = false;
    struct dirent *entry;
    while (!stop && (entry = readdir(dir)) != NULL) {
        if (!is_safe_recording_meta_name(entry->d_name)) {
            continue;
        }
        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        stop = search_scan_jsonl_file(ctx, path, "frame");
    }
    closedir(dir);
    return stop;
}

static esp_err_t search_get_handler_internal(httpd_req_t *req)
{
    char query[384] = {0};
    char label[64] = {0};
    char type[16] = "all";
    char text[32] = {0};
    int64_t from_ms = 0;
    int64_t to_ms = 0;
    uint32_t min_score = 0;
    uint32_t limit = 50;
    uint32_t cursor = 0;

    bool has_query = false;
    esp_err_t query_ret = read_optional_url_query(req, query, sizeof(query), &has_query);
    if (query_ret != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "search query is too long or unreadable");
    }
    if (has_query) {
        esp_err_t value_ret = form_query_key_value(query, "label", label, sizeof(label));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "label has invalid form encoding or is too long");
        }

        value_ret = httpd_query_key_value(query, "type", type, sizeof(type));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "type parameter is too long or malformed");
        }
        if (value_ret == ESP_OK && !search_type_filter_valid(type)) {
            return httpd_resp_send_err(
                req, HTTPD_400_BAD_REQUEST,
                "type must be all, history, event, recording, summary, or frame");
        }

        value_ret = httpd_query_key_value(query, "from_ms", text, sizeof(text));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "from_ms parameter is too long or malformed");
        }
        if (value_ret == ESP_OK &&
            !query_i64(query, "from_ms", 0, INT64_MAX, &from_ms)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "from_ms must be a non-negative integer");
        }

        text[0] = '\0';
        value_ret = httpd_query_key_value(query, "to_ms", text, sizeof(text));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "to_ms parameter is too long or malformed");
        }
        if (value_ret == ESP_OK && !query_i64(query, "to_ms", 0, INT64_MAX, &to_ms)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "to_ms must be a non-negative integer");
        }

        text[0] = '\0';
        value_ret = httpd_query_key_value(query, "min_score", text, sizeof(text));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "min_score parameter is too long or malformed");
        }
        if (value_ret == ESP_OK && !query_u32(query, "min_score", 0, 100, &min_score)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "min_score must be an integer in range 0..100");
        }

        text[0] = '\0';
        value_ret = httpd_query_key_value(query, "limit", text, sizeof(text));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit parameter is too long or malformed");
        }
        if (value_ret == ESP_OK && !query_u32(query, "limit", 1, 100, &limit)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "limit must be an integer in range 1..100");
        }

        text[0] = '\0';
        value_ret = httpd_query_key_value(query, "cursor", text, sizeof(text));
        if (value_ret != ESP_OK && value_ret != ESP_ERR_NOT_FOUND) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "cursor parameter is too long or malformed");
        }
        if (value_ret == ESP_OK && !query_u32(query, "cursor", 0, UINT32_MAX, &cursor)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "cursor must be a non-negative integer");
        }
    }
    if (to_ms > 0 && from_ms > to_ms) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "from_ms must not be greater than to_ms");
    }

    size_t cap = 2048U + (size_t)limit * (JSONL_TAIL_LINE_BYTES + 128U);
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no search buffer");
    }

    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    search_ctx_t ctx = {
        .writer = &writer,
        .label = label,
        .type_filter = type,
        .from_ms = from_ms,
        .to_ms = to_ms,
        .min_score = min_score,
        .limit = limit,
        .cursor = cursor,
    };

    json_writer_appendf(&writer,
                        "{\"ok\":true,\"index_version\":%" PRIu32
                        ",\"storage_backend\":",
                        (uint32_t)APP_JSONL_INDEX_VERSION);
    json_writer_append_escaped_string(&writer, s_storage_backend);
    json_writer_appendf(&writer,
                        ",\"tf_ready\":%s,\"storage_acceptance_ok\":%s"
                        ",\"sd_mounted\":%s,\"query\":{\"label\":",
                        storage_tf_ready() ? "true" : "false",
                        storage_acceptance_ok() ? "true" : "false",
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, label);
    json_writer_appendf(&writer, ",\"type\":");
    json_writer_append_escaped_string(&writer, type);
    json_writer_appendf(&writer,
                        ",\"from_ms\":%" PRId64 ",\"to_ms\":%" PRId64
                        ",\"min_score\":%" PRIu32 ",\"limit\":%" PRIu32
                        ",\"cursor\":%" PRIu32 "},\"results\":[",
                        from_ms, to_ms, min_score, limit, cursor);

    if (s_sd_mounted) {
        bool stop = search_scan_jsonl_file(&ctx, HISTORY_JSONL_PATH, "history");
        if (!stop) {
            stop = search_scan_jsonl_file(&ctx, HISTORY_JSONL_OLD_PATH, "history");
        }
        if (!stop) {
            stop = search_scan_jsonl_file(&ctx, RECORDING_INDEX_PATH, "recording");
        }
        if (!stop) {
            stop = search_scan_jsonl_file(&ctx, RECORDING_INDEX_OLD_PATH, "recording");
        }
        if (!stop) {
            stop = search_scan_jsonl_file(&ctx, RECORDING_SUMMARY_PATH, "summary");
        }
        if (!stop) {
            stop = search_scan_jsonl_file(&ctx, RECORDING_SUMMARY_OLD_PATH, "summary");
        }
        if (!stop) {
            search_scan_recording_sidecars(&ctx);
        }
    }

    uint64_t next_cursor_wide = (uint64_t)cursor + (uint64_t)ctx.returned;
    uint32_t next_cursor = next_cursor_wide > UINT32_MAX ?
                           UINT32_MAX : (uint32_t)next_cursor_wide;
    json_writer_appendf(&writer,
                        "],\"returned\":%" PRIu32 ",\"matched_seen\":%" PRIu32
                        ",\"next_cursor\":%" PRIu32 ",\"has_more\":%s}",
                        ctx.returned, ctx.matched, next_cursor,
                        ctx.has_more ? "true" : "false");

    if (!json_writer_ok(&writer)) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "search response exceeds safe buffer");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t search_async_handler(httpd_req_t *req)
{
    esp_err_t ret = search_get_handler_internal(req);
    file_download_reader_end();
    return ret;
}

static esp_err_t search_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, search_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no storage reader worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_frame_svg_get_handler_internal(httpd_req_t *req)
{
    char query[160] = {0};
    char name[96] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recording name invalid or TF not mounted");
    }
    uint32_t frame_index = 0;
    if (!query_u32(query, "frame", 1, UINT32_MAX, &frame_index) &&
        !query_u32(query, "index", 1, UINT32_MAX, &frame_index)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "frame/index must be a positive integer");
    }

    char *line = (char *)alloc_psram_buffer(4096);
    if (!line) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no metadata buffer");
    }
    if (!read_recording_meta_line(name, frame_index, line, 4096)) {
        free(line);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "frame metadata not found");
    }

    uint32_t width = 0;
    uint32_t height = 0;
    vision_result_t vision = {0};
    fill_vision_disabled(&vision);
    json_get_u32_field(line, "width", &width);
    json_get_u32_field(line, "height", &height);
    parse_vision_from_json_line(line, &vision);
    free(line);
    if (width == 0 || height == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "frame dimensions missing");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    uint8_t *jpeg = NULL;
    uint32_t jpeg_size = 0;
    esp_err_t err = read_mjpeg_frame_to_psram(path, frame_index, &jpeg, &jpeg_size);
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, esp_err_to_name(err));
    }
    esp_err_t ret = send_overlay_svg_response(req, jpeg, jpeg_size, width, height, &vision);
    free(jpeg);
    return ret;
}

static esp_err_t recording_frame_svg_async_handler(httpd_req_t *req)
{
    esp_err_t ret = recording_frame_svg_get_handler_internal(req);
    file_download_reader_end();
    return ret;
}

static esp_err_t recording_frame_svg_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, recording_frame_svg_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no storage reader worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_manifest_get_handler_internal(httpd_req_t *req)
{
    char query[160] = {0};
    char name[96] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recording name invalid or storage not mounted");
    }

    char meta_name[96];
    meta_name_for_recording(name, meta_name, sizeof(meta_name));

    char recording_path[384];
    char meta_path[384];
    snprintf(recording_path, sizeof(recording_path), "%s/%s", RECORDING_DIR, name);
    snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, meta_name);

    struct stat recording_stat = {0};
    struct stat meta_stat = {0};
    bool recording_exists = stat(recording_path, &recording_stat) == 0;
    bool meta_exists = stat(meta_path, &meta_stat) == 0;
    if (!recording_exists || !meta_exists) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "recording or metadata not found");
    }

    char json[1024];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"index_version\":%" PRIu32 ",\"name\":\"%s\","
             "\"raw_uri\":\"%s%s\",\"meta\":\"%s\",\"meta_uri\":\"%s%s\","
             "\"annotated_uri\":\"%s%s\","
             "\"frame_overlay_template\":\"%s?name=%s&frame={frame}\","
             "\"storage_backend\":\"%s\",\"recording_bytes\":%" PRIu64
             ",\"meta_bytes\":%" PRIu64 ",\"search_uri\":\"/api/search?type=frame&label=\"}",
             (uint32_t)APP_JSONL_INDEX_VERSION, name,
             RECORDING_URI_PREFIX, name, meta_name, RECORDING_META_URI_PREFIX, meta_name,
             RECORDING_ANNOTATED_URI_PREFIX, name,
             RECORDING_FRAME_SVG_URI, name,
             s_storage_backend, (uint64_t)recording_stat.st_size, (uint64_t)meta_stat.st_size);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t recording_manifest_async_handler(httpd_req_t *req)
{
    esp_err_t ret = recording_manifest_get_handler_internal(req);
    file_download_reader_end();
    return ret;
}

static esp_err_t recording_manifest_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, recording_manifest_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no storage reader worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_annotated_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!s_sd_mounted) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "storage not mounted");
    }

    char name[96] = {0};
    const char *tail = req->uri + strlen(RECORDING_ANNOTATED_URI_PREFIX);
    size_t len = strcspn(tail, "?");
    if (len == 0 || len >= sizeof(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recording name required");
    }
    memcpy(name, tail, len);
    name[len] = '\0';
    if (!is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recording name invalid");
    }

    char meta_name[96];
    meta_name_for_recording(name, meta_name, sizeof(meta_name));
    char *html = (char *)alloc_psram_buffer(6144);
    if (!html) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no annotated page buffer");
    }

    snprintf(html, 6144,
             "<!doctype html><html><head><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<title>Annotated recording</title><style>"
             ":root{color-scheme:dark;background:#0c1116;color:#edf2f7;font-family:system-ui,-apple-system,Segoe UI,sans-serif}"
             "body{margin:0}.top{position:sticky;top:0;background:#111821;border-bottom:1px solid #263241;padding:10px;z-index:2}"
             ".name{font-size:14px;color:#9fb0c3;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.bar{display:flex;gap:8px;align-items:center;margin-top:8px}"
             "button{min-width:44px;height:38px;border:1px solid #3a4a5d;background:#172231;color:#edf2f7;border-radius:6px;font-size:16px}"
             "input{flex:1;min-width:0}.wrap{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:12px;padding:12px}"
             ".stage{background:#05070a;border:1px solid #263241;border-radius:8px;min-height:220px;display:grid;place-items:center;overflow:hidden}"
             "img{display:block;max-width:100%%;height:auto}.panel{border:1px solid #263241;border-radius:8px;padding:12px;background:#111821}"
             ".kv{font-size:14px;line-height:1.7;color:#c8d3df}.det{margin-top:8px;padding-top:8px;border-top:1px solid #263241;color:#f5d76e}"
             "@media(max-width:760px){.wrap{grid-template-columns:1fr}.panel{order:-1}}</style></head>"
             "<body><div class=\"top\"><div class=\"name\" id=\"name\"></div><div class=\"bar\">"
             "<button onclick=\"step(-1)\">&#8249;</button><input id=\"slider\" type=\"range\" min=\"1\" value=\"1\" oninput=\"show(Number(this.value)-1)\">"
             "<button onclick=\"step(1)\">&#8250;</button><a id=\"raw\" target=\"_blank\"><button>raw</button></a></div></div>"
             "<main class=\"wrap\"><section class=\"stage\"><img id=\"frame\" alt=\"annotated frame\"></section>"
             "<aside class=\"panel\"><div class=\"kv\" id=\"meta\">loading</div><div class=\"det\" id=\"dets\"></div></aside></main>"
             "<script>const rec='%s',metaUri='%s%s';let rows=[],idx=0;"
             "name.textContent=rec;raw.href='%s%s';"
             "function esc(s){return String(s==null?'':s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]))}"
             "function label(d){let a=d.detections||[];return a.length?a.map(x=>esc(x.label)+' '+(x.score||0)+'%%').join('<br>'):'none'}"
             "function show(i){if(!rows.length)return;idx=Math.max(0,Math.min(rows.length-1,i));let r=rows[idx];slider.max=rows.length;slider.value=idx+1;"
             "let f=r.frame_index||idx+1;frame.src='%s?name='+encodeURIComponent(rec)+'&frame='+f+'&ts='+Date.now();"
             "meta.innerHTML='frame '+f+' / '+rows.length+'<br>seq '+(r.seq||0)+'<br>model '+esc(r.model||'')+'<br>time '+(r.time_ms||0)+' ms<br>detections '+(r.detection_count||0)+'<br>score '+(r.best_score||r.candidate_score||0)+'%%<br>inference '+(r.inference_ms||0)+' ms';"
             "dets.innerHTML=label(r)}"
             "function step(d){show(idx+d)}"
             "fetch(metaUri,{cache:'no-store'}).then(r=>r.text()).then(t=>{rows=t.trim().split(/\\n+/).filter(Boolean).map(x=>JSON.parse(x));show(0)}).catch(e=>{meta.textContent=String(e)})"
             "</script></body></html>",
             name, RECORDING_META_URI_PREFIX, meta_name, RECORDING_URI_PREFIX, name, RECORDING_FRAME_SVG_URI);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, html);
    free(html);
    return ret;
}

static esp_err_t dataset_frame_svg_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[224] = {0};
    char dataset[DATASET_NAME_MAX] = {0};
    char run_id[80] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "run_id", run_id, sizeof(run_id)) != ESP_OK ||
        !is_safe_snapshot_name(run_id)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "run_id invalid");
    }
    esp_err_t dataset_ret =
        httpd_query_key_value(query, "dataset", dataset, sizeof(dataset));
    if (dataset_ret != ESP_OK || !is_safe_dataset_name(dataset)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dataset invalid");
    }
    uint32_t frame_index = 0;
    if (!query_u32(query, "index", 1, UINT32_MAX, &frame_index) &&
        !query_u32(query, "frame", 1, UINT32_MAX, &frame_index)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "index/frame must be a positive integer");
    }

    dataset_frame_cache_t cached = {0};
    if (dataset_frame_cache_copy(run_id, dataset, frame_index, &cached)) {
        const uint8_t *jpeg_start = NULL;
        const uint8_t *jpeg_end = NULL;
        if (is_builtin_video_dataset(dataset)) {
            if (!builtin_video_frame_image(dataset, frame_index, &jpeg_start, &jpeg_end)) {
                return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                           "embedded dataset frame missing");
            }
            return send_overlay_svg_response(req, jpeg_start,
                                             (uint32_t)(jpeg_end - jpeg_start),
                                             cached.source_w, cached.source_h,
                                             &cached.vision);
        }
        char cached_path[512];
        snprintf(cached_path, sizeof(cached_path), "%s/%s/frames/frame_%05" PRIu32 ".jpg",
                 DATASET_ROOT_DIR, dataset, frame_index);
        uint32_t cached_jpeg_size = 0;
        uint8_t *cached_jpeg = read_file_to_psram(cached_path, &cached_jpeg_size);
        if (!cached_jpeg) {
            return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                       "cached dataset frame image not found");
        }
        esp_err_t cached_ret = send_overlay_svg_response(req, cached_jpeg, cached_jpeg_size,
                                                         cached.source_w, cached.source_h,
                                                         &cached.vision);
        free(cached_jpeg);
        return cached_ret;
    }
    if (!s_sd_mounted) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND,
                                   "dataset frame not cached and TF not mounted");
    }

    char *line = (char *)alloc_psram_buffer(4096);
    if (!line) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no dataset result buffer");
    }
    if (!read_dataset_result_line(run_id, frame_index, line, 4096)) {
        free(line);
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "dataset frame result not found");
    }
    uint32_t width = 0;
    uint32_t height = 0;
    vision_result_t vision = {0};
    fill_vision_disabled(&vision);
    json_get_u32_field(line, "source_w", &width);
    json_get_u32_field(line, "source_h", &height);
    parse_vision_from_json_line(line, &vision);
    free(line);
    if (width == 0 || height == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "frame dimensions missing");
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/%s/frames/frame_%05" PRIu32 ".jpg",
             DATASET_ROOT_DIR, dataset, frame_index);
    uint32_t jpeg_size = 0;
    uint8_t *jpeg = read_file_to_psram(path, &jpeg_size);
    if (!jpeg) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "dataset frame image not found");
    }
    esp_err_t ret = send_overlay_svg_response(req, jpeg, jpeg_size, width, height, &vision);
    free(jpeg);
    return ret;
}

static bool dataset_persistence_errno_is_storage_failure(int error_number)
{
    return error_number == ENOSPC || dataset_errno_is_storage_io(error_number);
}

static void dataset_run_set_persistence_error(dataset_run_status_t *status,
                                              const char *operation,
                                              int error_number)
{
    if (!status) {
        return;
    }
    int saved_errno = error_number ? error_number : EIO;
    snprintf(status->last_error, sizeof(status->last_error),
             "dataset result %s failed (errno=%d); incomplete output was not published",
             operation ? operation : "write", saved_errno);
    status->result_uri[0] = '\0';
    status->summary_uri[0] = '\0';
    if (dataset_persistence_errno_is_storage_failure(saved_errno)) {
        storage_latch_io_error(operation ? operation : "dataset result persistence",
                               saved_errno);
    }
}

static esp_err_t sync_and_close_file(FILE **file_ptr, bool sync_to_media,
                                     int *error_number)
{
    if (!file_ptr || !*file_ptr) {
        if (error_number) {
            *error_number = EBADF;
        }
        errno = EBADF;
        return ESP_ERR_INVALID_ARG;
    }

    FILE *file = *file_ptr;
    int saved_errno = 0;
    errno = 0;
    if (fflush(file) != 0) {
        saved_errno = errno ? errno : EIO;
    }
    if (saved_errno == 0 && sync_to_media) {
        int fd = fileno(file);
        errno = 0;
        if (fd < 0 || fsync(fd) != 0) {
            saved_errno = errno ? errno : (fd < 0 ? EBADF : EIO);
        }
    }
    errno = 0;
    if (fclose(file) != 0 && saved_errno == 0) {
        saved_errno = errno ? errno : EIO;
    }
    *file_ptr = NULL;
    if (error_number) {
        *error_number = saved_errno;
    }
    if (saved_errno != 0) {
        errno = saved_errno;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void dataset_run_task(void *arg)
{
    (void)arg;
    dataset_run_request_t req;
    while (true) {
        if (xQueueReceive(s_dataset_run_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        dataset_run_status_t status = {0};
        status.queued = false;
        status.running = true;
        status.done = false;
        status.limit = req.limit ? req.limit : CONFIG_APP_DATASET_RUN_MAX_FRAMES;
        if (status.limit > CONFIG_APP_DATASET_RUN_MAX_FRAMES) {
            status.limit = CONFIG_APP_DATASET_RUN_MAX_FRAMES;
        }
        status.stride = req.stride ? req.stride : 1;
        status.method = req.method;
        strlcpy(status.dataset, req.dataset, sizeof(status.dataset));
        strlcpy(status.run_id, req.run_id, sizeof(status.run_id));
        bool builtin_video = is_builtin_video_dataset(status.dataset);
        if (builtin_video) {
            uint32_t frames = builtin_video_frame_count(status.dataset);
            uint32_t available = frames ? 1U + (frames - 1U) / status.stride : 0;
            if (status.limit > available) {
                status.limit = available;
            }
        }
        bool persist_results = storage_acceptance_ok();
        status.started_ms = esp_timer_get_time() / 1000;
        if (persist_results) {
            snprintf(status.result_uri, sizeof(status.result_uri),
                     "/api/dataset/run/results?run_id=%s", status.run_id);
            snprintf(status.summary_uri, sizeof(status.summary_uri),
                     "/api/dataset/run/results?run_id=%s&type=summary", status.run_id);
        }
        dataset_frame_cache_clear();
        dataset_status_update(&status);

        if (!storage_acceptance_ok() && !builtin_video) {
            strlcpy(status.last_error,
                    "TF card is not mounted and write-verified; run storage retry first",
                    sizeof(status.last_error));
            status.running = false;
            status.done = true;
            status.finished_ms = esp_timer_get_time() / 1000;
            dataset_status_update(&status);
            continue;
        }

        char result_path[512];
        char result_temp_path[528];
        char summary_path[512];
        char summary_temp_path[528];
        int result_path_len = snprintf(result_path, sizeof(result_path), "%s/%s.jsonl",
                                       DATASET_RUN_DIR, status.run_id);
        int result_temp_len = snprintf(result_temp_path, sizeof(result_temp_path),
                                       "%s.part", result_path);
        int summary_path_len = snprintf(summary_path, sizeof(summary_path),
                                        "%s/%s_summary.json", DATASET_RUN_DIR,
                                        status.run_id);
        int summary_temp_len = snprintf(summary_temp_path, sizeof(summary_temp_path),
                                        "%s.part", summary_path);
        FILE *result = NULL;
        if (persist_results) {
            if (result_path_len < 0 || (size_t)result_path_len >= sizeof(result_path) ||
                result_temp_len < 0 || (size_t)result_temp_len >= sizeof(result_temp_path) ||
                summary_path_len < 0 || (size_t)summary_path_len >= sizeof(summary_path) ||
                summary_temp_len < 0 || (size_t)summary_temp_len >= sizeof(summary_temp_path)) {
                dataset_run_set_persistence_error(&status, "path construction", ENAMETOOLONG);
                status.running = false;
                status.done = true;
                status.finished_ms = esp_timer_get_time() / 1000;
                dataset_status_update(&status);
                continue;
            }
            errno = 0;
            esp_err_t dir_ret = ensure_dir(DATASET_RUN_DIR);
            if (dir_ret != ESP_OK) {
                int dir_errno = errno ? errno : EIO;
                dataset_run_set_persistence_error(&status, "directory preparation",
                                                  dir_errno);
                status.running = false;
                status.done = true;
                status.finished_ms = esp_timer_get_time() / 1000;
                dataset_status_update(&status);
                continue;
            }
            errno = 0;
            if (unlink(result_temp_path) != 0 && errno != ENOENT) {
                int unlink_errno = errno ? errno : EIO;
                dataset_run_set_persistence_error(&status, "temporary-file cleanup",
                                                  unlink_errno);
                status.running = false;
                status.done = true;
                status.finished_ms = esp_timer_get_time() / 1000;
                dataset_status_update(&status);
                continue;
            }
            errno = 0;
            result = fopen(result_temp_path, "w");
            if (!result) {
                int open_errno = errno ? errno : EIO;
                dataset_run_set_persistence_error(&status, "open", open_errno);
                status.running = false;
                status.done = true;
                status.finished_ms = esp_timer_get_time() / 1000;
                dataset_status_update(&status);
                continue;
            }
        }

        uint32_t *latencies = (uint32_t *)calloc(DATASET_RUN_LATENCY_CAP, sizeof(uint32_t));
        uint32_t *sorted = (uint32_t *)calloc(DATASET_RUN_LATENCY_CAP, sizeof(uint32_t));
        char *detections = (char *)alloc_psram_buffer(1280);
        char *top_k_json = (char *)alloc_psram_buffer(512);
        if (!latencies || !sorted || !detections || !top_k_json) {
            free(latencies);
            free(sorted);
            free(top_k_json);
            free(detections);
            if (result) {
                fclose(result);
                result = NULL;
                unlink(result_temp_path);
            }
            strlcpy(status.last_error, "dataset run buffer alloc failed", sizeof(status.last_error));
            status.running = false;
            status.done = true;
            status.finished_ms = esp_timer_get_time() / 1000;
            dataset_status_update(&status);
            continue;
        }
        uint64_t sum_analysis = 0;
        bool result_persistence_failed = false;
        bool result_published = false;
        bool summary_published = false;
        for (uint32_t i = 0; i < status.limit; i++) {
            dataset_run_status_t latest;
            dataset_status_copy(&latest);
            if (latest.cancel || s_storage_quiescing || http_server_is_stopping()) {
                strlcpy(status.last_error,
                        latest.cancel ? "cancelled" : "cancelled for device mode change",
                        sizeof(status.last_error));
                break;
            }

            uint32_t frame_index = 1 + i * status.stride;
            char frame_path[512];
            snprintf(frame_path, sizeof(frame_path), "%s/%s/frames/frame_%05" PRIu32 ".jpg",
                     DATASET_ROOT_DIR, status.dataset, frame_index);
            char overlay_uri[192];
            snprintf(overlay_uri, sizeof(overlay_uri),
                     "%s?run_id=%s&dataset=%s&index=%" PRIu32,
                     DATASET_FRAME_SVG_URI, status.run_id, status.dataset, frame_index);
            uint32_t jpeg_size = 0;
            uint8_t *jpeg = NULL;
            if (builtin_video) {
                const uint8_t *jpeg_start = NULL;
                const uint8_t *jpeg_end = NULL;
                if (builtin_video_frame_image(status.dataset, frame_index, &jpeg_start, &jpeg_end)) {
                    jpeg_size = (uint32_t)(jpeg_end - jpeg_start);
                    jpeg = alloc_psram_buffer(jpeg_size);
                    if (jpeg) {
                        memcpy(jpeg, jpeg_start, jpeg_size);
                    }
                }
            } else {
                jpeg = read_file_to_psram(frame_path, &jpeg_size);
            }
            if (!jpeg) {
                status.failed_frames++;
                if (result) {
                    errno = 0;
                    int write_ret = fprintf(
                        result,
                        "{\"index_version\":%" PRIu32 ",\"index\":%" PRIu32
                        ",\"ok\":false,\"dataset\":\"%s\",\"file\":\"frames/frame_%05" PRIu32 ".jpg\","
                        "\"overlay_uri\":\"%s\",\"error\":\"read failed\"}\n",
                        (uint32_t)APP_JSONL_INDEX_VERSION, frame_index,
                        status.dataset, frame_index, overlay_uri);
                    if (write_ret < 0 || fflush(result) != 0) {
                        int write_errno = errno ? errno : EIO;
                        dataset_run_set_persistence_error(
                            &status, "JSONL write", write_errno);
                        result_persistence_failed = true;
                    }
                }
                if (!result_persistence_failed && i == 0) {
                    strlcpy(status.last_error, "first dataset frame missing or unreadable", sizeof(status.last_error));
                }
                break;
            }

            vision_result_t vision = {0};
            uint32_t source_w = 0;
            uint32_t source_h = 0;
            esp_err_t err = run_jpeg_on_inference_queue(jpeg, jpeg_size, status.method,
                                                        &vision, &source_w, &source_h);
            if (!detections_to_json(detections, 1280, &vision) ||
                !top_k_to_json(top_k_json, 512, &vision)) {
                strlcpy(detections, "[]", 1280);
                strlcpy(top_k_json, "[]", 512);
                err = ESP_ERR_INVALID_SIZE;
            }
            bool ok = err == ESP_OK;
            if (ok) {
                status.ok_frames++;
                status.detection_total += (status.method == RECOGNITION_METHOD_TINYCLS ||
                                           status.method == RECOGNITION_METHOD_FISH31) ?
                                          vision.object_count : vision.detection_count;
                if (vision.analysis_ms > 0 && status.processed < DATASET_RUN_LATENCY_CAP) {
                    latencies[status.processed] = (uint32_t)vision.analysis_ms;
                    sum_analysis += (uint32_t)vision.analysis_ms;
                    if ((uint32_t)vision.analysis_ms > status.max_analysis_ms) {
                        status.max_analysis_ms = (uint32_t)vision.analysis_ms;
                    }
                }
                if (status.method == RECOGNITION_METHOD_TINYCLS ||
                    status.method == RECOGNITION_METHOD_FISH31) {
                    label_count_add_with_unknown(status.labels, vision.label, true);
                } else {
                    for (uint32_t d = 0; d < vision.detection_count && d < APP_MAX_DETECTIONS; d++) {
                        label_count_add(status.labels, vision.detections[d].label);
                    }
                }
            } else {
                status.failed_frames++;
                strlcpy(status.last_error, esp_err_to_name(err), sizeof(status.last_error));
            }
            if (ok) {
                dataset_frame_cache_store(&status, frame_index, &vision, source_w, source_h);
            }

            if (result) {
                errno = 0;
                int write_ret = fprintf(result,
                    "{\"index_version\":%" PRIu32 ",\"index\":%" PRIu32
                    ",\"ok\":%s,\"dataset\":\"%s\",\"file\":\"frames/frame_%05" PRIu32 ".jpg\","
                    "\"overlay_uri\":\"%s\","
                    "\"source_w\":%" PRIu32 ",\"source_h\":%" PRIu32 ",\"jpeg_bytes\":%" PRIu32
                    ",\"method\":\"%s\",\"model\":\"%s\",\"model_bytes\":%" PRIu32 ",\"input\":%" PRIu32
                    ",\"label\":\"%s\",\"object\":\"%s\""
                    ",\"box_min_score\":%" PRIu32 ",\"best_score\":%" PRIu32
                    ",\"candidate_score\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
                    ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64
                    ",\"detection_count\":%" PRIu32 ",\"detections\":%s,\"top_k\":%s,\"error\":\"%s\"}\n",
                    (uint32_t)APP_JSONL_INDEX_VERSION, frame_index,
                    ok ? "true" : "false", status.dataset, frame_index,
                    overlay_uri, source_w, source_h, jpeg_size,
                    recognition_method_name(status.method),
                    model_name_for_method(status.method),
                    model_bytes_for_method(status.method), model_input_size_for_method(status.method),
                    vision.label, vision.object,
                    vision.box_min_score, vision.object_score,
                    vision.candidate_score, vision.raw_candidate_count,
                    vision.inference_ms, vision.analysis_ms,
                    vision.detection_count, detections, top_k_json, ok ? "" : esp_err_to_name(err));
                if (write_ret < 0 || fflush(result) != 0) {
                    int write_errno = errno ? errno : EIO;
                    dataset_run_set_persistence_error(&status, "JSONL write",
                                                      write_errno);
                    result_persistence_failed = true;
                    break;
                }
            }

            status.processed++;
            if (ok) {
                status.last_frame_index = frame_index;
                strlcpy(status.last_overlay_uri, overlay_uri, sizeof(status.last_overlay_uri));
            }
            if (status.ok_frames > 0) {
                status.avg_analysis_ms = (uint32_t)(sum_analysis / status.ok_frames);
                uint32_t n = status.processed < DATASET_RUN_LATENCY_CAP ? status.processed : DATASET_RUN_LATENCY_CAP;
                memcpy(sorted, latencies, sizeof(uint32_t) * n);
                qsort(sorted, n, sizeof(uint32_t), compare_u32_for_qsort);
                uint32_t p95_index = n > 0 ? (n - 1) * 95 / 100 : 0;
                status.p95_analysis_ms = sorted[p95_index];
            }
            dataset_status_update(&status);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (result) {
            int close_errno = 0;
            if (sync_and_close_file(&result, true, &close_errno) != ESP_OK) {
                dataset_run_set_persistence_error(&status, "JSONL sync/close",
                                                  close_errno);
                result_persistence_failed = true;
            }
        }
        free(latencies);
        free(sorted);
        free(top_k_json);
        free(detections);

        char labels[512];
        if (!label_counts_to_json(labels, sizeof(labels), status.labels)) {
            strlcpy(labels, "[]", sizeof(labels));
            strlcpy(status.last_error, "dataset label summary exceeded safe buffer",
                    sizeof(status.last_error));
        }
        FILE *summary = NULL;
        if (persist_results && !result_persistence_failed) {
            errno = 0;
            if (unlink(summary_temp_path) != 0 && errno != ENOENT) {
                int unlink_errno = errno ? errno : EIO;
                dataset_run_set_persistence_error(
                    &status, "summary temporary-file cleanup", unlink_errno);
                result_persistence_failed = true;
            } else {
                errno = 0;
                summary = fopen(summary_temp_path, "w");
                if (!summary) {
                    int open_errno = errno ? errno : EIO;
                    dataset_run_set_persistence_error(&status, "summary open",
                                                      open_errno);
                    result_persistence_failed = true;
                }
            }
        }
        if (summary) {
            errno = 0;
            int summary_write_ret = fprintf(summary,
                    "{\"index_version\":%" PRIu32 ",\"run_id\":\"%s\",\"dataset\":\"%s\","
                    "\"overlay_endpoint\":\"%s\",\"processed\":%" PRIu32
                    ",\"ok_frames\":%" PRIu32 ",\"failed_frames\":%" PRIu32
                    ",\"detection_total\":%" PRIu32 ",\"avg_analysis_ms\":%" PRIu32
                    ",\"p95_analysis_ms\":%" PRIu32 ",\"max_analysis_ms\":%" PRIu32
                    ",\"method\":\"%s\",\"model\":\"%s\",\"model_bytes\":%" PRIu32 ",\"labels\":%s,"
                    "\"started_ms\":%" PRId64 ",\"finished_ms\":%" PRId64 ",\"error\":\"%s\"}\n",
                    (uint32_t)APP_JSONL_INDEX_VERSION, status.run_id, status.dataset,
                    DATASET_FRAME_SVG_URI, status.processed,
                    status.ok_frames, status.failed_frames, status.detection_total,
                    status.avg_analysis_ms, status.p95_analysis_ms, status.max_analysis_ms,
                    recognition_method_name(status.method), model_name_for_method(status.method),
                    model_bytes_for_method(status.method), labels,
                    status.started_ms, esp_timer_get_time() / 1000, status.last_error);
            if (summary_write_ret < 0) {
                int write_errno = errno ? errno : EIO;
                dataset_run_set_persistence_error(&status, "summary write",
                                                  write_errno);
                result_persistence_failed = true;
            }
            int close_errno = 0;
            if (sync_and_close_file(&summary, true, &close_errno) != ESP_OK) {
                dataset_run_set_persistence_error(&status, "summary sync/close",
                                                  close_errno);
                result_persistence_failed = true;
            }
            if (!result_persistence_failed) {
                errno = 0;
                if (rename(summary_temp_path, summary_path) != 0) {
                    int rename_errno = errno ? errno : EIO;
                    dataset_run_set_persistence_error(&status, "summary commit",
                                                      rename_errno);
                    result_persistence_failed = true;
                } else {
                    summary_published = true;
                }
            }
            /* Publish JSONL last. Its presence is the commit marker consumed by
             * the result endpoint, so a reader can never observe JSONL without
             * the matching summary. */
            if (!result_persistence_failed) {
                errno = 0;
                if (rename(result_temp_path, result_path) != 0) {
                    int rename_errno = errno ? errno : EIO;
                    dataset_run_set_persistence_error(&status, "JSONL commit",
                                                      rename_errno);
                    result_persistence_failed = true;
                } else {
                    result_published = true;
                }
            }
        }
        if (persist_results && result_persistence_failed) {
            (void)unlink(result_temp_path);
            (void)unlink(summary_temp_path);
            if (result_published) {
                errno = 0;
                if (unlink(result_path) != 0 && errno != ENOENT) {
                    int cleanup_errno = errno ? errno : EIO;
                    storage_latch_io_error("dataset JSONL rollback", cleanup_errno);
                    ESP_LOGE(TAG, "dataset JSONL rollback failed: errno=%d",
                             cleanup_errno);
                }
            }
            if (summary_published) {
                errno = 0;
                if (unlink(summary_path) != 0 && errno != ENOENT) {
                    int cleanup_errno = errno ? errno : EIO;
                    storage_latch_io_error("dataset summary rollback", cleanup_errno);
                    ESP_LOGE(TAG, "dataset summary rollback failed: errno=%d",
                             cleanup_errno);
                }
            }
        }
        status.queued = false;
        status.running = false;
        status.done = true;
        status.finished_ms = esp_timer_get_time() / 1000;
        dataset_status_update(&status);
        update_sd_info();
    }
}

static uint32_t count_dataset_frames(const char *dataset)
{
    if (!is_safe_dataset_name(dataset)) {
        return 0;
    }
    char frames_dir[512];
    snprintf(frames_dir, sizeof(frames_dir), "%s/%s/frames", DATASET_ROOT_DIR, dataset);
    DIR *dir = opendir(frames_dir);
    if (!dir) {
        return 0;
    }
    uint32_t count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (is_safe_snapshot_name(entry->d_name) &&
            (has_suffix(entry->d_name, ".jpg") || has_suffix(entry->d_name, ".jpeg"))) {
            count++;
        }
    }
    closedir(dir);
    return count;
}

static esp_err_t datasets_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    size_t cap = 2048;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no dataset buffer");
    }

    json_writer_t writer;
    json_writer_init(&writer, json, cap);
    json_writer_appendf(&writer, "{\"sd_mounted\":%s,\"storage_status\":",
                        s_sd_mounted ? "true" : "false");
    json_writer_append_escaped_string(&writer, s_storage_status);
    json_writer_appendf(&writer, ",\"datasets\":[{\"name\":");
    json_writer_append_escaped_string(&writer, BUILTIN_COCO_VIDEO_DATASET);
    json_writer_appendf(&writer,
                        ",\"frames\":%" PRIu32
                        ",\"method\":\"coco\",\"source\":\"firmware\",\"embedded\":true},"
                        "{\"name\":",
                        (uint32_t)BUILTIN_COCO_VIDEO_FRAMES);
    json_writer_append_escaped_string(&writer, BUILTIN_TINYCLS_VIDEO_DATASET);
    json_writer_appendf(&writer,
                        ",\"frames\":%" PRIu32
                        ",\"method\":\"tinycls\",\"source\":\"firmware\",\"embedded\":true},"
                        "{\"name\":",
                        (uint32_t)BUILTIN_TINYCLS_VIDEO_FRAMES);
    json_writer_append_escaped_string(&writer, BUILTIN_FISH31_VIDEO_DATASET);
    json_writer_appendf(&writer,
                        ",\"frames\":%" PRIu32
                        ",\"method\":\"fish31\",\"source\":\"firmware\",\"embedded\":true}",
                        (uint32_t)BUILTIN_FISH31_VIDEO_FRAMES);
    bool dataset_path_error = false;
    if (s_sd_mounted) {
        DIR *dir = opendir(DATASET_ROOT_DIR);
        if (dir) {
            struct dirent *entry;
            while (json_writer_ok(&writer) && (entry = readdir(dir)) != NULL) {
                if (!is_safe_dataset_name(entry->d_name) ||
                    is_builtin_video_dataset(entry->d_name)) {
                    continue;
                }
                uint32_t frames = count_dataset_frames(entry->d_name);
                char path[512];
                int path_len = snprintf(path, sizeof(path), "%s/%s",
                                        DATASET_ROOT_DIR, entry->d_name);
                if (path_len < 0 || (size_t)path_len >= sizeof(path)) {
                    dataset_path_error = true;
                    break;
                }
                json_writer_appendf(&writer, ",{\"name\":");
                json_writer_append_escaped_string(&writer, entry->d_name);
                json_writer_appendf(&writer, ",\"frames\":%" PRIu32 ",\"path\":", frames);
                json_writer_append_escaped_string(&writer, path);
                json_writer_appendf(&writer,
                                    ",\"source\":\"storage\",\"embedded\":false}");
            }
            closedir(dir);
        }
    }
    json_writer_appendf(&writer, "]}");
    if (dataset_path_error || !json_writer_ok(&writer)) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "dataset response exceeds safe buffer");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t ensure_dataset_directory(const char *path)
{
    errno = 0;
    esp_err_t ret = ensure_dir(path);
    int ensure_errno = errno;
    if (ret == ESP_ERR_INVALID_STATE) {
        errno = EISDIR;
        return ret;
    }
    if (ret != ESP_OK) {
        errno = ensure_errno ? ensure_errno : EIO;
        return ret;
    }

    /* Verify EEXIST races as well as freshly created directories. A regular
     * file at any parent component is a path conflict, not a TF media fault. */
    struct stat st = {0};
    errno = 0;
    if (stat(path, &st) != 0) {
        int stat_errno = errno ? errno : EIO;
        errno = stat_errno;
        return ESP_FAIL;
    }
    if (!S_ISDIR(st.st_mode)) {
        errno = EISDIR;
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t ensure_dataset_parent_dirs(const char *dataset, const char *relpath)
{
    if (!is_safe_dataset_name(dataset) || !is_safe_dataset_relpath(relpath)) {
        errno = EINVAL;
        return ESP_ERR_INVALID_ARG;
    }

    char dataset_dir[512];
    int dataset_len = snprintf(dataset_dir, sizeof(dataset_dir), "%s/%s",
                               DATASET_ROOT_DIR, dataset);
    if (dataset_len < 0 || (size_t)dataset_len >= sizeof(dataset_dir)) {
        errno = ENAMETOOLONG;
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret = ensure_dataset_directory(DATASET_ROOT_DIR);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ensure_dataset_directory(dataset_dir);
    if (ret != ESP_OK) {
        return ret;
    }

    /* The API and boot recovery both support the same bounded nesting depth.
     * Create every parent component so accepted paths cannot later fail with
     * ENOENT merely because they are not under the historical frames/ folder. */
    for (const char *separator = strchr(relpath, '/'); separator;
         separator = strchr(separator + 1, '/')) {
        size_t prefix_len = (size_t)(separator - relpath);
        char parent_dir[512];
        int parent_len = snprintf(parent_dir, sizeof(parent_dir), "%s/%.*s",
                                  dataset_dir, (int)prefix_len, relpath);
        if (parent_len < 0 || (size_t)parent_len >= sizeof(parent_dir)) {
            errno = ENAMETOOLONG;
            return ESP_ERR_INVALID_SIZE;
        }
        ret = ensure_dataset_directory(parent_dir);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t receive_dataset_upload(httpd_req_t *req, const char *temp_path,
                                        uint64_t *written_total)
{
    errno = 0;
    FILE *file = fopen(temp_path, "wb");
    if (!file) {
        return ESP_FAIL;
    }

    char buf[2048];
    size_t remaining = req->content_len;
    uint64_t total = 0;
    esp_err_t ret = ESP_OK;
    int operation_errno = 0;
    while (remaining > 0) {
        size_t want = remaining > sizeof(buf) ? sizeof(buf) : remaining;
        int recv_len = httpd_req_recv(req, buf, want);
        if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
            ret = ESP_ERR_TIMEOUT;
            break;
        }
        if (recv_len <= 0 || (size_t)recv_len > remaining) {
            ret = ESP_ERR_INVALID_RESPONSE;
            break;
        }
        if (fwrite(buf, 1, (size_t)recv_len, file) != (size_t)recv_len) {
            ret = ESP_FAIL;
            operation_errno = errno ? errno : EIO;
            break;
        }
        total += (uint64_t)recv_len;
        remaining -= (size_t)recv_len;
    }
    if (ret == ESP_OK && (fflush(file) != 0 || fsync(fileno(file)) != 0)) {
        ret = ESP_FAIL;
        operation_errno = errno ? errno : EIO;
    }
    errno = 0;
    if (fclose(file) != 0) {
        int close_errno = errno ? errno : EIO;
        if (ret == ESP_OK || operation_errno == 0) {
            ret = ESP_FAIL;
            operation_errno = close_errno;
        }
    }
    if (ret != ESP_OK) {
        unlink(temp_path);
        errno = operation_errno;
        return ret;
    }
    *written_total = total;
    errno = 0;
    return ESP_OK;
}

typedef struct {
    bool had_previous;
    bool previous_file_available;
    bool recovered_stale_backup;
    bool recovery_backup_retained;
    bool backup_cleanup_pending;
    int backup_cleanup_errno;
} dataset_upload_commit_result_t;

static esp_err_t dataset_upload_regular_file_state(const char *path, bool *exists)
{
    if (!path || !exists) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    if (stat(path, &st) == 0) {
        if (!S_ISREG(st.st_mode)) {
            errno = EISDIR;
            return ESP_ERR_INVALID_STATE;
        }
        *exists = true;
        return ESP_OK;
    }
    if (errno == ENOENT) {
        *exists = false;
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t commit_dataset_upload(const char *temp_path, const char *final_path,
                                       dataset_upload_commit_result_t *result)
{
    if (!temp_path || !final_path || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(result, 0, sizeof(*result));

    char backup_path[544];
    int backup_len = snprintf(backup_path, sizeof(backup_path), "%s%s",
                              final_path, DATASET_UPLOAD_BACKUP_SUFFIX);
    if (backup_len < 0 || (size_t)backup_len >= sizeof(backup_path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    bool final_exists = false;
    bool backup_exists = false;
    esp_err_t ret = dataset_upload_regular_file_state(final_path, &final_exists);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = dataset_upload_regular_file_state(backup_path, &backup_exists);
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     * A backup without a final file is the only known-good copy left by an
     * interrupted transaction. Restore it before starting another commit and
     * never delete it merely to make the backup pathname available.
     */
    if (!final_exists && backup_exists) {
        if (rename(backup_path, final_path) != 0) {
            result->had_previous = true;
            result->recovery_backup_retained = true;
            ESP_LOGE(TAG,
                     "dataset upload found a recovery backup but could not restore %s",
                     backup_path);
            return ESP_FAIL;
        }
        final_exists = true;
        backup_exists = false;
        result->recovered_stale_backup = true;
        ESP_LOGW(TAG, "restored interrupted dataset upload backup: %s", final_path);
    }

    result->had_previous = final_exists;
    result->previous_file_available = final_exists;

    /* Both files means the preceding commit installed its new final but did
     * not finish backup cleanup. The final protects the data, so this stale
     * backup may be removed before creating the next transaction backup. */
    if (final_exists && backup_exists) {
        if (unlink(backup_path) != 0) {
            ESP_LOGE(TAG, "dataset upload could not remove stale backup: %s", backup_path);
            return ESP_FAIL;
        }
        backup_exists = false;
    }

    if (final_exists && rename(final_path, backup_path) != 0) {
        return ESP_FAIL;
    }
    if (final_exists) {
        backup_exists = true;
        result->previous_file_available = false;
    }

    if (rename(temp_path, final_path) != 0) {
        int commit_errno = errno;
        if (backup_exists) {
            if (rename(backup_path, final_path) == 0) {
                result->previous_file_available = true;
            } else {
                int rollback_errno = errno;
                result->recovery_backup_retained = true;
                ESP_LOGE(TAG,
                         "dataset upload rollback failed; previous file retained at %s",
                         backup_path);
                errno = rollback_errno;
                return ESP_FAIL;
            }
        }
        errno = commit_errno;
        return ESP_FAIL;
    }

    /* The new final is now installed. Only at this point may its predecessor
     * be deleted. A cleanup failure does not invalidate the new final; retain
     * the backup and report the degraded-but-usable state to the caller. */
    if (backup_exists && unlink(backup_path) != 0) {
        result->backup_cleanup_pending = true;
        result->backup_cleanup_errno = errno;
        ESP_LOGW(TAG, "dataset upload committed but backup cleanup is pending: %s",
                 backup_path);
    }
    return ESP_OK;
}

static esp_err_t dataset_file_put_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (s_storage_quiescing || storage_transition_active()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(req, "storage maintenance is active; retry the upload shortly");
    }
    char query[256] = {0};
    char dataset[DATASET_NAME_MAX] = {0};
    char relpath[DATASET_PATH_MAX] = {0};
    if (!s_sd_mounted || !storage_acceptance_ok()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(req,
                                  "TF card is not mounted and write-verified; use storage retry first");
    }
    if (httpd_req_get_url_query_len(req) >= sizeof(query) ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "dataset", dataset, sizeof(dataset)) != ESP_OK ||
        httpd_query_key_value(query, "path", relpath, sizeof(relpath)) != ESP_OK ||
        !is_safe_dataset_name(dataset) || !is_safe_dataset_relpath(relpath)) {
        char message[192];
        snprintf(message, sizeof(message),
                 "dataset name/path is invalid or too long; file paths support at most %u nested directory levels so interrupted uploads remain recoverable",
                 (unsigned)DATASET_RECOVERY_MAX_DIR_DEPTH);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, message);
    }
    for (char *p = relpath; *p; p++) {
        if (*p == '\\') {
            *p = '/';
        }
    }

    size_t max_upload = (has_suffix(relpath, ".jpg") || has_suffix(relpath, ".jpeg")) ?
                        DATASET_IMAGE_UPLOAD_MAX_BYTES : DATASET_METADATA_UPLOAD_MAX_BYTES;
    if (req->content_len == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "upload body is empty; select a non-empty file and retry");
    }
    if (req->content_len > max_upload) {
        return httpd_resp_send_err(req, HTTPD_413_CONTENT_TOO_LARGE,
                                   "upload exceeds the file-type size limit; reduce the file size and retry");
    }
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (s_storage_quiescing || storage_transition_active()) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(req, "storage maintenance started; retry the upload shortly");
    }

    if (xSemaphoreTake(s_storage_lock,
                       pdMS_TO_TICKS(RECORDING_CLEANUP_LOCK_TIMEOUT_MS)) !=
        pdTRUE) {
        file_download_reader_end();
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(
            req, "storage is busy; wait for the current operation and retry the upload");
    }
    if (s_storage_quiescing || storage_transition_active() ||
        !storage_acceptance_ok()) {
        xSemaphoreGive(s_storage_lock);
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(
            req, "storage maintenance started before upload; no file was changed; retry shortly");
    }
    update_sd_info();
    uint64_t required_free = (uint64_t)req->content_len +
                             DATASET_UPLOAD_FREE_RESERVE_BYTES;
    if (s_sd_free_bytes < required_free) {
        xSemaphoreGive(s_storage_lock);
        file_download_reader_end();
        httpd_resp_set_status(req, "507 Insufficient Storage");
        return httpd_resp_sendstr(req,
                                  "TF card does not have enough free space for a safe transactional upload");
    }
    esp_err_t ret = ESP_OK;
    esp_err_t parent_dirs_ret = ESP_OK;
    int saved_errno = 0;
    errno = 0;
    uint64_t written_total = 0;
    dataset_upload_commit_result_t commit_result = {0};
    char path[512];
    char temp_path[544];
    int path_len = snprintf(path, sizeof(path), "%s/%s/%s",
                            DATASET_ROOT_DIR, dataset, relpath);
    int temp_len = path_len < 0 ? -1 :
                   snprintf(temp_path, sizeof(temp_path), "%s.upload.part", path);
    if (path_len < 0 || (size_t)path_len >= sizeof(path) ||
        temp_len < 0 || (size_t)temp_len >= sizeof(temp_path)) {
        ret = ESP_ERR_INVALID_SIZE;
    } else if (s_storage_quiescing || storage_transition_active() ||
               !storage_acceptance_ok()) {
        ret = ESP_ERR_INVALID_STATE;
    } else if ((parent_dirs_ret = ensure_dataset_parent_dirs(dataset, relpath)) != ESP_OK) {
        ret = parent_dirs_ret;
        saved_errno = errno;
    } else {
        errno = 0;
        if (unlink(temp_path) != 0 && errno != ENOENT) {
            saved_errno = errno ? errno : EIO;
            ret = saved_errno == EISDIR ? ESP_ERR_INVALID_STATE : ESP_FAIL;
        } else {
            errno = 0;
            ret = receive_dataset_upload(req, temp_path, &written_total);
            saved_errno = errno;
        }
        if (ret == ESP_OK) {
            errno = 0;
            ret = commit_dataset_upload(temp_path, path, &commit_result);
            saved_errno = errno;
        }
        if (ret != ESP_OK) {
            unlink(temp_path);
        }

        if (ret == ESP_OK) {
            if (commit_result.backup_cleanup_pending) {
                if (dataset_errno_is_storage_io(
                        commit_result.backup_cleanup_errno)) {
                    storage_latch_io_error("dataset upload backup cleanup",
                                           commit_result.backup_cleanup_errno);
                } else {
                    set_storage_status(
                        "TF ready; dataset backup cleanup pending errno=%d; inspect via USB",
                        commit_result.backup_cleanup_errno);
                }
            }
            update_sd_info();
            xSemaphoreGive(s_storage_lock);
            file_download_reader_end();

            char json[1024];
            json_writer_t writer;
            json_writer_init(&writer, json, sizeof(json));
            json_writer_appendf(&writer, "{\"ok\":true,\"dataset\":");
            json_writer_append_escaped_string(&writer, dataset);
            json_writer_appendf(&writer, ",\"path\":");
            json_writer_append_escaped_string(&writer, relpath);
            json_writer_appendf(
                &writer,
                ",\"bytes\":%" PRIu64
                ",\"recovered_stale_backup\":%s,\"backup_cleanup_pending\":%s",
                written_total,
                commit_result.recovered_stale_backup ? "true" : "false",
                commit_result.backup_cleanup_pending ? "true" : "false");
            if (commit_result.backup_cleanup_pending) {
                char backup_relpath[DATASET_PATH_MAX + 16];
                snprintf(backup_relpath, sizeof(backup_relpath), "%s.upload.bak", relpath);
                json_writer_appendf(&writer, ",\"backup_path\":");
                json_writer_append_escaped_string(&writer, backup_relpath);
                json_writer_appendf(&writer, ",\"warning\":");
                json_writer_append_escaped_string(
                    &writer,
                    "the new file is installed, but the previous backup could not be removed");
                json_writer_appendf(&writer, ",\"action\":");
                json_writer_append_escaped_string(
                    &writer,
                    "the new file remains usable; inspect the retained backup via USB before uploading this path again");
            }
            json_writer_appendf(&writer, "}");
            if (!json_writer_ok(&writer)) {
                return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                           "upload response exceeds safe buffer");
            }
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return http_send_cstr_chunked(req, json);
        }
    }

    xSemaphoreGive(s_storage_lock);
    file_download_reader_end();
    if (ret == ESP_ERR_TIMEOUT) {
        httpd_resp_set_status(req, "408 Request Timeout");
        return httpd_resp_sendstr(
            req,
            "upload timed out; the incomplete temporary file was removed and any existing dataset file was left unchanged; retry the upload");
    }
    if (ret == ESP_ERR_INVALID_RESPONSE) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "upload ended before the declared Content-Length; the incomplete temporary file was removed and any existing dataset file was left unchanged; retry with a stable connection");
    }
    if (ret == ESP_ERR_INVALID_STATE && saved_errno == 0) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_hdr(req, "Retry-After", "2");
        return httpd_resp_sendstr(
            req,
            "storage became unavailable before the upload started; no dataset file was changed; wait for storage recovery and retry");
    }
    if (saved_errno == EISDIR || saved_errno == EEXIST) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(
            req,
            "the requested dataset path conflicts with a directory; choose another file path or repair the dataset from USB/TF storage");
    }
    bool storage_io_failure =
        ret == ESP_FAIL && dataset_errno_is_storage_io(saved_errno);
    if (storage_io_failure) {
        storage_latch_io_error("dataset transactional upload", saved_errno);
    }
    bool storage_full = ret == ESP_FAIL && saved_errno == ENOSPC;
    if (storage_full) {
        storage_latch_io_error("dataset transactional upload", saved_errno);
    }
    if (commit_result.recovery_backup_retained) {
        char backup_relpath[DATASET_PATH_MAX + 16];
        snprintf(backup_relpath, sizeof(backup_relpath), "%s.upload.bak", relpath);
        char json[1024];
        json_writer_t writer;
        json_writer_init(&writer, json, sizeof(json));
        json_writer_appendf(
            &writer,
            "{\"ok\":false,\"error\":\"upload_commit_degraded\",\"dataset\":");
        json_writer_append_escaped_string(&writer, dataset);
        json_writer_appendf(&writer, ",\"path\":");
        json_writer_append_escaped_string(&writer, relpath);
        json_writer_appendf(&writer, ",\"recovery_path\":");
        json_writer_append_escaped_string(&writer, backup_relpath);
        json_writer_appendf(&writer, ",\"message\":");
        json_writer_append_escaped_string(
            &writer,
            "the previous file could not be restored to its normal path; its recovery backup was preserved");
        json_writer_appendf(&writer, ",\"action\":");
        json_writer_append_escaped_string(
            &writer,
            "do not upload this path again; use USB export or TF recovery to copy or rename the recovery backup, then run TF retry");
        json_writer_appendf(&writer, "}");
        if (!json_writer_ok(&writer)) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "upload degraded and recovery response overflowed");
        }
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return http_send_cstr_chunked(req, json);
    }
    if (storage_full) {
        httpd_resp_set_status(req, "507 Insufficient Storage");
        return httpd_resp_sendstr(
            req,
            "TF card became full during the upload; existing data and any recovery backup were preserved; free space, then use TF retry");
    }
    if (commit_result.had_previous && commit_result.previous_file_available) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "upload could not be committed; the previous file remains available at its normal path; run TF retry before uploading again");
    }
    if (ret == ESP_ERR_INVALID_ARG || ret == ESP_ERR_INVALID_SIZE) {
        return httpd_resp_send_err(
            req, HTTPD_400_BAD_REQUEST,
            "dataset path could not be represented safely; use a shorter path within the supported nesting depth");
    }
    if (ret == ESP_FAIL && saved_errno != 0 && !storage_io_failure) {
        char message[256];
        snprintf(message, sizeof(message),
                 "dataset upload could not use the requested path (errno=%d); existing data and recovery backups were left unchanged; inspect the dataset layout via USB and retry",
                 saved_errno);
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, message);
    }
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "upload could not be committed; no dataset file was changed; check TF health, run storage retry, and try again");
}

static esp_err_t dataset_run_start_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
    if (http_server_is_stopping() || s_storage_quiescing ||
        storage_transition_active()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "dataset service is quiescing");
    }
    if (export_mode_reject(req, "dataset run")) {
        return ESP_OK;
    }
    char query[160] = {0};
    char dataset[DATASET_NAME_MAX] = BUILTIN_FISH31_VIDEO_DATASET;
    char method_text[16] = {0};
    char limit_text[12] = {0};
    char stride_text[12] = {0};
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len >= sizeof(query) ||
        (query_len > 0 && httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "dataset run query is too long or unreadable");
    }
    if (query_len > 0) {
        esp_err_t dataset_ret = httpd_query_key_value(query, "dataset", dataset, sizeof(dataset));
        esp_err_t method_ret = httpd_query_key_value(query, "method", method_text, sizeof(method_text));
        esp_err_t limit_ret = httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text));
        esp_err_t stride_ret = httpd_query_key_value(query, "stride", stride_text, sizeof(stride_text));
        if ((dataset_ret != ESP_OK && dataset_ret != ESP_ERR_NOT_FOUND) ||
            (method_ret != ESP_OK && method_ret != ESP_ERR_NOT_FOUND) ||
            (limit_ret != ESP_OK && limit_ret != ESP_ERR_NOT_FOUND) ||
            (stride_ret != ESP_OK && stride_ret != ESP_ERR_NOT_FOUND)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "dataset run parameter is too long or malformed");
        }
    }
    recognition_method_t method = is_builtin_fish31_video_dataset(dataset) ?
                                  RECOGNITION_METHOD_FISH31 :
                                  (is_builtin_tinycls_video_dataset(dataset) ?
                                   RECOGNITION_METHOD_TINYCLS : RECOGNITION_METHOD_COCO);
    if (method_text[0] && !parse_validation_method_strict(method_text, &method)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "method must be fish31, tinycls, or coco");
    }
    bool builtin_video = is_builtin_video_dataset(dataset);
    if ((!s_sd_mounted && !builtin_video) || !is_safe_dataset_name(dataset)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dataset invalid or TF not mounted");
    }
    if (!recognition_method_uses_jpeg_inference(method) || !validation_selftest_method_available(method)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "method invalid or unavailable");
    }
    if (is_builtin_coco_video_dataset(dataset) && method != RECOGNITION_METHOD_COCO) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "coco_video_demo requires method=coco");
    }
    if (is_builtin_tinycls_video_dataset(dataset) && method != RECOGNITION_METHOD_TINYCLS) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "tinycls_marine_demo requires method=tinycls");
    }
    if (is_builtin_fish31_video_dataset(dataset) && method != RECOGNITION_METHOD_FISH31) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "fish31_video_demo requires method=fish31");
    }
    dataset_run_request_t run = {0};
    strlcpy(run.dataset, dataset, sizeof(run.dataset));
    run.method = method;
    run.limit = CONFIG_APP_DATASET_RUN_MAX_FRAMES;
    run.stride = 1;
    if (limit_text[0] &&
        !query_u32(query, "limit", 1, CONFIG_APP_DATASET_RUN_MAX_FRAMES, &run.limit)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "limit is outside the supported dataset frame range");
    }
    if (stride_text[0] && !query_u32(query, "stride", 1, 1000, &run.stride)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "stride must be an integer in range 1..1000");
    }
    if (builtin_video) {
        uint32_t frames = builtin_video_frame_count(run.dataset);
        uint32_t available = frames ? 1U + (frames - 1U) / run.stride : 0;
        if (run.limit > available) {
            run.limit = available;
        }
    }

    int64_t queued_ms = esp_timer_get_time() / 1000;
    snprintf(run.run_id, sizeof(run.run_id), "%s_%08" PRIx32 "_%010" PRId64,
             run.dataset, s_boot_id, queued_ms);
    dataset_run_status_t queued = {
        .queued = true,
        .running = false,
        .done = false,
        .method = run.method,
        .limit = run.limit,
        .stride = run.stride,
        .started_ms = queued_ms,
    };
    strlcpy(queued.dataset, run.dataset, sizeof(queued.dataset));
    strlcpy(queued.run_id, run.run_id, sizeof(queued.run_id));
    if (!dataset_status_try_queue(&queued)) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "dataset run already active");
    }

    if (!s_dataset_run_queue || xQueueSend(s_dataset_run_queue, &run, pdMS_TO_TICKS(100)) != pdTRUE) {
        queued.queued = false;
        queued.done = true;
        queued.finished_ms = esp_timer_get_time() / 1000;
        strlcpy(queued.last_error, "dataset queue failed", sizeof(queued.last_error));
        dataset_status_update(&queued);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "dataset queue failed");
    }

    char json[256];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":true,\"running\":false,\"done\":false,"
             "\"dataset\":\"%s\",\"method\":\"%s\",\"run_id\":\"%s\",\"limit\":%" PRIu32
             ",\"stride\":%" PRIu32 "}",
             run.dataset, recognition_method_name(run.method), run.run_id, run.limit, run.stride);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t dataset_run_status_handler(httpd_req_t *req)
{
    record_http_request(req);
    dataset_run_status_t status;
    dataset_status_copy(&status);
    char labels[512];
    if (!label_counts_to_json(labels, sizeof(labels), status.labels)) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "dataset labels exceed safe buffer");
    }
    const char *state = status.queued ? "queued" :
                        status.running ? "running" :
                        status.done ? "done" : "idle";
    char json[2200];
    snprintf(json, sizeof(json),
             "{\"index_version\":%" PRIu32 ",\"state\":\"%s\","
             "\"queued\":%s,\"running\":%s,\"done\":%s,"
             "\"dataset\":\"%s\",\"method\":\"%s\",\"run_id\":\"%s\",\"overlay_endpoint\":\"%s\","
             "\"result_uri\":\"%s\",\"summary_uri\":\"%s\",\"limit\":%" PRIu32
             ",\"stride\":%" PRIu32 ",\"processed\":%" PRIu32 ",\"ok_frames\":%" PRIu32
             ",\"failed_frames\":%" PRIu32 ",\"detection_total\":%" PRIu32
             ",\"avg_analysis_ms\":%" PRIu32 ",\"p95_analysis_ms\":%" PRIu32
             ",\"max_analysis_ms\":%" PRIu32 ",\"last_frame_index\":%" PRIu32
             ",\"last_overlay_uri\":\"%s\",\"labels\":%s,"
             "\"started_ms\":%" PRId64 ",\"finished_ms\":%" PRId64 ",\"error\":\"%s\"}",
             (uint32_t)APP_JSONL_INDEX_VERSION, state,
             status.queued ? "true" : "false",
             status.running ? "true" : "false", status.done ? "true" : "false",
             status.dataset, recognition_method_name(status.method), status.run_id, DATASET_FRAME_SVG_URI,
             status.result_uri, status.summary_uri,
             status.limit, status.stride, status.processed, status.ok_frames,
             status.failed_frames, status.detection_total,
             status.avg_analysis_ms, status.p95_analysis_ms, status.max_analysis_ms,
             status.last_frame_index, status.last_overlay_uri,
             labels, status.started_ms, status.finished_ms, status.last_error);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t dataset_run_results_async_handler(httpd_req_t *req)
{
    char query[128] = {0};
    char run_id[80] = {0};
    char type[16] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "run_id", run_id, sizeof(run_id)) != ESP_OK ||
        !is_safe_snapshot_name(run_id)) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "run_id invalid or TF not mounted");
    }
    esp_err_t type_ret = httpd_query_key_value(query, "type", type, sizeof(type));
    if (type_ret != ESP_OK && type_ret != ESP_ERR_NOT_FOUND) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "type is too long or malformed");
    }
    if (type_ret == ESP_OK && strcmp(type, "summary") != 0) {
        file_download_reader_end();
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "type must be omitted or summary");
    }

    char path[512];
    if (strcmp(type, "summary") == 0) {
        snprintf(path, sizeof(path), "%s/%s_summary.json", DATASET_RUN_DIR, run_id);
        return send_file_response_internal(req, path, "application/json", true);
    }
    snprintf(path, sizeof(path), "%s/%s.jsonl", DATASET_RUN_DIR, run_id);
    return send_file_response_internal(req, path, "application/x-ndjson", true);
}

static esp_err_t dataset_run_results_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (!file_download_reader_try_begin()) {
        return send_file_download_unavailable(req);
    }
    if (queue_async_request(req, dataset_run_results_async_handler) != ESP_OK) {
        file_download_reader_end();
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static esp_err_t stream_async_handler(httpd_req_t *req)
{
    if (s_storage_quiescing || http_server_is_stopping()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "stream service is stopping");
    }
    /*
     * 每个 /stream 连接都会进入自己的 HTTP worker。这里不做摄像头采集，
     * 只按 stream_max_fps 复制最新 JPEG 并分块发送 MJPEG，因此两台设备同时访问时不会互相抢摄像头。
     */
    uint8_t *client_buf = alloc_psram_buffer(s_frame_capacity);
    if (!client_buf) {
        stream_stats_record_error();
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "no stream buffer");
    }

    esp_err_t ret = ESP_OK;
    uint32_t last_seq = 0;
    int64_t last_send_ms = 0;
    const int64_t min_interval_ms = s_stream_max_fps > 0 ? 1000 / (int64_t)s_stream_max_fps : 0;
    char header[288];
    int stream_sockfd = httpd_req_to_sockfd(req);

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    stream_stats_client_begin();
    mark_network_activity();

    while (true) {
        if (s_storage_quiescing || http_server_is_stopping()) {
            break;
        }
        power_state_t state = s_power_state;
        if (state == POWER_STATE_STANDBY || state == POWER_STATE_STOPPING || state == POWER_STATE_ERROR) {
            break;
        }

        frame_meta_t meta = {0};
        if (!copy_latest_frame(client_buf, s_frame_capacity, &meta) || meta.seq == last_seq) {
            camera_cmd_t cmd;
            if (xQueuePeek(s_camera_cmd_queue, &cmd, 0) == pdTRUE && cmd == CAMERA_CMD_STANDBY) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t wait_ms = min_interval_ms - (now_ms - last_send_ms);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
            now_ms = esp_timer_get_time() / 1000;
            if (s_storage_quiescing || http_server_is_stopping()) {
                break;
            }
        }

        int len = snprintf(header, sizeof(header), STREAM_PART, meta.size, meta.seq,
                           meta.capture_ms, meta.encode_ms, meta.vision.motion_score,
                           meta.vision.object_score, meta.vision.scene);
        if (len <= 0 || len >= (int)sizeof(header)) {
            ret = ESP_FAIL;
            break;
        }

        ret = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (ret != ESP_OK) {
            break;
        }
        ret = httpd_resp_send_chunk(req, header, len);
        if (ret != ESP_OK) {
            break;
        }
        ret = http_send_stream_frame_chunked(req, client_buf, meta.size);
        if (ret != ESP_OK) {
            break;
        }

        last_seq = meta.seq;
        last_send_ms = now_ms;
        mark_network_activity();
        stream_stats_record_frame(meta.size, now_ms);
    }

    if (ret == ESP_OK && !http_server_is_stopping()) {
        (void)httpd_resp_send_chunk(req, NULL, 0);
    } else if (ret != ESP_OK) {
        stream_stats_record_error();
        if (stream_sockfd >= 0) {
            (void)httpd_sess_trigger_close(req->handle, stream_sockfd);
        }
    }

    stream_stats_client_end();
    mark_network_activity();
    free(client_buf);
    return ESP_OK;
}

static esp_err_t queue_async_request(httpd_req_t *req, async_req_handler_t handler)
{
    if (!req || !handler || !s_async_worker_ready || !s_async_req_queue) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!http_async_activity_try_begin()) {
        return ESP_ERR_INVALID_STATE;
    }

    httpd_req_t *copy = NULL;
    esp_err_t ret = httpd_req_async_handler_begin(req, &copy);
    if (ret != ESP_OK) {
        http_async_activity_end();
        return ret;
    }

    async_req_t async_req = {
        .req = copy,
        .handler = handler,
    };

    if (xSemaphoreTake(s_async_worker_ready, 0) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        http_async_activity_end();
        return ESP_ERR_NOT_FOUND;
    }

    if (xQueueSend(s_async_req_queue, &async_req, pdMS_TO_TICKS(100)) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        xSemaphoreGive(s_async_worker_ready);
        http_async_activity_end();
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t stream_get_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (http_server_is_stopping() || s_storage_quiescing ||
        storage_transition_active()) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req,
                                  "stream is temporarily paused for storage maintenance; retry shortly");
    }
    if (export_mode_reject(req, "stream")) {
        return ESP_OK;
    }
    power_state_t state = s_power_state;
    if (state == POWER_STATE_STANDBY || state == POWER_STATE_STOPPING || state == POWER_STATE_ERROR) {
        camera_cmd_t wake = CAMERA_CMD_WAKE;
        s_wake_requests++;
        xQueueReset(s_camera_cmd_queue);
        if (xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(20)) != pdTRUE) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            return httpd_resp_sendstr(req, "camera wake queue full");
        }
        set_power_state(POWER_STATE_STARTING);
    }

    if (queue_async_request(req, stream_async_handler) != ESP_OK) {
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no stream worker available");
    }

    return ESP_OK;
}

static esp_err_t power_api_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
    if (s_storage_quiescing || storage_transition_active()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "camera power cannot be changed during storage maintenance",
            "wait for the page to reconnect and try again");
    }
    char query[64] = {0};
    char cmd[24] = {0};
    camera_cmd_t camera_cmd;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "cmd", cmd, sizeof(cmd));
    }

    power_state_t previous = s_power_state;
    bool coalesced = false;
    if (strcmp(cmd, "standby") == 0) {
        camera_cmd = CAMERA_CMD_STANDBY;
        s_standby_requests++;
        coalesced = previous == POWER_STATE_STANDBY || previous == POWER_STATE_STOPPING;
    } else if (strcmp(cmd, "wake") == 0 || strcmp(cmd, "on") == 0) {
        camera_cmd = CAMERA_CMD_WAKE;
        s_wake_requests++;
        coalesced = previous == POWER_STATE_RUNNING || previous == POWER_STATE_STARTING;
    } else {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "supported cmd: wake, standby");
    }

    if (!coalesced) {
        xQueueReset(s_camera_cmd_queue);
        if (xQueueSend(s_camera_cmd_queue, &camera_cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "camera command queue full");
        }
        set_power_state(camera_cmd == CAMERA_CMD_WAKE ?
                        POWER_STATE_STARTING : POWER_STATE_STOPPING);
    }

    char json[160];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"requested\":\"%s\",\"current\":\"%s\",\"coalesced\":%s}",
             camera_cmd == CAMERA_CMD_WAKE ? "wake" : "standby",
             power_state_name(previous), coalesced ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t time_sync_post_handler(httpd_req_t *req)
{
    record_http_request(req);
    char query[96] = {0};
    char epoch_text[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "epoch_ms", epoch_text, sizeof(epoch_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "epoch_ms required");
    }

    errno = 0;
    char *end = NULL;
    uint64_t epoch_ms = strtoull(epoch_text, &end, 10);
    if (!epoch_text[0] || epoch_text[0] == '-' || errno == ERANGE ||
        !end || *end != '\0' ||
        epoch_ms < APP_MIN_VALID_EPOCH_MS || epoch_ms > APP_MAX_VALID_EPOCH_MS) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "epoch_ms must be between 2020 and 2100");
    }

    struct timeval tv = {
        .tv_sec = (time_t)(epoch_ms / 1000ULL),
        .tv_usec = (suseconds_t)((epoch_ms % 1000ULL) * 1000ULL),
    };
    if (settimeofday(&tv, NULL) != 0) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "settimeofday failed");
    }
    s_time_source = TIME_SOURCE_BROWSER;
    ESP_LOGI(TAG, "system time synchronized by browser: epoch_ms=%" PRIu64, epoch_ms);

    char json[128];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"time_source\":\"browser\",\"epoch_ms\":%" PRIu64 "}",
             wall_clock_epoch_ms());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t netmode_api_handler(httpd_req_t *req)
{
    if (req->method != HTTP_POST) {
        record_http_request(req);
        return reject_non_post_method(req);
    }
    return config_get_handler(req);
}

static void async_worker_task(void *arg)
{
    (void)arg;
    while (true) {
        xSemaphoreGive(s_async_worker_ready);

        async_req_t async_req;
        if (xQueueReceive(s_async_req_queue, &async_req, portMAX_DELAY) == pdTRUE) {
            async_req.handler(async_req.req);
            if (httpd_req_async_handler_complete(async_req.req) != ESP_OK) {
                ESP_LOGW(TAG, "async request complete failed");
            }
            http_async_activity_end();
        }
    }
}

static void history_task(void *arg)
{
    int64_t last_mount_attempt_ms = -10000;
    while (true) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (CONFIG_APP_SD_ENABLE && s_storage_mount_allowed && !s_storage_quiescing &&
            !storage_usb_owned() &&
            !s_sd_mounted && now_ms - last_mount_attempt_ms >= 10000) {
            last_mount_attempt_ms = now_ms;
            esp_err_t ret = storage_mount();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "TF card unavailable, retrying in 10s: %s", esp_err_to_name(ret));
            }
        }

        history_item_t item;
        bool received = xQueueReceive(s_history_queue, &item,
                                      s_storage_quiescing ? 0 : pdMS_TO_TICKS(250)) == pdTRUE;
        if (received) {
            __atomic_store_n(&s_history_worker_busy, true, __ATOMIC_RELEASE);
            history_store_item(&item);
            free(item.jpeg);
            __atomic_store_n(&s_history_worker_busy, false, __ATOMIC_RELEASE);
        }
        if (s_storage_quiescing && !received) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static bool recording_segment_should_close(const recording_segment_t *segment, int64_t now_ms)
{
    if (!segment || !segment->writer) {
        return false;
    }
    uint32_t segment_ms = s_recording_segment_ms;
    return s_app_mode != APP_MODE_FIELD || !s_sd_mounted || !s_recording_enabled ||
           !storage_acceptance_ok() ||
           (segment_ms > 0 && now_ms - segment->start_ms >= (int64_t)segment_ms);
}

static void recording_close_pair(recording_segment_t *raw_segment,
                                 recording_segment_t *annotated_segment)
{
    /*
     * Close annotated first. If a reset/power event lands between the two closes,
     * startup cleanup can ignore an annotated-only fragment; a raw-only fragment
     * would otherwise show up as a customer-visible missing inference video.
     */
    recording_close_segment(annotated_segment);
    recording_close_segment(raw_segment);
}

static void recording_task(void *arg)
{
    (void)arg;
    static recording_segment_t raw_segment;
    static recording_segment_t annotated_segment;
    static frame_meta_t last_inference_meta = {0};
    static recognition_method_t last_inference_method = RECOGNITION_METHOD_OFF;
    static bool have_last_inference = false;

    while (true) {
        if (__atomic_exchange_n(&s_recording_reset_requested, false, __ATOMIC_ACQ_REL)) {
            recording_close_pair(&raw_segment, &annotated_segment);
            /* drain 旧 segment 残留的队列帧，防止新 segment 的 annotated 帧因队列满被丢弃 */
            recording_item_t drain;
            while (xQueueReceive(s_recording_queue, &drain, 0) == pdTRUE) {
                if (drain.kind == RECORDING_KIND_ANNOTATED) {
                    coco_espdl_free_jpeg(drain.jpeg);
                } else {
                    free(drain.jpeg);
                }
                if (drain.finalize_done) {
                    xSemaphoreGive(drain.finalize_done);
                }
            }
            cleanup_recording_temp_files();
            memset(&last_inference_meta, 0, sizeof(last_inference_meta));
            last_inference_method = RECOGNITION_METHOD_OFF;
            have_last_inference = false;
            s_segment_sequence = 0;
            s_current_segment_base[0] = '\0';
            ESP_LOGI(TAG, "recording_task: reset for new field session");
        }

        recording_item_t item = {0};
        if (xQueueReceive(s_recording_queue, &item, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (item.finalize) {
                recording_close_pair(&raw_segment, &annotated_segment);
                memset(&last_inference_meta, 0, sizeof(last_inference_meta));
                last_inference_method = RECOGNITION_METHOD_OFF;
                have_last_inference = false;
                s_segment_sequence = 0;
                s_current_segment_base[0] = '\0';
                if (item.finalize_done) {
                    xSemaphoreGive(item.finalize_done);
                }
                continue;
            }
            if (s_app_mode == APP_MODE_FIELD && storage_acceptance_ok() &&
                s_recording_enabled) {
                if (item.kind == RECORDING_KIND_ANNOTATED) {
                    last_inference_meta = item.meta;
                    last_inference_method = item.method;
                    have_last_inference = true;
                    if (item.jpeg) {
                        coco_espdl_free_jpeg(item.jpeg);
                    }
                } else {
                    int64_t item_ms = item.meta.timestamp_ms ?
                        item.meta.timestamp_ms : esp_timer_get_time() / 1000;
                    if (recording_segment_should_close(&raw_segment, item_ms) ||
                        recording_segment_should_close(&annotated_segment, item_ms)) {
                        recording_close_pair(&raw_segment, &annotated_segment);
                    }
                    recording_write_item(&raw_segment, &item);

                    frame_meta_t ann_meta = item.meta;
                    recognition_method_t ann_method = item.method;
                    if (ann_method == RECOGNITION_METHOD_OFF) {
                        ann_method = recognition_method_or_fallback(s_recognition_method);
                    }
                    if (have_last_inference && last_inference_method == ann_method) {
                        ann_meta.vision = last_inference_meta.vision;
                    }

                    uint8_t *annotated_jpeg = NULL;
                    size_t annotated_size = 0;
                    esp_err_t ann_err = generate_annotated_jpeg_from_vision(
                        item.jpeg, item.jpeg_size, ann_method, &ann_meta.vision,
                        item.meta.width, item.meta.height,
                        &annotated_jpeg, &annotated_size);
                    recording_item_t ann_item = {
                        .meta = ann_meta,
                        .jpeg = annotated_jpeg ? annotated_jpeg : item.jpeg,
                        .jpeg_size = annotated_jpeg ? (uint32_t)annotated_size : item.jpeg_size,
                        .kind = RECORDING_KIND_ANNOTATED,
                        .method = ann_method,
                    };
                    if (ann_err != ESP_OK || !ann_item.jpeg || ann_item.jpeg_size == 0) {
                        ann_item.jpeg = item.jpeg;
                        ann_item.jpeg_size = item.jpeg_size;
                    }
                    recording_write_item(&annotated_segment, &ann_item);
                    if (annotated_jpeg) {
                        coco_espdl_free_jpeg(annotated_jpeg);
                    }
                    if (raw_segment.writer && annotated_segment.writer &&
                        raw_segment.frames != annotated_segment.frames) {
                        ESP_LOGW(TAG,
                                 "recording_task: frame count mismatch raw=%" PRIu32
                                 " ann=%" PRIu32 " base=%s",
                                 raw_segment.frames, annotated_segment.frames,
                                 s_current_segment_base);
                    }
                }
            }
            if (item.kind != RECORDING_KIND_ANNOTATED) {
                free(item.jpeg);
            }
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        if (recording_segment_should_close(&raw_segment, now_ms) ||
            recording_segment_should_close(&annotated_segment, now_ms)) {
            recording_close_pair(&raw_segment, &annotated_segment);
        }
    }
}

static bool enrichment_should_cancel(void *arg)
{
    (void)arg;
    bool manual_request = recording_enrichment_has_request();
    recording_enrichment_status_t enrichment = {0};
    recording_enrichment_get_status(&enrichment);
    bool enrichment_active = manual_request || enrichment.running;
    if (!CONFIG_APP_ENRICHMENT_ENABLE || s_storage_quiescing ||
        storage_transition_active() || storage_mode_request_pending() ||
        s_app_mode != APP_MODE_SERVER ||
        !s_network_active || s_network_shutdown_for_idle ||
        !s_sd_mounted || s_power_state != POWER_STATE_STANDBY ||
        stream_stats_client_count() > 0 ||
        __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) > 0 ||
        (s_history_queue && uxQueueMessagesWaiting(s_history_queue) > 0) ||
        (s_recording_queue && uxQueueMessagesWaiting(s_recording_queue) > 0)) {
        return true;
    }
    if (!enrichment_active &&
        (inference_worker_busy() ||
         (s_inference_queue && uxQueueMessagesWaiting(s_inference_queue) > 0))) {
        return true;
    }

    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    if (dataset.queued || dataset.running) {
        return true;
    }
    return !enrichment_active;
}

static void enrichment_task(void *arg)
{
    (void)arg;
    while (true) {
        if (enrichment_should_cancel(NULL)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (xSemaphoreTake(s_storage_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        esp_err_t ret = ESP_ERR_INVALID_STATE;
        if (!enrichment_should_cancel(NULL)) {
            ret = recording_enrichment_process_next(
                RECORDING_DIR, CONFIG_APP_ENRICHMENT_INITIAL_STRIDE,
                s_box_min_score, s_jpeg_quality,
                enrichment_should_cancel, NULL);
            if (ret == ESP_OK) {
                recording_enrichment_status_t status = {0};
                recording_enrichment_get_status(&status);
                if (status.output_name[0]) {
                    bool index_failed = false;
                    remove_recording_index_rows(status.output_name, &index_failed);
                    if (index_failed) {
                        ESP_LOGW(TAG, "could not remove stale annotated index for %s",
                                 status.output_name);
                    }
                }
                reconcile_recording_indexes();
                update_sd_info();
            }
        }
        xSemaphoreGive(s_storage_lock);

        uint32_t delay_ms = ret == ESP_OK ? 1000U :
                            (ret == ESP_ERR_NOT_FOUND ? 10000U : 2000U);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void __attribute__((unused)) resegment_task(void *arg)
{
    (void)arg;
    while (true) {
        recording_resegment_status_t status = {0};
        resegment_status_copy(&status);
        if (!status.requested || status.target_ms == 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (resegment_should_pause()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (xSemaphoreTake(s_storage_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (!resegment_should_pause()) {
            resegment_recordings_to_target(status.target_ms);
        }
        xSemaphoreGive(s_storage_lock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t usb_mode_start_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
#if !CONFIG_APP_USB_MSC_ENABLE
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "USB MSC disabled");
#else
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "USB") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=USB required");
    }
    if (s_app_mode == APP_MODE_USB_EXPORT || s_storage_quiescing ||
        storage_transition_active()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "USB export already active or pending");
    }
    if (s_app_mode != APP_MODE_SERVER) {
        return send_customer_action_json(
            req, "409 Conflict", "wrong_mode",
            "USB export can start only from Web server mode",
            "leave Ethernet export or field mode, return to server mode, then retry");
    }
    if (!storage_transition_try_acquire(STORAGE_TRANSITION_USB_EXPORT)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage ownership transition was requested first",
            "wait for the current operation to finish and retry");
    }
    storage_request_set(&s_usb_export_requested);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":true,\"mode\":\"usb_export_pending\","
        "\"note\":\"camera and recording will pause; Web stays online; computer should enumerate P4_BUOY\"}");
#endif
}

static esp_err_t usb_restore_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
#if !CONFIG_APP_USB_MSC_ENABLE
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "USB MSC disabled");
#else
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "RESTORE") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=RESTORE required");
    }
    if (s_app_mode != APP_MODE_USB_EXPORT) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "USB export is not active");
    }
    if (s_storage_quiescing || storage_transition_active()) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "USB restore is already running or another storage operation is pending",
            "wait for the current operation to finish and refresh status");
    }
    if (!storage_transition_try_acquire(STORAGE_TRANSITION_USB_RESTORE)) {
        return send_customer_action_json(
            req, "409 Conflict", "storage_busy",
            "another storage ownership transition was requested first",
            "wait for the current operation to finish and refresh status");
    }
    __atomic_store_n(&s_usb_restore_auto_blocked, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_usb_restore_manual_requested, true, __ATOMIC_RELEASE);
    storage_request_set(&s_usb_restore_requested);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":true,\"mode\":\"usb_restore_pending\","
        "\"note\":\"USB MSC will be soft-disconnected; TF will be remounted and write-verified; Web stays online\"}");
#endif
}

static esp_err_t system_reboot_handler(httpd_req_t *req)
{
    record_http_request(req);
    if (req->method != HTTP_POST) {
        return reject_non_post_method(req);
    }
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "REBOOT") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=REBOOT required");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    http_send_cstr_chunked(req, "{\"ok\":true,\"rebooting\":true}");
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
    return ESP_OK;
}

static esp_err_t storage_release_usb_sdmmc_card(void);

static esp_err_t storage_init_usb_sdmmc_card(void)
{
    if (s_usb_sd_card_initialized && s_usb_sd_card &&
        s_usb_sdmmc_slot_initialized && s_sd_pwr_ctrl) {
        return ESP_OK;
    }
    if (s_sd_mounted) {
        ESP_LOGE(TAG, "USB TF init rejected while application filesystem is mounted");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_usb_sd_card || s_usb_sdmmc_slot_initialized ||
        s_usb_sdmmc_host_initialized || s_sd_pwr_ctrl) {
        esp_err_t cleanup_ret = storage_release_usb_sdmmc_card();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "USB TF init blocked by pending teardown: %s",
                     esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
    }

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_APP_SD_LDO_IO_ID,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_pwr_ctrl);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "USB TF LDO init failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_GOTO_ON_ERROR(storage_reset_card_power(), fail, TAG, "USB TF power reset failed");

    /* USB must not silently select a faster/wider bus than the application
     * has just proved writable. Some cards enumerate and read successfully at
     * 4-bit/40 MHz but time out on their first real write. Besides corrupting
     * the MSC transfer, that stalls the shared controller used by Hosted on
     * Slot 1 and causes the whole device to restart. */
    int usb_width = 1;
    int usb_freq_khz = CONFIG_APP_USB_MSC_SD_FREQ_KHZ < 10000 ?
                       CONFIG_APP_USB_MSC_SD_FREQ_KHZ : 10000;
    if (strcmp(s_sd_mount_mode, "sdmmc_4bit") == 0) {
        usb_width = 4;
        usb_freq_khz = CONFIG_APP_USB_MSC_SD_FREQ_KHZ < CONFIG_APP_SD_MAX_FREQ_KHZ ?
                       CONFIG_APP_USB_MSC_SD_FREQ_KHZ : CONFIG_APP_SD_MAX_FREQ_KHZ;
    } else if (strncmp(s_sd_mount_mode, "sdspi", 5) == 0 && usb_freq_khz > 5000) {
        usb_freq_khz = 5000;
    }
    ESP_LOGI(TAG, "USB TF profile selected from write-verified %s: SDMMC %d-bit at %d kHz",
             s_sd_mount_mode, usb_width, usb_freq_khz);

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = usb_freq_khz;
    host.unaligned_multi_block_rw_max_chunk_size = 8;
    host.pwr_ctrl_handle = s_sd_pwr_ctrl;
    bool host_owned_by_hosted = false;
#if CONFIG_ESP_HOSTED_ENABLED && CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    if (s_hosted_sdmmc_host_active) {
        host.init = storage_sdmmc_host_init_already_done;
        host.deinit = storage_sdmmc_host_deinit_keep_hosted;
        host_owned_by_hosted = true;
    }
#endif

    ret = host.init();
    if (ret == ESP_OK) {
        s_usb_sdmmc_host_initialized = !host_owned_by_hosted;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SDMMC host already initialized before USB handoff");
        ret = ESP_OK;
    }
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "USB SDMMC host init failed");

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = usb_width;
    slot_config.clk = CONFIG_APP_SD_PIN_CLK;
    slot_config.cmd = CONFIG_APP_SD_PIN_CMD;
    slot_config.d0 = CONFIG_APP_SD_PIN_D0;
    if (slot_config.width == 4) {
        slot_config.d1 = CONFIG_APP_SD_PIN_D1;
        slot_config.d2 = CONFIG_APP_SD_PIN_D2;
        slot_config.d3 = CONFIG_APP_SD_PIN_D3;
    }
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ret = sdmmc_host_init_slot(host.slot, &slot_config);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "USB SDMMC slot init failed");
    s_usb_sdmmc_slot_initialized = true;

    s_usb_sd_card = calloc(1, sizeof(*s_usb_sd_card));
    ESP_GOTO_ON_FALSE(s_usb_sd_card, ESP_ERR_NO_MEM, fail, TAG,
                      "USB SDMMC card allocation failed");
    ret = sdmmc_card_init(&host, s_usb_sd_card);
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "USB SDMMC card init failed");

    s_usb_sd_card_initialized = true;
    ESP_LOGI(TAG, "USB TF ready: SDMMC %d-bit at %d kHz", usb_width, usb_freq_khz);
    sdmmc_card_print_info(stdout, s_usb_sd_card);
    return ESP_OK;

fail:
    {
        esp_err_t init_ret = ret;
        esp_err_t cleanup_ret = storage_release_usb_sdmmc_card();
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "USB TF init failed (%s) and ownership cleanup remains pending (%s)",
                     esp_err_to_name(init_ret), esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
        return init_ret;
    }
}

static esp_err_t storage_release_usb_sdmmc_card(void)
{
    /* Once teardown starts the card must never be re-exposed, even when a
     * later Slot/Host/LDO release step needs a Web-triggered retry. */
    s_usb_sd_card_initialized = false;
    if (s_usb_sdmmc_slot_initialized) {
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        host.slot = SDMMC_HOST_SLOT_0;
        esp_err_t slot_ret;
        if (!s_usb_sdmmc_host_initialized) {
            /* USB reused Hosted's global controller. Release only Slot 0;
             * tearing down the whole host here would also kill the C6 link. */
            slot_ret = sdmmc_host_deinit_slot(host.slot);
        } else if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
            slot_ret = host.deinit_p(host.slot);
        } else {
            slot_ret = host.deinit();
        }
        if (slot_ret == ESP_ERR_INVALID_STATE || slot_ret == ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "USB SDMMC slot was already released: %s",
                     esp_err_to_name(slot_ret));
            slot_ret = ESP_OK;
        }
        if (slot_ret != ESP_OK) {
            ESP_LOGE(TAG, "USB SDMMC slot release failed: %s",
                     esp_err_to_name(slot_ret));
            return slot_ret;
        }
        s_usb_sdmmc_slot_initialized = false;
        s_usb_sdmmc_host_initialized = false;
    } else if (s_usb_sdmmc_host_initialized) {
        esp_err_t host_ret = sdmmc_host_deinit();
        if (host_ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "USB SDMMC host was already released");
            host_ret = ESP_OK;
        }
        if (host_ret != ESP_OK) {
            ESP_LOGE(TAG, "USB SDMMC host release failed: %s",
                     esp_err_to_name(host_ret));
            return host_ret;
        }
        s_usb_sdmmc_host_initialized = false;
    }
    if (s_sd_pwr_ctrl) {
        esp_err_t ldo_ret = sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
        if (ldo_ret != ESP_OK) {
            ESP_LOGE(TAG, "USB SD LDO release failed: %s", esp_err_to_name(ldo_ret));
            return ldo_ret;
        }
        s_sd_pwr_ctrl = NULL;
    }
    free(s_usb_sd_card);
    s_usb_sd_card = NULL;
    return ESP_OK;
}

static esp_err_t recording_finalize_sync(TickType_t timeout)
{
    if (!s_recording_queue || !s_recording_finalize_done) {
        return ESP_ERR_INVALID_STATE;
    }
    while (xSemaphoreTake(s_recording_finalize_done, 0) == pdTRUE) {
    }
    recording_item_t item = {
        .finalize = true,
        .finalize_done = s_recording_finalize_done,
    };
    if (xQueueSend(s_recording_queue, &item, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return xSemaphoreTake(s_recording_finalize_done, timeout) == pdTRUE ?
           ESP_OK : ESP_ERR_TIMEOUT;
}

static void start_async_workers(void)
{
    s_async_worker_ready = xSemaphoreCreateCounting(CONFIG_APP_MAX_STREAM_CLIENTS, 0);
    ESP_ERROR_CHECK(s_async_worker_ready ? ESP_OK : ESP_ERR_NO_MEM);

    s_async_req_queue = xQueueCreate(CONFIG_APP_MAX_STREAM_CLIENTS, sizeof(async_req_t));
    ESP_ERROR_CHECK(s_async_req_queue ? ESP_OK : ESP_ERR_NO_MEM);

    for (int i = 0; i < CONFIG_APP_MAX_STREAM_CLIENTS; i++) {
        BaseType_t ok = xTaskCreate(async_worker_task, "stream_worker",
                                    CONFIG_APP_ASYNC_WORKER_TASK_STACK_SIZE,
                                    NULL, 5, &s_async_worker_handles[i]);
        ESP_ERROR_CHECK(ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);
    }
}

static esp_err_t wait_for_http_quiescence(uint32_t timeout_ms)
{
    int64_t deadline_ms = esp_timer_get_time() / 1000 + (int64_t)timeout_ms;
    for (;;) {
        dataset_run_status_t dataset = {0};
        recording_enrichment_status_t enrichment = {0};
        dataset_status_copy(&dataset);
        recording_enrichment_get_status(&enrichment);
        bool enrichment_pending = recording_enrichment_has_request();
        uint32_t async_active = http_async_activity_count();
        uint32_t stream_clients = stream_stats_client_count();
        uint32_t downloads = __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE);
        uint32_t validation_jobs = __atomic_load_n(&s_validation_active_jobs, __ATOMIC_ACQUIRE);
        UBaseType_t history_queued = s_history_queue ? uxQueueMessagesWaiting(s_history_queue) : 0;
        UBaseType_t recording_queued = s_recording_queue ? uxQueueMessagesWaiting(s_recording_queue) : 0;
        bool history_busy = __atomic_load_n(&s_history_worker_busy, __ATOMIC_ACQUIRE);
        if (async_active == 0 && stream_clients == 0 && downloads == 0 && validation_jobs == 0 &&
            !dataset.queued && !dataset.running && !enrichment_pending &&
            !enrichment.running &&
            history_queued == 0 && !history_busy && recording_queued == 0) {
            return ESP_OK;
        }
        if (esp_timer_get_time() / 1000 >= deadline_ms) {
            ESP_LOGE(TAG,
                     "HTTP quiesce timeout: async=%" PRIu32 ", stream=%" PRIu32
                     ", downloads=%" PRIu32 ", validation=%" PRIu32
                     ", dataset=%u/%u, enrichment=%u/%u, history=%u/%u, recording_q=%u, async_q=%u",
                     async_active, stream_clients, downloads, validation_jobs,
                     (unsigned)dataset.queued, (unsigned)dataset.running,
                     (unsigned)enrichment_pending,
                     (unsigned)enrichment.running,
                     (unsigned)history_busy, (unsigned)history_queued,
                     (unsigned)recording_queued,
                     (unsigned)(s_async_req_queue ? uxQueueMessagesWaiting(s_async_req_queue) : 0));
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(HTTP_STOP_QUIESCE_POLL_MS));
    }
}

static esp_err_t stop_webserver(uint32_t timeout_ms)
{
    http_server_set_stopping(true);
    esp_err_t ret = wait_for_http_quiescence(timeout_ms);
    if (ret != ESP_OK) {
        http_server_set_stopping(false);
        return ret;
    }

    httpd_handle_t server = s_server;
    if (server) {
        ret = httpd_stop(server);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "HTTP server stop failed: %s", esp_err_to_name(ret));
            http_server_set_stopping(false);
            return ret;
        }
        if (s_server == server) {
            s_server = NULL;
        }
        ESP_LOGI(TAG, "HTTP server stopped");
    }

    if (s_mdns_started) {
#if CONFIG_APP_MDNS_ENABLE
        mdns_free();
#endif
        s_mdns_started = false;
        ESP_LOGI(TAG, "mDNS stopped");
    }
    s_http_server_ready = false;
    return ESP_OK;
}

static esp_err_t mdns_start_runtime(void)
{
#if CONFIG_APP_MDNS_ENABLE
    if (s_mdns_started) {
        return ESP_OK;
    }
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mdns_hostname_set(CONFIG_APP_HOSTNAME);
    if (ret == ESP_OK) {
        ret = mdns_instance_name_set("ESP32-P4 Buoy Vision");
    }
    if (ret == ESP_OK) {
        mdns_txt_item_t txt[] = {
            {"board", CONFIG_IDF_TARGET},
            {"path", "/"},
        };
        ret = mdns_service_add("ESP32-P4 Buoy Vision", "_http", "_tcp", 80,
                               txt, sizeof(txt) / sizeof(txt[0]));
    }
    if (ret != ESP_OK) {
        mdns_free();
        ESP_LOGW(TAG, "mDNS advertise failed: %s", esp_err_to_name(ret));
        return ret;
    }
    s_mdns_started = true;
    ESP_LOGI(TAG, "mDNS URL: http://%s.local/", CONFIG_APP_HOSTNAME);
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

static esp_err_t start_webserver(void)
{
    if (s_server && s_http_server_ready) {
        ESP_LOGI(TAG, "HTTP server already running");
        mdns_start_runtime();
        http_server_set_stopping(false);
        return ESP_OK;
    }
    if (s_server) {
        httpd_handle_t partial_server = s_server;
        esp_err_t cleanup_ret = httpd_stop(partial_server);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "incomplete HTTP server cleanup retry failed: %s",
                     esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
        if (s_server == partial_server) {
            s_server = NULL;
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = CONFIG_APP_MAX_STREAM_CLIENTS + 6;
    int max_http_sockets = CONFIG_LWIP_MAX_SOCKETS - 3;
    if (config.max_open_sockets > max_http_sockets) {
        config.max_open_sockets = max_http_sockets;
    }
    config.lru_purge_enable = true;
    config.backlog_conn = CONFIG_APP_MAX_STREAM_CLIENTS + 2;
    config.max_uri_handlers = 80;
    config.send_wait_timeout = HTTP_FILE_SEND_WAIT_TIMEOUT_SEC;
    config.uri_match_fn = httpd_uri_match_wildcard;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        s_server = NULL;
        s_http_server_ready = false;
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
    };
    const httpd_uri_t healthz = {
        .uri = "/healthz",
        .method = HTTP_GET,
        .handler = healthz_get_handler,
    };
    const httpd_uri_t validate = {
        .uri = "/validate",
        .method = HTTP_GET,
        .handler = validate_get_handler,
    };
    const httpd_uri_t validate_demo_01 = {
        .uri = "/validate/demo_01.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_demo_02 = {
        .uri = "/validate/demo_02.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_demo_03 = {
        .uri = "/validate/demo_03.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_demo_04 = {
        .uri = "/validate/demo_04.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_tiny_01 = {
        .uri = "/validate/tiny_01.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_tiny_02 = {
        .uri = "/validate/tiny_02.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_tiny_03 = {
        .uri = "/validate/tiny_03.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_tiny_04 = {
        .uri = "/validate/tiny_04.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_fish31_01 = {
        .uri = "/validate/fish31_01.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_fish31_02 = {
        .uri = "/validate/fish31_02.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_fish31_03 = {
        .uri = "/validate/fish31_03.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_fish31_04 = {
        .uri = "/validate/fish31_04.jpg",
        .method = HTTP_GET,
        .handler = validate_demo_jpg_handler,
    };
    const httpd_uri_t validate_run_get = {
        .uri = "/api/validate/run",
        .method = HTTP_GET,
        .handler = validation_run_get_rejected_handler,
    };
    const httpd_uri_t validate_run_post = {
        .uri = "/api/validate/run",
        .method = HTTP_POST,
        .handler = validation_run_post_handler,
    };
    const httpd_uri_t validate_status = {
        .uri = "/api/validate/status",
        .method = HTTP_GET,
        .handler = validation_status_get_handler,
    };
    const httpd_uri_t validate_overlay = {
        .uri = "/api/validate/overlay.svg",
        .method = HTTP_GET,
        .handler = validation_overlay_get_handler,
    };
    const httpd_uri_t search = {
        .uri = "/api/search",
        .method = HTTP_GET,
        .handler = search_get_handler,
    };
    const httpd_uri_t frame = {
        .uri = "/api/frame.jpg",
        .method = HTTP_GET,
        .handler = frame_get_handler,
    };
    const httpd_uri_t vision = {
        .uri = "/api/vision",
        .method = HTTP_GET,
        .handler = vision_api_handler,
    };
    const httpd_uri_t vision_post = {
        .uri = "/api/vision",
        .method = HTTP_POST,
        .handler = vision_api_handler,
    };
    const httpd_uri_t recognition = {
        .uri = "/api/recognition",
        .method = HTTP_GET,
        .handler = recognition_api_handler,
    };
    const httpd_uri_t recognition_post = {
        .uri = "/api/recognition",
        .method = HTTP_POST,
        .handler = recognition_api_handler,
    };
    const httpd_uri_t config_api = {
        .uri = "/api/config",
        .method = HTTP_GET,
        .handler = config_get_handler,
    };
    const httpd_uri_t config_api_post = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = config_get_handler,
    };
    const httpd_uri_t history = {
        .uri = "/api/history",
        .method = HTTP_GET,
        .handler = history_get_handler,
    };
    const httpd_uri_t timeline = {
        .uri = "/api/timeline",
        .method = HTTP_GET,
        .handler = timeline_get_handler,
    };
    const httpd_uri_t timeline_delete = {
        .uri = "/api/timeline",
        .method = HTTP_DELETE,
        .handler = timeline_delete_handler,
    };
    const httpd_uri_t history_file = {
        .uri = "/api/history.jsonl",
        .method = HTTP_GET,
        .handler = history_file_get_handler,
    };
    const httpd_uri_t recordings = {
        .uri = "/api/recordings",
        .method = HTTP_GET,
        .handler = recordings_get_handler,
    };
    const httpd_uri_t recordings_delete = {
        .uri = "/api/recordings",
        .method = HTTP_DELETE,
        .handler = recordings_delete_handler,
    };
    const httpd_uri_t cleanup_recordings = {
        .uri = "/api/recordings/cleanup",
        .method = HTTP_POST,
        .handler = cleanup_recordings_post_handler,
    };
    const httpd_uri_t cleanup_recordings_get = {
        .uri = "/api/recordings/cleanup",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t recording_frame_svg = {
        .uri = RECORDING_FRAME_SVG_URI,
        .method = HTTP_GET,
        .handler = recording_frame_svg_get_handler,
    };
    const httpd_uri_t recording_manifest = {
        .uri = RECORDING_MANIFEST_URI,
        .method = HTTP_GET,
        .handler = recording_manifest_get_handler,
    };
    const httpd_uri_t recording_delete = {
        .uri = "/api/recording",
        .method = HTTP_DELETE,
        .handler = recording_delete_handler,
    };
    const httpd_uri_t recording_delete_get = {
        .uri = "/api/recording",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "DELETE",
    };
    const httpd_uri_t recording_enrich = {
        .uri = "/api/recording/enrich",
        .method = HTTP_POST,
        .handler = recording_enrich_handler,
    };
    const httpd_uri_t recording_enrich_get = {
        .uri = "/api/recording/enrich",
        .method = HTTP_GET,
        .handler = recording_enrich_handler,
    };
    const httpd_uri_t storage_records_delete = {
        .uri = "/api/storage/records",
        .method = HTTP_DELETE,
        .handler = storage_records_delete_handler,
    };
    const httpd_uri_t storage_records_delete_get = {
        .uri = "/api/storage/records",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "DELETE",
    };
    const httpd_uri_t storage_files = {
        .uri = "/api/storage/files",
        .method = HTTP_GET,
        .handler = storage_files_get_handler,
    };
    const httpd_uri_t storage_format = {
        .uri = "/api/storage/format",
        .method = HTTP_POST,
        .handler = storage_format_handler,
    };
    const httpd_uri_t storage_format_get = {
        .uri = "/api/storage/format",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t storage_remount = {
        .uri = "/api/storage/remount",
        .method = HTTP_POST,
        .handler = storage_remount_handler,
    };
    const httpd_uri_t storage_remount_get = {
        .uri = "/api/storage/remount",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t storage_retry = {
        .uri = "/api/storage/retry",
        .method = HTTP_POST,
        .handler = storage_retry_handler,
    };
    const httpd_uri_t storage_retry_get = {
        .uri = "/api/storage/retry",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t field_mode_start = {
        .uri = "/api/mode/field",
        .method = HTTP_POST,
        .handler = field_mode_start_handler,
    };
    const httpd_uri_t field_mode_start_get = {
        .uri = "/api/mode/field",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t export_mode_start = {
        .uri = "/api/mode/export",
        .method = HTTP_POST,
        .handler = export_mode_start_handler,
    };
    const httpd_uri_t export_mode_start_get = {
        .uri = "/api/mode/export",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t usb_mode_start = {
        .uri = "/api/mode/usb",
        .method = HTTP_POST,
        .handler = usb_mode_start_handler,
    };
    const httpd_uri_t usb_mode_start_get = {
        .uri = "/api/mode/usb",
        .method = HTTP_GET,
        .handler = usb_mode_start_handler,
    };
    const httpd_uri_t usb_restore = {
        .uri = "/api/mode/usb/restore",
        .method = HTTP_POST,
        .handler = usb_restore_handler,
    };
    const httpd_uri_t usb_restore_get = {
        .uri = "/api/mode/usb/restore",
        .method = HTTP_GET,
        .handler = usb_restore_handler,
    };
    const httpd_uri_t datasets = {
        .uri = "/api/datasets",
        .method = HTTP_GET,
        .handler = datasets_get_handler,
    };
    const httpd_uri_t dataset_file = {
        .uri = "/api/dataset/file",
        .method = HTTP_PUT,
        .handler = dataset_file_put_handler,
    };
    const httpd_uri_t dataset_file_get = {
        .uri = "/api/dataset/file",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "PUT",
    };
    const httpd_uri_t dataset_run_start = {
        .uri = "/api/dataset/run/start",
        .method = HTTP_POST,
        .handler = dataset_run_start_handler,
    };
    const httpd_uri_t dataset_run_start_get = {
        .uri = "/api/dataset/run/start",
        .method = HTTP_GET,
        .handler = dataset_run_start_handler,
    };
    const httpd_uri_t dataset_run_status = {
        .uri = "/api/dataset/run/status",
        .method = HTTP_GET,
        .handler = dataset_run_status_handler,
    };
    const httpd_uri_t dataset_run_results = {
        .uri = "/api/dataset/run/results",
        .method = HTTP_GET,
        .handler = dataset_run_results_handler,
    };
    const httpd_uri_t dataset_frame_svg = {
        .uri = DATASET_FRAME_SVG_URI,
        .method = HTTP_GET,
        .handler = dataset_frame_svg_get_handler,
    };
    const httpd_uri_t power = {
        .uri = "/api/power",
        .method = HTTP_GET,
        .handler = power_api_handler,
    };
    const httpd_uri_t power_post = {
        .uri = "/api/power",
        .method = HTTP_POST,
        .handler = power_api_handler,
    };
    const httpd_uri_t time_sync = {
        .uri = "/api/time/sync",
        .method = HTTP_POST,
        .handler = time_sync_post_handler,
    };
    const httpd_uri_t time_sync_get = {
        .uri = "/api/time/sync",
        .method = HTTP_GET,
        .handler = reject_get_for_mutation_handler,
        .user_ctx = "POST",
    };
    const httpd_uri_t system_reboot = {
        .uri = "/api/system/reboot",
        .method = HTTP_POST,
        .handler = system_reboot_handler,
    };
    const httpd_uri_t system_reboot_get = {
        .uri = "/api/system/reboot",
        .method = HTTP_GET,
        .handler = system_reboot_handler,
    };
    const httpd_uri_t netmode = {
        .uri = "/api/netmode",
        .method = HTTP_GET,
        .handler = netmode_api_handler,
    };
    const httpd_uri_t netmode_post = {
        .uri = "/api/netmode",
        .method = HTTP_POST,
        .handler = netmode_api_handler,
    };
    const httpd_uri_t stream = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_get_handler,
    };
    const httpd_uri_t snapshot = {
        .uri = SNAPSHOT_URI_PREFIX "*",
        .method = HTTP_GET,
        .handler = snapshot_get_handler,
    };
    const httpd_uri_t recording = {
        .uri = RECORDING_URI_PREFIX "*",
        .method = HTTP_GET,
        .handler = recording_get_handler,
    };
    const httpd_uri_t recording_meta = {
        .uri = RECORDING_META_URI_PREFIX "*",
        .method = HTTP_GET,
        .handler = recording_meta_get_handler,
    };
    const httpd_uri_t recording_annotated = {
        .uri = RECORDING_ANNOTATED_URI_PREFIX "*",
        .method = HTTP_GET,
        .handler = recording_annotated_get_handler,
    };

    #define REGISTER_HTTP_URI(uri_config) do {                                      \
        ret = httpd_register_uri_handler(s_server, &(uri_config));                   \
        if (ret != ESP_OK) {                                                         \
            ESP_LOGE(TAG, "HTTP URI registration failed for %s: %s",              \
                     (uri_config).uri, esp_err_to_name(ret));                        \
            goto fail;                                                               \
        }                                                                            \
    } while (0)

    REGISTER_HTTP_URI(root);
    REGISTER_HTTP_URI(validate);
    REGISTER_HTTP_URI(validate_demo_01);
    REGISTER_HTTP_URI(validate_demo_02);
    REGISTER_HTTP_URI(validate_demo_03);
    REGISTER_HTTP_URI(validate_demo_04);
    REGISTER_HTTP_URI(validate_tiny_01);
    REGISTER_HTTP_URI(validate_tiny_02);
    REGISTER_HTTP_URI(validate_tiny_03);
    REGISTER_HTTP_URI(validate_tiny_04);
    REGISTER_HTTP_URI(validate_fish31_01);
    REGISTER_HTTP_URI(validate_fish31_02);
    REGISTER_HTTP_URI(validate_fish31_03);
    REGISTER_HTTP_URI(validate_fish31_04);
    REGISTER_HTTP_URI(validate_run_get);
    REGISTER_HTTP_URI(validate_run_post);
    REGISTER_HTTP_URI(validate_status);
    REGISTER_HTTP_URI(validate_overlay);
    REGISTER_HTTP_URI(search);
    REGISTER_HTTP_URI(healthz);
    REGISTER_HTTP_URI(status);
    REGISTER_HTTP_URI(frame);
    REGISTER_HTTP_URI(vision);
    REGISTER_HTTP_URI(vision_post);
    REGISTER_HTTP_URI(recognition);
    REGISTER_HTTP_URI(recognition_post);
    REGISTER_HTTP_URI(config_api);
    REGISTER_HTTP_URI(config_api_post);
    REGISTER_HTTP_URI(history);
    REGISTER_HTTP_URI(timeline);
    REGISTER_HTTP_URI(timeline_delete);
    REGISTER_HTTP_URI(history_file);
    REGISTER_HTTP_URI(recordings);
    REGISTER_HTTP_URI(recordings_delete);
    REGISTER_HTTP_URI(cleanup_recordings);
    REGISTER_HTTP_URI(cleanup_recordings_get);
    REGISTER_HTTP_URI(recording_frame_svg);
    REGISTER_HTTP_URI(recording_manifest);
    REGISTER_HTTP_URI(recording_delete);
    REGISTER_HTTP_URI(recording_delete_get);
    REGISTER_HTTP_URI(recording_enrich);
    REGISTER_HTTP_URI(recording_enrich_get);
    REGISTER_HTTP_URI(storage_records_delete);
    REGISTER_HTTP_URI(storage_records_delete_get);
    REGISTER_HTTP_URI(storage_files);
    REGISTER_HTTP_URI(storage_format);
    REGISTER_HTTP_URI(storage_format_get);
    REGISTER_HTTP_URI(storage_remount);
    REGISTER_HTTP_URI(storage_remount_get);
    REGISTER_HTTP_URI(storage_retry);
    REGISTER_HTTP_URI(storage_retry_get);
    REGISTER_HTTP_URI(field_mode_start);
    REGISTER_HTTP_URI(field_mode_start_get);
    REGISTER_HTTP_URI(export_mode_start);
    REGISTER_HTTP_URI(export_mode_start_get);
    REGISTER_HTTP_URI(usb_mode_start);
    REGISTER_HTTP_URI(usb_mode_start_get);
    REGISTER_HTTP_URI(usb_restore);
    REGISTER_HTTP_URI(usb_restore_get);
    REGISTER_HTTP_URI(datasets);
    REGISTER_HTTP_URI(dataset_file);
    REGISTER_HTTP_URI(dataset_file_get);
    REGISTER_HTTP_URI(dataset_run_start);
    REGISTER_HTTP_URI(dataset_run_start_get);
    REGISTER_HTTP_URI(dataset_run_status);
    REGISTER_HTTP_URI(dataset_run_results);
    REGISTER_HTTP_URI(dataset_frame_svg);
    REGISTER_HTTP_URI(power);
    REGISTER_HTTP_URI(power_post);
    REGISTER_HTTP_URI(time_sync);
    REGISTER_HTTP_URI(time_sync_get);
    REGISTER_HTTP_URI(system_reboot);
    REGISTER_HTTP_URI(system_reboot_get);
    REGISTER_HTTP_URI(netmode);
    REGISTER_HTTP_URI(netmode_post);
    REGISTER_HTTP_URI(stream);
    REGISTER_HTTP_URI(snapshot);
    REGISTER_HTTP_URI(recording_annotated);
    REGISTER_HTTP_URI(recording);
    REGISTER_HTTP_URI(recording_meta);
    #undef REGISTER_HTTP_URI

    http_server_set_stopping(false);
    s_http_server_ready = true;
    mdns_start_runtime();
    open_network_access_window("HTTP server ready");
    return ESP_OK;

fail:
    #undef REGISTER_HTTP_URI
    if (s_server) {
        httpd_handle_t failed_server = s_server;
        esp_err_t stop_ret = httpd_stop(failed_server);
        if (stop_ret == ESP_OK && s_server == failed_server) {
            s_server = NULL;
        } else if (stop_ret != ESP_OK) {
            ESP_LOGE(TAG, "partial HTTP server cleanup failed: %s",
                     esp_err_to_name(stop_ret));
        }
    }
    s_http_server_ready = false;
    http_server_set_stopping(false);
    return ret;
}

static esp_err_t refresh_webserver_after_storage_transition(const char *reason)
{
    http_server_set_stopping(true);
    httpd_handle_t server = s_server;
    if (server) {
        esp_err_t stop_ret = httpd_stop(server);
        if (stop_ret != ESP_OK) {
            ESP_LOGE(TAG, "HTTP refresh stop failed after %s: %s",
                     reason ? reason : "storage transition",
                     esp_err_to_name(stop_ret));
            http_server_set_stopping(false);
            return stop_ret;
        }
        if (s_server == server) {
            s_server = NULL;
        }
        ESP_LOGI(TAG, "HTTP server stopped for %s refresh",
                 reason ? reason : "storage transition");
    }
    if (s_mdns_started) {
#if CONFIG_APP_MDNS_ENABLE
        mdns_free();
#endif
        s_mdns_started = false;
        ESP_LOGI(TAG, "mDNS stopped for %s refresh",
                 reason ? reason : "storage transition");
    }
    s_http_server_ready = false;
    esp_err_t ret = start_webserver();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server refreshed after %s",
                 reason ? reason : "storage transition");
    }
    return ret;
}

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == ETH_EVENT) {
        esp_eth_handle_t eth_handle = event_data ? *(esp_eth_handle_t *)event_data : NULL;
        switch (event_id) {
        case ETHERNET_EVENT_START:
            s_eth_started = true;
            s_eth_runtime_ready = true;
            strlcpy(s_eth_last_error, "ok", sizeof(s_eth_last_error));
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_CONNECTED: {
            uint8_t mac_addr[6] = {0};
            if (eth_handle) {
                esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            }
            s_eth_link_up = true;
            s_eth_got_ip = false;
            s_eth_static_fallback = false;
            s_eth_link_up_ms = esp_timer_get_time() / 1000;
            strlcpy(s_eth_ip_addr, "0.0.0.0", sizeof(s_eth_ip_addr));
            mark_network_activity();
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
                     mac_addr[4], mac_addr[5]);
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            s_eth_link_up = false;
            s_eth_got_ip = false;
            s_eth_static_fallback = false;
            s_eth_link_up_ms = 0;
            strlcpy(s_eth_ip_addr, "0.0.0.0", sizeof(s_eth_ip_addr));
            ESP_LOGI(TAG, "Ethernet Link Down");
            break;
        case ETHERNET_EVENT_STOP:
            s_eth_started = false;
            s_eth_runtime_ready = false;
            s_eth_link_up = false;
            s_eth_got_ip = false;
            ESP_LOGI(TAG, "Ethernet Stopped");
            break;
        default:
            break;
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_eth_ip_addr, sizeof(s_eth_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
        s_eth_got_ip = true;
        mark_network_activity();
        ESP_LOGI(TAG, "Ethernet Got IP Address");
        ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "ETH URL: http://%s/", s_eth_ip_addr);
        return;
    }

    if (s_network_shutdown_for_idle && event_base == WIFI_EVENT) {
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_network_mode == NETWORK_MODE_STA || s_network_mode == NETWORK_MODE_APSTA) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_network_mode == NETWORK_MODE_STA || s_network_mode == NETWORK_MODE_APSTA) {
            s_reconnect_count++;
        }
        if ((s_network_mode == NETWORK_MODE_STA || s_network_mode == NETWORK_MODE_APSTA) &&
            (WIFI_MAXIMUM_RETRY == 0 || s_retry_num < WIFI_MAXIMUM_RETRY)) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "retry Wi-Fi connection (%d)", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        s_ap_clients++;
        mark_network_activity();
        ESP_LOGI(TAG, "AP client connected, clients=%" PRIu32, s_ap_clients);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        if (s_ap_clients > 0) {
            s_ap_clients--;
        }
        mark_network_activity();
        ESP_LOGI(TAG, "AP client disconnected, clients=%" PRIu32, s_ap_clients);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_sta_ip_addr, sizeof(s_sta_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
        if (!network_mode_has_ap(s_network_mode)) {
            strlcpy(s_ip_addr, s_sta_ip_addr, sizeof(s_ip_addr));
        }
        mark_network_activity();
        ESP_LOGI(TAG, "got sta ip: %s", s_sta_ip_addr);
        ESP_LOGI(TAG, "STA URL: http://%s/", s_sta_ip_addr);
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool network_mode_has_sta(network_mode_t mode)
{
    return mode == NETWORK_MODE_STA || mode == NETWORK_MODE_APSTA;
}

static bool wifi_ap_supported(void)
{
#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
    return true;
#else
    return false;
#endif
}

static bool network_mode_has_ap(network_mode_t mode)
{
    return wifi_ap_supported() && (mode == NETWORK_MODE_SOFTAP || mode == NETWORK_MODE_APSTA);
}

static bool parse_ipv4_addr_text(const char *text, esp_ip4_addr_t *addr)
{
    unsigned int a = 0;
    unsigned int b = 0;
    unsigned int c = 0;
    unsigned int d = 0;
    char extra = '\0';
    if (!text || !addr || sscanf(text, "%u.%u.%u.%u%c", &a, &b, &c, &d, &extra) != 4) {
        return false;
    }
    if (a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    IP4_ADDR(addr, a, b, c, d);
    return true;
}

static esp_err_t eth_apply_static_fallback(void)
{
#if CONFIG_APP_ETH_ENABLE
    if (!s_eth_netif) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_ip_info_t ip_info = {0};
    ESP_RETURN_ON_FALSE(parse_ipv4_addr_text(CONFIG_APP_ETH_STATIC_FALLBACK_IP, &ip_info.ip),
                        ESP_ERR_INVALID_ARG, TAG, "bad Ethernet fallback IP");
    ESP_RETURN_ON_FALSE(parse_ipv4_addr_text(CONFIG_APP_ETH_STATIC_FALLBACK_NETMASK, &ip_info.netmask),
                        ESP_ERR_INVALID_ARG, TAG, "bad Ethernet fallback netmask");
    IP4_ADDR(&ip_info.gw, 0, 0, 0, 0);

    esp_err_t stop_ret = esp_netif_dhcpc_stop(s_eth_netif);
    if (stop_ret != ESP_OK && stop_ret != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGW(TAG, "Ethernet DHCP client stop returned %s", esp_err_to_name(stop_ret));
    }
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(s_eth_netif, &ip_info),
                        TAG, "set Ethernet static fallback IP failed");

    snprintf(s_eth_ip_addr, sizeof(s_eth_ip_addr), IPSTR, IP2STR(&ip_info.ip));
    s_eth_got_ip = true;
    s_eth_static_fallback = true;
    s_network_active = true;
    mark_network_activity();
    strlcpy(s_eth_last_error, "static fallback", sizeof(s_eth_last_error));
    ESP_LOGW(TAG, "ETH static fallback: http://%s/", s_eth_ip_addr);
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void eth_dhcp_fallback_tick(int64_t now_ms)
{
#if CONFIG_APP_ETH_ENABLE
    if (!s_eth_started || !s_eth_link_up || s_eth_got_ip || s_eth_static_fallback ||
        !s_eth_netif || CONFIG_APP_ETH_DHCP_TIMEOUT_MS <= 0 || s_eth_link_up_ms <= 0) {
        return;
    }
    if (now_ms - s_eth_link_up_ms < CONFIG_APP_ETH_DHCP_TIMEOUT_MS) {
        return;
    }
    esp_err_t ret = eth_apply_static_fallback();
    if (ret != ESP_OK) {
        snprintf(s_eth_last_error, sizeof(s_eth_last_error), "%s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Ethernet static fallback failed: %s", s_eth_last_error);
    }
#else
    (void)now_ms;
#endif
}

static esp_err_t eth_init_runtime(void)
{
#if CONFIG_APP_ETH_ENABLE
    if (s_eth_handle && s_eth_started) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = APP_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = APP_ETH_PHY_RST_GPIO;

    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = APP_ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = APP_ETH_MDIO_GPIO;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = APP_ETH_RMII_CLK_GPIO;
#if defined(SOC_EMAC_USE_MULTI_IO_MUX) && SOC_EMAC_USE_MULTI_IO_MUX
    emac_config.emac_dataif_gpio.rmii.tx_en_num = APP_ETH_RMII_TX_EN_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd0_num = APP_ETH_RMII_TXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.txd1_num = APP_ETH_RMII_TXD1_GPIO;
    emac_config.emac_dataif_gpio.rmii.crs_dv_num = APP_ETH_RMII_CRS_DV_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd0_num = APP_ETH_RMII_RXD0_GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd1_num = APP_ETH_RMII_RXD1_GPIO;
#endif

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (!mac) {
        ret = ESP_FAIL;
        goto fail;
    }
    /*
     * ESP-IDF 6.x uses the generic IEEE 802.3 PHY driver for IP101-class PHYs.
     * The board-specific identity is still captured by the IP101GRI address,
     * reset GPIO, RMII pins, and MDC/MDIO pins above.
     */
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    if (!phy) {
        mac->del(mac);
        ret = ESP_FAIL;
        goto fail;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (ret != ESP_OK) {
        mac->del(mac);
        phy->del(phy);
        goto fail;
    }

    if (!s_eth_netif) {
        esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
        s_eth_netif = esp_netif_new(&netif_cfg);
        if (!s_eth_netif) {
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
    }

    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    if (!s_eth_glue) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    ret = esp_netif_attach(s_eth_netif, s_eth_glue);
    if (ret != ESP_OK) {
        goto fail;
    }

    if (!s_eth_handlers_registered) {
        ret = esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                  &event_handler, NULL, NULL);
        if (ret != ESP_OK) {
            goto fail;
        }
        ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                  &event_handler, NULL, NULL);
        if (ret != ESP_OK) {
            goto fail;
        }
        s_eth_handlers_registered = true;
    }

    ret = esp_eth_start(s_eth_handle);
    if (ret != ESP_OK) {
        goto fail;
    }

    s_eth_runtime_ready = true;
    s_network_active = true;
    strlcpy(s_eth_last_error, "ok", sizeof(s_eth_last_error));
    ESP_LOGI(TAG, "Ethernet runtime started; DHCP timeout=%d ms, fallback=http://%s/",
             CONFIG_APP_ETH_DHCP_TIMEOUT_MS, CONFIG_APP_ETH_STATIC_FALLBACK_IP);
    return ESP_OK;

fail:
    s_eth_runtime_ready = false;
    snprintf(s_eth_last_error, sizeof(s_eth_last_error), "%s", esp_err_to_name(ret));
    ESP_LOGW(TAG, "Ethernet runtime init failed: %s", s_eth_last_error);
    return ret;
#else
    strlcpy(s_eth_last_error, "disabled", sizeof(s_eth_last_error));
    return ESP_OK;
#endif
}

static void eth_stop_runtime(const char *reason)
{
#if CONFIG_APP_ETH_ENABLE
    if (!s_eth_handle || !s_eth_started) {
        return;
    }
    esp_err_t ret = esp_eth_stop(s_eth_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Ethernet stop requested: %s", reason ? reason : "shutdown");
    } else {
        ESP_LOGW(TAG, "Ethernet stop failed: %s", esp_err_to_name(ret));
    }
    s_eth_started = false;
    s_eth_runtime_ready = false;
    s_eth_link_up = false;
    s_eth_got_ip = false;
    s_eth_static_fallback = false;
    s_eth_link_up_ms = 0;
    strlcpy(s_eth_ip_addr, "0.0.0.0", sizeof(s_eth_ip_addr));
    strlcpy(s_eth_last_error, reason ? reason : "stopped", sizeof(s_eth_last_error));
#else
    (void)reason;
#endif
}

static esp_err_t configure_ap_static_ip(void)
{
    if (!s_ap_netif) {
        return ESP_OK;
    }

    esp_netif_ip_info_t ip_info = {0};
    ESP_RETURN_ON_FALSE(parse_ipv4_addr_text(CONFIG_APP_AP_STATIC_IP, &ip_info.ip),
                        ESP_ERR_INVALID_ARG, TAG, "bad AP static IP");
    ESP_RETURN_ON_FALSE(parse_ipv4_addr_text(CONFIG_APP_AP_NETMASK, &ip_info.netmask),
                        ESP_ERR_INVALID_ARG, TAG, "bad AP netmask");
    ESP_RETURN_ON_FALSE(parse_ipv4_addr_text(CONFIG_APP_AP_GATEWAY, &ip_info.gw),
                        ESP_ERR_INVALID_ARG, TAG, "bad AP gateway");

    esp_err_t stop_ret = esp_netif_dhcps_stop(s_ap_netif);
    if (stop_ret != ESP_OK) {
        ESP_LOGD(TAG, "AP DHCP server stop returned %s", esp_err_to_name(stop_ret));
    }
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(s_ap_netif, &ip_info), TAG, "set AP static IP failed");
    esp_err_t start_ret = esp_netif_dhcps_start(s_ap_netif);
    if (start_ret != ESP_OK) {
        ESP_LOGW(TAG, "AP DHCP server start returned %s", esp_err_to_name(start_ret));
    }
    snprintf(s_ap_ip_addr, sizeof(s_ap_ip_addr), IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

static esp_err_t wifi_apply_mode(network_mode_t mode)
{
    /*
     * Wi-Fi 模式切换会短暂断开当前连接，所以调用者要先让摄像头 standby。
     * 这里按“停止 -> 配置 STA/AP -> 启动 -> 等待 STA 结果”的顺序执行；
     * SoftAP 的 IP 固定使用 esp_netif 默认的 192.168.4.1，方便手机野外直连。
     */
    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        ESP_RETURN_ON_FALSE(s_wifi_event_group, ESP_ERR_NO_MEM, TAG, "no wifi event group");
    }

    if (!wifi_ap_supported() && mode != NETWORK_MODE_STA) {
        ESP_LOGW(TAG, "SoftAP is not supported by this Wi-Fi backend; falling back to STA");
        mode = NETWORK_MODE_STA;
    }

    if (s_wifi_started) {
        esp_wifi_stop();
        s_wifi_started = false;
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    s_retry_num = 0;
    s_ap_clients = 0;
    strlcpy(s_sta_ip_addr, "0.0.0.0", sizeof(s_sta_ip_addr));

    bool have_router_ssid = s_router_ssid[0] != '\0';
    if (network_mode_has_sta(mode) && !have_router_ssid) {
        if (wifi_ap_supported()) {
            ESP_LOGW(TAG, "router SSID is empty; using SoftAP until the Web page saves router settings");
            mode = NETWORK_MODE_SOFTAP;
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    }

    wifi_config_t sta_config = {0};
    strlcpy((char *)sta_config.sta.ssid, s_router_ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, s_router_password, sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = s_router_password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASSWORD,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = CONFIG_APP_AP_CHANNEL,
            .max_connection = CONFIG_APP_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(WIFI_AP_PASSWORD) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    wifi_mode_t wifi_mode = WIFI_MODE_STA;
    if (mode == NETWORK_MODE_SOFTAP) {
        wifi_mode = WIFI_MODE_AP;
    } else if (mode == NETWORK_MODE_APSTA) {
        wifi_mode = WIFI_MODE_APSTA;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(wifi_mode), TAG, "set wifi mode failed");
    if (network_mode_has_sta(mode)) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG, "set sta config failed");
    }
    if (network_mode_has_ap(mode)) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "set ap config failed");
        ESP_RETURN_ON_ERROR(configure_ap_static_ip(), TAG, "configure ap static ip failed");
    }

    s_network_mode = mode;
    s_rescue_ap_active = false;
    s_network_active = true;
    s_network_shutdown_for_idle = false;
    esp_err_t start_ret = esp_wifi_start();
    if (start_ret != ESP_OK) {
        s_network_active = false;
    }
    ESP_RETURN_ON_ERROR(start_ret, TAG, "wifi start failed");
    s_wifi_started = true;
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "disable wifi power save failed");
    mark_network_activity();
    save_setting_u8("netmode", (uint8_t)mode);

    if (network_mode_has_ap(mode)) {
        strlcpy(s_ip_addr, s_ap_ip_addr, sizeof(s_ip_addr));
    }

    if (mode == NETWORK_MODE_APSTA) {
        ESP_LOGI(TAG, "AP+STA started; AP HTTP service can open immediately while STA joins %s in background",
                 s_router_ssid);
    } else if (network_mode_has_sta(mode)) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(10000));
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to %s", s_router_ssid);
        } else {
            ESP_LOGW(TAG, "STA not connected yet; mode=%s", network_mode_name(mode));
            if (mode == NETWORK_MODE_STA && wifi_ap_supported()) {
                /*
                 * STA 是默认工作方式，但现场路由器不可达、密码错误或 DHCP 异常时，
                 * 纯 STA 会让用户看起来“没有 Web 服务”。这里自动切到 APSTA 作为救援链路：
                 * NVS 仍保存用户选择的 STA，状态接口用 rescue_ap=true 标识当前额外开的热点。
                 */
                ESP_LOGW(TAG, "enabling rescue SoftAP %s at %s", WIFI_AP_SSID, s_ap_ip_addr);
                esp_wifi_stop();
                s_wifi_started = false;
                vTaskDelay(pdMS_TO_TICKS(200));
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
                s_retry_num = 0;
                ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "set rescue APSTA failed");
                ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG, "set rescue sta failed");
                ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "set rescue ap failed");
                ESP_RETURN_ON_ERROR(configure_ap_static_ip(), TAG, "configure rescue ap ip failed");
                s_network_active = true;
                s_network_shutdown_for_idle = false;
                esp_err_t rescue_start_ret = esp_wifi_start();
                if (rescue_start_ret != ESP_OK) {
                    s_network_active = false;
                }
                ESP_RETURN_ON_ERROR(rescue_start_ret, TAG, "start rescue APSTA failed");
                s_wifi_started = true;
                ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG,
                                    "disable rescue wifi power save failed");
                mark_network_activity();
                s_rescue_ap_active = true;
                strlcpy(s_ip_addr, s_ap_ip_addr, sizeof(s_ip_addr));
            } else if (mode == NETWORK_MODE_APSTA) {
                strlcpy(s_ip_addr, s_ap_ip_addr, sizeof(s_ip_addr));
            }
        }
    }

    ESP_LOGI(TAG, "network mode: %s, rescue_ap=%d, sta=%s, ap=%s",
             network_mode_name(mode), s_rescue_ap_active, s_sta_ip_addr, s_ap_ip_addr);
    if (network_mode_has_ap(mode) || s_rescue_ap_active) {
        ESP_LOGI(TAG, "AP URL: http://%s/", s_ap_ip_addr);
    }
    if (strcmp(s_sta_ip_addr, "0.0.0.0") != 0) {
        ESP_LOGI(TAG, "STA URL: http://%s/", s_sta_ip_addr);
    } else if (network_mode_has_sta(mode)) {
        ESP_LOGI(TAG, "STA URL: pending router DHCP");
    }
    return ESP_OK;
}

static esp_err_t wifi_init_runtime(void)
{
    esp_err_t ret = ESP_OK;
    if (!s_wifi_event_group) {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group) {
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
    }

#if CONFIG_ESP_HOSTED_ENABLED
    /*
     * ESP-Hosted creates the transport during component startup, but the SPI
     * slave link is only proven usable after reconfigure/connect. Calling this
     * explicitly on first boot avoids a silent "no AP, no STA" failure when the
     * C6 side is not running the matching SPI firmware.
     */
    ret = (esp_err_t)esp_hosted_init();
    if (ret != ESP_OK) {
        goto fail;
    }
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    s_hosted_sdmmc_host_active = true;
#endif
    ret = (esp_err_t)esp_hosted_connect_to_slave();
    if (ret != ESP_OK) {
        goto fail;
    }
#endif

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }
#ifdef CONFIG_ESP_WIFI_SOFTAP_SUPPORT
    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }
#else
    s_ap_netif = NULL;
#endif
    if (!s_sta_netif) {
        ret = ESP_FAIL;
        goto fail;
    }

    if (!s_wifi_initialized) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            goto fail;
        }
        s_wifi_initialized = true;
    }

    if (!s_wifi_handlers_registered) {
        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                  &event_handler, NULL, &instance_any_id);
        if (ret != ESP_OK) {
            goto fail;
        }
        ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                  &event_handler, NULL, &instance_got_ip);
        if (ret != ESP_OK) {
            goto fail;
        }
        s_wifi_handlers_registered = true;
    }

    ret = init_time_sync();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SNTP init failed: %s; browser fallback remains available",
                 esp_err_to_name(ret));
    }

    ret = wifi_apply_mode(s_network_mode);
    if (ret != ESP_OK) {
        goto fail;
    }

    s_wifi_runtime_ready = true;
    strlcpy(s_wifi_last_error, "ok", sizeof(s_wifi_last_error));
    return ESP_OK;

fail:
    s_wifi_runtime_ready = false;
    s_wifi_init_failures++;
    snprintf(s_wifi_last_error, sizeof(s_wifi_last_error), "%s", esp_err_to_name(ret));
    ESP_LOGW(TAG, "Wi-Fi runtime init failed (%" PRIu32 "): %s",
             s_wifi_init_failures, s_wifi_last_error);
    return ret;
}

static esp_err_t wifi_reinit_after_storage_window(void)
{
    /*
     * 存储窗口会调用 esp_wifi_deinit() 和 esp_hosted_deinit() 释放共享外设资源。
     * 恢复时不能重复创建 esp_netif 或重复注册事件，只需要重新初始化 remote
     * Wi-Fi 后复用原来的 AP+STA 配置。
     */
    return wifi_init_runtime();
}

static esp_err_t wifi_shutdown_for_storage_window(void)
{
    esp_err_t finalize_ret = recording_finalize_sync(pdMS_TO_TICKS(5000));
    if (finalize_ret != ESP_OK) {
        ESP_LOGE(TAG, "recording finalize before storage window failed: %s",
                 esp_err_to_name(finalize_ret));
        return finalize_ret;
    }
    esp_err_t stop_ret = stop_webserver(HTTP_STOP_QUIESCE_TIMEOUT_MS);
    if (stop_ret != ESP_OK) {
        ESP_LOGE(TAG, "storage-window HTTP shutdown aborted: %s", esp_err_to_name(stop_ret));
        return stop_ret;
    }
    s_network_active = false;
    s_network_shutdown_for_idle = true;
    s_ap_clients = 0;
    clear_web_clients();

    if (s_wifi_started) {
        esp_err_t ret = esp_wifi_stop();
        ESP_LOGI(TAG, "Wi-Fi stop for storage window: %s", esp_err_to_name(ret));
        if (ret != ESP_OK) {
            esp_err_t web_ret = start_webserver();
            s_network_active = web_ret == ESP_OK;
            s_network_shutdown_for_idle = false;
            if (web_ret == ESP_OK) {
                mark_network_activity();
                open_network_access_window("Wi-Fi stop failed; Web restored");
            } else {
                ESP_LOGE(TAG, "Web restore after Wi-Fi stop failure failed: %s",
                         esp_err_to_name(web_ret));
            }
            return ret;
        }
        s_wifi_started = false;
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    esp_err_t deinit_ret = s_wifi_initialized ? esp_wifi_deinit() : ESP_OK;
    ESP_LOGI(TAG, "Wi-Fi deinit for storage window: %s", esp_err_to_name(deinit_ret));
    if (deinit_ret != ESP_OK) {
        esp_err_t restore_ret = wifi_init_runtime();
        if (restore_ret == ESP_OK) {
            esp_err_t web_ret = start_webserver();
            s_network_active = web_ret == ESP_OK;
            s_network_shutdown_for_idle = false;
            if (web_ret == ESP_OK) {
                mark_network_activity();
                open_network_access_window("Wi-Fi deinit failed; Web restored");
            } else {
                ESP_LOGE(TAG, "HTTP restore after Wi-Fi deinit failure failed: %s",
                         esp_err_to_name(web_ret));
            }
        } else {
            ESP_LOGE(TAG, "Web restore after Wi-Fi deinit failure also failed: %s",
                     esp_err_to_name(restore_ret));
        }
        return deinit_ret;
    }
    s_wifi_initialized = false;
    s_wifi_runtime_ready = false;
    strlcpy(s_ip_addr, "0.0.0.0", sizeof(s_ip_addr));
    strlcpy(s_sta_ip_addr, "0.0.0.0", sizeof(s_sta_ip_addr));
    return ESP_OK;
}

static void wifi_stop_for_export_mode(void)
{
    s_ap_clients = 0;
    clear_web_clients();
    if (s_wifi_started) {
        esp_err_t ret = esp_wifi_stop();
        ESP_LOGI(TAG, "Wi-Fi stop for export mode: %s", esp_err_to_name(ret));
        s_wifi_started = false;
    }
    s_wifi_runtime_ready = false;
    strlcpy(s_ip_addr, s_eth_ip_addr[0] ? s_eth_ip_addr : "0.0.0.0", sizeof(s_ip_addr));
    strlcpy(s_sta_ip_addr, "0.0.0.0", sizeof(s_sta_ip_addr));
}

static storage_runtime_snapshot_t storage_pause_capture_runtime(void)
{
    storage_runtime_snapshot_t snapshot = {
        .vision_enabled = s_vision_enabled,
        .history_enabled = s_history_enabled,
        .recording_enabled = s_recording_enabled,
        .recognition_method = s_recognition_method,
        .power_state = s_power_state,
    };

    s_vision_enabled = false;
    s_history_enabled = false;
    s_recording_enabled = false;
    cancel_dataset_for_storage_handoff();

    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    if (!s_camera_cmd_queue ||
        xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "storage maintenance could not queue camera standby");
    }
    return snapshot;
}

static void storage_restore_capture_runtime(const storage_runtime_snapshot_t *snapshot,
                                            bool wake_camera)
{
    if (!snapshot) {
        return;
    }
    s_vision_enabled = snapshot->vision_enabled;
    s_history_enabled = snapshot->history_enabled;
    s_recording_enabled = snapshot->recording_enabled;
    s_recognition_method = snapshot->recognition_method;

    bool was_requested_running = snapshot->power_state == POWER_STATE_RUNNING ||
                                 snapshot->power_state == POWER_STATE_STARTING ||
                                 snapshot->power_state == POWER_STATE_ERROR;
    if (wake_camera && was_requested_running) {
        camera_cmd_t wake = CAMERA_CMD_WAKE;
        if (!s_camera_cmd_queue ||
            xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "storage maintenance could not restore camera wake state");
        }
    }
}

static esp_err_t storage_format_mounted_tf(void)
{
    if (!storage_tf_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_vfs_fat_mount_config_t cfg = {
        .format_if_mount_failed = true,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
        .use_one_fat = true,
    };
    bool locked = false;
    if (s_storage_lock) {
        xSemaphoreTake(s_storage_lock, portMAX_DELAY);
        locked = true;
    }

    ESP_LOGW(TAG, "Formatting TF card at %s after all storage users quiesced",
             CONFIG_APP_SD_MOUNT_POINT);
    esp_err_t ret = esp_vfs_fat_sdcard_format_cfg(CONFIG_APP_SD_MOUNT_POINT,
                                                  s_sd_card, &cfg);
    if (ret == ESP_OK) {
        storage_clear_write_health();
        ret = storage_prepare_dirs_after_mount(
            s_sd_mount_mode[0] ? s_sd_mount_mode : "formatted");
        if (ret == ESP_OK) {
            s_sd_format_count++;
        }
    }
    if (ret != ESP_OK) {
        record_sd_mount_error("format", ret);
    }

    if (locked) {
        xSemaphoreGive(s_storage_lock);
    }
    return ret;
}

static void storage_service_task(void *arg)
{
    (void)arg;
    storage_service_request_t req;
    while (true) {
        if (xQueueReceive(s_storage_service_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (storage_transition_owner() != STORAGE_TRANSITION_MAINTENANCE) {
            s_storage_service_last_error_code = ESP_ERR_INVALID_STATE;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "discarded stale maintenance request without transition ownership");
            continue;
        }
        if (s_app_mode != APP_MODE_SERVER || storage_usb_owned() ||
            storage_mode_request_pending()) {
            s_storage_service_last_error_code = ESP_ERR_INVALID_STATE;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "storage maintenance cancelled because another mode transition is pending");
            storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
            continue;
        }

        s_storage_service_runs++;
        s_storage_service_last_mount_ok = false;
        s_storage_service_last_error_code = 0;
        strlcpy(s_storage_service_last_mode, "none", sizeof(s_storage_service_last_mode));
        s_storage_quiescing = true;
        /*
         * A USB callback can arrive after the request was admitted.  Recheck
         * after publishing quiescing so a deferred ownership event wins before
         * capture, FAT, Hosted, or SDMMC state is changed.
         */
        if (storage_mode_request_pending()) {
            s_storage_service_last_error_code = ESP_ERR_INVALID_STATE;
            s_storage_quiescing = false;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "storage maintenance deferred for a pending mode/USB transition");
            storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
            continue;
        }
        storage_runtime_snapshot_t runtime = storage_pause_capture_runtime();

        /*
         * Give the HTTP handler time to send the queued response, then wait for
         * every capture/storage user to release the card before touching FAT.
         */
        vTaskDelay(pdMS_TO_TICKS(500));
        set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK,
                                  "pausing capture and waiting for storage users");
        esp_err_t quiesce_ret = wait_for_usb_quiescence(10000);
        if (quiesce_ret == ESP_OK) {
            quiesce_ret = recording_finalize_sync(pdMS_TO_TICKS(5000));
        }
        bool transition_pending = storage_mode_request_pending();
        if (quiesce_ret == ESP_OK && transition_pending) {
            quiesce_ret = ESP_ERR_INVALID_STATE;
        }
        if (quiesce_ret != ESP_OK) {
            s_storage_service_last_error_code = quiesce_ret;
            storage_restore_capture_runtime(&runtime, true);
            s_storage_quiescing = false;
            if (transition_pending) {
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "maintenance deferred for a pending mode/USB transition");
            } else {
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "storage users did not stop safely; maintenance cancelled: %s",
                    esp_err_to_name(quiesce_ret));
            }
            open_network_access_window("storage maintenance cancelled safely");
            storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
            continue;
        }

        set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK,
                                  "temporarily closing HTTP and Wi-Fi");
        esp_err_t shutdown_ret = wifi_shutdown_for_storage_window();
        if (shutdown_ret != ESP_OK) {
            s_storage_service_last_error_code = shutdown_ret;
            storage_restore_capture_runtime(&runtime, true);
            s_storage_quiescing = false;
            set_storage_service_state(STORAGE_SERVICE_ERROR,
                                      "storage/network quiesce failed; handoff aborted: %s",
                                      esp_err_to_name(shutdown_ret));
            open_network_access_window("storage handoff aborted");
            storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
            continue;
        }

        /*
         * Hosted deinit tears down the global SDMMC host shared by both slots.
         * Remove FAT and Slot 0 while that host is still valid.
         */
        esp_err_t hosted_ret = storage_unmount(
            "before Hosted maintenance shutdown");
        if (hosted_ret != ESP_OK) {
            ESP_LOGE(TAG,
                     "storage maintenance stopped before Hosted shutdown because teardown failed: %s",
                     esp_err_to_name(hosted_ret));
        }
        if (hosted_ret == ESP_OK) {
            set_storage_service_state(STORAGE_SERVICE_HOSTED_DOWN,
                                      "deinitializing ESP-Hosted transport");
#if CONFIG_ESP_HOSTED_ENABLED
            hosted_ret = (esp_err_t)esp_hosted_deinit();
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
            if (hosted_ret == ESP_OK) {
                s_hosted_sdmmc_host_active = false;
            }
#endif
            if (hosted_ret != ESP_OK) {
                ESP_LOGE(TAG,
                         "esp_hosted_deinit for storage failed: %s; TF mount will not be attempted",
                         esp_err_to_name(hosted_ret));
            }
            vTaskDelay(pdMS_TO_TICKS(300));
#endif
        }

        set_storage_service_state(
            STORAGE_SERVICE_MOUNTING,
            req.format_requested ? "mounting TF for requested format" :
                                   "probing TF while Hosted is down");
        bool old_flash_fallback = s_storage_flash_fallback_enabled;
        s_storage_flash_fallback_enabled = false;
        esp_err_t maintenance_ret = hosted_ret == ESP_OK ?
                                    storage_mount_internal(req.format_requested) : hosted_ret;
        if (maintenance_ret == ESP_OK && req.format_requested) {
            maintenance_ret = storage_format_mounted_tf();
        }
        s_storage_flash_fallback_enabled = old_flash_fallback;
        s_storage_service_last_error_code = maintenance_ret;
        if (maintenance_ret == ESP_OK) {
            strlcpy(s_storage_service_last_mode, s_sd_mount_mode, sizeof(s_storage_service_last_mode));
            set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                                      req.format_requested ?
                                      "TF format and write verification passed via %s" :
                                      "TF probe and write verification passed via %s",
                                      s_sd_mount_mode);
            update_sd_info();
            vTaskDelay(pdMS_TO_TICKS(req.hold_ms));
        } else {
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                req.format_requested ? "TF format failed safely: %s" : "TF probe failed: %s",
                esp_err_to_name(maintenance_ret));
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        set_storage_service_state(STORAGE_SERVICE_UNMOUNTING, "releasing TF before network recovery");
        esp_err_t recovery_unmount_ret = storage_unmount("Wi-Fi restore");
        if (recovery_unmount_ret != ESP_OK) {
            s_storage_service_last_error_code = recovery_unmount_ret;
            ESP_LOGE(TAG, "TF teardown before network recovery failed: %s",
                     esp_err_to_name(recovery_unmount_ret));
        }

        set_storage_service_state(STORAGE_SERVICE_RESTORING_NETWORK, "restarting ESP-Hosted and AP+STA");
        esp_err_t wifi_ret = wifi_reinit_after_storage_window();
        if (wifi_ret == ESP_OK) {
            esp_err_t app_mount_ret = recovery_unmount_ret == ESP_OK ?
                                      storage_mount() : recovery_unmount_ret;
            bool storage_ready = app_mount_ret == ESP_OK && storage_acceptance_ok();
            if (!storage_ready && app_mount_ret == ESP_OK) {
                app_mount_ret = ESP_FAIL;
            }
            s_storage_service_last_mount_ok = storage_ready;
            if (storage_ready) {
                strlcpy(s_storage_service_last_mode, s_sd_mount_mode,
                        sizeof(s_storage_service_last_mode));
                update_sd_info();
            }
            esp_err_t web_ret = start_webserver();
            storage_restore_capture_runtime(&runtime, true);
            s_storage_quiescing = false;
            s_network_shutdown_for_idle = false;
            s_network_active = web_ret == ESP_OK;
            if (web_ret == ESP_OK) {
                mark_network_activity();
                open_network_access_window(storage_ready ?
                                           "storage maintenance complete" :
                                           "storage maintenance needs attention");
            }
            if (maintenance_ret == ESP_OK && storage_ready && web_ret == ESP_OK) {
                set_storage_service_state(
                    STORAGE_SERVICE_IDLE,
                    req.format_requested ?
                    "format complete; TF restored via %s; Web and capture resumed" :
                    "remount complete via %s; Web and capture resumed",
                    s_storage_service_last_mode);
            } else {
                esp_err_t final_ret = web_ret != ESP_OK ? web_ret :
                                      (maintenance_ret != ESP_OK ? maintenance_ret : app_mount_ret);
                s_storage_service_last_error_code = final_ret;
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    web_ret == ESP_OK ?
                    "maintenance finished with %s; Web restored; check TF and use online retry" :
                    "storage restored but HTTP restart failed: %s; automatic retry remains enabled",
                    esp_err_to_name(final_ret));
            }
        } else {
            esp_err_t web_ret = start_webserver();
            storage_restore_capture_runtime(&runtime, true);
            s_storage_quiescing = false;
            s_network_active = web_ret == ESP_OK;
            s_network_shutdown_for_idle = false;
            s_storage_service_last_error_code = wifi_ret;
            if (web_ret == ESP_OK) {
                mark_network_activity();
                open_network_access_window("Wi-Fi restore failed; Ethernet Web recovery active");
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "Wi-Fi restore failed: %s; Ethernet Web recovery and automatic retry remain active",
                    esp_err_to_name(wifi_ret));
            } else {
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "Wi-Fi restore failed: %s; HTTP restart also failed: %s",
                    esp_err_to_name(wifi_ret), esp_err_to_name(web_ret));
            }
        }
        storage_transition_release(STORAGE_TRANSITION_MAINTENANCE);
    }
}

static esp_err_t wait_for_enrichment_idle(uint32_t timeout_ms)
{
    int64_t deadline_ms = esp_timer_get_time() / 1000 + timeout_ms;
    while (esp_timer_get_time() / 1000 < deadline_ms) {
        recording_enrichment_status_t enrichment = {0};
        recording_enrichment_get_status(&enrichment);
        if (!recording_enrichment_has_request() && !enrichment.running) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static void enter_offline_tf_capture_mode(void)
{
    ESP_LOGI(TAG, "entering offline TF capture mode after idle network window");
    const recognition_method_t previous_method = s_recognition_method;
    const bool previous_vision = s_vision_enabled;
    const bool previous_history = s_history_enabled;
    const bool previous_recording = s_recording_enabled;
    const uint32_t previous_box_min_score = s_box_min_score;
    const uint32_t previous_inference_interval_ms = s_inference_interval_ms;
    const uint32_t previous_jpeg_quality = s_jpeg_quality;
    const power_state_t previous_power = s_power_state;
    const bool previous_flash_fallback = s_storage_flash_fallback_enabled;
    s_storage_quiescing = true;
    cancel_dataset_for_storage_handoff();
    set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK,
                              "waiting for active storage and HTTP work before field recording");
    if (wait_for_enrichment_idle(10000) != ESP_OK) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "enrichment did not stop; field transition aborted");
        ESP_LOGE(TAG, "FIELD transition aborted because enrichment did not stop");
        s_storage_quiescing = false;
        open_network_access_window("FIELD transition aborted");
        return;
    }

    set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK,
                              "closing HTTP, Wi-Fi and Ethernet for field recording");
    esp_err_t shutdown_ret = wifi_shutdown_for_storage_window();
    if (shutdown_ret != ESP_OK) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "storage/network quiesce failed; field transition aborted: %s",
                                  esp_err_to_name(shutdown_ret));
        ESP_LOGE(TAG, "FIELD transition aborted during HTTP shutdown: %s",
                 esp_err_to_name(shutdown_ret));
        s_storage_quiescing = false;
        open_network_access_window("FIELD transition aborted");
        return;
    }
    eth_stop_runtime("field capture");
    coco_espdl_release_background();

    s_app_mode = APP_MODE_FIELD;
    s_recognition_method = recognition_method_or_fallback(s_recognition_method);
    if (s_recognition_method == RECOGNITION_METHOD_OFF ||
        s_recognition_method == RECOGNITION_METHOD_MLP ||
        s_recognition_method == RECOGNITION_METHOD_YOLO11 ||
        s_recognition_method == RECOGNITION_METHOD_YOLO26) {
        s_recognition_method = fish31_espdl_available() ? RECOGNITION_METHOD_FISH31 :
                               recognition_method_or_fallback(preferred_recognition_method());
    }
    s_vision_enabled = true;
    s_history_enabled = false;
    s_recording_enabled = true;
    s_box_min_score = 50;
    s_inference_interval_ms = 0;
    s_jpeg_quality = 70;
    s_last_inference_ms = 0;
    s_last_recording_frame_ms = 0;
    s_recording_reset_requested = true;

    set_storage_service_state(STORAGE_SERVICE_HOSTED_DOWN,
                              "idle timeout; deinitializing ESP-Hosted transport");
    esp_err_t hosted_ret = ESP_OK;
#if CONFIG_ESP_HOSTED_ENABLED
    bool tf_on_sdmmc = strcmp(s_storage_backend, "tf_sdmmc") == 0;
    if (tf_on_sdmmc) {
        ESP_LOGI(TAG, "keeping ESP-Hosted SDMMC host initialized because TF is mounted via SDMMC");
    } else {
        hosted_ret = (esp_err_t)esp_hosted_deinit();
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
        if (hosted_ret == ESP_OK) {
            s_hosted_sdmmc_host_active = false;
        }
#endif
    }
    if (hosted_ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_hosted_deinit for offline TF capture failed: %s",
                 esp_err_to_name(hosted_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
#endif

    set_storage_service_state(STORAGE_SERVICE_MOUNTING,
                              "preparing TF for offline camera recording");
    s_storage_flash_fallback_enabled = false;
    esp_err_t mount_ret = hosted_ret;
    if (mount_ret == ESP_OK && s_sd_mounted && s_sd_card && storage_acceptance_ok()) {
        /*
         * Reuse the healthy mount during the server-to-field transition.
         * This avoids a FatFS drive-slot leak seen when unmounting and
         * immediately mounting the same card, and preserves SDMMC bus state
         * when the TF socket is running in 4-bit mode.
         */
        char mount_mode[sizeof(s_sd_mount_mode)];
        strlcpy(mount_mode, s_sd_mount_mode, sizeof(mount_mode));
        mount_ret = storage_prepare_dirs_after_mount(mount_mode);
    } else if (mount_ret == ESP_OK) {
        mount_ret = storage_unmount("server-to-field storage reset");
        if (mount_ret == ESP_OK) {
            mount_ret = storage_mount();
        }
    }
    if (mount_ret == ESP_OK && !storage_acceptance_ok()) {
        mount_ret = ESP_ERR_INVALID_SIZE;
    }
    s_storage_service_last_error_code = mount_ret;

    if (mount_ret == ESP_OK) {
        s_storage_service_last_mount_ok = true;
        strlcpy(s_storage_service_last_mode, s_sd_mount_mode, sizeof(s_storage_service_last_mode));
        update_sd_info();
        s_storage_quiescing = false;
        set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                                  "offline TF capture active via %s; reboot to reopen AP+STA",
                                  s_sd_mount_mode);
        camera_cmd_t wake = CAMERA_CMD_WAKE;
        xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "offline TF capture active via %s; reboot to reopen AP+STA",
                 s_sd_mount_mode);
    } else {
        s_storage_service_last_mount_ok = false;
        ESP_LOGW(TAG, "offline TF capture TF mount failed: %s; restoring Web service",
                 esp_err_to_name(mount_ret));

        s_app_mode = APP_MODE_SERVER;
        s_recognition_method = previous_method;
        s_vision_enabled = previous_vision;
        s_history_enabled = previous_history;
        s_recording_enabled = previous_recording;
        s_box_min_score = previous_box_min_score;
        s_inference_interval_ms = previous_inference_interval_ms;
        s_jpeg_quality = previous_jpeg_quality;
        s_storage_flash_fallback_enabled = previous_flash_fallback;
        set_storage_service_state(STORAGE_SERVICE_RESTORING_NETWORK,
                                  "TF unavailable; restoring Web recovery service");

        esp_err_t wifi_ret = wifi_reinit_after_storage_window();
        if (wifi_ret == ESP_OK) {
            esp_err_t web_ret = start_webserver();
            if (CONFIG_APP_ETH_ENABLE && !s_eth_started) {
                esp_err_t eth_ret = eth_init_runtime();
                if (eth_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Ethernet restore after FIELD failure: %s",
                             esp_err_to_name(eth_ret));
                }
            }
            s_storage_quiescing = false;
            s_network_shutdown_for_idle = false;
            s_network_active = web_ret == ESP_OK;
            if (web_ret == ESP_OK) {
                mark_network_activity();
                open_network_access_window("FIELD storage failure recovery");
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "FIELD aborted because TF failed (%s); Web restored, check the card and use TF retry",
                    esp_err_to_name(mount_ret));
            } else {
                set_storage_service_state(
                    STORAGE_SERVICE_ERROR,
                    "FIELD aborted because TF failed (%s); HTTP restart failed (%s)",
                    esp_err_to_name(mount_ret), esp_err_to_name(web_ret));
            }
            if (previous_power == POWER_STATE_RUNNING ||
                previous_power == POWER_STATE_STARTING ||
                previous_power == POWER_STATE_ERROR) {
                camera_cmd_t wake = CAMERA_CMD_WAKE;
                xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(100));
            }
        } else {
            esp_err_t web_ret = start_webserver();
            s_storage_quiescing = false;
            s_network_shutdown_for_idle = false;
            s_network_active = web_ret == ESP_OK;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                web_ret == ESP_OK ?
                "FIELD TF failure (%s); Wi-Fi failed (%s), Ethernet Web recovery active" :
                "FIELD TF failure (%s) and network restore failure (%s)",
                esp_err_to_name(mount_ret), esp_err_to_name(wifi_ret));
            ESP_LOGE(TAG, "Web restore after FIELD failure failed: %s",
                     esp_err_to_name(wifi_ret));
        }
    }
}

static void enter_export_mode(void)
{
    ESP_LOGI(TAG, "entering export mode: stopping capture and Wi-Fi, keeping Ethernet HTTP");
    s_app_mode = APP_MODE_EXPORT;
    s_vision_enabled = false;
    s_recording_enabled = false;
    s_history_enabled = false;
    s_inference_interval_ms = 600000;
    s_last_recording_frame_ms = 0;

    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(100));

    /*
     * Give recording_task one pass to close active .part files after the
     * app_mode/recording gate changed, then keep the HTTP server open on ETH.
     */
    vTaskDelay(pdMS_TO_TICKS(1200));

    if (wait_for_enrichment_idle(10000) != ESP_OK) {
        ESP_LOGE(TAG, "EXPORT transition continuing after enrichment stop timeout");
    }
    coco_espdl_release_background();

    wifi_stop_for_export_mode();
    if (!s_eth_started) {
        esp_err_t eth_ret = eth_init_runtime();
        if (eth_ret != ESP_OK) {
            ESP_LOGW(TAG, "export mode Ethernet init failed: %s", esp_err_to_name(eth_ret));
        }
    }
    esp_err_t web_ret = start_webserver();
    s_network_active = web_ret == ESP_OK;
    s_network_shutdown_for_idle = false;
    if (web_ret == ESP_OK) {
        mark_network_activity();
        open_network_access_window("export mode");
        set_storage_service_state(STORAGE_SERVICE_IDLE,
                                  "export mode active; Ethernet HTTP download only");
    } else {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "export mode entered but HTTP start failed: %s",
                                  esp_err_to_name(web_ret));
    }
}

static void usb_host_event_callback(bool connected, void *arg)
{
    (void)arg;
    __atomic_store_n(&s_usb_restore_auto_blocked, false, __ATOMIC_RELEASE);
    if (connected && s_app_mode == APP_MODE_USB_EXPORT) {
        __atomic_store_n(&s_usb_host_seen_during_export, true,
                         __ATOMIC_RELEASE);
    }
    if (connected && CONFIG_APP_USB_MSC_AUTO_EXPORT &&
        !s_usb_auto_export_suppressed &&
        usb_auto_export_allowed_mode(s_app_mode) &&
        !storage_usb_owned()) {
        bool admitted = storage_transition_reserve_event(
            STORAGE_TRANSITION_USB_EXPORT, &s_usb_export_requested);
        if (admitted) {
            ESP_LOGI(TAG, "USB1 host detected; queued writable TF export");
        } else {
            ESP_LOGI(TAG, "USB1 host detected during %s; deferring TF export",
                     storage_transition_name(storage_transition_owner()));
        }
    } else if (!connected && s_app_mode == APP_MODE_USB_EXPORT &&
               storage_usb_owned()) {
        if (!__atomic_load_n(&s_usb_restore_auto_blocked, __ATOMIC_ACQUIRE)) {
            bool admitted = storage_transition_reserve_event(
                STORAGE_TRANSITION_USB_RESTORE, &s_usb_restore_requested);
            if (admitted) {
                ESP_LOGI(TAG, "USB1 host inactive; queued automatic TF restore");
            } else {
                ESP_LOGI(TAG, "USB1 host inactive during %s; deferring TF restore",
                         storage_transition_name(storage_transition_owner()));
            }
        } else {
            ESP_LOGW(TAG, "USB1 host inactive; automatic restore is blocked until the next USB edge or Web retry");
        }
        s_usb_auto_export_suppressed = false;
    } else if (!connected) {
        s_usb_auto_export_suppressed = false;
    }
}

static void cancel_dataset_for_storage_handoff(void)
{
    if (!s_dataset_lock) {
        return;
    }
    xSemaphoreTake(s_dataset_lock, portMAX_DELAY);
    if (s_dataset_status.queued || s_dataset_status.running) {
        s_dataset_status.cancel = true;
    }
    xSemaphoreGive(s_dataset_lock);
}

static esp_err_t wait_for_usb_quiescence(uint32_t timeout_ms)
{
    int64_t deadline_ms = esp_timer_get_time() / 1000 + timeout_ms;
    int64_t quiet_since_ms = -1;
    while (esp_timer_get_time() / 1000 < deadline_ms) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        dataset_run_status_t dataset = {0};
        recording_enrichment_status_t enrichment = {0};
        dataset_status_copy(&dataset);
        recording_enrichment_get_status(&enrichment);
        bool enrichment_pending = recording_enrichment_has_request();
        bool camera_idle = s_power_state == POWER_STATE_STANDBY ||
                           s_power_state == POWER_STATE_ERROR;
        bool queues_idle = (!s_history_queue || uxQueueMessagesWaiting(s_history_queue) == 0) &&
                           (!s_recording_queue || uxQueueMessagesWaiting(s_recording_queue) == 0) &&
                           (!s_inference_queue || uxQueueMessagesWaiting(s_inference_queue) == 0);
        bool history_idle = !__atomic_load_n(&s_history_worker_busy, __ATOMIC_ACQUIRE);
        uint32_t stream_clients = stream_stats_client_count();
        uint32_t async_active = http_async_activity_count();
        uint32_t downloads = __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE);
        if (camera_idle && queues_idle && history_idle && !inference_worker_busy() &&
            !dataset.queued && !dataset.running && !enrichment_pending &&
            !enrichment.running && stream_clients == 0 && async_active == 0 &&
            downloads == 0) {
            if (quiet_since_ms < 0) {
                quiet_since_ms = now_ms;
            }
            if (now_ms - quiet_since_ms >= 300) {
                return ESP_OK;
            }
        } else {
            quiet_since_ms = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

#if CONFIG_APP_USB_MSC_ENABLE
static void restore_usb_previous_runtime(void)
{
    s_vision_enabled = s_usb_prev_vision_enabled;
    s_history_enabled = s_usb_prev_history_enabled;
    s_recording_enabled = s_usb_prev_recording_enabled;
    s_recognition_method = s_usb_prev_recognition_method;
    if (s_usb_prev_power_state == POWER_STATE_RUNNING) {
        camera_cmd_t wake = CAMERA_CMD_WAKE;
        if (xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "camera wake queue full while restoring USB runtime state");
        }
    }
}
#endif

static void enter_usb_export_mode(void)
{
#if CONFIG_APP_USB_MSC_ENABLE
    ESP_LOGI(TAG, "entering writable USB TF export mode with Web kept online");
    usb_msc_export_status_t initial_usb_status = {0};
    usb_msc_export_get_status(&initial_usb_status);
    __atomic_store_n(&s_usb_host_seen_during_export,
                     initial_usb_status.host_connected, __ATOMIC_RELEASE);
    s_storage_quiescing = true;
    s_usb_auto_export_suppressed = false;
    s_storage_mount_allowed = false;
    s_usb_prev_vision_enabled = s_vision_enabled;
    s_usb_prev_history_enabled = s_history_enabled;
    s_usb_prev_recording_enabled = s_recording_enabled;
    s_usb_prev_recognition_method = s_recognition_method;
    s_usb_prev_power_state = s_power_state;
    s_vision_enabled = false;
    s_history_enabled = false;
    s_recording_enabled = false;
    cancel_dataset_for_storage_handoff();

    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t ret = wait_for_usb_quiescence(8000);
    if (ret == ESP_OK) {
        ret = recording_finalize_sync(pdMS_TO_TICKS(5000));
    }
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "quiesce failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s; TF remains with application", s_usb_last_error);
        restore_usb_previous_runtime();
        __atomic_store_n(&s_usb_host_seen_during_export, false,
                         __ATOMIC_RELEASE);
        s_storage_mount_allowed = true;
        s_storage_quiescing = false;
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "USB handoff failed before TF export: %s",
                                  esp_err_to_name(ret));
        return;
    }
    coco_espdl_release_background();
    errno = 0;
    ret = recover_incomplete_recordings();
    if (ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) || !storage_acceptance_ok()) {
            snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                     "recording recovery failed before USB export: %s",
                     esp_err_to_name(ret));
            ESP_LOGE(TAG, "%s errno=%d; TF remains with application",
                     s_usb_last_error, helper_errno);
            restore_usb_previous_runtime();
            __atomic_store_n(&s_usb_host_seen_during_export, false,
                             __ATOMIC_RELEASE);
            s_storage_mount_allowed = true;
            s_storage_quiescing = false;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "USB export stopped because recording recovery hit TF I/O error: %s",
                esp_err_to_name(ret));
            return;
        }
        ESP_LOGW(TAG, "recording recovery before USB export degraded: %s errno=%d",
                 esp_err_to_name(ret), helper_errno);
    }
    errno = 0;
    ret = reconcile_recording_indexes();
    if (ret != ESP_OK) {
        int helper_errno = errno ? errno : EIO;
        if (storage_errno_is_media_failure(helper_errno) || !storage_acceptance_ok()) {
            snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                     "recording index reconciliation failed before USB export: %s",
                     esp_err_to_name(ret));
            ESP_LOGE(TAG, "%s errno=%d; TF remains with application",
                     s_usb_last_error, helper_errno);
            restore_usb_previous_runtime();
            __atomic_store_n(&s_usb_host_seen_during_export, false,
                             __ATOMIC_RELEASE);
            s_storage_mount_allowed = true;
            s_storage_quiescing = false;
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "USB export stopped because recording index recovery hit TF I/O error: %s",
                esp_err_to_name(ret));
            return;
        }
        ESP_LOGW(TAG, "recording index reconciliation before USB export degraded: %s errno=%d",
                 esp_err_to_name(ret), helper_errno);
    }
    cleanup_recording_temp_files();

    s_last_recording_frame_ms = 0;
    s_network_active = true;
    s_network_shutdown_for_idle = false;

    ret = storage_unmount("USB mass-storage handoff");
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "TF unmount failed: %s; USB export was cancelled",
                 esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s", s_usb_last_error);
        restore_usb_previous_runtime();
        __atomic_store_n(&s_usb_host_seen_during_export, false,
                         __ATOMIC_RELEASE);
        s_storage_mount_allowed = true;
        s_storage_quiescing = false;
        set_storage_service_state(
            STORAGE_SERVICE_ERROR,
            "USB handoff stopped because application storage teardown failed: %s",
            esp_err_to_name(ret));
        return;
    }

    s_app_mode = APP_MODE_USB_EXPORT;

    ret = storage_init_usb_sdmmc_card();
    if (ret == ESP_OK) {
        ret = usb_msc_export_attach_sdmmc(s_usb_sd_card);
    }
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "TF attach failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s; trying to restore application storage while Web stays online", s_usb_last_error);
        esp_err_t release_ret = storage_release_usb_sdmmc_card();
        if (release_ret != ESP_OK) {
            /* Keep ownership quarantined in USB_EXPORT until every SDMMC/LDO
             * teardown step succeeds.  The next Web restore can retry the
             * idempotent detach/release sequence without remounting early. */
            s_usb_storage_ready = true;
            snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                     "USB attach failed and SDMMC teardown failed: %s; storage remains quarantined",
                     esp_err_to_name(release_ret));
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "USB attach cleanup failed: %s; do not remount or remove TF; unplug USB and retry restore in Web",
                esp_err_to_name(release_ret));
            s_storage_quiescing = false;
            return;
        }
        s_usb_storage_ready = false;
        s_app_mode = APP_MODE_SERVER;
        __atomic_store_n(&s_usb_host_seen_during_export, false,
                         __ATOMIC_RELEASE);
        s_storage_mount_allowed = true;
        esp_err_t remount_ret = storage_mount();
        if (remount_ret != ESP_OK) {
            snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                     "USB attach failed; TF restore failed: %s", esp_err_to_name(remount_ret));
            set_storage_service_state(STORAGE_SERVICE_ERROR,
                                      "USB attach failed; TF restore failed: %s",
                                      esp_err_to_name(remount_ret));
        } else {
            update_sd_info();
            strlcpy(s_usb_last_error, "USB attach failed; TF restored to application",
                    sizeof(s_usb_last_error));
            set_storage_service_state(STORAGE_SERVICE_IDLE,
                                      "USB attach failed; TF restored to application");
        }
        restore_usb_previous_runtime();
        s_storage_quiescing = false;
        return;
    }

    s_usb_storage_ready = true;
    strlcpy(s_usb_last_error, "ok", sizeof(s_usb_last_error));
    set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                               "USB host owns writable TF; Web restore or unplug returns TF to application; Web stays online");
    s_storage_quiescing = false;
    esp_err_t web_ret = refresh_webserver_after_storage_transition("USB export");
    if (web_ret != ESP_OK) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "USB export succeeded, but HTTP refresh failed: %s; retry from STA/AP or reboot if Web is unavailable",
                                  esp_err_to_name(web_ret));
    }
    mark_network_activity();
    open_network_access_window("USB export active");
    ESP_LOGI(TAG, "USB writable TF ready; Web restore or unplug returns TF to application");
#endif
}

static esp_err_t enter_usb_app_restore_mode(bool suppress_reexport)
{
#if CONFIG_APP_USB_MSC_ENABLE
    ESP_LOGI(TAG, "restoring TF ownership from USB to application");
    s_storage_quiescing = true;
    if (suppress_reexport) {
        s_usb_auto_export_suppressed = true;
    }
    s_vision_enabled = false;
    s_history_enabled = false;
    s_recording_enabled = false;
    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(150));

    esp_err_t ret = usb_msc_export_detach_storage();
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "USB detach failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s", s_usb_last_error);
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "USB detach failed: %s", esp_err_to_name(ret));
        s_storage_quiescing = false;
        return ret;
    }

    esp_err_t release_ret = storage_release_usb_sdmmc_card();
    if (release_ret != ESP_OK) {
        s_usb_storage_ready = true;
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "USB detached but SDMMC teardown failed: %s; storage remains quarantined",
                 esp_err_to_name(release_ret));
        set_storage_service_state(
            STORAGE_SERVICE_ERROR,
            "USB detached but SDMMC teardown failed: %s; remount is blocked; retry restore in Web",
            esp_err_to_name(release_ret));
        s_storage_quiescing = false;
        return release_ret;
    }

    s_usb_storage_ready = false;
    s_app_mode = APP_MODE_SERVER;
    __atomic_store_n(&s_usb_host_seen_during_export, false,
                     __ATOMIC_RELEASE);
    s_storage_mount_allowed = true;
    vTaskDelay(pdMS_TO_TICKS(250));
    usb_msc_export_status_t usb_status = {0};
    usb_msc_export_get_status(&usb_status);
    s_usb_auto_export_suppressed = suppress_reexport || usb_status.host_connected;
    if (suppress_reexport) {
        __atomic_store_n(&s_usb_export_requested, false, __ATOMIC_RELEASE);
    }
    esp_err_t mount_ret = storage_mount();
    if (mount_ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "USB detached; TF restore failed: %s; use Web retry after checking the card",
                 esp_err_to_name(mount_ret));
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "USB detached; TF restore failed: %s; Web remains online",
                                  esp_err_to_name(mount_ret));
    } else {
        update_sd_info();
        strlcpy(s_usb_last_error, "restored to application", sizeof(s_usb_last_error));
        set_storage_service_state(STORAGE_SERVICE_IDLE,
                                  "USB detached; TF restored to application");
    }
    /* Detach succeeded, so the device is back in SERVER mode even when TF
     * remount needs attention. Restore the user's camera/model/capture state
     * now; a later Web TF retry must not snapshot permanently disabled flags. */
    restore_usb_previous_runtime();
    esp_err_t web_ret = refresh_webserver_after_storage_transition("USB restore");
    if (web_ret != ESP_OK) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "USB restored TF, but HTTP restart failed: %s; reboot if Web is unavailable",
                                  esp_err_to_name(web_ret));
        if (mount_ret == ESP_OK) {
            mount_ret = web_ret;
        }
    }
    s_storage_quiescing = false;
    mark_network_activity();
    open_network_access_window(mount_ret == ESP_OK ?
                               "USB restore complete" : "USB restore needs attention");
    return mount_ret;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void retry_application_storage(void)
{
    if (storage_transition_owner() != STORAGE_TRANSITION_RETRY) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "TF retry discarded without transition ownership");
        return;
    }
    if (s_app_mode != APP_MODE_SERVER || storage_usb_owned() || s_storage_quiescing ||
        storage_mode_request_pending()) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "TF retry rejected in current mode");
        return;
    }

    ESP_LOGI(TAG, "retrying application TF mount with a bounded HTTP pause");
    s_storage_quiescing = true;
    set_storage_service_state(STORAGE_SERVICE_MOUNTING,
                              "quiescing camera and storage users for Web TF retry");
    storage_runtime_snapshot_t runtime = storage_pause_capture_runtime();

    /* Allow the 202 response to leave the socket before stopping HTTP. */
    vTaskDelay(pdMS_TO_TICKS(500));

    esp_err_t ret = wait_for_usb_quiescence(10000);
    if (ret == ESP_OK && s_recording_queue) {
        ret = recording_finalize_sync(pdMS_TO_TICKS(5000));
    }
    bool web_stopped = false;
    if (ret == ESP_OK) {
        ret = stop_webserver(HTTP_STOP_QUIESCE_TIMEOUT_MS);
        web_stopped = ret == ESP_OK;
    }
    if (ret == ESP_OK) {
        ret = storage_unmount("Web TF retry");
        if (ret == ESP_OK) {
            s_storage_mount_allowed = true;
            ret = storage_mount();
        }
    }

    esp_err_t web_ret = ESP_OK;
    if (web_stopped) {
        web_ret = start_webserver();
        if (ret == ESP_OK && web_ret != ESP_OK) {
            ret = web_ret;
        }
    }
    storage_restore_capture_runtime(&runtime, true);
    if (ret == ESP_OK && storage_acceptance_ok()) {
        update_sd_info();
        strlcpy(s_usb_last_error, "TF retry passed write verification",
                sizeof(s_usb_last_error));
        set_storage_service_state(STORAGE_SERVICE_IDLE,
                                  "TF retry passed write/read verification via %s",
                                  s_sd_mount_mode);
    } else {
        if (ret == ESP_OK) {
            ret = ESP_FAIL;
        }
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "TF retry failed: %s; check or replace the card",
                 esp_err_to_name(ret));
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  web_ret == ESP_OK ?
                                  "TF retry failed: %s; HTTP restored without reboot" :
                                  "TF retry finished but HTTP restart failed: %s; automatic retry remains enabled",
                                  esp_err_to_name(web_ret == ESP_OK ? ret : web_ret));
    }

    s_storage_quiescing = false;
    mark_network_activity();
    open_network_access_window(ret == ESP_OK ?
                               "TF retry complete" : "TF retry needs attention");
}

static storage_transition_t storage_transition_claim_pending_request(void)
{
    storage_transition_t owner = storage_transition_owner();
    if (owner != STORAGE_TRANSITION_NONE) {
        return owner;
    }

    if (storage_request_pending(&s_storage_retry_requested) &&
        storage_transition_try_acquire(STORAGE_TRANSITION_RETRY)) {
        return STORAGE_TRANSITION_RETRY;
    }
    owner = storage_transition_owner();
    if (owner != STORAGE_TRANSITION_NONE) {
        return owner;
    }
    if (storage_request_pending(&s_usb_restore_requested) &&
        storage_transition_try_acquire(STORAGE_TRANSITION_USB_RESTORE)) {
        return STORAGE_TRANSITION_USB_RESTORE;
    }
    owner = storage_transition_owner();
    if (owner != STORAGE_TRANSITION_NONE) {
        return owner;
    }
    if (storage_request_pending(&s_usb_export_requested) &&
        storage_transition_try_acquire(STORAGE_TRANSITION_USB_EXPORT)) {
        return STORAGE_TRANSITION_USB_EXPORT;
    }
    owner = storage_transition_owner();
    if (owner != STORAGE_TRANSITION_NONE) {
        return owner;
    }
    if (storage_request_pending(&s_field_mode_requested) &&
        storage_transition_try_acquire(STORAGE_TRANSITION_FIELD)) {
        return STORAGE_TRANSITION_FIELD;
    }
    owner = storage_transition_owner();
    if (owner != STORAGE_TRANSITION_NONE) {
        return owner;
    }
    if (storage_request_pending(&s_export_mode_requested) &&
        storage_transition_try_acquire(STORAGE_TRANSITION_EXPORT)) {
        return STORAGE_TRANSITION_EXPORT;
    }
    return storage_transition_owner();
}

static void network_watchdog_tick(void)
{
    storage_transition_t owner = storage_transition_owner();
    /* The storage worker owns Hosted/SDMMC until it has restored everything. */
    if (owner == STORAGE_TRANSITION_MAINTENANCE) {
        field_idle_pause_latch();
        return;
    }

#if CONFIG_APP_USB_MSC_ENABLE
    if (owner == STORAGE_TRANSITION_NONE) {
        usb_msc_export_status_t usb_status = {0};
        usb_msc_export_get_status(&usb_status);
        if (CONFIG_APP_USB_MSC_AUTO_EXPORT &&
            usb_status.host_connected && !s_usb_storage_ready &&
            !s_usb_auto_export_suppressed &&
            usb_auto_export_allowed_mode(s_app_mode)) {
            storage_transition_reserve_event(
                STORAGE_TRANSITION_USB_EXPORT, &s_usb_export_requested);
        } else if (!usb_status.host_connected &&
                   s_app_mode == APP_MODE_USB_EXPORT &&
                   storage_usb_owned()) {
            __atomic_store_n(&s_usb_restore_auto_blocked, false,
                             __ATOMIC_RELEASE);
            storage_transition_reserve_event(
                STORAGE_TRANSITION_USB_RESTORE, &s_usb_restore_requested);
        }
    }
#endif

    owner = storage_transition_claim_pending_request();
    if (owner == STORAGE_TRANSITION_MAINTENANCE) {
        field_idle_pause_latch();
        return;
    }

    if (owner == STORAGE_TRANSITION_RETRY) {
        if (!storage_request_take(&s_storage_retry_requested)) {
            storage_transition_release(STORAGE_TRANSITION_RETRY);
            field_idle_pause_latch();
            return;
        }
        retry_application_storage();
        storage_transition_release(STORAGE_TRANSITION_RETRY);
        field_idle_pause_latch();
        return;
    }
    if (owner == STORAGE_TRANSITION_USB_EXPORT) {
        if (!storage_request_take(&s_usb_export_requested)) {
            storage_transition_release(STORAGE_TRANSITION_USB_EXPORT);
            field_idle_pause_latch();
            return;
        }
        if (usb_auto_export_allowed_mode(s_app_mode) &&
            !storage_usb_owned() && !s_storage_quiescing) {
            enter_usb_export_mode();
        }
        storage_transition_release(STORAGE_TRANSITION_USB_EXPORT);
        field_idle_pause_latch();
        return;
    }
    if (owner == STORAGE_TRANSITION_USB_RESTORE) {
        if (!storage_request_take(&s_usb_restore_requested)) {
            storage_transition_release(STORAGE_TRANSITION_USB_RESTORE);
            field_idle_pause_latch();
            return;
        }
        bool manual_restore = __atomic_exchange_n(
            &s_usb_restore_manual_requested, false, __ATOMIC_ACQ_REL);
        esp_err_t restore_ret = ESP_ERR_INVALID_STATE;
        if (s_app_mode == APP_MODE_USB_EXPORT && !s_storage_quiescing) {
            restore_ret = enter_usb_app_restore_mode(manual_restore);
        }
        if (restore_ret == ESP_OK) {
            __atomic_store_n(&s_usb_restore_auto_blocked, false, __ATOMIC_RELEASE);
            if (manual_restore) {
                s_usb_auto_export_suppressed = true;
                __atomic_store_n(&s_usb_export_requested, false, __ATOMIC_RELEASE);
            }
        } else if (storage_usb_owned()) {
            __atomic_store_n(&s_usb_restore_auto_blocked, true, __ATOMIC_RELEASE);
            ESP_LOGE(TAG,
                     "USB restore %s attempt latched after failure (%s); waiting for a new USB edge or Web retry",
                     manual_restore ? "manual" : "automatic",
                     esp_err_to_name(restore_ret));
        }
        storage_transition_release(STORAGE_TRANSITION_USB_RESTORE);
        field_idle_pause_latch();
        return;
    }
    if (owner == STORAGE_TRANSITION_FIELD) {
        if (!storage_request_pending(&s_field_mode_requested)) {
            storage_transition_release(STORAGE_TRANSITION_FIELD);
            field_idle_pause_latch();
            return;
        }
        dataset_run_status_t dataset = {0};
        dataset_status_copy(&dataset);
        if (__atomic_load_n(&s_validation_active_jobs, __ATOMIC_ACQUIRE) > 0 ||
            dataset.queued || dataset.running || inference_worker_busy()) {
            ESP_LOGD(TAG, "FIELD mode remains pending while validation/dataset inference is active");
            field_idle_pause_latch();
            return;
        }
        if (!storage_request_take(&s_field_mode_requested)) {
            storage_transition_release(STORAGE_TRANSITION_FIELD);
            field_idle_pause_latch();
            return;
        }
        if ((s_app_mode == APP_MODE_SERVER || s_app_mode == APP_MODE_EXPORT) &&
            !storage_usb_owned() && !s_storage_quiescing &&
            storage_acceptance_ok()) {
            ESP_LOGI(TAG, "manual FIELD_MODE request accepted");
            enter_offline_tf_capture_mode();
        } else {
            set_storage_service_state(
                STORAGE_SERVICE_ERROR,
                "FIELD request cancelled because TF or application mode is no longer ready");
            open_network_access_window("FIELD request needs attention");
        }
        storage_transition_release(STORAGE_TRANSITION_FIELD);
        field_idle_pause_latch();
        return;
    }
    if (owner == STORAGE_TRANSITION_EXPORT) {
        if (!storage_request_take(&s_export_mode_requested)) {
            storage_transition_release(STORAGE_TRANSITION_EXPORT);
            field_idle_pause_latch();
            return;
        }
        if (s_app_mode == APP_MODE_SERVER && !storage_usb_owned() &&
            !s_storage_quiescing) {
            ESP_LOGI(TAG, "manual EXPORT_MODE request accepted");
            enter_export_mode();
        }
        storage_transition_release(STORAGE_TRANSITION_EXPORT);
        field_idle_pause_latch();
        return;
    }
    if (owner != STORAGE_TRANSITION_NONE) {
        field_idle_pause_latch();
        return;
    }

    if (!s_field_auto_enable) {
        field_idle_pause_latch();
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    uint32_t web_clients = web_client_count(now_ms);
    uint32_t file_download_clients = __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE);
    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    uint32_t stream_clients = stream_stats_client_count();
    uint32_t validation_jobs = __atomic_load_n(&s_validation_active_jobs, __ATOMIC_ACQUIRE);
    bool acceptance_ok = storage_acceptance_ok();
    const char *pause_reason = field_idle_pause_reason_for_state(
        now_ms, acceptance_ok, web_clients, stream_clients, validation_jobs,
        file_download_clients, &dataset, inference_worker_busy());
    if (pause_reason) {
        field_idle_pause_latch();
        return;
    }

    if (__atomic_exchange_n(&s_field_idle_pause_latched, false,
                            __ATOMIC_ACQ_REL)) {
        field_idle_reanchor_after_pause(now_ms);
        ESP_LOGI(TAG, "automatic FIELD countdown restarted after busy state cleared");
        return;
    }

    int64_t last_activity_ms = __atomic_load_n(
        &s_last_network_activity_ms, __ATOMIC_ACQUIRE);
    int64_t last_web_client_ms = __atomic_load_n(
        &s_last_web_client_activity_ms, __ATOMIC_ACQUIRE);
    if (last_web_client_ms > last_activity_ms) {
        last_activity_ms = last_web_client_ms;
    }
    int64_t idle_ms = last_activity_ms > 0 ? now_ms - last_activity_ms : now_ms;
    if (idle_ms < (int64_t)s_field_idle_timeout_ms) {
        return;
    }

    if (!storage_transition_try_acquire(STORAGE_TRANSITION_FIELD)) {
        field_idle_pause_latch();
        return;
    }

    ESP_LOGI(TAG,
             "network idle shutdown: idle_ms=%" PRId64 ", web_clients=%" PRIu32
             ", stream_clients=%" PRIu32 ", downloads=%" PRIu32
             ", validation_jobs=%" PRIu32,
              idle_ms, web_clients, stream_clients, file_download_clients,
              validation_jobs);
    enter_offline_tf_capture_mode();
    storage_transition_release(STORAGE_TRANSITION_FIELD);
    field_idle_pause_latch();
}

static void network_task(void *arg)
{
    /*
     * 网络任务独立于 HTTP handler：网页请求只把目标模式写入队列，真正切换
     * 在这个任务里完成。这样即使 Wi-Fi 重启耗时较长，HTTP handler 也能快速返回。
    */
    int64_t last_wifi_retry_ms = -10000;
    int64_t last_http_retry_ms = -10000;
    while (true) {
        network_mode_t mode;
        if (xQueueReceive(s_netmode_queue, &mode,
                          pdMS_TO_TICKS(CONFIG_APP_NETWORK_WATCHDOG_PERIOD_MS)) == pdTRUE) {
            if (s_app_mode == APP_MODE_EXPORT || s_app_mode == APP_MODE_FIELD ||
                s_app_mode == APP_MODE_USB_EXPORT) {
                ESP_LOGW(TAG, "ignoring network mode switch while app_mode=%s",
                         app_mode_name(s_app_mode));
                continue;
            }
            bool reconfigure = s_wifi_reconfigure_requested;
            s_wifi_reconfigure_requested = false;
            if (mode == s_network_mode && !reconfigure) {
                continue;
            }

            camera_cmd_t standby = CAMERA_CMD_STANDBY;
            xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(50));
            set_power_state(POWER_STATE_STOPPING);
            vTaskDelay(pdMS_TO_TICKS(1000));

            esp_err_t ret = wifi_apply_mode(mode);
            if (ret == ESP_OK) {
                s_netmode_switches++;
            } else {
                ESP_LOGE(TAG, "network mode switch failed: %s", esp_err_to_name(ret));
            }
        } else {
            int64_t now_ms = esp_timer_get_time() / 1000;
            eth_dhcp_fallback_tick(now_ms);
            if (s_storage_mount_allowed && !s_wifi_runtime_ready && !s_network_shutdown_for_idle &&
                s_app_mode != APP_MODE_EXPORT && s_app_mode != APP_MODE_FIELD &&
                s_app_mode != APP_MODE_USB_EXPORT &&
                now_ms - last_wifi_retry_ms >= 10000) {
                last_wifi_retry_ms = now_ms;
                ESP_LOGI(TAG, "retrying Wi-Fi runtime startup");
                esp_err_t ret = wifi_init_runtime();
                if (ret == ESP_OK) {
                    esp_err_t web_ret = start_webserver();
                    if (web_ret == ESP_OK) {
                        ESP_LOGI(TAG, "Camera web server recovered: http://%s/", s_ip_addr);
                    } else {
                        ESP_LOGW(TAG, "HTTP recovery after Wi-Fi startup failed: %s",
                                 esp_err_to_name(web_ret));
                    }
                }
            }
            bool network_interface_ready = s_wifi_runtime_ready || s_eth_started;
            if (network_interface_ready && !s_http_server_ready && !s_storage_quiescing &&
                !s_network_shutdown_for_idle && s_app_mode != APP_MODE_FIELD &&
                now_ms - last_http_retry_ms >= 5000) {
                last_http_retry_ms = now_ms;
                esp_err_t web_ret = start_webserver();
                if (web_ret == ESP_OK) {
                    s_network_active = true;
                    mark_network_activity();
                    ESP_LOGI(TAG, "HTTP service recovered without reboot");
                } else {
                    ESP_LOGW(TAG, "HTTP service retry failed: %s",
                             esp_err_to_name(web_ret));
                }
            }
            network_watchdog_tick();
            if (!s_storage_boot_probe_queued &&
                CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS > 0 &&
                now_ms >= CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS &&
                !s_sd_mounted &&
                s_storage_service_mode == STORAGE_SERVICE_IDLE &&
                web_client_count(now_ms) == 0 &&
                s_ap_clients == 0 &&
                stream_stats_client_count() == 0 &&
                s_storage_service_queue) {
                storage_service_request_t req = {
                    .hold_ms = 1500,
                    .format_requested = false,
                };
                if (queue_storage_service_request(&req)) {
                    s_storage_boot_probe_queued = true;
                    ESP_LOGI(TAG, "queued automatic idle storage timeshare probe");
                }
            }
        }
    }
}

static void eth_fallback_task(void *arg)
{
    (void)arg;
    while (true) {
        eth_dhcp_fallback_tick(esp_timer_get_time() / 1000);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    load_runtime_settings();
    int64_t boot_now_ms = esp_timer_get_time() / 1000;
    __atomic_store_n(&s_last_network_activity_ms, boot_now_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&s_last_web_client_activity_ms, boot_now_ms, __ATOMIC_RELEASE);
    s_network_boot_window_until_ms = 0;
    strlcpy(s_ap_ip_addr, CONFIG_APP_AP_STATIC_IP, sizeof(s_ap_ip_addr));
    ESP_LOGI(TAG,
             "Network access window<=%" PRIu32 " ms after HTTP starts, idle timeout=%" PRIu32
             " ms, fixed AP URL=http://%s/",
             network_access_window_ms(), s_field_idle_timeout_ms, s_ap_ip_addr);
    log_acceleration_status();

    s_frame_capacity = CONFIG_APP_FRAME_BUFFER_BYTES;
    s_latest_jpeg = alloc_psram_buffer(s_frame_capacity);
    ESP_ERROR_CHECK(s_latest_jpeg ? ESP_OK : ESP_ERR_NO_MEM);

    s_history_records = (history_record_t *)heap_caps_calloc(CONFIG_APP_HISTORY_MAX_RECORDS,
                                                             sizeof(history_record_t),
                                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_history_records) {
        s_history_records = (history_record_t *)calloc(CONFIG_APP_HISTORY_MAX_RECORDS, sizeof(history_record_t));
    }
    ESP_ERROR_CHECK(s_history_records ? ESP_OK : ESP_ERR_NO_MEM);

    s_frame_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_frame_lock ? ESP_OK : ESP_ERR_NO_MEM);

    s_history_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_history_lock ? ESP_OK : ESP_ERR_NO_MEM);

    s_validation_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_validation_lock ? ESP_OK : ESP_ERR_NO_MEM);

    s_dataset_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_dataset_lock ? ESP_OK : ESP_ERR_NO_MEM);

    s_dataset_frame_cache = (dataset_frame_cache_t *)heap_caps_calloc(
        CONFIG_APP_DATASET_RUN_MAX_FRAMES, sizeof(dataset_frame_cache_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_dataset_frame_cache) {
        s_dataset_frame_cache = (dataset_frame_cache_t *)calloc(
            CONFIG_APP_DATASET_RUN_MAX_FRAMES, sizeof(dataset_frame_cache_t));
    }
    ESP_ERROR_CHECK(s_dataset_frame_cache ? ESP_OK : ESP_ERR_NO_MEM);

    s_storage_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_storage_lock ? ESP_OK : ESP_ERR_NO_MEM);

    s_camera_cmd_queue = xQueueCreate(4, sizeof(camera_cmd_t));
    ESP_ERROR_CHECK(s_camera_cmd_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_netmode_queue = xQueueCreate(1, sizeof(network_mode_t));
    ESP_ERROR_CHECK(s_netmode_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_history_queue = xQueueCreate(HISTORY_QUEUE_DEPTH, sizeof(history_item_t));
    ESP_ERROR_CHECK(s_history_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_recording_queue = xQueueCreate(CONFIG_APP_RECORDING_QUEUE_DEPTH,
                                     sizeof(recording_item_t));
    ESP_ERROR_CHECK(s_recording_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_recording_finalize_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_recording_finalize_done ? ESP_OK : ESP_ERR_NO_MEM);
    recording_enrichment_init(CONFIG_APP_ENRICHMENT_ENABLE);

    s_inference_queue = xQueueCreate(1, sizeof(inference_job_t));
    ESP_ERROR_CHECK(s_inference_queue ? ESP_OK : ESP_ERR_NO_MEM);
    recording_enrichment_set_infer_callback(enrichment_infer_annotate_cb, NULL);

    s_dataset_run_queue = xQueueCreate(1, sizeof(dataset_run_request_t));
    ESP_ERROR_CHECK(s_dataset_run_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_storage_service_queue = xQueueCreate(1, sizeof(storage_service_request_t));
    ESP_ERROR_CHECK(s_storage_service_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_recording_cleanup_queue = xQueueCreate(
        1, sizeof(recording_cleanup_request_t));
    ESP_ERROR_CHECK(s_recording_cleanup_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_boot_id = esp_random();

    start_async_workers();

    BaseType_t history_ok = xTaskCreate(history_task, "history_task",
                                        CONFIG_APP_HISTORY_TASK_STACK_SIZE,
                                        NULL, CONFIG_APP_HISTORY_TASK_PRIORITY,
                                        &s_history_task_handle);
    ESP_ERROR_CHECK(history_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t recording_ok = xTaskCreate(recording_task, "recording_task",
                                          CONFIG_APP_RECORDING_TASK_STACK_SIZE,
                                          NULL, CONFIG_APP_HISTORY_TASK_PRIORITY,
                                          &s_recording_task_handle);
    ESP_ERROR_CHECK(recording_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

#if CONFIG_APP_ENRICHMENT_ENABLE
    BaseType_t enrichment_ok = xTaskCreate(enrichment_task, "recording_enrich",
                                           CONFIG_APP_ENRICHMENT_TASK_STACK_SIZE,
                                           NULL, 1, &s_enrichment_task_handle);
    ESP_ERROR_CHECK(enrichment_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);
#endif

    resegment_status_update(false, false, false, 0, 0, 0,
                            "disabled: existing recordings are preserved");

    BaseType_t inference_ok = xTaskCreate(inference_task, "inference_task",
                                          CONFIG_APP_INFERENCE_TASK_STACK_SIZE,
                                          NULL, CONFIG_APP_INFERENCE_TASK_PRIORITY,
                                          &s_inference_task_handle);
    ESP_ERROR_CHECK(inference_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t dataset_ok = xTaskCreate(dataset_run_task, "dataset_task",
                                        CONFIG_APP_INFERENCE_TASK_STACK_SIZE + 4096,
                                        NULL, CONFIG_APP_INFERENCE_TASK_PRIORITY - 1,
                                        &s_dataset_task_handle);
    ESP_ERROR_CHECK(dataset_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t storage_svc_ok = xTaskCreate(storage_service_task, "storage_svc",
                                            6144, NULL, 4,
                                            &s_storage_service_task_handle);
    ESP_ERROR_CHECK(storage_svc_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t cleanup_ok = xTaskCreate(
        recording_cleanup_task, "recording_cleanup", 6144, NULL, 3,
        &s_recording_cleanup_task_handle);
    ESP_ERROR_CHECK(cleanup_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t ok = xTaskCreate(camera_task, "camera_task",
                                CONFIG_APP_CAMERA_TASK_STACK_SIZE,
                                NULL, CONFIG_APP_CAMERA_TASK_PRIORITY,
                                &s_camera_task_handle);
    ESP_ERROR_CHECK(ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*
     * ESP-Hosted has initialized the SDMMC controller before app_main, but the
     * Wi-Fi RPC link is not active yet. Mount Slot 0 now with the official
     * host_sdcard_with_hosted init/deinit shim. Keep the background storage
     * task gated until this first attempt has completed.
     */
    esp_err_t storage_ret = storage_mount();
    s_storage_mount_allowed = true;
    if (storage_ret != ESP_OK) {
        ESP_LOGE(TAG, "TF SDSPI mount failed before Wi-Fi startup: %s",
                 esp_err_to_name(storage_ret));
    }
#if CONFIG_APP_USB_MSC_ENABLE
    esp_err_t usb_ret = usb_msc_export_init(usb_host_event_callback, NULL);
    if (usb_ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error), "%s", esp_err_to_name(usb_ret));
        ESP_LOGE(TAG, "USB MSC initialization failed: %s", s_usb_last_error);
    } else {
        strlcpy(s_usb_last_error, "waiting for USB1 host", sizeof(s_usb_last_error));
    }
#else
    strlcpy(s_usb_last_error, "disabled", sizeof(s_usb_last_error));
#endif
    esp_err_t eth_ret = eth_init_runtime();
    if (eth_ret != ESP_OK) {
        ESP_LOGE(TAG, "Ethernet runtime unavailable: %s", esp_err_to_name(eth_ret));
    }
    bool eth_available = false;
#if CONFIG_APP_ETH_ENABLE
    eth_available = eth_ret == ESP_OK;
    if (eth_available) {
        BaseType_t eth_task_ok = xTaskCreate(eth_fallback_task, "eth_fallback",
                                             3072, NULL, 4,
                                             &s_eth_fallback_task_handle);
        ESP_ERROR_CHECK(eth_task_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);
        esp_err_t web_ret = start_webserver();
        if (web_ret == ESP_OK) {
            ESP_LOGI(TAG, "Ethernet HTTP server active; DHCP pending, fallback=http://%s/",
                     CONFIG_APP_ETH_STATIC_FALLBACK_IP);
        } else {
            ESP_LOGW(TAG, "Ethernet is active but HTTP startup failed: %s; watchdog will retry",
                     esp_err_to_name(web_ret));
        }
    }
#endif
    esp_err_t wifi_ret = wifi_init_runtime();
    if (wifi_ret == ESP_OK) {
        esp_err_t web_ret = start_webserver();
        if (web_ret == ESP_OK) {
            ESP_LOGI(TAG, "Camera web server started; AP=http://%s/ STA=%s ETH=%s",
                     s_ap_ip_addr, s_sta_ip_addr, s_eth_ip_addr);
        } else {
            ESP_LOGW(TAG, "Wi-Fi is active but HTTP startup failed: %s; watchdog will retry",
                     esp_err_to_name(web_ret));
        }
    } else if (!eth_available) {
        ESP_LOGE(TAG, "Wi-Fi runtime unavailable: %s; AP+STA HTTP access needs ESP-Hosted/C6, serial image validation stays active",
                 esp_err_to_name(wifi_ret));
    } else {
        ESP_LOGW(TAG, "Wi-Fi runtime unavailable: %s; Ethernet HTTP remains active",
                 esp_err_to_name(wifi_ret));
    }

    /*
     * Start retry/watchdog logic only after the first Hosted transaction has
     * returned. Otherwise network_task can enter wifi_init_runtime() in
     * parallel with app_main and reset the C6/SDIO transport mid-handshake.
     */
    BaseType_t network_ok = xTaskCreate(network_task, "network_task",
                                        8192, NULL, 5, &s_network_task_handle);
    ESP_ERROR_CHECK(network_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    /*
     * Customer firmware keeps model validation on the /validate page instead
     * of running a boot self-test. The self-test eagerly loads COCO and can
     * starve the hardware JPEG encoder before the user opens live preview.
     */
}
