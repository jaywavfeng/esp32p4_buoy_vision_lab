/*
 * ESP32-P4-WIFI6-DEV-KIT-A LAN camera server.
 *
 * Architecture:
 * - one camera task captures and encodes frames continuously
 * - the latest JPEG frame is copied into a shared PSRAM cache
 * - each /stream client runs on an async HTTP worker task
 * - power commands are queued to the camera task for fast HTTP response
 * - recognition_method controls off/mlp/yolo26/yolo11 at runtime
 * - network_mode controls sta/softap/apsta at runtime and is stored in NVS
 *
 * 中文结构说明：
 * - 摄像头任务负责 Wake/Standby、采集、编码和发布最新帧，网页命令只入队，不阻塞 HTTP。
 * - 独立 inference_task 负责 YOLO11/YOLO26 长耗时推理，摄像头和 /stream 不等待模型跑完。
 * - HTTP worker 并发服务多个 /stream 客户端，所有客户端读取同一份最新帧缓存。
 * - 历史任务把抽帧识别结果写入 RAM 环形队列和 TF 卡，并按数量/索引大小自动淘汰旧数据。
 * - 识别方法运行时可切换：off 用来测纯图传，mlp 是可乐/雪碧轻量 baseline，yolo26/yolo11 是自训练 Coke/Sprite 的 ESP-DL 量化模型后端。
 * - 无线模式运行时可切换：STA 连路由器，SoftAP 让板子自己开热点，APSTA 同时保留两条链路做稳定性对比。
 */

#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
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
#include "recording_enrichment.h"
#include "usb_msc_export.h"

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
#define CONFIG_APP_DEFAULT_RECOGNITION_METHOD 4
#endif
#ifndef CONFIG_APP_SD_BUS_WIDTH
#define CONFIG_APP_SD_BUS_WIDTH 4
#endif
#ifndef CONFIG_APP_SD_MAX_FREQ_KHZ
#define CONFIG_APP_SD_MAX_FREQ_KHZ 20000
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
#define SETTINGS_VERSION 12
#define YOLO26_MODEL_NAME "coke-sprite-yolo26n-416-p4"
#define YOLO11_MODEL_NAME "coke-sprite-yolo11n-416-p4"
#define COCO_MODEL_NAME "coco-yolo11n-320-s8-v3-p4"
#define YOLO26_INPUT_SIZE 416U
#define YOLO11_INPUT_SIZE 416U
#define COCO_INPUT_SIZE 320U
#define MLP_MODEL_BYTES (sizeof(CAN_CLASSIFIER_W1) + sizeof(CAN_CLASSIFIER_B1) + \
                         sizeof(CAN_CLASSIFIER_W2) + sizeof(CAN_CLASSIFIER_B2))
#define APP_MAX_DETECTIONS 8U
#define APP_YOLO_NMS_THRESHOLD_X100 70U

#define STREAM_BOUNDARY_TEXT CONFIG_APP_HTTP_PART_BOUNDARY
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY_TEXT
#define STREAM_BOUNDARY "\r\n--" STREAM_BOUNDARY_TEXT "\r\n"
#define STREAM_PART "Content-Type: image/jpeg\r\nContent-Length: %" PRIu32 "\r\nX-Frame-Seq: %" PRIu32 "\r\nX-Capture-Latency-Ms: %" PRId64 "\r\nX-Encode-Ms: %" PRId64 "\r\nX-Motion-Score: %" PRIu32 "\r\nX-Object-Score: %" PRIu32 "\r\nX-Scene: %s\r\n\r\n"

#define HISTORY_ROOT_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4"
#define HISTORY_SNAPSHOT_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/snapshots"
#define HISTORY_JSONL_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.jsonl"
#define HISTORY_JSONL_OLD_PATH CONFIG_APP_SD_MOUNT_POINT "/esp32p4/history.old.jsonl"
#define RECORDING_DIR CONFIG_APP_SD_MOUNT_POINT "/esp32p4/recordings"
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
#define HISTORY_QUEUE_DEPTH 8
#define RECORDING_LABEL_BUCKETS 8
#define JSONL_TAIL_LINE_BYTES 2048
#define DATASET_NAME_MAX 32
#define DATASET_PATH_MAX 160
#define DATASET_RUN_LATENCY_CAP 512
#define BUILTIN_COCO_VIDEO_DATASET "coco_video_demo"
#define BUILTIN_COCO_VIDEO_FRAMES 16U
#define BOARD_IMAGE_VALIDATION_MAX_ANALYSIS_MS 2500
#define APP_JSONL_INDEX_VERSION 4U
#define APP_RECORDING_MIN_FREE_BYTES (512ULL * 1024ULL * 1024ULL)
#define APP_RECORDING_MIN_FREE_PERCENT 5U
#define APP_MIN_VALID_EPOCH_MS 1577836800000ULL
#define APP_MAX_VALID_EPOCH_MS 4102444800000ULL

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
} recognition_method_t;

typedef enum {
    VALIDATION_SAMPLE_NONE = 0,
    VALIDATION_SAMPLE_COKE,
    VALIDATION_SAMPLE_SPRITE,
    VALIDATION_SAMPLE_DEMO_01,
    VALIDATION_SAMPLE_DEMO_02,
    VALIDATION_SAMPLE_DEMO_03,
    VALIDATION_SAMPLE_DEMO_04,
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
    bool motion;
    char scene[16];
    char color[16];
    char label[24];
    char object[16];
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
    bool finalize;
    SemaphoreHandle_t finalize_done;
} recording_item_t;

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

typedef struct {
    uint32_t hold_ms;
    bool format_if_failed;
    bool reboot_after;
} storage_service_request_t;

typedef struct {
    SemaphoreHandle_t done;
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
    int64_t completed_ms;
} validation_context_t;

/*
 * /validate 页面使用的“最近一次验证结果”缓存。
 * HTTP 请求 /api/validate/run 会等待 inference_task 跑完模型；随后 /api/validate/overlay.svg
 * 只需要读取这里的结果并画 SVG，不再重复推理，避免一个页面刷新触发多次 YOLO。
 */
typedef struct {
    bool valid;
    uint32_t id;
    validation_sample_t sample;
    recognition_method_t method;
    uint32_t box_min_score;
    vision_result_t vision;
    uint32_t source_w;
    uint32_t source_h;
    uint32_t jpeg_size;
    int64_t queued_ms;
    int64_t completed_ms;
} validation_cache_t;

/*
 * 推理任务队列只保存“要分析的一帧 JPEG 副本”和当时的元数据。
 * 摄像头任务发布图传后立刻返回采集循环，YOLO11/YOLO26 这种秒级推理由 inference_task 独立完成。
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

static const char *TAG = "wifi_camera_web";
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_eth_netif;
static esp_eth_handle_t s_eth_handle;
static esp_eth_netif_glue_handle_t s_eth_glue;
static httpd_handle_t s_server;
static QueueHandle_t s_camera_cmd_queue;
static QueueHandle_t s_async_req_queue;
static QueueHandle_t s_history_queue;
static QueueHandle_t s_recording_queue;
static QueueHandle_t s_inference_queue;
static QueueHandle_t s_netmode_queue;
static QueueHandle_t s_dataset_run_queue;
static QueueHandle_t s_storage_service_queue;
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
static TaskHandle_t s_recording_task_handle;
static TaskHandle_t s_enrichment_task_handle;
static TaskHandle_t s_network_task_handle;
static TaskHandle_t s_eth_fallback_task_handle;
static TaskHandle_t s_validation_selftest_task_handle;
static TaskHandle_t s_dataset_task_handle;
static TaskHandle_t s_storage_service_task_handle;

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
static volatile bool s_mdns_started;
static volatile app_mode_t s_app_mode = APP_MODE_SERVER;
static volatile bool s_storage_quiescing;
static int64_t s_last_network_activity_ms;
static int64_t s_network_boot_window_until_ms;
static volatile bool s_storage_mount_allowed;
static bool s_video_hw_ready;
static volatile power_state_t s_power_state = CONFIG_APP_BOOT_STANDBY ? POWER_STATE_STANDBY : POWER_STATE_STARTING;
static volatile bool s_vision_enabled = true;
static volatile bool s_history_enabled = CONFIG_APP_HISTORY_ENABLE;
static volatile bool s_recording_enabled = CONFIG_APP_RECORDING_ENABLE;
static volatile uint32_t s_box_min_score = CONFIG_APP_CAN_BOX_MIN_SCORE;
static volatile uint32_t s_stream_max_fps = CONFIG_APP_STREAM_MAX_FPS;
static volatile uint32_t s_inference_interval_ms = CONFIG_APP_INFERENCE_INTERVAL_MS;
static volatile uint32_t s_history_sample_interval_ms = CONFIG_APP_HISTORY_SAMPLE_INTERVAL_MS;
static volatile uint32_t s_jpeg_quality = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
static volatile recognition_method_t s_recognition_method = CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 4 ? RECOGNITION_METHOD_COCO :
                                                            (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 3 ? RECOGNITION_METHOD_YOLO11 :
                                                             (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 2 ? RECOGNITION_METHOD_YOLO26 :
                                                              (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 0 ? RECOGNITION_METHOD_OFF :
                                                               RECOGNITION_METHOD_MLP)));
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
static volatile bool s_sd_format_requested;
static volatile uint32_t s_sd_format_count;
static volatile storage_service_mode_t s_storage_service_mode = STORAGE_SERVICE_IDLE;
static char s_storage_service_status[128] = "idle";
static volatile uint32_t s_storage_service_runs;
static volatile int s_storage_service_last_error_code;
static volatile bool s_storage_service_last_mount_ok;
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
static dataset_run_status_t s_dataset_status;
static dataset_frame_cache_t *s_dataset_frame_cache;

static uint8_t s_prev_luma_grid[VISION_GRID_N];
static bool s_prev_luma_valid;

static volatile uint32_t s_requests;
static volatile uint32_t s_frames_total;
static volatile uint32_t s_capture_errors;
static volatile uint32_t s_frame_drops;
static volatile uint32_t s_stream_clients;
static volatile uint32_t s_file_download_clients;
static volatile uint32_t s_stream_errors;
static volatile uint32_t s_stream_frames_total;
static volatile uint64_t s_stream_bytes_total;
static volatile uint32_t s_capture_fps_x100;
static volatile uint32_t s_stream_fps_x100;
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
static volatile uint32_t s_recording_summary_count;
static volatile uint32_t s_recording_current_frames;
static volatile uint64_t s_recording_bytes;
static volatile uint64_t s_recording_current_bytes;
static int64_t s_last_recording_frame_ms;
static char s_recording_current_uri[128];
static char s_usb_last_error[96] = "not initialized";
static volatile bool s_usb_storage_ready;

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
static bool queue_yolo_inference(const uint8_t *jpeg, uint32_t jpeg_size,
                                 const frame_meta_t *meta, recognition_method_t method);
static void recording_maybe_queue(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta);
static bool network_mode_has_ap(network_mode_t mode);
static void mark_network_activity(void);
static void open_network_access_window(const char *reason);
static void record_http_request(void);
static esp_err_t eth_init_runtime(void);
static void log_acceleration_status(void);
static void dataset_status_copy(dataset_run_status_t *out);
static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end);
static esp_err_t queue_async_request(httpd_req_t *req, async_req_handler_t handler);
static uint32_t remove_recording_index_rows(const char *name, bool *failed);

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
    return recognition_method_name(method);
}

static uint32_t model_input_size_for_method(recognition_method_t method)
{
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
    return false;
}

static void detections_to_json(char *buf, size_t size, const vision_result_t *vision)
{
    if (!buf || size == 0) {
        return;
    }
    size_t off = 0;
    buf[0] = '\0';
    int n = snprintf(buf + off, size - off, "[");
    if (n < 0) {
        return;
    }
    off += (size_t)n;
    uint32_t count = vision ? vision->detection_count : 0;
    if (count > APP_MAX_DETECTIONS) {
        count = APP_MAX_DETECTIONS;
    }
    for (uint32_t i = 0; i < count && off < size; i++) {
        const vision_detection_t *d = &vision->detections[i];
        n = snprintf(buf + off, size - off,
                     "%s{\"label\":\"%s\",\"class_id\":%" PRIu32 ",\"score\":%" PRIu32 ","
                     "\"x\":%" PRIu32 ",\"y\":%" PRIu32 ",\"w\":%" PRIu32 ",\"h\":%" PRIu32 "}",
                     i == 0 ? "" : ",", d->label, d->class_id, d->score,
                     d->x, d->y, d->w, d->h);
        if (n < 0) {
            break;
        }
        off += (size_t)n;
    }
    if (off < size) {
        snprintf(buf + off, size - off, "]");
    } else {
        buf[size - 1] = '\0';
    }
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

    int n = snprintf(svg, svg_cap,
                     "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 %" PRIu32 " %" PRIu32 "\">"
                     "<image href=\"%s\" x=\"0\" y=\"0\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" preserveAspectRatio=\"none\"/>"
                     "<rect x=\"0\" y=\"0\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" fill=\"none\" stroke=\"#000\" stroke-width=\"2\"/>",
                     source_w, source_h, data_uri, source_w, source_h, source_w, source_h);
    free(data_uri);

    uint32_t count = vision->detection_count;
    if (count > APP_MAX_DETECTIONS) {
        count = APP_MAX_DETECTIONS;
    }
    for (uint32_t i = 0; i < count && n > 0 && (size_t)n < svg_cap; i++) {
        const vision_detection_t *d = &vision->detections[i];
        uint32_t text_y = d->y > 24 ? d->y - 8 : d->y + d->h + 28;
        if (text_y > source_h - 4) {
            text_y = source_h > 8 ? source_h - 8 : source_h;
        }
        n += snprintf(svg + n, svg_cap - n,
                      "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" "
                      "fill=\"none\" stroke=\"#64d68a\" stroke-width=\"4\"/>"
                      "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"430\" height=\"34\" fill=\"#000\" fill-opacity=\"0.75\"/>"
                      "<text x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" fill=\"#64d68a\" font-size=\"24\" font-family=\"Arial,sans-serif\">"
                      "%s %" PRIu32 "%% / threshold %" PRIu32 "%%</text>",
                      d->x, d->y, d->w, d->h,
                      d->x, text_y > 28 ? text_y - 28 : 0,
                      d->x + 8, text_y,
                      d->label, d->score, vision->box_min_score);
    }

    bool draw_candidate = count == 0 && vision->candidate_score > 0 &&
                          vision->object_w > 0 && vision->object_h > 0;
    if (draw_candidate && n > 0 && (size_t)n < svg_cap) {
        uint32_t text_y = vision->object_y > 24 ? vision->object_y - 8 :
                          vision->object_y + vision->object_h + 28;
        if (text_y > source_h - 4) {
            text_y = source_h > 8 ? source_h - 8 : source_h;
        }
        n += snprintf(svg + n, svg_cap - n,
                      "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"%" PRIu32 "\" height=\"%" PRIu32 "\" "
                      "fill=\"none\" stroke=\"#ffcc66\" stroke-width=\"4\" stroke-dasharray=\"10 8\"/>"
                      "<rect x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" width=\"520\" height=\"34\" fill=\"#000\" fill-opacity=\"0.75\"/>"
                      "<text x=\"%" PRIu32 "\" y=\"%" PRIu32 "\" fill=\"#ffcc66\" font-size=\"24\" font-family=\"Arial,sans-serif\">"
                      "%s %" PRIu32 "%% / threshold %" PRIu32 "%%</text>",
                      vision->object_x, vision->object_y,
                      vision->object_w, vision->object_h,
                      vision->object_x, text_y > 28 ? text_y - 28 : 0,
                      vision->object_x + 8, text_y,
                      vision->label, vision->candidate_score, vision->box_min_score);
    } else if (count == 0 && n > 0 && (size_t)n < svg_cap) {
        n += snprintf(svg + n, svg_cap - n,
                      "<rect x=\"16\" y=\"16\" width=\"560\" height=\"42\" fill=\"#000\" fill-opacity=\"0.75\"/>"
                      "<text x=\"28\" y=\"46\" fill=\"#ffcc66\" font-size=\"24\" font-family=\"Arial,sans-serif\">"
                      "no candidate / threshold %" PRIu32 "%%</text>",
                      vision->box_min_score);
    }
    if (n > 0 && (size_t)n < svg_cap) {
        snprintf(svg + n, svg_cap - n, "</svg>");
    }

    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, svg);
    free(svg);
    return ret;
}

static const char s_index_html[] =
"<!doctype html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>ESP32-P4 摄像头</title><style>"
":root{color-scheme:dark light;--bg:#101418;--panel:#191f24;--line:#2d3740;--text:#eef3f7;--muted:#9aa8b3;--ok:#64d68a;--bad:#ff7b7b;--accent:#7cc7ff}"
"*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:var(--bg);color:var(--text)}"
"main{width:min(1180px,100%);margin:0 auto;padding:18px;display:grid;gap:14px}"
".top,.histHead{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap}.title{font-size:22px;font-weight:700}.title.small{font-size:18px}.sub{color:var(--muted);font-size:13px;margin-top:3px}"
".view{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:14px;align-items:start}.video{background:#050708;border:1px solid var(--line);border-radius:8px;min-height:220px;overflow:hidden;display:grid;place-items:center}"
".videoFrame{position:relative;width:100%}img{display:block;width:100%;height:auto;min-height:220px;object-fit:contain}.boxes{position:absolute;inset:0;pointer-events:none}.bbox{position:absolute;border:2px solid var(--accent);box-shadow:0 0 0 1px #0008;pointer-events:none}.bbox span{position:absolute;left:0;top:-24px;background:#000b;color:var(--accent);font-size:12px;padding:3px 6px;border-radius:4px;white-space:nowrap}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}"
".card,.history{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px}.label{color:var(--muted);font-size:12px}.value{font-size:22px;font-weight:700;margin-top:4px}.wide{grid-column:1/-1}"
".actions{display:flex;gap:8px;flex-wrap:wrap;align-items:center}button,select,input{border:1px solid var(--line);background:#22303a;color:var(--text);border-radius:8px;padding:9px 13px;font-weight:650}button,select{cursor:pointer}input{width:110px}button:hover,select:hover{border-color:var(--accent)}button.active{border-color:var(--accent);color:var(--accent)}"
".toggle{display:flex;align-items:center;gap:8px;border:1px solid var(--line);background:#17212a;border-radius:8px;padding:8px 11px;font-weight:650}.toggle input{width:18px;height:18px;accent-color:var(--accent)}"
".rows{display:grid;gap:8px;margin-top:10px}.row{display:grid;grid-template-columns:90px minmax(0,1fr) 168px;gap:10px;align-items:center;border-top:1px solid var(--line);padding-top:8px;font-size:13px}.row a{color:var(--accent);text-decoration:none}.tag{font-weight:700}.recordTitle{font-size:15px;font-weight:700}.recordActions{display:grid;gap:6px}.recordActions a,.recordActions button,.recordActions span{width:100%;text-align:center}.recordActions a,.recordActions span{padding:7px 8px;border:1px solid var(--line);border-radius:7px}.ok{color:var(--ok)}.badText{color:var(--bad)}.muted{color:var(--muted)}.logRow{grid-template-columns:120px 1fr 110px}"
"@media(max-width:880px){.view{grid-template-columns:1fr}.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.row{grid-template-columns:1fr}.recordActions{display:flex;flex-wrap:wrap}.recordActions a,.recordActions button,.recordActions span{width:auto;flex:1 1 140px}}"
"</style></head><body><main>"
"<section class=\"top\"><div><div class=\"title\">ESP32-P4 摄像头控制</div><div class=\"sub\"><span id=\"ip\">--</span> | <span id=\"mode\">--</span></div></div>"
"<div class=\"actions\"><label class=\"toggle\"><input id=\"visionToggle\" type=\"checkbox\" onchange=\"setVision(this.checked)\"><span>识别 Vision</span></label><select id=\"methodSelect\" onchange=\"setMethod(this.value)\"><option value=\"off\">关闭 Off</option><option value=\"mlp\">MLP</option><option value=\"coco\">COCO YOLO11n 320</option><option value=\"yolo26\">YOLO26 Coke/Sprite</option><option value=\"yolo11\">YOLO11 Coke/Sprite</option></select><select id=\"netSelect\" onchange=\"setNetMode(this.value)\"><option value=\"sta\">STA</option><option value=\"softap\">热点 SoftAP</option><option value=\"apsta\">AP+STA</option></select><button id=\"wakeBtn\" onclick=\"cmd('wake')\">唤醒 Wake</button><button id=\"standbyBtn\" onclick=\"cmd('standby')\">待机 Standby</button><a href=\"/validate\">手机验证</a></div></section>"
"<section class=\"view\"><div class=\"video\"><div class=\"videoFrame\"><img id=\"stream\" src=\"/stream\" alt=\"camera stream\"><div id=\"boxes\" class=\"boxes\"></div></div></div>"
"<aside><div class=\"grid\">"
"<div class=\"card\"><div class=\"label\">采集帧率 Capture FPS</div><div class=\"value\" id=\"cfps\">--</div></div>"
"<div class=\"card\"><div class=\"label\">传输帧率 Send FPS</div><div class=\"value\" id=\"sfps\">--</div></div>"
"<div class=\"card\"><div class=\"label\">客户端 Clients</div><div class=\"value\" id=\"clients\">--</div></div>"
"<div class=\"card\"><div class=\"label\">帧年龄 Frame Age</div><div class=\"value\" id=\"age\">--</div></div>"
"<div class=\"card\"><div class=\"label\">采集延时 Latency</div><div class=\"value\" id=\"lat\">--</div></div>"
"<div class=\"card\"><div class=\"label\">推理延时 Inference</div><div class=\"value\" id=\"infer\">--</div></div>"
"<div class=\"card\"><div class=\"label\">图像大小 JPEG</div><div class=\"value\" id=\"jpg\">--</div></div>"
"<div class=\"card\"><div class=\"label\">运动 Motion</div><div class=\"value\" id=\"motion\">--</div></div>"
"<div class=\"card\"><div class=\"label\">场景 Scene</div><div class=\"value\" id=\"scene\">--</div></div>"
"<div class=\"card wide\"><div class=\"label\">板端识别 Vision</div><div class=\"value\" id=\"vision\">--</div><div class=\"sub\" id=\"vision2\"></div></div>"
"<div class=\"card wide\"><div class=\"label\">无线与实验 Link Lab</div><div class=\"sub\" id=\"netlab\">--</div></div>"
"<div class=\"card wide\"><div class=\"label\">存储 Storage</div><div class=\"sub\" id=\"storage\">--</div></div>"
"<div class=\"card wide\"><div class=\"label\">系统 System</div><div class=\"sub\" id=\"sys\">--</div><div class=\"sub\" id=\"err\"></div></div>"
"</div></aside></section>"
"<section class=\"history\"><div class=\"histHead\"><div><div class=\"title small\">调试参数 Debug Config</div><div class=\"sub\">这些参数会写入 NVS，重启后继续生效。</div></div><button onclick=\"applyConfig()\">应用 Apply</button></div>"
"<div class=\"actions\"><label>框阈值 <input id=\"boxMin\" type=\"number\" min=\"50\" max=\"100\"></label><label>推流FPS <input id=\"streamFps\" type=\"number\" min=\"1\" max=\"30\"></label><label>推理间隔ms <input id=\"infMs\" type=\"number\" min=\"0\" max=\"600000\"></label><label>历史间隔ms <input id=\"histMs\" type=\"number\" min=\"250\" max=\"600000\"></label><label>JPEG质量 <input id=\"jpegQ\" type=\"number\" min=\"1\" max=\"100\"></label><label class=\"toggle\"><input id=\"historyToggle\" type=\"checkbox\"><span>历史 History</span></label><label class=\"toggle\"><input id=\"recordingToggle\" type=\"checkbox\"><span>录像 Recording</span></label></div><div class=\"sub\" id=\"configMsg\">--</div></section>"
"<section class=\"history\"><div class=\"histHead\"><div><div class=\"title small\">监控记录 Timeline</div><div class=\"sub\" id=\"timelineMeta\">--</div></div><div class=\"actions\"><button onclick=\"loadTimeline()\">刷新</button><button onclick=\"startVideoValidation()\">视频验证</button><button onclick=\"remountTf()\">TF重挂载</button><button onclick=\"formatTf()\">格式化TF</button><button onclick=\"clearRecords()\">清空记录</button></div></div><div class=\"sub\" id=\"datasetMeta\">--</div><div id=\"timelineList\" class=\"rows\"></div></section>"
"<section class=\"history\"><div class=\"histHead\"><div><div class=\"title small\">实验日志 Experiment Log</div><div class=\"sub\">自动记录识别方法、无线模式、RTT、RSSI、内存和图传状态，便于对比稳定性。</div></div><button onclick=\"experimentLog=[];renderExperimentLog()\">清空 Clear</button></div><div id=\"experimentList\" class=\"rows\"></div></section>"
"</main><script>"
"let lastMode='',experimentLog=[],lastLogKey='',lastLogAt=0,timeSyncAttempted=false;function esc(v){return String(v==null?'':v).replace(/[&<>\"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[m]))}"
"function keepInput(el,val){if(document.activeElement!==el)el.value=val}"
"function streamUrl(){return '/stream?ts='+Date.now()}function setMode(m){wakeBtn.className=m==='running'?'active':'';standbyBtn.className=m==='standby'?'active':''}"
"async function load(){try{const t=performance.now();let r=await fetch('/api/status?ts='+Date.now(),{cache:'no-store'});let s=await r.json();paint(s,Math.round(performance.now()-t))}catch(e){err.textContent='status offline'}}"
"function cname(o){return o==='coke'?'可乐 Coke':(o==='sprite'?'雪碧 Sprite':(!o||o==='unknown'?'未知 Unknown':o))}"
"function paintObject(s){let v=s.vision||{},ds=v.detections||[];boxes.innerHTML='';if(!ds.length&&v.object_count>0)ds=[{label:v.object,score:v.object_score,x:v.object_x,y:v.object_y,w:v.object_w,h:v.object_h}];if(!s.width||!s.height)return;ds.forEach(d=>{if(!d.w||!d.h)return;let b=document.createElement('div');b.className='bbox';b.style.left=(100*d.x/s.width)+'%';b.style.top=(100*d.y/s.height)+'%';b.style.width=(100*d.w/s.width)+'%';b.style.height=(100*d.h/s.height)+'%';let sp=document.createElement('span');sp.textContent=cname(d.label)+' '+d.score;b.appendChild(sp);boxes.appendChild(b)})}"
"function paint(s,rtt){ip.textContent=s.mdns_url||s.eth_url||('http://'+s.ip+'/');mode.textContent=(s.app_mode||'server')+' | '+s.power_mode+' | '+s.network_mode+(s.rescue_ap?' + rescue AP':'')+' | '+s.recognition_method;setMode(s.power_mode);visionToggle.checked=!!s.vision_enabled;methodSelect.value=s.recognition_method||'mlp';netSelect.value=s.network_mode||'sta';historyToggle.checked=!!s.history_enabled;recordingToggle.checked=!!s.recording_enabled;keepInput(boxMin,s.config.box_min_score);keepInput(streamFps,s.config.stream_max_fps);keepInput(infMs,s.config.inference_interval_ms);keepInput(histMs,s.config.history_sample_interval_ms);keepInput(jpegQ,s.config.jpeg_quality);if(lastMode!=='running'&&s.power_mode==='running'){stream.src=streamUrl()}lastMode=s.power_mode;"
"cfps.textContent=(s.capture_fps_x100/100).toFixed(1);sfps.textContent=(s.stream_fps_x100/100).toFixed(1);clients.textContent=s.stream_clients+'/'+s.max_stream_clients;age.textContent=s.last_frame_age_ms+' ms';"
"lat.textContent=s.last_capture_ms+' ms';infer.textContent=(s.vision.inference_ms||0)+' ms';jpg.textContent=Math.round(s.last_jpeg_bytes/1024)+' KB';motion.textContent=s.vision.motion?'是 YES':'否 NO';scene.textContent=s.vision.scene;"
"vision.textContent=cname(s.vision.object)+' / '+s.vision.label;vision2.textContent='模型 '+s.vision.model+' | 模型大小 '+Math.round((s.model_info?.bytes||s.model_bytes||0)/1024)+' KB | 输入 '+(s.model_info?.input_size||s.config.yolo_input_size||0)+' | 检测框 '+(s.vision.detection_count||0)+'/'+(s.model_info?.max_detections||8)+' | raw候选 '+(s.vision.raw_candidate_count||0)+' | NMS '+(s.model_info?.nms_threshold||70)+' | 候选分 '+s.vision.candidate_score+' | 画框阈值 '+s.vision.box_min_score+' | 目标分 '+s.vision.object_score+' | 可乐 '+s.vision.coke_score+' | 雪碧 '+s.vision.sprite_score+' | 未知 '+s.vision.unknown_score+' | 运动 '+s.vision.motion_score+' | 边缘 '+s.vision.edge_score+' | 亮度 '+s.vision.avg_luma+' | 推理 '+s.vision.inference_ms+' ms | 分析 '+s.vision.analysis_ms+' ms';"
"netlab.textContent='模式 '+s.network_mode+' | 救援AP '+(s.rescue_ap?'开':'关')+' | STA '+s.sta_ip+' | AP '+s.ap_ip+' | AP客户端 '+s.ap_clients+' | 重连 '+s.reconnect_count+' | 推理FPS '+(s.inference_fps_x100/100).toFixed(1)+' | 推理忙 '+(s.inference_busy?'是':'否')+' | 队列 '+s.inference_queue_depth+' | 已完成 '+s.inference_jobs_completed+' | 丢弃推理帧 '+s.dropped_inference_frames+' | 队列丢弃 '+s.inference_queue_drops+' | 模型 '+Math.round(s.model_bytes/1024)+' KB | YOLO输入 '+s.config.yolo_input_size;"
"storage.textContent=(s.file_storage_mounted?'存储可用':'存储不可用')+' | 后端 '+(s.storage_backend||'none')+' | TF '+(s.tf_card_mounted?'已挂载':'未挂载')+' | '+s.storage_status+' | 存储窗口 '+((s.storage_service&&s.storage_service.mode)||'--')+' '+((s.storage_service&&s.storage_service.status)||'')+' | 模式 '+(s.sd_mount_mode||'--')+' | 错误 '+(s.sd_last_error||'--')+' | 历史 '+s.history_saved+' | 录像段 '+s.recording_segments+' | 录像帧 '+s.recording_frames+' | 当前 '+(s.recording_current_uri||'--')+' '+s.recording_current_frames+' 帧 | 视频验证 '+((s.dataset_run&&s.dataset_run.running)?('运行 '+s.dataset_run.processed+'/'+s.dataset_run.ok_frames):'空闲')+' | 剩余 '+Math.round(s.sd_free_bytes/1048576)+' MB';"
"sys.textContent='帧 '+s.frame_seq+' | 已发 '+s.stream_frames+' | 流量 '+Math.round(s.stream_bytes/1024)+' KB | RSSI '+s.rssi_dbm+' dBm | 网页RTT '+rtt+' ms | 堆 '+Math.round(s.free_heap/1024)+'/'+Math.round(s.min_free_heap/1024)+' KB | PSRAM '+Math.round(s.free_psram/1024)+'/'+Math.round(s.min_free_psram/1024)+' KB';"
"err.textContent=(s.camera_error||'')+' | time '+(s.time_source||'unsynced');paintObject(s);appendExperiment(s,rtt);syncBrowserClock(s)}"
"async function cmd(c){await fetch('/api/power?cmd='+c,{cache:'no-store'}).catch(()=>{});if(c==='standby'){lastMode='standby';stream.removeAttribute('src')}if(c==='wake'){setTimeout(()=>{stream.src=streamUrl()},800)}setTimeout(load,150)}"
"async function setVision(on){await fetch('/api/vision?enabled='+(on?1:0),{cache:'no-store'}).catch(()=>{});setTimeout(load,150);setTimeout(loadHistory,250)}"
"async function setMethod(m){await fetch('/api/recognition?method='+encodeURIComponent(m),{cache:'no-store'}).catch(()=>{});setTimeout(load,150)}"
"async function setNetMode(m){await fetch('/api/netmode?mode='+encodeURIComponent(m),{cache:'no-store'}).catch(()=>{});setTimeout(load,500);setTimeout(load,2500)}"
"async function applyConfig(){let q=new URLSearchParams();q.set('box_min_score',boxMin.value);q.set('stream_max_fps',streamFps.value);q.set('inference_interval_ms',infMs.value);q.set('history_sample_interval_ms',histMs.value);q.set('jpeg_quality',jpegQ.value);q.set('history',historyToggle.checked?1:0);q.set('recording',recordingToggle.checked?1:0);let r=await fetch('/api/config?'+q.toString(),{cache:'no-store'}).catch(()=>null);configMsg.textContent=r?'已应用，新的推流FPS会在下次打开 /stream 时生效':'配置失败';setTimeout(load,150);setTimeout(loadTimeline,400)}"
"function labelSummary(labels){return (labels||[]).map(x=>esc(x.label)+' '+x.count).join('，')||'无目标'}"
"function fmtBytes(v){v=Number(v)||0;if(v>=1048576)return (v/1048576).toFixed(1)+' MB';if(v>=1024)return Math.round(v/1024)+' KB';return v+' B'}"
"function fmtBoot(ms){ms=Math.max(0,Number(ms)||0);let s=Math.floor(ms/1000),h=Math.floor(s/3600),m=Math.floor((s%3600)/60);return '本次启动 + '+String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s%60).padStart(2,'0')}"
"function recordingTime(d){if(Number(d.start_epoch_ms)>0){let a=new Date(Number(d.start_epoch_ms)),b=new Date(Number(d.end_epoch_ms||d.start_epoch_ms));return a.toLocaleString()+' - '+b.toLocaleTimeString()}return fmtBoot(d.start_ms)}"
"function recordingTitle(d){return '监控录像 · '+(Number(d.start_epoch_ms)>0?new Date(Number(d.start_epoch_ms)).toLocaleString():fmtBoot(d.start_ms))}"
"function recordingBoot(d){let m=String(d.name||'').match(/_b([0-9a-f]+)_/i);return m?m[1]:''}"
"function recordingBounds(d){let s=Number(d.start_ms)||0,e=Number(d.end_ms)||0;if(e<=s)e=s+(Number(d.duration_ms)||0);return {start:s,end:e,duration:Math.max(0,e-s)}}"
"function pairRecordings(records){let raws=(records||[]).filter(x=>x.kind==='raw').sort((a,b)=>(Number(a.start_ms)||0)-(Number(b.start_ms)||0)),anns=(records||[]).filter(x=>x.kind==='annotated'),used=new Set(),pairs=[];raws.forEach(raw=>{let rb=recordingBounds(raw),boot=recordingBoot(raw),best=null;anns.forEach(ann=>{if(used.has(ann.name)||!boot||recordingBoot(ann)!==boot)return;let ab=recordingBounds(ann),shorter=Math.min(rb.duration,ab.duration);if(shorter<=0)return;let overlap=Math.max(0,Math.min(rb.end,ab.end)-Math.max(rb.start,ab.start)),ratio=overlap/shorter,delta=Math.abs(rb.start-ab.start);if(ratio<.5)return;if(!best||ratio>best.ratio||(ratio===best.ratio&&delta<best.delta))best={ann:ann,ratio:ratio,delta:delta}});if(best){used.add(best.ann.name);pairs.push({raw:raw,annotated:best.ann})}else pairs.push({raw:raw,annotated:null})});anns.forEach(ann=>{if(!used.has(ann.name))pairs.push({raw:null,annotated:ann})});return pairs.map(p=>{let d=p.raw||p.annotated;return {type:'recording_pair',data:p,sort_time:Number(d.start_epoch_ms)||Number(d.start_ms)||0}})}"
"function recordingDownload(d,label){return d?'<a href=\"'+esc(d.uri)+'\" download>'+label+'</a>':'<span class=\"muted\">暂无'+label.replace('下载','')+'</span>'}"
"function rowForRecordingPair(p){let raw=p.raw,ann=p.annotated,base=raw||ann,stats=ann||raw,duration=Math.round((Number((raw||ann).duration_ms)||0)/1000),rawInfo=raw?fmtBytes(raw.bytes)+' · '+(raw.frames||0)+' 帧':'暂无',annInfo=ann?fmtBytes(ann.bytes)+' · '+(ann.frames||0)+' 帧':'暂无',primary=raw||ann,secondary=raw&&ann?ann:null;return '<div class=\"row\"><div class=\"tag\">监控录像</div><div><div class=\"recordTitle\">'+esc(recordingTitle(base))+'</div><div class=\"sub\">'+esc(recordingTime(base))+' | 时长 '+duration+' s</div><div class=\"sub\">原始视频 '+rawInfo+' | 标注视频 '+annInfo+'</div><div class=\"sub\">命中帧 '+(stats.hit_frames||0)+' | 检测 '+(stats.detection_total||0)+' | '+labelSummary(stats.labels)+'</div></div><div class=\"recordActions\">'+recordingDownload(raw,'下载原始视频')+recordingDownload(ann,'下载标注视频')+'<button onclick=\"deleteRecording(\\''+esc(primary.name)+'\\',\\''+esc(secondary?secondary.name:'')+'\\')\">删除此录像</button></div></div>'}"
"async function syncBrowserClock(s){if(timeSyncAttempted)return;let board=Number(s.epoch_ms)||0;if(!s.time_synced||Math.abs(Date.now()-board)>300000){timeSyncAttempted=true;await fetch('/api/time/sync?epoch_ms='+Date.now(),{method:'POST',cache:'no-store'}).catch(()=>{timeSyncAttempted=false})}}"
"function rowForTimeline(x){if(x.type==='recording_pair')return rowForRecordingPair(x.data);let d=x.data||{},ds=d.detections||[],det=ds.length?ds.map(y=>cname(y.label)+' '+y.score+'%').join('，'):'无正式框',snap=d.snapshot?'<a href=\"'+esc(d.snapshot)+'\" target=\"_blank\">快照</a>':'';return '<div class=\"row\"><div class=\"tag\">识别</div><div>'+esc(d.source||'camera')+' / '+esc(d.recognition_method)+'<div class=\"sub\">'+det+' | 检测 '+(d.detection_count||0)+' | 推理 '+(d.inference_ms||0)+' ms | 分析 '+(d.analysis_ms||0)+' ms</div></div><div>'+snap+'</div></div>'}"
"async function loadTimeline(){try{let ts=Date.now(),rs=await Promise.all([fetch('/api/timeline?limit=50&ts='+ts,{cache:'no-store'}),fetch('/api/recordings?limit=100&ts='+ts,{cache:'no-store'})]),h=await rs[0].json(),rec=await rs[1].json(),pairs=pairRecordings(rec.recordings||[]),events=(h.timeline||[]).filter(x=>x.type!=='summary'&&x.type!=='recording').map(x=>{let d=x.data||{};x.sort_time=Number(d.epoch_ms)||Number(d.time_ms)||0;return x}),rows=events.concat(pairs).sort((a,b)=>(b.sort_time||0)-(a.sort_time||0)).slice(0,50);timelineMeta.textContent='TF '+(h.sd_mounted?'已挂载':'未挂载')+' | '+h.storage_status+' | 历史 '+h.history_saved+' | 监控录像 '+pairs.length;timelineList.innerHTML=rows.map(rowForTimeline).join('')||'<div class=\"row\"><div class=\"muted\">暂无监控记录</div></div>'}catch(e){timelineMeta.textContent='监控记录离线'}}"
"async function deleteRecording(n,p){if(!confirm('删除这段监控录像的原始视频、标注视频及全部索引？'))return;let q=new URLSearchParams({name:n,confirm:'DELETE'});if(p)q.set('paired_name',p);let r=await fetch('/api/recording?'+q.toString(),{method:'DELETE',cache:'no-store'}).catch(()=>null);let j=r?await r.json().catch(()=>null):null;if(!r||!r.ok||!j||!j.ok){datasetMeta.textContent='删除失败：'+(j&&j.error?j.error:(r?'HTTP '+r.status:'网络错误'));return}datasetMeta.textContent='已删除 '+j.deleted_files+' 个文件，清理 '+j.removed_index_rows+' 条索引，释放 '+fmtBytes(j.freed_bytes);await loadTimeline()}"
"async function clearRecords(){if(!confirm('清空全部监控记录？'))return;let r=await fetch('/api/storage/records?scope=all&confirm=DELETE',{method:'DELETE',cache:'no-store'}).catch(()=>null);let j=r?await r.json().catch(()=>null):null;if(!r||!r.ok||!j||!j.ok){datasetMeta.textContent='清空失败：'+(r?'HTTP '+r.status:'网络错误');return}datasetMeta.textContent='已清理 '+j.deleted_files+' 个文件，释放 '+fmtBytes(j.freed_bytes);await loadTimeline()}"
"async function remountTf(){if(!confirm('将关闭热点/网页并重新挂载TF卡，维护后板子会自动重启恢复热点，确认？'))return;let r=await fetch('/api/storage/remount?confirm=REMOUNT&hold_ms=2000',{method:'POST',cache:'no-store'}).catch(()=>null);datasetMeta.textContent=r?await r.text():'TF重挂载请求失败';}"
"async function formatTf(){if(!confirm('格式化TF卡会清空卡内数据；若TF未挂载，板子会进入维护窗口并自动重启，确认？'))return;let r=await fetch('/api/storage/format?confirm=FORMAT',{method:'POST',cache:'no-store'}).catch(()=>null);datasetMeta.textContent=r?await r.text():'格式化请求失败';setTimeout(loadTimeline,800)}"
"async function startVideoValidation(){let r=await fetch('/api/dataset/run/start?dataset=coco_video_demo&limit=16&stride=1',{cache:'no-store'}).catch(()=>null);datasetMeta.textContent=r?await r.text():'视频验证启动失败';setTimeout(checkVideoValidation,500)}"
"async function checkVideoValidation(){try{let r=await fetch('/api/dataset/run/status?ts='+Date.now(),{cache:'no-store'});let s=await r.json();datasetMeta.textContent='视频验证 '+(s.state||'idle')+' | '+(s.dataset||'--')+' | 帧 '+s.processed+'/'+s.limit+' | ok '+s.ok_frames+' | 失败 '+s.failed_frames+' | 平均 '+s.avg_analysis_ms+' ms | P95 '+s.p95_analysis_ms+' ms | '+labelSummary(s.labels)+' | '+(s.error||'');if(s.queued||s.running)setTimeout(checkVideoValidation,1000)}catch(e){datasetMeta.textContent='视频验证状态离线'}}"
"function appendExperiment(s,rtt){let now=Date.now();let key=s.network_mode+'|'+s.recognition_method+'|'+s.power_mode+'|'+s.stream_clients+'|'+Math.floor(now/5000);if(key===lastLogKey&&now-lastLogAt<5000)return;lastLogKey=key;lastLogAt=now;experimentLog.unshift({t:new Date().toLocaleTimeString(),mode:s.network_mode,method:s.recognition_method,power:s.power_mode,rtt:rtt,rssi:s.rssi_dbm,cfps:s.capture_fps_x100,sfps:s.stream_fps_x100,heap:s.free_heap,psram:s.free_psram,clients:s.stream_clients,ap:s.ap_clients,err:s.stream_errors,drop:s.dropped_inference_frames,inf:s.vision?s.vision.inference_ms:0});if(experimentLog.length>24)experimentLog.pop();renderExperimentLog()}"
"function renderExperimentLog(){experimentList.innerHTML=experimentLog.map(x=>'<div class=\"row logRow\"><div class=\"tag\">'+esc(x.t)+'</div><div>'+esc(x.mode)+' / '+esc(x.method)+' / '+esc(x.power)+'<div class=\"sub\">RTT '+x.rtt+' ms | RSSI '+x.rssi+' dBm | Capture '+(x.cfps/100).toFixed(1)+' FPS | Send '+(x.sfps/100).toFixed(1)+' FPS | 推理 '+x.inf+' ms | heap '+Math.round(x.heap/1024)+' KB | PSRAM '+Math.round(x.psram/1024)+' KB</div></div><div>流 '+x.clients+' / AP '+x.ap+'</div></div>').join('')||'<div class=\"row\"><div class=\"muted\">等待状态采样</div></div>'}"
"setInterval(load,500);setInterval(loadTimeline,3000);setInterval(checkVideoValidation,5000);load();loadTimeline();checkVideoValidation();</script></body></html>";

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
"<div class=\"sub\">点击“板端推理”后，固件会把内嵌 JPEG 复制到 PSRAM 并投进 inference_task 队列；完成后返回 JSON 和带框 SVG。</div></div>"
"<a href=\"/\">返回首页</a></section>"
"<section class=\"panel\"><div class=\"actions\"><label>识别方法 <select id=\"method\"><option value=\"coco\">COCO YOLO11n 320 fast</option></select></label><label>验证阈值 <input id=\"valBox\" type=\"number\" min=\"1\" max=\"100\" value=\"50\"></label><span class=\"sub\">COCO classic multi-object samples from val2017; Coke/Sprite APIs remain available only for historical comparison.</span></div></section>"
"<section class=\"panel\"><div class=\"title\">真实 COCO 视频验证</div><div class=\"actions\"><button id=\"videoStart\" onclick=\"startVideoVal()\">运行 16 帧板端推理</button><button id=\"videoPlay\" onclick=\"toggleVideoPlayback()\" disabled>播放</button><span class=\"videoProgress\" id=\"videoProgress\">0 / 16</span><a href=\"/api/datasets\" target=\"_blank\">查看数据集</a></div><div class=\"sub\" id=\"videoVal\">固件内置商店过道连续视频帧；运行时逐帧显示板端标注结果，完成后以 1 FPS 循环播放。</div></section>"
"<section class=\"grid\">"
"<div class=\"card\"><div class=\"sample\"><img src=\"/validate/demo_01.jpg\" alt=\"COCO classic sample 1\"></div><div class=\"hint\">COCO 经典 01：多人、餐桌、杯子、碗、披萨<div class=\"actions\"><button class=\"valBtn\" onclick=\"runVal('demo_01')\">板端推理</button><a href=\"/validate/demo_01.jpg\" target=\"_blank\">打开原图</a></div></div></div>"
"<div class=\"card\"><div class=\"sample\"><img src=\"/validate/demo_02.jpg\" alt=\"COCO classic sample 2\"></div><div class=\"hint\">COCO 经典 02：多人、椅子、瓶子、杯子、餐桌<div class=\"actions\"><button class=\"valBtn\" onclick=\"runVal('demo_02')\">板端推理</button><a href=\"/validate/demo_02.jpg\" target=\"_blank\">打开原图</a></div></div></div>"
"<div class=\"card\"><div class=\"sample\"><img src=\"/validate/demo_03.jpg\" alt=\"COCO classic sample 3\"></div><div class=\"hint\">COCO 经典 03：多人、椅子、瓶子、餐桌、雨伞<div class=\"actions\"><button class=\"valBtn\" onclick=\"runVal('demo_03')\">板端推理</button><a href=\"/validate/demo_03.jpg\" target=\"_blank\">打开原图</a></div></div></div>"
"<div class=\"card\"><div class=\"sample\"><img src=\"/validate/demo_04.jpg\" alt=\"COCO classic sample 4\"></div><div class=\"hint\">COCO 经典 04：多人、雨伞、碗、杯子、餐桌<div class=\"actions\"><button class=\"valBtn\" onclick=\"runVal('demo_04')\">板端推理</button><a href=\"/validate/demo_04.jpg\" target=\"_blank\">打开原图</a></div></div></div>"
"</section>"
"<section class=\"result\"><div class=\"boxed\"><img id=\"boxed\" alt=\"boxed validation result\"><div id=\"empty\" class=\"sub\">等待点击样例图进行板端推理</div></div><aside class=\"panel\"><div class=\"title\">推理结果</div><div class=\"kv\" id=\"summary\"></div><pre id=\"raw\"></pre></aside></section>"
"</main><script>"
"function esc(v){return String(v==null?'':v).replace(/[&<>\"]/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[m]))}"
"function cname(o){return o==='coke'?'可乐 Coke':(o==='sprite'?'雪碧 Sprite':(!o||o==='unknown'?'未知 Unknown':o))}"
"function detRows(ds){if(!ds||!ds.length)return '<div class=\"row\">正式检测框：0</div>';return '<div class=\"row\">正式检测框 '+ds.length+'</div>'+ds.map((d,i)=>'<div class=\"row\"><b>#'+(i+1)+'</b> '+cname(d.label)+' '+d.score+'% <span class=\"sub\">x='+d.x+' y='+d.y+' w='+d.w+' h='+d.h+'</span></div>').join('')}"
"let validationBusy=false,validationSeq=0,videoBusy=false,videoRunId='',videoDataset='coco_video_demo',videoFrames=[],videoFrameIndex=0,videoTimer=null,videoPollToken=0;"
"function setBusy(b){validationBusy=b;document.querySelectorAll('.valBtn').forEach(x=>x.disabled=b||videoBusy);method.disabled=b||videoBusy;valBox.disabled=b||videoBusy;videoStart.disabled=b||videoBusy;videoPlay.disabled=b||videoBusy||!videoFrames.some(Boolean)}"
"function setVideoBusy(b){videoBusy=b;document.querySelectorAll('.valBtn').forEach(x=>x.disabled=b||validationBusy);method.disabled=b||validationBusy;valBox.disabled=b||validationBusy;videoStart.disabled=b||validationBusy;videoPlay.disabled=b||!videoFrames.some(Boolean)}"
"function stopVideoPlayback(){if(videoTimer){clearInterval(videoTimer);videoTimer=null}videoPlay.textContent='播放'}"
"function clearVideoFrames(){stopVideoPlayback();videoFrames.forEach(x=>{if(x)URL.revokeObjectURL(x)});videoFrames=[];videoFrameIndex=0;videoProgress.textContent='0 / 16';videoPlay.disabled=true}"
"function showVideoFrame(i){if(!videoFrames.length)return;let n=videoFrames.length;for(let step=0;step<n;step++){let k=(i+step+n)%n;if(videoFrames[k]){videoFrameIndex=k;boxed.src=videoFrames[k];empty.style.display='none';videoProgress.textContent=(k+1)+' / '+n;return}}}"
"function startVideoPlayback(){if(!videoFrames.some(Boolean))return;stopVideoPlayback();showVideoFrame(videoFrameIndex);videoPlay.textContent='暂停';videoTimer=setInterval(()=>showVideoFrame(videoFrameIndex+1),1000)}"
"function toggleVideoPlayback(){if(videoTimer)stopVideoPlayback();else startVideoPlayback()}"
"function videoOverlayUri(index){return '/api/dataset/frame.svg?run_id='+encodeURIComponent(videoRunId)+'&dataset='+encodeURIComponent(videoDataset)+'&index='+index+'&ts='+Date.now()}"
"async function ensureVideoFrame(index,token){let slot=index-1;if(videoFrames[slot])return videoFrames[slot];let r=await fetch(videoOverlayUri(index),{cache:'no-store'});if(!r.ok)throw new Error('overlay '+index+' HTTP '+r.status);let blob=await r.blob();if(token!==videoPollToken)return '';let url=URL.createObjectURL(blob);videoFrames[slot]=url;return url}"
"async function runVal(sample){if(validationBusy||videoBusy)return;stopVideoPlayback();let seq=++validationSeq;setBusy(true);summary.innerHTML='<div class=\"row\">正在把 '+sample+' 图片送入板端推理队列，请等待，本次完成前按钮会暂时锁定。</div>';raw.textContent='';boxed.removeAttribute('src');empty.style.display='block';let m=method.value;let threshold=Math.max(1,Math.min(100,Number(valBox.value)||50));let t=performance.now();try{let r=await fetch('/api/validate/run?sample='+encodeURIComponent(sample)+'&method='+encodeURIComponent(m)+'&box_min_score='+encodeURIComponent(threshold)+'&ts='+Date.now(),{cache:'no-store'});let j=await r.json();let dt=Math.round(performance.now()-t);if(seq!==validationSeq)return;if(!j.ok){summary.innerHTML='<div class=\"row bad\">验证失败：'+esc(j.error||r.status)+'</div>';raw.textContent=JSON.stringify(j,null,2);return}empty.style.display='none';boxed.src=j.overlay+'?id='+encodeURIComponent(j.id)+'&ts='+Date.now();let v=j.vision||{};let ds=v.detections||j.detections||[];summary.innerHTML='<div class=\"row '+(j.matched?'ok':'bad')+'\">期望 '+esc(j.expected)+'，识别 '+cname(v.object)+'，'+(j.matched?'命中':'未命中')+'</div>'+'<div class=\"row\">方法 '+esc(j.method)+' | 模型 '+esc(v.model)+' | 模型 '+Math.round((j.model_bytes||0)/1024)+' KB | 输入 '+(j.model_input_size||0)+'</div>'+'<div class=\"row\">候选分 '+v.candidate_score+' | 验证阈值 '+v.box_min_score+' | NMS '+j.nms_threshold+' | raw候选 '+j.raw_candidate_count+'</div>'+'<div class=\"row\">推理 '+v.inference_ms+' ms | 分析 '+v.analysis_ms+' ms | 网页等待 '+dt+' ms</div>'+'<div class=\"row\">原图 '+j.source_w+'x'+j.source_h+' | JPEG '+Math.round(j.jpeg_bytes/1024)+' KB</div>'+detRows(ds);raw.textContent=JSON.stringify(j,null,2)}catch(e){if(seq===validationSeq){summary.innerHTML='<div class=\"row bad\">验证请求失败</div>';raw.textContent=String(e)}}finally{if(seq===validationSeq)setBusy(false)}}"
"function labelSummary(labels){return (labels||[]).map(x=>esc(x.label)+' '+x.count).join('，')||'无目标'}"
"async function startVideoVal(){if(videoBusy||validationBusy)return;clearVideoFrames();videoPollToken++;let token=videoPollToken;setVideoBusy(true);boxed.removeAttribute('src');empty.style.display='block';empty.textContent='正在等待第一帧板端推理结果';summary.innerHTML='<div class=\"row\">正在启动真实 COCO 视频验证...</div>';raw.textContent='';try{let r=await fetch('/api/dataset/run/start?dataset='+encodeURIComponent(videoDataset)+'&limit=16&stride=1',{cache:'no-store'});let text=await r.text();let j={};try{j=JSON.parse(text)}catch(e){}if(!r.ok||!j.ok)throw new Error(j.error||text||('HTTP '+r.status));videoRunId=j.run_id;videoVal.textContent='已排队 '+videoRunId;setTimeout(()=>pollVideoVal(token),200)}catch(e){videoVal.textContent='启动失败：'+e;summary.innerHTML='<div class=\"row bad\">视频验证启动失败</div>';setVideoBusy(false)}}"
"async function preloadVideoFrames(s,token){for(let i=0;i<s.processed;i++){if(token!==videoPollToken)return;let index=1+i*(s.stride||1);await ensureVideoFrame(index,token)}if(token!==videoPollToken)return;videoPlay.disabled=false;videoFrameIndex=0;startVideoPlayback()}"
"async function pollVideoVal(token){if(token!==videoPollToken||!videoRunId)return;try{let r=await fetch('/api/dataset/run/status?ts='+Date.now(),{cache:'no-store'});let s=await r.json();if(s.run_id!==videoRunId)throw new Error('run_id mismatch');let links=(!s.running&&s.result_uri)?' | <a href=\"'+esc(s.result_uri)+'\" target=\"_blank\">JSONL</a> | <a href=\"'+esc(s.summary_uri)+'\" target=\"_blank\">summary</a>':'';videoVal.innerHTML='状态 '+esc(s.state||'idle')+' | 帧 '+s.processed+'/'+s.limit+' | ok '+s.ok_frames+' | 失败 '+s.failed_frames+' | 平均 '+s.avg_analysis_ms+' ms | P95 '+s.p95_analysis_ms+' ms | 最大 '+s.max_analysis_ms+' ms | 检测 '+s.detection_total+' | '+labelSummary(s.labels)+' | '+esc(s.error||'')+links;videoProgress.textContent=s.processed+' / '+s.limit;summary.innerHTML='<div class=\"row '+(s.failed_frames?'bad':'ok')+'\">真实视频板端推理 '+s.ok_frames+'/'+s.limit+'</div><div class=\"row\">检测 '+s.detection_total+' | '+labelSummary(s.labels)+'</div><div class=\"row\">平均 '+s.avg_analysis_ms+' ms | P95 '+s.p95_analysis_ms+' ms | 最大 '+s.max_analysis_ms+' ms</div>';raw.textContent=JSON.stringify(s,null,2);if(s.last_frame_index>0){let url=await ensureVideoFrame(s.last_frame_index,token);if(url&&token===videoPollToken){boxed.src=url;empty.style.display='none';videoProgress.textContent=s.last_frame_index+' / '+s.limit}}if(s.queued||s.running){setTimeout(()=>pollVideoVal(token),400);return}if(s.done){await preloadVideoFrames(s,token);setVideoBusy(false);return}setVideoBusy(false)}catch(e){if(token===videoPollToken){videoVal.textContent='视频验证状态错误：'+e;summary.innerHTML='<div class=\"row bad\">视频验证失败</div>';setVideoBusy(false)}}}"
"</script></body></html>";

extern const uint8_t validate_coke_jpg_start[] asm("_binary_coke_01_jpg_start");
extern const uint8_t validate_coke_jpg_end[] asm("_binary_coke_01_jpg_end");
extern const uint8_t validate_sprite_jpg_start[] asm("_binary_sprite_01_jpg_start");
extern const uint8_t validate_sprite_jpg_end[] asm("_binary_sprite_01_jpg_end");
extern const uint8_t validate_demo_01_jpg_start[] asm("_binary_demo_01_jpg_start");
extern const uint8_t validate_demo_01_jpg_end[] asm("_binary_demo_01_jpg_end");
extern const uint8_t validate_demo_02_jpg_start[] asm("_binary_demo_02_jpg_start");
extern const uint8_t validate_demo_02_jpg_end[] asm("_binary_demo_02_jpg_end");
extern const uint8_t validate_demo_03_jpg_start[] asm("_binary_demo_03_jpg_start");
extern const uint8_t validate_demo_03_jpg_end[] asm("_binary_demo_03_jpg_end");
extern const uint8_t validate_demo_04_jpg_start[] asm("_binary_demo_04_jpg_start");
extern const uint8_t validate_demo_04_jpg_end[] asm("_binary_demo_04_jpg_end");

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
 * - 当前固件加载的模型文件见 main/CMakeLists.txt，均为本工程训练并量化的 Coke/Sprite 模型。
 * - C++ 桥接层负责 JPEG/RGB 解码、letterbox 预处理、ESP-DL 推理、NMS 后处理和坐标映射。
 * - C 主程序只关心统一的 vision_result_t，这样网页、历史记录和 API 不需要知道底层是 MLP、YOLO26 还是 YOLO11。
 */
static bool recognition_method_is_yolo(recognition_method_t method)
{
    return method == RECOGNITION_METHOD_YOLO26 ||
           method == RECOGNITION_METHOD_YOLO11 ||
           method == RECOGNITION_METHOD_COCO;
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
     * YOLO26 和 YOLO11 都已经替换为自训练 Coke/Sprite 两类模型。
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

static void classify_yolo26_jpeg(const uint8_t *jpeg, uint32_t jpeg_size, vision_result_t *result)
{
    yolo26_espdl_result_t det = {0};
    vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
    fill_yolo_pending(result, RECOGNITION_METHOD_YOLO26);

    esp_err_t err = yolo26_espdl_detect_jpeg(jpeg, jpeg_size, &det);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "YOLO26 inference failed: %s", esp_err_to_name(err));
        strlcpy(result->label, "yolo26-error", sizeof(result->label));
        result->unknown_score = 100;
        return;
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
                                det.source_w, det.source_h, s_box_min_score, false, result);
}

static void classify_yolo11_jpeg(const uint8_t *jpeg, uint32_t jpeg_size, vision_result_t *result)
{
    yolo11_espdl_result_t det = {0};
    vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
    fill_yolo_pending(result, RECOGNITION_METHOD_YOLO11);

    esp_err_t err = yolo11_espdl_detect_jpeg(jpeg, jpeg_size, &det);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "YOLO11 inference failed: %s", esp_err_to_name(err));
        strlcpy(result->label, "yolo11-error", sizeof(result->label));
        result->unknown_score = 100;
        return;
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
                                det.source_w, det.source_h, s_box_min_score, false, result);
}

static void classify_coco_jpeg(const uint8_t *jpeg, uint32_t jpeg_size, vision_result_t *result)
{
    coco_espdl_result_t det = {0};
    vision_detection_t candidates[APP_MAX_DETECTIONS] = {0};
    fill_yolo_pending(result, RECOGNITION_METHOD_COCO);

    esp_err_t err = coco_espdl_detect_jpeg(jpeg, jpeg_size, &det);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "COCO inference failed: %s", esp_err_to_name(err));
        strlcpy(result->label, "coco-error", sizeof(result->label));
        result->unknown_score = 100;
        return;
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
                                det.source_w, det.source_h, s_box_min_score, false, result);
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

static void update_stream_fps(int64_t now_ms)
{
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

    if (!vision || !s_frame_lock || !recognition_method_is_yolo(method)) {
        return false;
    }

    xSemaphoreTake(s_frame_lock, portMAX_DELAY);
    if (s_have_completed_yolo_vision &&
        s_last_completed_yolo_method == method &&
        is_completed_yolo_result(&s_last_completed_yolo_vision, method)) {
        *vision = s_last_completed_yolo_vision;
        ok = true;
    }
    xSemaphoreGive(s_frame_lock);
    return ok;
}

static void update_latest_vision_from_inference(const vision_result_t *vision,
                                                recognition_method_t method)
{
    if (!vision || !s_frame_lock || !recognition_method_is_yolo(method)) {
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
    if (is_completed_yolo_result(vision, method)) {
        s_last_completed_yolo_vision = *vision;
        s_last_completed_yolo_method = method;
        s_have_completed_yolo_vision = true;
    }
    xSemaphoreGive(s_frame_lock);
}

static void update_sd_info(void)
{
    if (!s_sd_mounted) {
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
        return;
    }

    if (esp_vfs_fat_info(CONFIG_APP_SD_MOUNT_POINT, &s_sd_total_bytes, &s_sd_free_bytes) != ESP_OK) {
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
    }
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
    return storage_tf_ready();
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
    struct stat st = {0};
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode) ? ESP_OK : ESP_FAIL;
    }

    if (mkdir(path, 0775) == 0 || errno == EEXIST) {
        return ESP_OK;
    }
    return ESP_FAIL;
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

static esp_err_t rotate_history_index_if_needed(void)
{
    struct stat st = {0};
    if (stat(HISTORY_JSONL_PATH, &st) != 0) {
        return ESP_OK;
    }

    const off_t max_bytes = (off_t)CONFIG_APP_HISTORY_INDEX_MAX_KB * 1024;
    if (st.st_size <= max_bytes) {
        return ESP_OK;
    }

    unlink(HISTORY_JSONL_OLD_PATH);
    if (rename(HISTORY_JSONL_PATH, HISTORY_JSONL_OLD_PATH) != 0) {
        s_history_sd_errors++;
        set_storage_status("history rotation failed: errno=%d", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static uint32_t count_snapshot_files(void)
{
    uint32_t count = 0;
    DIR *dir = opendir(HISTORY_SNAPSHOT_DIR);
    if (!dir) {
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (is_safe_snapshot_name(entry->d_name)) {
            count++;
        }
    }
    closedir(dir);
    return count;
}

static bool find_oldest_snapshot(char *name, size_t name_size)
{
    DIR *dir = opendir(HISTORY_SNAPSHOT_DIR);
    if (!dir) {
        return false;
    }

    bool found = false;
    time_t oldest_time = 0;
    char oldest_name[96] = {0};
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (!is_safe_snapshot_name(entry->d_name)) {
            continue;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, entry->d_name);
        struct stat st = {0};
        if (stat(path, &st) != 0 || S_ISDIR(st.st_mode)) {
            continue;
        }

        if (!found || st.st_mtime < oldest_time ||
            (st.st_mtime == oldest_time && strcmp(entry->d_name, oldest_name) < 0)) {
            found = true;
            oldest_time = st.st_mtime;
            strlcpy(oldest_name, entry->d_name, sizeof(oldest_name));
        }
    }

    closedir(dir);
    if (found) {
        strlcpy(name, oldest_name, name_size);
    }
    return found;
}

static void cleanup_old_snapshots(void)
{
    if (!s_sd_mounted || CONFIG_APP_HISTORY_MAX_SNAPSHOTS <= 0) {
        return;
    }

    uint32_t count = count_snapshot_files();
    while (count > CONFIG_APP_HISTORY_MAX_SNAPSHOTS) {
        char oldest[96] = {0};
        if (!find_oldest_snapshot(oldest, sizeof(oldest))) {
            break;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, oldest);
        if (unlink(path) == 0) {
            s_history_files_deleted++;
            count--;
        } else {
            s_history_sd_errors++;
            break;
        }
    }
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

static bool is_safe_dataset_relpath(const char *path)
{
    if (!path || !path[0] || strlen(path) >= DATASET_PATH_MAX || strstr(path, "..")) {
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

static void rotate_jsonl_if_needed(const char *path, const char *old_path)
{
    struct stat st = {0};
    if (!path || !old_path || stat(path, &st) != 0) {
        return;
    }

    const off_t max_bytes = (off_t)CONFIG_APP_HISTORY_INDEX_MAX_KB * 1024;
    if (st.st_size <= max_bytes) {
        return;
    }

    unlink(old_path);
    if (rename(path, old_path) != 0) {
        s_recording_sd_errors++;
        set_storage_status("jsonl rotation failed: errno=%d", errno);
    }
}

static uint32_t count_recording_files(void)
{
    uint32_t count = 0;
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (is_safe_recording_name(entry->d_name)) {
            count++;
        }
    }
    closedir(dir);
    return count;
}

static bool find_oldest_recording(char *name, size_t name_size)
{
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return false;
    }

    bool found = false;
    time_t oldest_time = 0;
    char oldest_name[96] = {0};
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (!is_safe_recording_name(entry->d_name)) {
            continue;
        }

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        struct stat st = {0};
        if (stat(path, &st) != 0 || S_ISDIR(st.st_mode)) {
            continue;
        }

        if (!found || st.st_mtime < oldest_time ||
            (st.st_mtime == oldest_time && strcmp(entry->d_name, oldest_name) < 0)) {
            found = true;
            oldest_time = st.st_mtime;
            strlcpy(oldest_name, entry->d_name, sizeof(oldest_name));
        }
    }

    closedir(dir);
    if (found) {
        strlcpy(name, oldest_name, name_size);
    }
    return found;
}

static void cleanup_old_recordings(void)
{
    if (!s_sd_mounted || CONFIG_APP_RECORDING_MAX_SEGMENTS <= 0) {
        return;
    }

    update_sd_info();
    uint32_t count = count_recording_files();
    uint64_t min_free = recording_min_free_bytes();
    while (count > CONFIG_APP_RECORDING_MAX_SEGMENTS ||
           (min_free > 0 && s_sd_free_bytes > 0 && s_sd_free_bytes < min_free)) {
        char oldest[96] = {0};
        if (!find_oldest_recording(oldest, sizeof(oldest))) {
            break;
        }

        char path[384];
        struct stat st = {0};
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, oldest);
        stat(path, &st);
        if (unlink(path) == 0) {
            char meta_name[96];
            char meta_path[384];
            meta_name_for_recording(oldest, meta_name, sizeof(meta_name));
            snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, meta_name);
            unlink(meta_path);
            bool index_failed = false;
            remove_recording_index_rows(oldest, &index_failed);
            if (index_failed) {
                s_recording_sd_errors++;
            }
            s_recording_files_deleted++;
            count--;
            if (st.st_size > 0) {
                if ((uint64_t)st.st_size > UINT64_MAX - s_sd_free_bytes) {
                    s_sd_free_bytes = UINT64_MAX;
                } else {
                    s_sd_free_bytes += (uint64_t)st.st_size;
                }
            }
            update_sd_info();
        } else {
            s_recording_sd_errors++;
            break;
        }
    }
}

static void label_count_add(label_count_t *labels, const char *label)
{
    if (!labels || !label || !label[0] || strcmp(label, "unknown") == 0) {
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

static void label_counts_to_json(char *buf, size_t size, const label_count_t *labels)
{
    size_t off = 0;
    off += snprintf(buf + off, size - off, "[");
    bool first = true;
    for (uint32_t i = 0; i < RECORDING_LABEL_BUCKETS && off < size; i++) {
        if (!labels[i].count) {
            continue;
        }
        off += snprintf(buf + off, size - off, "%s{\"label\":\"%s\",\"count\":%" PRIu32 "}",
                        first ? "" : ",", labels[i].label, labels[i].count);
        first = false;
    }
    snprintf(buf + off, size - off, "]");
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

    rotate_jsonl_if_needed(RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH);
    FILE *file = fopen(RECORDING_INDEX_PATH, "a");
    if (!file) {
        s_recording_sd_errors++;
        set_storage_status("open recording index failed: errno=%d", errno);
        return ESP_FAIL;
    }

    char labels[512];
    label_counts_to_json(labels, sizeof(labels), seg->labels);
    uint64_t end_epoch_ms = seg->start_epoch_ms;
    if (end_epoch_ms > 0 && seg->last_ms > seg->start_ms) {
        end_epoch_ms += (uint64_t)(seg->last_ms - seg->start_ms);
    }
    fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"kind\":\"%s\",\"container\":\"avi\",\"codec\":\"mjpeg\","
            "\"name\":\"%s\",\"uri\":\"%s\",\"meta\":\"%s\",\"meta_uri\":\"%s\","
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
            seg->kind == RECORDING_KIND_ANNOTATED ? seg->uri : "",
            RECORDING_MANIFEST_URI, seg->name,
            RECORDING_FRAME_SVG_URI, seg->name, s_storage_backend,
            seg->start_ms, seg->last_ms,
            seg->start_epoch_ms, end_epoch_ms, time_source_name(seg->time_source),
            seg->last_ms > seg->start_ms ? seg->last_ms - seg->start_ms : 0,
            seg->frames, seg->bytes, seg->hit_frames, seg->detection_total, labels);
    fclose(file);
    return ESP_OK;
}

static esp_err_t append_recording_summary_jsonl(const recording_segment_t *seg)
{
    if (!seg || !seg->name[0] || seg->frames == 0) {
        return ESP_OK;
    }

    rotate_jsonl_if_needed(RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_OLD_PATH);
    FILE *file = fopen(RECORDING_SUMMARY_PATH, "a");
    if (!file) {
        s_recording_sd_errors++;
        set_storage_status("open summary index failed: errno=%d", errno);
        return ESP_FAIL;
    }

    char labels[512];
    label_counts_to_json(labels, sizeof(labels), seg->labels);
    uint64_t end_epoch_ms = seg->start_epoch_ms;
    if (end_epoch_ms > 0 && seg->last_ms > seg->start_ms) {
        end_epoch_ms += (uint64_t)(seg->last_ms - seg->start_ms);
    }
    fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"period_ms\":%d,\"kind\":\"%s\","
            "\"segment\":\"%s\",\"uri\":\"%s\",\"annotated_uri\":\"%s\",\"manifest_uri\":\"%s?name=%s\","
            "\"first_frame_overlay_uri\":\"%s?name=%s&frame=1\",\"storage_backend\":\"%s\","
            "\"start_ms\":%" PRId64
            ",\"end_ms\":%" PRId64 ",\"start_epoch_ms\":%" PRIu64
            ",\"end_epoch_ms\":%" PRIu64 ",\"clock_source\":\"%s\""
            ",\"frames\":%" PRIu32 ",\"hit_frames\":%" PRIu32
            ",\"detection_total\":%" PRIu32 ",\"labels\":%s}\n",
            (uint32_t)APP_JSONL_INDEX_VERSION, CONFIG_APP_SUMMARY_INTERVAL_MS,
            seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
            seg->name, seg->uri,
            seg->kind == RECORDING_KIND_ANNOTATED ? seg->uri : "",
            RECORDING_MANIFEST_URI, seg->name,
            RECORDING_FRAME_SVG_URI, seg->name, s_storage_backend, seg->start_ms, seg->last_ms,
            seg->start_epoch_ms, end_epoch_ms, time_source_name(seg->time_source),
            seg->frames, seg->hit_frames, seg->detection_total, labels);
    fclose(file);
    s_recording_summary_count++;
    return ESP_OK;
}

static void recording_close_segment(recording_segment_t *seg)
{
    if (!seg || (!seg->writer && !seg->meta_file)) {
        return;
    }

    esp_err_t close_ret = ESP_OK;
    if (seg->writer) {
        if (seg->frames > 0) {
            uint64_t duration_ms = seg->last_ms > seg->start_ms ?
                (uint64_t)(seg->last_ms - seg->start_ms) : 0;
            avi_mjpeg_writer_set_duration_ms(seg->writer, duration_ms);
            close_ret = avi_mjpeg_writer_close(seg->writer);
        } else {
            avi_mjpeg_writer_abort(seg->writer);
            unlink(seg->part_path);
        }
    }
    if (seg->meta_file) {
        fflush(seg->meta_file);
        fsync(fileno(seg->meta_file));
        fclose(seg->meta_file);
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
            s_recording_sd_errors++;
            set_storage_status("AVI finalize failed: %s", esp_err_to_name(close_ret));
            ESP_LOGE(TAG,
                     "recording segment finalize failed: kind=%s name=%s frames=%" PRIu32
                     " bytes=%" PRIu64 " error=%s",
                     seg->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                     seg->name, seg->frames, seg->bytes, esp_err_to_name(close_ret));
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
        append_recording_segment_jsonl(seg);
        append_recording_summary_jsonl(seg);
        cleanup_old_recordings();
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

    if (ensure_dir(RECORDING_DIR) != ESP_OK) {
        s_recording_sd_errors++;
        set_storage_status("recording mkdir failed: errno=%d", errno);
        return ESP_FAIL;
    }

    memset(seg, 0, sizeof(*seg));
    seg->kind = kind;
    seg->start_ms = now_ms;
    seg->last_ms = now_ms;
    seg->start_epoch_ms = wall_clock_epoch_ms();
    seg->time_source = seg->start_epoch_ms > 0 ? s_time_source : TIME_SOURCE_UNSYNCED;
    snprintf(seg->name, sizeof(seg->name), "%s_b%08" PRIx32 "_t%010" PRId64 ".avi",
             kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw", s_boot_id, now_ms);
    meta_name_for_recording(seg->name, seg->meta_name, sizeof(seg->meta_name));
    snprintf(seg->uri, sizeof(seg->uri), RECORDING_URI_PREFIX "%s", seg->name);
    snprintf(seg->meta_uri, sizeof(seg->meta_uri), RECORDING_META_URI_PREFIX "%s", seg->meta_name);

    snprintf(seg->final_path, sizeof(seg->final_path), "%s/%s", RECORDING_DIR, seg->name);
    snprintf(seg->part_path, sizeof(seg->part_path), "%s/%s.part", RECORDING_DIR, seg->name);
    uint32_t fps = kind == RECORDING_KIND_ANNOTATED ? 1U : CONFIG_APP_FIELD_RECORDING_MAX_FPS;
    if (fps == 0) {
        fps = 1;
    }
    esp_err_t ret = avi_mjpeg_writer_open(&seg->writer, seg->part_path, seg->final_path,
                                          width, height, fps);
    if (ret != ESP_OK) {
        s_recording_sd_errors++;
        set_storage_status("open AVI failed: %s", esp_err_to_name(ret));
        memset(seg, 0, sizeof(*seg));
        return ret;
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, seg->meta_name);
    seg->meta_file = fopen(path, "w");
    if (!seg->meta_file) {
        avi_mjpeg_writer_abort(seg->writer);
        seg->writer = NULL;
        unlink(seg->part_path);
        s_recording_sd_errors++;
        set_storage_status("open recording meta failed: errno=%d", errno);
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

static void append_recording_event(const recording_segment_t *seg,
                                   const recording_item_t *item,
                                   uint32_t frame_index,
                                   int64_t now_ms)
{
    if (!seg || !item || seg->kind != RECORDING_KIND_ANNOTATED) {
        return;
    }
    FILE *file = fopen(EVENT_INDEX_PATH, "a");
    if (!file) {
        s_recording_sd_errors++;
        return;
    }
    char detections[1280];
    detections_to_json(detections, sizeof(detections), &item->meta.vision);
    fprintf(file,
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
    fclose(file);
}

static void recording_write_item(recording_segment_t *seg, const recording_item_t *item)
{
    if (!seg || !item || !item->jpeg || !item->jpeg_size || !s_sd_mounted) {
        return;
    }

    int64_t now_ms = item->meta.timestamp_ms ? item->meta.timestamp_ms : esp_timer_get_time() / 1000;
    if (!seg->writer ||
        (CONFIG_APP_RECORDING_SEGMENT_MS > 0 &&
         now_ms - seg->start_ms >= (int64_t)CONFIG_APP_RECORDING_SEGMENT_MS) ||
        (CONFIG_APP_SUMMARY_INTERVAL_MS > 0 &&
         now_ms - seg->start_ms >= (int64_t)CONFIG_APP_SUMMARY_INTERVAL_MS)) {
        recording_close_segment(seg);
        if (recording_open_segment(seg, now_ms, item->kind,
                                   item->meta.width, item->meta.height) != ESP_OK) {
            return;
        }
    }

    esp_err_t write_ret = avi_mjpeg_writer_add_frame(seg->writer, item->jpeg, item->jpeg_size);
    if (write_ret != ESP_OK) {
        s_recording_sd_errors++;
        set_storage_status("AVI frame write failed: %s", esp_err_to_name(write_ret));
        if (s_recording_sd_errors <= 4U || (s_recording_sd_errors % 32U) == 0U) {
            ESP_LOGE(TAG,
                     "AVI frame write failed: kind=%s name=%s seq=%" PRIu32
                     " jpeg=%" PRIu32 " errors=%" PRIu32 " error=%s",
                     item->kind == RECORDING_KIND_ANNOTATED ? "annotated" : "raw",
                     seg->name, item->meta.seq, item->jpeg_size,
                     s_recording_sd_errors, esp_err_to_name(write_ret));
        }
        return;
    }

    seg->frames++;
    seg->last_ms = now_ms;
    seg->bytes += item->jpeg_size;
    recording_segment_add_vision(seg, &item->meta.vision);
    if (seg->meta_file) {
        char detections[1280];
        detections_to_json(detections, sizeof(detections), &item->meta.vision);
        int meta_ret = fprintf(seg->meta_file,
                "{\"index_version\":%" PRIu32 ",\"segment\":\"%s\",\"segment_uri\":\"%s\","
                "\"kind\":\"%s\","
                "\"meta_uri\":\"%s\",\"overlay_uri\":\"%s?name=%s&frame=%" PRIu32 "\","
                "\"storage_backend\":\"%s\",\"frame_index\":%" PRIu32 ",\"seq\":%" PRIu32
                ",\"time_ms\":%" PRId64 ",\"epoch_ms\":%" PRIu64
                ",\"jpeg_bytes\":%" PRIu32
                ",\"width\":%" PRIu32 ",\"height\":%" PRIu32
                ",\"model\":\"%s\",\"box_min_score\":%" PRIu32 ",\"best_score\":%" PRIu32
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
                item->meta.vision.model, item->meta.vision.box_min_score,
                item->meta.vision.object_score, item->meta.vision.candidate_score,
                item->meta.vision.raw_candidate_count,
                item->meta.vision.inference_ms, item->meta.vision.analysis_ms,
                item->meta.vision.detection_count, detections);
        if (meta_ret < 0) {
            s_recording_sd_errors++;
            ESP_LOGE(TAG, "recording metadata write failed: name=%s frame=%" PRIu32
                     " errno=%d ferror=%d",
                     seg->meta_name, seg->frames, errno, ferror(seg->meta_file));
        }
        if ((seg->frames % 16U) == 0) {
            errno = 0;
            if (fflush(seg->meta_file) != 0 || fsync(fileno(seg->meta_file)) != 0) {
                s_recording_sd_errors++;
                ESP_LOGE(TAG, "recording metadata sync failed: name=%s frame=%" PRIu32
                         " errno=%d",
                         seg->meta_name, seg->frames, errno);
            }
        }
    }
    append_recording_event(seg, item, seg->frames, now_ms);
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

static void recording_maybe_queue(const uint8_t *jpeg, uint32_t jpeg_size, const frame_meta_t *meta)
{
    if (s_storage_quiescing || s_app_mode != APP_MODE_FIELD ||
        !CONFIG_APP_RECORDING_ENABLE || !s_recording_enabled || !s_sd_mounted ||
        !storage_acceptance_ok() ||
        !s_recording_queue || !jpeg || !jpeg_size || !meta || meta->seq == 0) {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;
    uint32_t max_fps = CONFIG_APP_FIELD_RECORDING_MAX_FPS;
    int64_t min_interval_ms = max_fps > 0 ? 1000 / max_fps : 0;
    if (s_last_recording_frame_ms != 0 && now_ms - s_last_recording_frame_ms < min_interval_ms) {
        return;
    }
    s_last_recording_frame_ms = now_ms;

    recording_item_t item = {
        .meta = *meta,
        .jpeg_size = jpeg_size,
        .kind = RECORDING_KIND_RAW,
    };
    item.jpeg = alloc_psram_buffer(jpeg_size);
    if (!item.jpeg) {
        s_recording_dropped++;
        return;
    }
    memcpy(item.jpeg, jpeg, jpeg_size);

    if (xQueueSend(s_recording_queue, &item, 0) != pdTRUE) {
        free(item.jpeg);
        s_recording_dropped++;
        return;
    }
    s_recording_queued++;
    if (s_recording_queued == 1 || (s_recording_queued % 100U) == 0U) {
        ESP_LOGI(TAG, "raw recording queue progress: queued=%" PRIu32
                 " seq=%" PRIu32 " jpeg=%" PRIu32,
                 s_recording_queued, meta->seq, jpeg_size);
    }
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

static void recover_incomplete_recordings(void)
{
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (!is_safe_snapshot_name(entry->d_name) || !has_suffix(entry->d_name, ".avi.part")) {
            continue;
        }
        char part_path[384];
        char final_path[384];
        char final_name[128];
        strlcpy(final_name, entry->d_name, sizeof(final_name));
        final_name[strlen(final_name) - strlen(".part")] = '\0';
        snprintf(part_path, sizeof(part_path), "%s/%s", RECORDING_DIR, entry->d_name);
        snprintf(final_path, sizeof(final_path), "%s/%s", RECORDING_DIR, final_name);
        esp_err_t ret = avi_mjpeg_recover_part(part_path, final_path);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "recovered interrupted recording %s", final_name);
        } else {
            ESP_LOGW(TAG, "could not recover %s: %s", entry->d_name, esp_err_to_name(ret));
            struct stat st = {0};
            if ((ret == ESP_ERR_INVALID_RESPONSE ||
                 (ret == ESP_FAIL && stat(part_path, &st) == 0 && st.st_size <= 224)) &&
                stat(part_path, &st) == 0) {
                char corrupt_path[400];
                snprintf(corrupt_path, sizeof(corrupt_path), "%s.corrupt", part_path);
                unlink(corrupt_path);
                if (rename(part_path, corrupt_path) == 0) {
                    ESP_LOGW(TAG, "quarantined unusable recording part %s (%" PRId64 " bytes)",
                             entry->d_name, (int64_t)st.st_size);
                }
            }
        }
    }
    closedir(dir);
}

static bool recording_index_contains_name(const char *name)
{
    const char *paths[] = {RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH};
    char needle[128];
    snprintf(needle, sizeof(needle), "\"name\":\"%s\"", name);
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        return false;
    }
    bool found = false;
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]) && !found; i++) {
        FILE *file = fopen(paths[i], "r");
        if (!file) {
            continue;
        }
        while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
            if (strstr(line, needle)) {
                found = true;
                break;
            }
        }
        fclose(file);
    }
    free(line);
    return found;
}

static int64_t json_number_from_line(const char *line, const char *key)
{
    char needle[48];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *value = strstr(line, needle);
    return value ? atoll(value + strlen(needle)) : -1;
}

static void recovered_segment_read_meta(recording_segment_t *seg, const char *path)
{
    FILE *file = fopen(path, "r");
    if (!file) {
        return;
    }
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        fclose(file);
        return;
    }

    int64_t first_ms = -1;
    int64_t last_ms = -1;
    while (fgets(line, JSONL_TAIL_LINE_BYTES, file)) {
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
    free(line);
    fclose(file);
    if (first_ms >= 0 && last_ms >= first_ms) {
        seg->start_ms = first_ms;
        seg->last_ms = last_ms;
    }
}

static void reconcile_recording_indexes(void)
{
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (!is_safe_recording_name(entry->d_name) ||
            !has_suffix(entry->d_name, ".avi")) {
            continue;
        }
        bool already_indexed = recording_index_contains_name(entry->d_name);

        char path[384];
        snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, entry->d_name);
        avi_mjpeg_info_t info = {0};
        if (avi_mjpeg_probe(path, &info) != ESP_OK) {
            continue;
        }
        recording_segment_t *seg = (recording_segment_t *)calloc(1, sizeof(*seg));
        if (!seg) {
            break;
        }
        seg->kind = strncmp(entry->d_name, "annotated_", strlen("annotated_")) == 0 ?
            RECORDING_KIND_ANNOTATED : RECORDING_KIND_RAW;
        strlcpy(seg->name, entry->d_name, sizeof(seg->name));
        meta_name_for_recording(seg->name, seg->meta_name, sizeof(seg->meta_name));
        snprintf(seg->uri, sizeof(seg->uri), RECORDING_URI_PREFIX "%s", seg->name);
        snprintf(seg->meta_uri, sizeof(seg->meta_uri), RECORDING_META_URI_PREFIX "%s",
                 seg->meta_name);
        const char *time_mark = strstr(seg->name, "_t");
        seg->start_ms = time_mark ? atoll(time_mark + 2) : 0;
        seg->last_ms = seg->start_ms + (int64_t)info.duration_ms;
        seg->frames = info.frame_count;
        seg->bytes = info.file_bytes;

        char meta_path[384];
        snprintf(meta_path, sizeof(meta_path), "%s/%s", RECORDING_DIR, seg->meta_name);
        recovered_segment_read_meta(seg, meta_path);
        if (seg->last_ms > seg->start_ms) {
            uint64_t expected_duration_ms = (uint64_t)(seg->last_ms - seg->start_ms);
            uint64_t duration_delta_ms = info.duration_ms > expected_duration_ms ?
                info.duration_ms - expected_duration_ms :
                expected_duration_ms - info.duration_ms;
            if (duration_delta_ms > 5U) {
                esp_err_t retime_ret =
                    avi_mjpeg_retime_file(path, expected_duration_ms);
                if (retime_ret != ESP_OK) {
                    ESP_LOGW(TAG, "could not retime recovered AVI %s: %s",
                             seg->name, esp_err_to_name(retime_ret));
                }
            }
        }
        if (!already_indexed && append_recording_segment_jsonl(seg) == ESP_OK) {
            append_recording_summary_jsonl(seg);
            ESP_LOGI(TAG,
                     "reconciled recording index: %s frames=%" PRIu32
                     " duration_ms=%" PRId64,
                     seg->name, seg->frames,
                     seg->last_ms > seg->start_ms ? seg->last_ms - seg->start_ms : 0);
        }
        free(seg);
    }
    closedir(dir);
}

static void append_field_session(void)
{
    if (s_field_session_started) {
        return;
    }
    FILE *file = fopen(SESSION_INDEX_PATH, "a");
    if (!file) {
        s_recording_sd_errors++;
        return;
    }
    uint64_t epoch_ms = wall_clock_epoch_ms();
    fprintf(file,
            "{\"index_version\":%" PRIu32 ",\"session\":\"b%08" PRIx32 "\","
            "\"start_ms\":%" PRId64 ",\"start_epoch_ms\":%" PRIu64
            ",\"clock\":\"%s\","
            "\"mode\":\"field\",\"storage_backend\":\"%s\"}\n",
            (uint32_t)APP_JSONL_INDEX_VERSION, s_boot_id,
            esp_timer_get_time() / 1000, epoch_ms,
            epoch_ms > 0 ? time_source_name(s_time_source) : "boot_relative",
            s_storage_backend);
    fclose(file);
    s_field_session_started = true;
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
    close(fd);
    if (unlink(path) != 0) {
        ESP_LOGW(TAG, "TF write self-test cleanup failed: errno=%d", errno);
    }
    ESP_LOGI(TAG, "TF write self-test passed: %u bytes fsync/reopen/verify",
             (unsigned)TEST_BYTES);
    return ESP_OK;
}

static void storage_ensure_usb_volume_label(void)
{
#if CONFIG_FATFS_USE_LABEL
    if (!s_sd_card) {
        return;
    }
    BYTE pdrv = ff_diskio_get_pdrv_card(s_sd_card);
    if (pdrv == 0xff) {
        ESP_LOGW(TAG, "cannot resolve TF FatFS drive for volume label");
        return;
    }
    char drive[8];
    char requested[24];
    char label[24] = {0};
    DWORD serial = 0;
    snprintf(drive, sizeof(drive), "%u:", (unsigned)pdrv);
    FRESULT result = f_getlabel(drive, label, &serial);
    if (result != FR_OK) {
        ESP_LOGW(TAG, "TF volume label read failed: fresult=%d", (int)result);
        return;
    }
    if (strcmp(label, "P4_BUOY") == 0) {
        return;
    }
    snprintf(requested, sizeof(requested), "%u:P4_BUOY", (unsigned)pdrv);
    result = f_setlabel(requested);
    if (result == FR_OK) {
        ESP_LOGI(TAG, "TF volume label set to P4_BUOY");
    } else {
        ESP_LOGW(TAG, "TF volume label update failed: fresult=%d", (int)result);
    }
#endif
}

static esp_err_t storage_prepare_dirs_after_mount(const char *mode)
{
    update_sd_info();
    storage_ensure_usb_volume_label();
    strlcpy(s_sd_mount_mode, mode, sizeof(s_sd_mount_mode));
    s_sd_last_error[0] = '\0';
    s_sd_last_error_code = 0;
    if (s_app_mode == APP_MODE_SERVER) {
        /*
         * SERVER_MODE does not create new monitoring data, but it must make an
         * interrupted field segment playable before exposing the card.
         */
        recover_incomplete_recordings();
        reconcile_recording_indexes();
        set_storage_status("mounted on %s; server read-only", mode);
        return ESP_OK;
    }

    if (ensure_dir(HISTORY_ROOT_DIR) != ESP_OK ||
        ensure_dir(HISTORY_SNAPSHOT_DIR) != ESP_OK ||
        ensure_dir(RECORDING_DIR) != ESP_OK ||
        ensure_dir(DATASET_ROOT_DIR) != ESP_OK ||
        ensure_dir(DATASET_RUN_DIR) != ESP_OK) {
        set_storage_status("mkdir failed: errno=%d", errno);
        s_history_sd_errors++;
        return ESP_FAIL;
    }
    esp_err_t write_test_ret = storage_write_selftest();
    if (write_test_ret != ESP_OK) {
        s_recording_sd_errors++;
        set_storage_status("TF write self-test failed: %s", esp_err_to_name(write_test_ret));
        return write_test_ret;
    }

    rotate_history_index_if_needed();
    rotate_jsonl_if_needed(RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH);
    rotate_jsonl_if_needed(RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_OLD_PATH);
    cleanup_old_snapshots();
    cleanup_old_recordings();
    recover_incomplete_recordings();
    reconcile_recording_indexes();
    append_field_session();
    update_sd_info();
    set_storage_status("mounted on %s; field recording enabled", mode);
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

static esp_err_t __attribute__((unused)) storage_mount_sdmmc_width(int width, bool format_if_failed)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_failed,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = CONFIG_APP_SD_MAX_FREQ_KHZ;
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
             s_sd_attempts, mode, CONFIG_APP_SD_MAX_FREQ_KHZ,
             CONFIG_APP_SD_PIN_CLK, CONFIG_APP_SD_PIN_CMD,
             CONFIG_APP_SD_PIN_D0, CONFIG_APP_SD_PIN_D1, CONFIG_APP_SD_PIN_D2,
             CONFIG_APP_SD_PIN_D3, CONFIG_APP_SD_LDO_IO_ID, format_if_failed);

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &s_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TF mount failed on %s: %s - %s", mode, esp_err_to_name(ret), sd_error_hint(ret));
        record_sd_mount_error(mode, ret);
        return ret;
    }

    s_sd_mounted = true;
    strlcpy(s_storage_backend, "tf_sdmmc", sizeof(s_storage_backend));
    ESP_LOGI(TAG, "TF card mounted at %s via SDMMC %d-bit", CONFIG_APP_SD_MOUNT_POINT, slot_config.width);
    sdmmc_card_print_info(stdout, s_sd_card);
    return storage_prepare_dirs_after_mount(mode);
}

static esp_err_t storage_mount_sdspi(spi_host_device_t spi_host, bool format_if_failed)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_failed,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = spi_host;
    host.max_freq_khz = CONFIG_APP_SD_MAX_FREQ_KHZ;
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
             s_sd_attempts, mode, CONFIG_APP_SD_MAX_FREQ_KHZ,
             CONFIG_APP_SD_PIN_CMD, CONFIG_APP_SD_PIN_D0,
             CONFIG_APP_SD_PIN_CLK, CONFIG_APP_SD_PIN_D3, CONFIG_APP_SD_LDO_IO_ID, format_if_failed);

    ret = esp_vfs_fat_sdspi_mount(CONFIG_APP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TF mount failed on %s: %s - %s", mode, esp_err_to_name(ret), sd_error_hint(ret));
        record_sd_mount_error(mode, ret);
        spi_bus_free(host.slot);
        return ret;
    }

    s_sd_mounted = true;
    s_sd_spi_host = spi_host;
    strlcpy(s_storage_backend, "tf_sdspi", sizeof(s_storage_backend));
    ESP_LOGI(TAG, "TF card mounted at %s via SDSPI%d", CONFIG_APP_SD_MOUNT_POINT, (int)spi_host + 1);
    sdmmc_card_print_info(stdout, s_sd_card);
    return storage_prepare_dirs_after_mount(mode);
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

static esp_err_t storage_mount(void)
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
        final_ret = ESP_OK;
        goto out;
    }

    esp_err_t ret = ESP_FAIL;

    /*
     * Prefer SDMMC Slot 0 for sustained field recording/export throughput.
     * If the board revision or host state rejects SDMMC, fall back to the
     * older SDSPI2 path so the service still comes up.
     */
    s_sd_format_requested = false;
    for (int attempt = 0; attempt < 3 && ret != ESP_OK; attempt++) {
        if (s_sd_pwr_ctrl) {
            sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
            s_sd_pwr_ctrl = NULL;
        }
        sd_pwr_ctrl_ldo_config_t ldo_config = {
            .ldo_chan_id = CONFIG_APP_SD_LDO_IO_ID,
        };
        ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_pwr_ctrl);
        if (ret != ESP_OK) {
            set_storage_status("sd ldo failed: %s", esp_err_to_name(ret));
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = storage_reset_card_power();
        bool card_power_ok = ret == ESP_OK;
        if (card_power_ok) {
#if CONFIG_APP_SD_USE_SDMMC
            ret = storage_mount_sdmmc_width(CONFIG_APP_SD_BUS_WIDTH, false);
            if (ret != ESP_OK && CONFIG_APP_SD_BUS_WIDTH > 1) {
                ESP_LOGW(TAG, "TF SDMMC %d-bit mount failed, trying SDMMC 1-bit before SPI fallback",
                         CONFIG_APP_SD_BUS_WIDTH);
                ret = storage_mount_sdmmc_width(1, false);
            }
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "TF SDMMC mount failed, falling back to SDSPI2");
            }
#endif
        }
        if (card_power_ok && ret != ESP_OK) {
            ret = storage_mount_sdspi(SPI2_HOST, false);
        }
        if (ret == ESP_OK) {
            break;
        }
        if (s_sd_pwr_ctrl) {
            sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
            s_sd_pwr_ctrl = NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    if (ret != ESP_OK) {
        s_history_sd_errors++;
    }
    final_ret = ret;

out:
    if (locked) {
        xSemaphoreGive(s_storage_lock);
    }
    return final_ret;
}

static void storage_unmount(const char *reason)
{
    bool locked = false;
    if (s_storage_lock) {
        xSemaphoreTake(s_storage_lock, portMAX_DELAY);
        locked = true;
    }

    if (s_sd_mounted && s_sd_card) {
        ESP_LOGI(TAG, "Unmounting TF card: %s", reason ? reason : "no reason");
        esp_vfs_fat_sdcard_unmount(CONFIG_APP_SD_MOUNT_POINT, s_sd_card);
        if (strcmp(s_storage_backend, "tf_sdspi") == 0) {
            esp_err_t bus_ret = spi_bus_free(s_sd_spi_host);
            if (bus_ret != ESP_OK && bus_ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "SPI bus free failed: %s", esp_err_to_name(bus_ret));
            }
        }
        s_sd_card = NULL;
        s_sd_mounted = false;
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
        strlcpy(s_storage_backend, "none", sizeof(s_storage_backend));
        set_storage_status("unmounted for %s", reason ? reason : "storage switch");
    } else if (s_sd_mounted && s_flash_wl_handle != WL_INVALID_HANDLE) {
        ESP_LOGI(TAG, "Unmounting flash fallback storage: %s", reason ? reason : "no reason");
        esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_APP_SD_MOUNT_POINT, s_flash_wl_handle);
        s_flash_wl_handle = WL_INVALID_HANDLE;
        s_sd_mounted = false;
        s_sd_total_bytes = 0;
        s_sd_free_bytes = 0;
        strlcpy(s_storage_backend, "none", sizeof(s_storage_backend));
        set_storage_status("flash fallback unmounted for %s", reason ? reason : "storage switch");
    }
    if (s_sd_pwr_ctrl) {
        esp_err_t ldo_ret = sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
        if (ldo_ret != ESP_OK) {
            ESP_LOGW(TAG, "SD LDO release failed: %s", esp_err_to_name(ldo_ret));
        }
        s_sd_pwr_ctrl = NULL;
    }

    if (locked) {
        xSemaphoreGive(s_storage_lock);
    }
}

static void history_record_to_json(char *buf, size_t size, const history_record_t *record)
{
    char fallback_dets[] = "[]";
    char *dets = (char *)alloc_psram_buffer(1280);
    if (!dets) {
        dets = fallback_dets;
    }
    if (dets != fallback_dets) {
        detections_to_json(dets, 1280, &record->vision);
    }
    snprintf(buf, size,
             "{\"index_version\":%" PRIu32 ",\"storage_backend\":\"%s\","
             "\"seq\":%" PRIu32 ",\"time_ms\":%" PRId64 ",\"stored_ms\":%" PRId64 ","
              "\"source\":\"%s\","
             "\"width\":%" PRIu32 ",\"height\":%" PRIu32 ",\"jpeg_bytes\":%" PRIu32 ","
             "\"capture_ms\":%" PRId64 ",\"encode_ms\":%" PRId64 ","
             "\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64 ","
             "\"recognition_method\":\"%s\",\"network_mode\":\"%s\",\"rssi_dbm\":%d,"
              "\"model_bytes\":%" PRIu32 ",\"model_input_size\":%" PRIu32 ","
             "\"free_heap\":%" PRIu32 ",\"min_free_heap\":%" PRIu32 ","
             "\"free_psram\":%" PRIu32 ",\"min_free_psram\":%" PRIu32 ","
             "\"label\":\"%s\",\"object\":\"%s\",\"scene\":\"%s\",\"color\":\"%s\",\"model\":\"%s\",\"motion\":%s,"
             "\"motion_score\":%" PRIu32 ",\"edge_score\":%" PRIu32 ",\"avg_luma\":%" PRIu32 ","
             "\"avg_r\":%" PRIu32 ",\"avg_g\":%" PRIu32 ",\"avg_b\":%" PRIu32 ","
             "\"object_score\":%" PRIu32 ",\"candidate_score\":%" PRIu32 ",\"box_min_score\":%" PRIu32 ","
              "\"object_count\":%" PRIu32 ",\"detection_count\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32 ","
              "\"best_score\":%" PRIu32 ",\"detections\":%s,"
              "\"object_x\":%" PRIu32 ",\"object_y\":%" PRIu32 ",\"object_w\":%" PRIu32 ",\"object_h\":%" PRIu32 ","
             "\"coke_score\":%" PRIu32 ",\"sprite_score\":%" PRIu32 ","
             "\"unknown_score\":%" PRIu32 ","
              "\"snapshot\":\"%s\"}",
               (uint32_t)APP_JSONL_INDEX_VERSION, s_storage_backend,
               record->seq, record->timestamp_ms, record->stored_ms,
              record->source[0] ? record->source : "camera",
              record->width, record->height, record->jpeg_size,
              record->capture_ms, record->encode_ms,
              record->vision.inference_ms, record->vision.analysis_ms,
              recognition_method_name(record->recognition_method), network_mode_name(record->network_mode),
              record->rssi_dbm,
              record->model_bytes, record->model_input_size,
              record->free_heap, record->min_free_heap,
              record->free_psram, record->min_free_psram,
              record->vision.label, record->vision.object, record->vision.scene, record->vision.color, record->vision.model,
             record->vision.motion ? "true" : "false",
             record->vision.motion_score, record->vision.edge_score, record->vision.avg_luma,
             record->vision.avg_r, record->vision.avg_g, record->vision.avg_b,
              record->vision.object_score, record->vision.candidate_score, record->vision.box_min_score,
              record->vision.object_count, record->vision.detection_count, record->vision.raw_candidate_count,
              record->vision.object_score, dets,
              record->vision.object_x, record->vision.object_y, record->vision.object_w, record->vision.object_h,
             record->vision.coke_score, record->vision.sprite_score,
              record->vision.unknown_score,
              record->snapshot);
    if (dets != fallback_dets) {
        free(dets);
    }
}

static esp_err_t append_history_jsonl(const history_record_t *record)
{
    rotate_history_index_if_needed();

    FILE *file = fopen(HISTORY_JSONL_PATH, "a");
    if (!file) {
        s_history_sd_errors++;
        set_storage_status("open history failed: errno=%d", errno);
        return ESP_FAIL;
    }

    char *line = (char *)alloc_psram_buffer(4096);
    if (!line) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    history_record_to_json(line, 4096, record);
    fprintf(file, "%s\n", line);
    free(line);
    fclose(file);
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
    FILE *file = fopen(path, "wb");
    if (!file) {
        s_history_sd_errors++;
        set_storage_status("open snapshot failed: errno=%d", errno);
        return ESP_FAIL;
    }

    size_t written = fwrite(item->jpeg, 1, item->jpeg_size, file);
    fclose(file);
    if (written != item->jpeg_size) {
        unlink(path);
        s_history_sd_errors++;
        set_storage_status("short snapshot write");
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
        save_snapshot_jpeg(item, item->record.snapshot, sizeof(item->record.snapshot));
        append_history_jsonl(&item->record);
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

static bool queue_yolo_inference(const uint8_t *jpeg, uint32_t jpeg_size,
                                 const frame_meta_t *meta, recognition_method_t method)
{
    if (s_storage_quiescing) {
        return false;
    }
    if (!s_inference_queue || !jpeg || !jpeg_size || !meta || !recognition_method_is_yolo(method)) {
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

static void validation_cache_update(validation_context_t *ctx)
{
    if (!ctx || !s_validation_lock) {
        return;
    }

    xSemaphoreTake(s_validation_lock, portMAX_DELAY);
    s_validation_last.valid = true;
    ctx->id = ++s_validation_id;
    s_validation_last.id = ctx->id;
    s_validation_last.sample = ctx->sample;
    s_validation_last.method = ctx->method;
    s_validation_last.vision = ctx->vision;
    s_validation_last.source_w = ctx->source_w;
    s_validation_last.source_h = ctx->source_h;
    s_validation_last.jpeg_size = ctx->jpeg_size;
    s_validation_last.queued_ms = ctx->queued_ms;
    s_validation_last.completed_ms = ctx->completed_ms;
    xSemaphoreGive(s_validation_lock);
}

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
            uint32_t source_w = 0;
            uint32_t source_h = 0;
            int64_t start_us = esp_timer_get_time();
            esp_err_t err = classify_validation_yolo_jpeg(job.jpeg, job.jpeg_size, job.method,
                                                          job.box_min_score ? job.box_min_score : s_box_min_score,
                                                          &vision, &source_w, &source_h);
            vision.analysis_ms = (esp_timer_get_time() - start_us) / 1000;

            if (ctx) {
                ctx->err = err;
                ctx->vision = vision;
                ctx->source_w = source_w;
                ctx->source_h = source_h;
                ctx->completed_ms = esp_timer_get_time() / 1000;
                validation_cache_update(ctx);
                if (vision.detection_count > 0) {
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
                     !recognition_method_is_yolo(job.method);

        if (!stale) {
            int64_t start_us = esp_timer_get_time();
            if (job.method == RECOGNITION_METHOD_YOLO11) {
                classify_yolo11_jpeg(job.jpeg, job.jpeg_size, &vision);
            } else if (job.method == RECOGNITION_METHOD_COCO) {
                classify_coco_jpeg(job.jpeg, job.jpeg_size, &vision);
            } else {
                classify_yolo26_jpeg(job.jpeg, job.jpeg_size, &vision);
            }
            vision.analysis_ms = (esp_timer_get_time() - start_us) / 1000;

            job.meta.vision = vision;
            update_latest_vision_from_inference(&vision, job.method);
            history_maybe_queue(job.jpeg, job.jpeg_size, &job.meta, job.method, "camera",
                                vision.detection_count > 0);
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
        set_camera_error("open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
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

    s_camera.fd = fd;
    s_camera.width = format.fmt.pix.width;
    s_camera.height = format.fmt.pix.height;
    s_camera.pixel_format = format.fmt.pix.pixelformat;

    if (s_camera.pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_config_t encoder_config = {
            .width = s_camera.width,
            .height = s_camera.height,
            .pixel_format = s_camera.pixel_format,
            .quality = (uint8_t)s_jpeg_quality,
        };
        ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &s_camera.encoder), fail, TAG, "failed to init JPEG encoder");
        ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(s_camera.encoder, &s_camera.jpeg_buf, &s_camera.jpeg_buf_size),
                          fail, TAG, "failed to alloc JPEG buffer");
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
    return ESP_OK;

fail:
    set_camera_error("camera open failed: %s (errno=%d)",
                     esp_err_to_name(ret == ESP_OK ? ESP_FAIL : ret), errno);
    camera_release_device(fd, false);
    return ret == ESP_OK ? ESP_FAIL : ret;
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
    int64_t now_for_interval_ms = esp_timer_get_time() / 1000;
    bool yolo_busy = method_enabled && method_is_yolo && inference_worker_busy();
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
    bool need_yolo = run_recognition && method_is_yolo;
    if (method_enabled && !run_recognition) {
        frame_meta_t previous = {0};
        if (method_is_yolo && copy_completed_yolo_vision(&vision, method)) {
            /* 复用上一次完成的 YOLO 结果，让视频流在下一次慢速推理结束前仍有稳定标注。 */
        } else if (copy_latest_meta(&previous)) {
            vision = previous.vision;
        } else if (method_is_yolo) {
            fill_yolo_pending(&vision, method);
        } else {
            fill_vision_disabled(&vision);
        }
    }

    if (s_camera.pixel_format == V4L2_PIX_FMT_JPEG) {
        jpeg_ptr = raw_ptr;
        jpeg_size = buf.bytesused;
        if (need_yolo) {
            strlcpy(vision.scene, "jpeg", sizeof(vision.scene));
            strlcpy(vision.color, "unknown", sizeof(vision.color));
            fill_yolo_pending(&vision, method);
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
            if (method == RECOGNITION_METHOD_MLP) {
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

    if (need_yolo && jpeg_ptr && jpeg_size > 0) {
        frame_meta_t previous = {0};
        if (copy_completed_yolo_vision(&vision, method)) {
            /* 新任务已经投递时，画面继续显示上一轮真实 YOLO 结果，避免 waiting 闪烁。 */
        } else if (copy_latest_meta(&previous) && is_completed_yolo_result(&previous.vision, method)) {
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
        recording_maybe_queue(jpeg_ptr, jpeg_size, &meta);
        if (need_yolo) {
            queue_yolo_inference(jpeg_ptr, jpeg_size, &meta, method);
        } else if (run_recognition) {
            history_maybe_queue(jpeg_ptr, jpeg_size, &meta, method, "camera",
                                meta.vision.detection_count > 0);
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

static void save_setting_u32(const char *key, uint32_t value)
{
    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, key, value);
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

static void load_runtime_settings(void)
{
    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
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
        s_recognition_method = CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 4 ? RECOGNITION_METHOD_COCO :
                               (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 3 ? RECOGNITION_METHOD_YOLO11 :
                                (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 2 ? RECOGNITION_METHOD_YOLO26 :
                                 (CONFIG_APP_DEFAULT_RECOGNITION_METHOD == 0 ? RECOGNITION_METHOD_OFF :
                                  RECOGNITION_METHOD_MLP)));
        if (s_recognition_method == RECOGNITION_METHOD_YOLO26 && !yolo26_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        } else if (s_recognition_method == RECOGNITION_METHOD_YOLO11 && !yolo11_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        } else if (s_recognition_method == RECOGNITION_METHOD_COCO && !coco_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        }
        s_vision_enabled = s_recognition_method != RECOGNITION_METHOD_OFF;
        s_history_enabled = CONFIG_APP_HISTORY_ENABLE;
        s_recording_enabled = CONFIG_APP_RECORDING_ENABLE;
        s_box_min_score = CONFIG_APP_CAN_BOX_MIN_SCORE;
        s_stream_max_fps = CONFIG_APP_STREAM_MAX_FPS;
        s_inference_interval_ms = CONFIG_APP_INFERENCE_INTERVAL_MS;
        s_history_sample_interval_ms = CONFIG_APP_HISTORY_SAMPLE_INTERVAL_MS;
        s_jpeg_quality = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
        nvs_set_u32(nvs, "version", SETTINGS_VERSION);
        nvs_set_u8(nvs, "method", (uint8_t)s_recognition_method);
        nvs_set_u8(nvs, "netmode", (uint8_t)s_network_mode);
        nvs_set_u8(nvs, "vision", s_vision_enabled ? 1 : 0);
        nvs_set_u8(nvs, "history", s_history_enabled ? 1 : 0);
        nvs_set_u8(nvs, "recording", s_recording_enabled ? 1 : 0);
        nvs_set_u32(nvs, "box_min", s_box_min_score);
        nvs_set_u32(nvs, "stream_fps", s_stream_max_fps);
        nvs_set_u32(nvs, "inf_ms", s_inference_interval_ms);
        nvs_set_u32(nvs, "hist_ms", s_history_sample_interval_ms);
        nvs_set_u32(nvs, "jpeg_q", s_jpeg_quality);
        nvs_commit(nvs);
        nvs_close(nvs);
        return;
    }

    uint8_t method = (uint8_t)s_recognition_method;
    if (nvs_get_u8(nvs, "method", &method) == ESP_OK && method <= RECOGNITION_METHOD_COCO) {
        s_recognition_method = (recognition_method_t)method;
        if (s_recognition_method == RECOGNITION_METHOD_YOLO26 && !yolo26_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        } else if (s_recognition_method == RECOGNITION_METHOD_YOLO11 && !yolo11_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        } else if (s_recognition_method == RECOGNITION_METHOD_COCO && !coco_espdl_available()) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
        }
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
    nvs_close(nvs);
}

static void mark_network_activity(void)
{
    __atomic_store_n(&s_last_network_activity_ms, esp_timer_get_time() / 1000,
                     __ATOMIC_RELEASE);
}

static void open_network_access_window(const char *reason)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    __atomic_store_n(&s_last_network_activity_ms, now_ms, __ATOMIC_RELEASE);
    s_network_boot_window_until_ms = now_ms + CONFIG_APP_NETWORK_BOOT_WINDOW_MS;
    ESP_LOGI(TAG, "network access window opened for %d ms%s%s",
             CONFIG_APP_NETWORK_BOOT_WINDOW_MS,
             reason && reason[0] ? ": " : "",
             reason && reason[0] ? reason : "");
}

static void record_http_request(void)
{
    s_requests++;
    mark_network_activity();
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

static esp_err_t root_get_handler(httpd_req_t *req)
{
    record_http_request();
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, s_index_html);
}

static bool export_mode_reject(httpd_req_t *req, const char *feature)
{
    if (s_app_mode != APP_MODE_EXPORT) {
        return false;
    }
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_set_type(req, "text/plain");
    char msg[128];
    snprintf(msg, sizeof(msg), "%s disabled in export mode", feature ? feature : "feature");
    httpd_resp_sendstr(req, msg);
    return true;
}

static esp_err_t validate_get_handler(httpd_req_t *req)
{
    record_http_request();
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

static esp_err_t validate_coke_jpg_handler(httpd_req_t *req)
{
    record_http_request();
    return send_embedded_jpeg(req, validate_coke_jpg_start, validate_coke_jpg_end);
}

static esp_err_t validate_sprite_jpg_handler(httpd_req_t *req)
{
    record_http_request();
    return send_embedded_jpeg(req, validate_sprite_jpg_start, validate_sprite_jpg_end);
}

static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end);

static esp_err_t validate_demo_jpg_handler(httpd_req_t *req)
{
    record_http_request();
    validation_sample_t sample = VALIDATION_SAMPLE_NONE;
    if (strcmp(req->uri, "/validate/demo_01.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_01;
    } else if (strcmp(req->uri, "/validate/demo_02.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_02;
    } else if (strcmp(req->uri, "/validate/demo_03.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_03;
    } else if (strcmp(req->uri, "/validate/demo_04.jpg") == 0) {
        sample = VALIDATION_SAMPLE_DEMO_04;
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
    default:
        return "none";
    }
}

static const char *validation_sample_expected(validation_sample_t sample)
{
    if (sample == VALIDATION_SAMPLE_COKE || sample == VALIDATION_SAMPLE_SPRITE) {
        return validation_sample_name(sample);
    }
    if (sample == VALIDATION_SAMPLE_DEMO_01 ||
        sample == VALIDATION_SAMPLE_DEMO_02 ||
        sample == VALIDATION_SAMPLE_DEMO_03 ||
        sample == VALIDATION_SAMPLE_DEMO_04) {
        return "object";
    }
    return "none";
}

static const char *validation_sample_image_uri(validation_sample_t sample)
{
    switch (sample) {
    case VALIDATION_SAMPLE_SPRITE:
        return "/validate/sprite.jpg";
    case VALIDATION_SAMPLE_DEMO_01:
        return "/validate/demo_01.jpg";
    case VALIDATION_SAMPLE_DEMO_02:
        return "/validate/demo_02.jpg";
    case VALIDATION_SAMPLE_DEMO_03:
        return "/validate/demo_03.jpg";
    case VALIDATION_SAMPLE_DEMO_04:
        return "/validate/demo_04.jpg";
    case VALIDATION_SAMPLE_COKE:
    default:
        return "/validate/coke.jpg";
    }
}

static bool validation_sample_image(validation_sample_t sample, const uint8_t **start, const uint8_t **end)
{
    if (!start || !end) {
        return false;
    }
    if (sample == VALIDATION_SAMPLE_COKE) {
        *start = validate_coke_jpg_start;
        *end = validate_coke_jpg_end;
        return *end > *start;
    }
    if (sample == VALIDATION_SAMPLE_SPRITE) {
        *start = validate_sprite_jpg_start;
        *end = validate_sprite_jpg_end;
        return *end > *start;
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
    return false;
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
    return false;
}

static void run_board_image_validation_case(validation_sample_t sample,
                                            recognition_method_t method,
                                            uint32_t threshold,
                                            const char *trial)
{
    const uint8_t *jpg_start = NULL;
    const uint8_t *jpg_end = NULL;

    if (!recognition_method_is_yolo(method) || !validation_selftest_method_available(method)) {
        ESP_LOGW(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        return;
    }
    if (!validation_sample_image(sample, &jpg_start, &jpg_end)) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable: embedded sample missing",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        return;
    }

    const uint32_t jpeg_size = (uint32_t)(jpg_end - jpg_start);
    validation_context_t *ctx = (validation_context_t *)calloc(1, sizeof(validation_context_t));
    if (!ctx) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable: no context memory",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        return;
    }

    ctx->done = xSemaphoreCreateBinary();
    if (!ctx->done) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable: no semaphore",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        free(ctx);
        return;
    }
    ctx->sample = sample;
    ctx->method = method;
    ctx->box_min_score = threshold;
    ctx->jpeg_size = jpeg_size;
    ctx->queued_ms = esp_timer_get_time() / 1000;

    inference_job_t job = {
        .jpeg_size = jpeg_size,
        .method = method,
        .box_min_score = threshold,
        .validation = true,
        .validation_sample = sample,
        .validation_ctx = ctx,
        .queued_ms = ctx->queued_ms,
    };
    fill_yolo_pending(&job.meta.vision, method);
    job.jpeg = alloc_psram_buffer(jpeg_size);
    if (!job.jpeg) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable: no jpeg buffer",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        vSemaphoreDelete(ctx->done);
        free(ctx);
        return;
    }
    memcpy(job.jpeg, jpg_start, jpeg_size);

    bool queued = false;
    for (int attempt = 0; attempt < 10 && !queued; attempt++) {
        queued = s_inference_queue &&
                 xQueueSend(s_inference_queue, &job, pdMS_TO_TICKS(500)) == pdTRUE;
    }
    if (!queued) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s unavailable: inference queue busy",
                 trial, validation_sample_name(sample), recognition_method_name(method));
        free(job.jpeg);
        vSemaphoreDelete(ctx->done);
        free(ctx);
        return;
    }

    s_inference_jobs_queued++;
    if (xSemaphoreTake(ctx->done, pdMS_TO_TICKS(45000)) != pdTRUE) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s timeout: sample=%s method=%s jpeg=%" PRIu32,
                 trial, validation_sample_name(sample), recognition_method_name(method), jpeg_size);
        return;
    }

    const vision_result_t vision = ctx->vision;
    const uint32_t source_w = ctx->source_w;
    const uint32_t source_h = ctx->source_h;
    const int64_t queued_wait_ms = ctx->completed_ms > ctx->queued_ms ?
                                   ctx->completed_ms - ctx->queued_ms : 0;
    const bool latency_ok = vision.analysis_ms > 0 &&
                            vision.analysis_ms < BOARD_IMAGE_VALIDATION_MAX_ANALYSIS_MS;
    const char *top_label = vision.detection_count > 0 ? vision.detections[0].label : "none";
    const uint32_t top_score = vision.detection_count > 0 ? vision.detections[0].score : 0;

    ESP_LOGI(TAG,
             "BOARD_IMAGE_VALIDATION trial=%s sample=%s method=%s latency_ok=%s model=%s model_bytes=%" PRIu32
             " input=%" PRIu32 " source=%" PRIu32 "x%" PRIu32 " jpeg=%" PRIu32
             " inference_ms=%" PRId64 " analysis_ms=%" PRId64 " queue_total_ms=%" PRId64
             " detections=%" PRIu32 " top=%s top_score=%" PRIu32,
             trial, validation_sample_name(sample), recognition_method_name(method),
             latency_ok ? "true" : "false", model_name_for_method(method),
             model_bytes_for_method(method), model_input_size_for_method(method),
             source_w, source_h, jpeg_size, vision.inference_ms, vision.analysis_ms,
             queued_wait_ms, vision.detection_count, top_label, top_score);

    if (!latency_ok) {
        ESP_LOGE(TAG, "BOARD_IMAGE_VALIDATION trial=%s method=%s latency requirement failed: analysis_ms=%" PRId64,
                 trial, recognition_method_name(method), vision.analysis_ms);
    }

    vSemaphoreDelete(ctx->done);
    free(ctx);
}

static void validation_selftest_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(500));

    typedef struct {
        validation_sample_t sample;
        recognition_method_t method;
        uint32_t threshold;
    } validation_selftest_case_t;

    static const validation_selftest_case_t cases[] = {
        {VALIDATION_SAMPLE_DEMO_01, RECOGNITION_METHOD_COCO, 50},
        {VALIDATION_SAMPLE_DEMO_02, RECOGNITION_METHOD_COCO, 50},
        {VALIDATION_SAMPLE_DEMO_03, RECOGNITION_METHOD_COCO, 50},
        {VALIDATION_SAMPLE_DEMO_04, RECOGNITION_METHOD_COCO, 50},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        run_board_image_validation_case(cases[i].sample, cases[i].method, cases[i].threshold, "warmup");
        vTaskDelay(pdMS_TO_TICKS(100));
        run_board_image_validation_case(cases[i].sample, cases[i].method, cases[i].threshold, "measure");
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    vTaskDelete(NULL);
}

static validation_sample_t parse_validation_sample(const char *text)
{
    if (strcmp(text, "coke") == 0 || strcmp(text, "cola") == 0) {
        return VALIDATION_SAMPLE_COKE;
    }
    if (strcmp(text, "sprite") == 0) {
        return VALIDATION_SAMPLE_SPRITE;
    }
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
    return VALIDATION_SAMPLE_NONE;
}

static recognition_method_t parse_validation_method(const char *text)
{
    if (strcmp(text, "yolo26") == 0) {
        return RECOGNITION_METHOD_YOLO26;
    }
    if (strcmp(text, "yolo11") == 0) {
        return RECOGNITION_METHOD_YOLO11;
    }
    if (strcmp(text, "coco") == 0) {
        return RECOGNITION_METHOD_COCO;
    }
    return recognition_method_is_yolo(s_recognition_method) ? s_recognition_method : RECOGNITION_METHOD_COCO;
}

static esp_err_t validation_json_error(httpd_req_t *req, const char *status, const char *error)
{
    char json[160];
    snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"%s\"}", error);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t validation_run_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (export_mode_reject(req, "validation run")) {
        return ESP_OK;
    }
    char query[160] = {0};
    char sample_text[16] = {0};
    char method_text[16] = {0};
    char box_min_text[8] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "sample", sample_text, sizeof(sample_text));
        httpd_query_key_value(query, "method", method_text, sizeof(method_text));
        httpd_query_key_value(query, "box_min_score", box_min_text, sizeof(box_min_text));
    }

    validation_sample_t sample = parse_validation_sample(sample_text);
    if (sample == VALIDATION_SAMPLE_NONE) {
        return validation_json_error(req, "400 Bad Request", "sample must be coke, sprite, demo_01, demo_02, demo_03, or demo_04");
    }

    recognition_method_t method = parse_validation_method(method_text);
    if (method == RECOGNITION_METHOD_YOLO26 && !yolo26_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "yolo26 model unavailable");
    }
    if (method == RECOGNITION_METHOD_YOLO11 && !yolo11_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "yolo11 model unavailable");
    }
    if (method == RECOGNITION_METHOD_COCO && !coco_espdl_available()) {
        return validation_json_error(req, "503 Service Unavailable", "coco model unavailable");
    }
    if (!recognition_method_is_yolo(method)) {
        return validation_json_error(req, "400 Bad Request", "validation supports coco, yolo11, or yolo26");
    }
    uint32_t validation_box_min_score = 50;
    if (box_min_text[0]) {
        validation_box_min_score = clamp_u32((uint32_t)strtoul(box_min_text, NULL, 10), 1, 100);
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

    validation_context_t *ctx = (validation_context_t *)calloc(1, sizeof(validation_context_t));
    if (!ctx) {
        return validation_json_error(req, "500 Internal Server Error", "no validation context");
    }
    ctx->done = xSemaphoreCreateBinary();
    if (!ctx->done) {
        free(ctx);
        return validation_json_error(req, "500 Internal Server Error", "no validation semaphore");
    }
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
    fill_yolo_pending(&job.meta.vision, method);
    job.jpeg = alloc_psram_buffer(jpeg_size);
    if (!job.jpeg) {
        vSemaphoreDelete(ctx->done);
        free(ctx);
        return validation_json_error(req, "500 Internal Server Error", "no validation jpeg buffer");
    }
    memcpy(job.jpeg, jpg_start, jpeg_size);

    if (!s_inference_queue || xQueueSend(s_inference_queue, &job, pdMS_TO_TICKS(1500)) != pdTRUE) {
        free(job.jpeg);
        vSemaphoreDelete(ctx->done);
        free(ctx);
        s_inference_queue_drops++;
        return validation_json_error(req, "409 Conflict", "inference queue busy");
    }

    s_inference_jobs_queued++;
    /* 验证请求是用户主动点击触发的，所以这里等待模型完成，再把完整 JSON 返回给网页。 */
    xSemaphoreTake(ctx->done, portMAX_DELAY);

    vision_result_t vision = ctx->vision;
    uint32_t source_w = ctx->source_w;
    uint32_t source_h = ctx->source_h;
    uint32_t result_id = ctx->id;
    int64_t queued_ms = ctx->queued_ms;
    int64_t completed_ms = ctx->completed_ms;
    esp_err_t run_err = ctx->err;
    vSemaphoreDelete(ctx->done);
    free(ctx);

    bool expected_any_object = strcmp(validation_sample_expected(sample), "object") == 0;
    bool matched = expected_any_object ?
                   (vision.object_count > 0 && vision.detection_count > 0) :
                   (vision.object_count > 0 && strcmp(vision.object, validation_sample_name(sample)) == 0);
    char detections_json[1280];
    detections_to_json(detections_json, sizeof(detections_json), &vision);
    const size_t json_cap = 4096;
    char *json = (char *)alloc_psram_buffer(json_cap);
    if (!json) {
        return validation_json_error(req, "500 Internal Server Error", "no validation json buffer");
    }

    snprintf(json, json_cap,
             "{"
             "\"ok\":%s,\"id\":%" PRIu32 ",\"sample\":\"%s\",\"expected\":\"%s\","
             "\"method\":\"%s\",\"matched\":%s,\"source_image\":\"%s\","
             "\"overlay\":\"/api/validate/overlay.svg\",\"source_w\":%" PRIu32 ",\"source_h\":%" PRIu32 ","
             "\"jpeg_bytes\":%" PRIu32 ",\"model_bytes\":%" PRIu32 ",\"model_input_size\":%" PRIu32 ","
             "\"nms_threshold\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32 ",\"detection_count\":%" PRIu32 ","
             "\"detections\":%s,"
             "\"queued_ms\":%" PRId64 ",\"completed_ms\":%" PRId64 ","
             "\"error\":\"%s\","
             "\"vision\":{\"label\":\"%s\",\"object\":\"%s\",\"model\":\"%s\","
             "\"object_score\":%" PRIu32 ",\"candidate_score\":%" PRIu32 ",\"box_min_score\":%" PRIu32 ","
             "\"object_count\":%" PRIu32 ",\"detection_count\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32 ","
             "\"detections\":%s,"
             "\"object_x\":%" PRIu32 ",\"object_y\":%" PRIu32 ","
             "\"object_w\":%" PRIu32 ",\"object_h\":%" PRIu32 ","
             "\"coke_score\":%" PRIu32 ",\"sprite_score\":%" PRIu32 ",\"unknown_score\":%" PRIu32 ","
             "\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64 "}"
             "}",
             run_err == ESP_OK ? "true" : "false", result_id,
             validation_sample_name(sample), validation_sample_expected(sample),
             recognition_method_name(method), matched ? "true" : "false",
             validation_sample_image_uri(sample),
             source_w, source_h, jpeg_size,
             model_bytes_for_method(method), model_input_size_for_method(method),
             (uint32_t)APP_YOLO_NMS_THRESHOLD_X100, vision.raw_candidate_count, vision.detection_count,
             detections_json,
             queued_ms, completed_ms,
             run_err == ESP_OK ? "" : esp_err_to_name(run_err),
             vision.label, vision.object, vision.model,
             vision.object_score, vision.candidate_score, vision.box_min_score,
             vision.object_count, vision.detection_count, vision.raw_candidate_count,
             detections_json,
             vision.object_x, vision.object_y,
             vision.object_w, vision.object_h,
             vision.coke_score, vision.sprite_score, vision.unknown_score,
             vision.inference_ms, vision.analysis_ms);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t validation_overlay_get_handler(httpd_req_t *req)
{
    record_http_request();
    bool has_requested_id = false;
    uint32_t requested_id = 0;
    char query[64] = {0};
    char id_text[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "id", id_text, sizeof(id_text)) == ESP_OK) {
        requested_id = (uint32_t)strtoul(id_text, NULL, 10);
        has_requested_id = requested_id > 0;
    }

    validation_cache_t cache = {0};
    if (s_validation_lock) {
        xSemaphoreTake(s_validation_lock, portMAX_DELAY);
        cache = s_validation_last;
        xSemaphoreGive(s_validation_lock);
    }
    if (!cache.valid || cache.source_w == 0 || cache.source_h == 0) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no validation result");
    }
    if (has_requested_id && cache.id != requested_id) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "validation result id mismatch");
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
    record_http_request();
    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"mode\":\"%s\",\"tf_mounted\":%s,\"storage\":\"%s\"}",
             app_mode_name(s_app_mode),
             s_sd_mounted ? "true" : "false", s_storage_backend);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    record_http_request();
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
    int64_t age_ms = have_frame ? now_ms - meta.timestamp_ms : -1;
    int64_t last_network_activity_ms = __atomic_load_n(
        &s_last_network_activity_ms, __ATOMIC_ACQUIRE);
    int64_t network_idle_ms = last_network_activity_ms > 0 ?
                              now_ms - last_network_activity_ms : -1;
    int64_t network_boot_remaining_ms = s_network_boot_window_until_ms > now_ms ?
                                        s_network_boot_window_until_ms - now_ms : 0;
    power_state_t state = s_power_state;
    usb_msc_export_status_t usb_status = {0};
    usb_msc_export_get_status(&usb_status);
    uint32_t history_count = 0;
    uint32_t free_heap = 0;
    uint32_t min_free_heap = 0;
    uint32_t free_psram = 0;
    uint32_t min_free_psram = 0;
    int rssi = wifi_rssi();
    sample_memory_stats(&free_heap, &min_free_heap, &free_psram, &min_free_psram);
    char detections_json[1280];
    char dataset_labels_json[512];
    char ap_url[40] = {0};
    char sta_url[40] = {0};
    char eth_url[40] = {0};
    char mdns_url[64] = {0};
    char access_urls[256] = {0};
    char enrichment_json[768] = {0};
    const char *primary_ip = s_ip_addr;
    dataset_run_status_t dataset_status;
    recording_enrichment_status_t enrichment_status = {0};
    detections_to_json(detections_json, sizeof(detections_json), &meta.vision);
    dataset_status_copy(&dataset_status);
    recording_enrichment_get_status(&enrichment_status);
    label_counts_to_json(dataset_labels_json, sizeof(dataset_labels_json), dataset_status.labels);
    bool tf_ready = storage_tf_ready();
    bool acceptance_ok = storage_acceptance_ok();
    uint64_t min_recording_free = recording_min_free_bytes();
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
    snprintf(access_urls, sizeof(access_urls),
             "{\"mdns\":\"%s\",\"ap\":\"%s\",\"sta\":\"%s\",\"eth\":\"%s\"}",
             mdns_url, ap_url, sta_url, eth_url);
    snprintf(enrichment_json, sizeof(enrichment_json),
             "{\"enabled\":%s,\"running\":%s,\"cancelled\":%s,"
             "\"raw_name\":\"%s\",\"output_name\":\"%s\","
             "\"pass_stride\":%" PRIu32 ",\"completed_stride\":%" PRIu32
             ",\"frame_index\":%" PRIu32 ",\"frame_count\":%" PRIu32
             ",\"inferred_frames\":%" PRIu32 ",\"output_frames\":%" PRIu32
             ",\"inference_coverage_x1000\":%" PRIu32
             ",\"passes_completed\":%" PRIu32 ",\"last_error\":\"%s\"}",
             enrichment_status.enabled ? "true" : "false",
             enrichment_status.running ? "true" : "false",
             enrichment_status.cancelled ? "true" : "false",
             enrichment_status.raw_name, enrichment_status.output_name,
             enrichment_status.pass_stride, enrichment_status.completed_stride,
             enrichment_status.frame_index, enrichment_status.frame_count,
             enrichment_status.inferred_frames, enrichment_status.output_frames,
             enrichment_status.inference_coverage_x1000,
             enrichment_status.passes_completed, enrichment_status.last_error);

    if (s_history_lock) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        history_count = s_history_count;
        xSemaphoreGive(s_history_lock);
    }

    if (meta.pixel_format) {
        fourcc_to_str(meta.pixel_format, fourcc);
    }

    snprintf(json, json_cap,
             "{"
             "\"ip\":\"%s\",\"target\":\"%s\",\"power_mode\":\"%s\","
             "\"app_mode\":\"%s\","
             "\"time_synced\":%s,\"time_source\":\"%s\",\"epoch_ms\":%" PRIu64 ","
             "\"network_mode\":\"%s\",\"recognition_method\":\"%s\",\"rescue_ap\":%s,"
             "\"sta_ip\":\"%s\",\"ap_ip\":\"%s\",\"eth_ip\":\"%s\",\"ap_clients\":%" PRIu32 ","
             "\"wifi_runtime_ready\":%s,\"wifi_started\":%s,\"wifi_init_failures\":%" PRIu32
             ",\"wifi_last_error\":\"%s\","
             "\"network_active\":%s,\"network_idle_ms\":%" PRId64 ","
             "\"network_boot_window_remaining_ms\":%" PRId64 ","
             "\"network_shutdown_for_idle\":%s,\"network_reopen_requires_reboot\":%s,"
             "\"hostname\":\"%s\",\"mdns_url\":\"%s\","
             "\"ap_url\":\"%s\",\"sta_url\":\"%s\",\"eth_url\":\"%s\",\"access_urls\":%s,"
              "\"eth_enabled\":%s,\"eth_started\":%s,\"eth_link_up\":%s,"
              "\"eth_got_ip\":%s,\"eth_static_fallback\":%s,\"eth_last_error\":\"%s\","
              "\"usb_msc_enabled\":%s,\"usb_initialized\":%s,\"usb_host_connected\":%s,"
              "\"usb_export_requested\":%s,\"usb_storage_owner\":\"%s\",\"usb_writable\":%s,"
              "\"storage_quiescing\":%s,\"file_download_clients\":%" PRIu32
              ",\"usb_last_error\":\"%s\",\"enrichment\":%s,"
             "\"config\":{\"box_min_score\":%" PRIu32 ",\"stream_max_fps\":%" PRIu32 ","
             "\"inference_interval_ms\":%" PRIu32 ",\"history_sample_interval_ms\":%" PRIu32 ","
             "\"jpeg_quality\":%" PRIu32 ","
             "\"yolo_input_size\":%" PRIu32 ",\"yolo_model_loaded\":%s,"
             "\"yolo26_available\":%s,\"yolo11_available\":%s,\"coco_available\":%s},"
             "\"camera_ready\":%s,\"video_hw_ready\":%s,"
             "\"camera_error\":\"%s\",\"vision_enabled\":%s,\"history_enabled\":%s,"
             "\"width\":%" PRIu32 ",\"height\":%" PRIu32 ",\"pixel_format\":\"%s\","
             "\"sensor_frame_rate\":%" PRIu32 ",\"capture_fps_x100\":%" PRIu32 ",\"stream_fps_x100\":%" PRIu32 ","
             "\"last_capture_ms\":%" PRId64 ",\"last_encode_ms\":%" PRId64 ","
             "\"last_frame_age_ms\":%" PRId64 ",\"last_jpeg_bytes\":%" PRIu32 ",\"frame_seq\":%" PRIu32 ","
             "\"frames\":%" PRIu32 ",\"capture_errors\":%" PRIu32 ",\"frame_drops\":%" PRIu32 ","
             "\"stream_clients\":%" PRIu32 ",\"max_stream_clients\":%d,\"stream_errors\":%" PRIu32 ","
             "\"stream_frames\":%" PRIu32 ",\"stream_bytes\":%" PRIu64 ","
             "\"inference_frames\":%" PRIu32 ",\"inference_fps_x100\":%" PRIu32 ","
             "\"dropped_inference_frames\":%" PRIu32 ",\"inference_busy\":%s,"
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
             "\"recording_summary_count\":%" PRIu32 ",\"recording_bytes\":%" PRIu64 ","
             "\"recording_current_uri\":\"%s\",\"recording_current_frames\":%" PRIu32 ","
             "\"recording_current_bytes\":%" PRIu64 ","
             "\"sd_mounted\":%s,\"file_storage_mounted\":%s,\"tf_card_mounted\":%s,"
             "\"tf_required\":true,\"tf_ready\":%s,\"storage_acceptance_ok\":%s,"
             "\"storage_index_version\":%" PRIu32 ",\"tf_min_accept_bytes\":%" PRIu64 ","
             "\"recording_min_free_bytes\":%" PRIu64 ","
             "\"storage_backend\":\"%s\",\"storage_status\":\"%s\",\"sd_mount_mode\":\"%s\","
             "\"sd_last_error\":\"%s\",\"sd_last_error_code\":%d,\"sd_attempts\":%" PRIu32
             ",\"sd_format_count\":%" PRIu32 ","
             "\"storage_service\":{\"mode\":\"%s\",\"status\":\"%s\",\"runs\":%" PRIu32
             ",\"last_mount_ok\":%s,\"last_mode\":\"%s\",\"last_error_code\":%d},"
             "\"sd_total_bytes\":%" PRIu64 ",\"sd_free_bytes\":%" PRIu64 ","
             "\"dataset_run\":{\"state\":\"%s\",\"queued\":%s,\"running\":%s,\"done\":%s,"
             "\"dataset\":\"%s\",\"run_id\":\"%s\","
             "\"processed\":%" PRIu32 ",\"ok_frames\":%" PRIu32 ",\"failed_frames\":%" PRIu32
             ",\"detection_total\":%" PRIu32 ",\"avg_analysis_ms\":%" PRIu32
             ",\"p95_analysis_ms\":%" PRIu32 ",\"max_analysis_ms\":%" PRIu32
             ",\"last_frame_index\":%" PRIu32 ",\"last_overlay_uri\":\"%s\","
             "\"labels\":%s,\"error\":\"%s\"},"
             "\"uptime_ms\":%" PRId64 ",\"rssi_dbm\":%d,"
             "\"free_heap\":%" PRIu32 ",\"min_free_heap\":%" PRIu32 ","
             "\"free_psram\":%" PRIu32 ",\"min_free_psram\":%" PRIu32 ","
             "\"vision\":{\"label\":\"%s\",\"object\":\"%s\",\"scene\":\"%s\",\"color\":\"%s\",\"model\":\"%s\",\"motion\":%s,"
             "\"motion_score\":%" PRIu32 ",\"edge_score\":%" PRIu32 ",\"avg_luma\":%" PRIu32 ","
             "\"avg_r\":%" PRIu32 ",\"avg_g\":%" PRIu32 ",\"avg_b\":%" PRIu32 ","
             "\"object_score\":%" PRIu32 ",\"candidate_score\":%" PRIu32 ",\"box_min_score\":%" PRIu32 ","
             "\"object_count\":%" PRIu32 ",\"detection_count\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32 ","
             "\"detections\":%s,"
             "\"object_x\":%" PRIu32 ",\"object_y\":%" PRIu32 ",\"object_w\":%" PRIu32 ",\"object_h\":%" PRIu32 ","
             "\"coke_score\":%" PRIu32 ",\"sprite_score\":%" PRIu32 ","
             "\"unknown_score\":%" PRIu32 ","
             "\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64 "}"
             "}",
             primary_ip, CONFIG_IDF_TARGET, power_state_name(state),
             app_mode_name(s_app_mode),
             epoch_ms > 0 && s_time_source != TIME_SOURCE_UNSYNCED ? "true" : "false",
             time_source_name(s_time_source), epoch_ms,
             network_mode_name(s_network_mode), recognition_method_name(s_recognition_method),
             s_rescue_ap_active ? "true" : "false",
             s_sta_ip_addr, s_ap_ip_addr, s_eth_ip_addr, s_ap_clients,
             s_wifi_runtime_ready ? "true" : "false",
             s_wifi_started ? "true" : "false",
             s_wifi_init_failures, s_wifi_last_error,
             s_network_active ? "true" : "false", network_idle_ms,
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
               usb_status.host_connected ? "true" : "false",
               s_usb_export_requested ? "true" : "false",
               s_usb_storage_ready ? "usb" : (s_sd_mounted ? "app" : "none"),
               usb_status.writable ? "true" : "false",
               s_storage_quiescing ? "true" : "false",
               __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE),
               s_usb_last_error, enrichment_json,
              s_box_min_score, s_stream_max_fps, (unsigned long)s_inference_interval_ms,
             s_history_sample_interval_ms, s_jpeg_quality,
             (unsigned long)active_yolo_input_size(), active_yolo_available() ? "true" : "false",
             yolo26_espdl_available() ? "true" : "false",
             yolo11_espdl_available() ? "true" : "false",
             coco_espdl_available() ? "true" : "false",
             state == POWER_STATE_RUNNING ? "true" : "false",
             s_video_hw_ready ? "true" : "false",
             s_camera_error, s_vision_enabled ? "true" : "false", s_history_enabled ? "true" : "false",
             meta.width, meta.height, fourcc, meta.sensor_fps, s_capture_fps_x100, s_stream_fps_x100,
             meta.capture_ms, meta.encode_ms, age_ms, meta.size, meta.seq,
             s_frames_total, s_capture_errors, s_frame_drops,
             s_stream_clients, CONFIG_APP_MAX_STREAM_CLIENTS, s_stream_errors,
             s_stream_frames_total, (uint64_t)s_stream_bytes_total,
             s_inference_frames_total, s_inference_fps_x100,
             s_inference_dropped_frames, s_inference_worker_busy ? "true" : "false",
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
             s_recording_summary_count, (uint64_t)s_recording_bytes,
             s_recording_current_uri, s_recording_current_frames,
             (uint64_t)s_recording_current_bytes,
             s_sd_mounted ? "true" : "false", s_sd_mounted ? "true" : "false",
             (s_sd_mounted && s_sd_card) ? "true" : "false",
             tf_ready ? "true" : "false", acceptance_ok ? "true" : "false",
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
             dataset_status.dataset, dataset_status.run_id,
             dataset_status.processed, dataset_status.ok_frames, dataset_status.failed_frames,
             dataset_status.detection_total, dataset_status.avg_analysis_ms,
             dataset_status.p95_analysis_ms, dataset_status.max_analysis_ms,
             dataset_status.last_frame_index, dataset_status.last_overlay_uri,
             dataset_labels_json, dataset_status.last_error,
             now_ms, rssi,
             free_heap, min_free_heap, free_psram, min_free_psram,
             meta.vision.label, meta.vision.object, meta.vision.scene, meta.vision.color, meta.vision.model,
             meta.vision.motion ? "true" : "false",
             meta.vision.motion_score, meta.vision.edge_score, meta.vision.avg_luma,
             meta.vision.avg_r, meta.vision.avg_g, meta.vision.avg_b,
             meta.vision.object_score, meta.vision.candidate_score, meta.vision.box_min_score,
             meta.vision.object_count, meta.vision.detection_count, meta.vision.raw_candidate_count,
             detections_json,
             meta.vision.object_x, meta.vision.object_y, meta.vision.object_w, meta.vision.object_h,
             meta.vision.coke_score, meta.vision.sprite_score,
             meta.vision.unknown_score,
             meta.vision.inference_ms,
             meta.vision.analysis_ms);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t frame_get_handler(httpd_req_t *req)
{
    record_http_request();
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

static esp_err_t vision_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char enabled[8] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "enabled", enabled, sizeof(enabled));
    }

    if (strcmp(enabled, "1") == 0 || strcmp(enabled, "true") == 0 || strcmp(enabled, "on") == 0) {
        s_vision_enabled = true;
        if (s_recognition_method == RECOGNITION_METHOD_OFF) {
            s_recognition_method = coco_espdl_available() ? RECOGNITION_METHOD_COCO : RECOGNITION_METHOD_MLP;
            save_setting_u8("method", (uint8_t)s_recognition_method);
        }
        s_last_inference_ms = 0;
        save_setting_u8("vision", 1);
    } else if (strcmp(enabled, "0") == 0 || strcmp(enabled, "false") == 0 || strcmp(enabled, "off") == 0) {
        s_vision_enabled = false;
        s_recognition_method = RECOGNITION_METHOD_OFF;
        s_last_inference_ms = 0;
        save_setting_u8("method", (uint8_t)s_recognition_method);
        save_setting_u8("vision", 0);
        s_prev_luma_valid = false;
    }

    char json[96];
    snprintf(json, sizeof(json), "{\"ok\":true,\"vision_enabled\":%s}", s_vision_enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t recognition_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[96] = {0};
    char method_text[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "method", method_text, sizeof(method_text));
    }

    if (strcmp(method_text, "off") == 0) {
        s_recognition_method = RECOGNITION_METHOD_OFF;
        s_vision_enabled = false;
        s_prev_luma_valid = false;
    } else if (strcmp(method_text, "mlp") == 0) {
        s_recognition_method = RECOGNITION_METHOD_MLP;
        s_vision_enabled = true;
    } else if (strcmp(method_text, "yolo26") == 0) {
        if (!yolo26_espdl_available()) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            return httpd_resp_sendstr(req,
                                      "yolo26 board backend is unavailable; use mlp or check models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl");
        }
        s_recognition_method = RECOGNITION_METHOD_YOLO26;
        s_vision_enabled = true;
    } else if (strcmp(method_text, "yolo11") == 0) {
        if (!yolo11_espdl_available()) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            return httpd_resp_sendstr(req, "yolo11 board backend is unavailable; check yolo11_coke_sprite_416_s8_p4.espdl");
        }
        s_recognition_method = RECOGNITION_METHOD_YOLO11;
        s_vision_enabled = true;
    } else if (strcmp(method_text, "coco") == 0) {
        if (!coco_espdl_available()) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            return httpd_resp_sendstr(req, "coco board backend is unavailable; enable COCO YOLO11n 320 flash model");
        }
        s_recognition_method = RECOGNITION_METHOD_COCO;
        s_vision_enabled = true;
    } else if (method_text[0] != '\0') {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "supported method: off, mlp, coco, yolo26, yolo11");
    }

    if (method_text[0] != '\0') {
        s_last_inference_ms = 0;
    }
    save_setting_u8("method", (uint8_t)s_recognition_method);
    save_setting_u8("vision", s_vision_enabled ? 1 : 0);

    char json[160];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"recognition_method\":\"%s\",\"vision_enabled\":%s}",
             recognition_method_name(s_recognition_method), s_vision_enabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static bool query_u32(const char *query, const char *key, uint32_t min_value,
                      uint32_t max_value, uint32_t *out_value)
{
    char text[24] = {0};
    if (!query || httpd_query_key_value(query, key, text, sizeof(text)) != ESP_OK) {
        return false;
    }
    int value = atoi(text);
    if (value < (int)min_value) {
        value = (int)min_value;
    } else if (value > (int)max_value) {
        value = (int)max_value;
    }
    *out_value = (uint32_t)value;
    return true;
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[320] = {0};
    char text[32] = {0};
    bool has_query = httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK;

    if (has_query && httpd_query_key_value(query, "method", text, sizeof(text)) == ESP_OK) {
        if (strcmp(text, "off") == 0) {
            s_recognition_method = RECOGNITION_METHOD_OFF;
            s_vision_enabled = false;
            s_prev_luma_valid = false;
        } else if (strcmp(text, "mlp") == 0) {
            s_recognition_method = RECOGNITION_METHOD_MLP;
            s_vision_enabled = true;
        } else if (strcmp(text, "yolo26") == 0) {
            if (!yolo26_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req,
                                          "yolo26 board backend is unavailable; use mlp or check models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl");
            }
            s_recognition_method = RECOGNITION_METHOD_YOLO26;
            s_vision_enabled = true;
        } else if (strcmp(text, "yolo11") == 0) {
            if (!yolo11_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "yolo11 board backend is unavailable; check yolo11_coke_sprite_416_s8_p4.espdl");
            }
            s_recognition_method = RECOGNITION_METHOD_YOLO11;
            s_vision_enabled = true;
        } else if (strcmp(text, "coco") == 0) {
            if (!coco_espdl_available()) {
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "coco board backend is unavailable; enable COCO YOLO11n 320 flash model");
            }
            s_recognition_method = RECOGNITION_METHOD_COCO;
            s_vision_enabled = true;
        }
        s_last_inference_ms = 0;
        save_setting_u8("method", (uint8_t)s_recognition_method);
        save_setting_u8("vision", s_vision_enabled ? 1 : 0);
    }

    if (has_query && httpd_query_key_value(query, "vision", text, sizeof(text)) == ESP_OK) {
        s_vision_enabled = strcmp(text, "0") != 0 && strcmp(text, "false") != 0 && strcmp(text, "off") != 0;
        if (!s_vision_enabled) {
            s_recognition_method = RECOGNITION_METHOD_OFF;
            s_prev_luma_valid = false;
        } else if (s_recognition_method == RECOGNITION_METHOD_OFF) {
            s_recognition_method = coco_espdl_available() ? RECOGNITION_METHOD_COCO : RECOGNITION_METHOD_MLP;
        }
        s_last_inference_ms = 0;
        save_setting_u8("method", (uint8_t)s_recognition_method);
        save_setting_u8("vision", s_vision_enabled ? 1 : 0);
    }

    if (has_query && httpd_query_key_value(query, "history", text, sizeof(text)) == ESP_OK) {
        s_history_enabled = strcmp(text, "0") != 0 && strcmp(text, "false") != 0 && strcmp(text, "off") != 0;
        save_setting_u8("history", s_history_enabled ? 1 : 0);
    }
    if (has_query && httpd_query_key_value(query, "recording", text, sizeof(text)) == ESP_OK) {
        s_recording_enabled = strcmp(text, "0") != 0 && strcmp(text, "false") != 0 && strcmp(text, "off") != 0;
        save_setting_u8("recording", s_recording_enabled ? 1 : 0);
    }

    if (has_query && httpd_query_key_value(query, "netmode", text, sizeof(text)) == ESP_OK) {
        network_mode_t mode = s_network_mode;
        if (strcmp(text, "sta") == 0) {
            mode = NETWORK_MODE_STA;
        } else if (strcmp(text, "softap") == 0 || strcmp(text, "ap") == 0) {
            mode = NETWORK_MODE_SOFTAP;
        } else if (strcmp(text, "apsta") == 0) {
            mode = NETWORK_MODE_APSTA;
        }
        if (s_netmode_queue && mode != s_network_mode) {
            xQueueOverwrite(s_netmode_queue, &mode);
        }
    }

    /*
     * /api/config 同时支持“读取”和“带查询参数写入”。所有调试参数都写入 NVS，
     * 现场测试时可以在网页上反复切换阈值、推理间隔和无线模式，复位后仍保留最后一次设置。
     */
    uint32_t value = 0;
    if (has_query && (query_u32(query, "box_min_score", 50, 100, &value) ||
                      query_u32(query, "box_min", 50, 100, &value))) {
        s_box_min_score = value;
        save_setting_u32("box_min", s_box_min_score);
    }
    if (has_query && (query_u32(query, "stream_max_fps", 1, 30, &value) ||
                      query_u32(query, "stream_fps", 1, 30, &value))) {
        s_stream_max_fps = value;
        save_setting_u32("stream_fps", s_stream_max_fps);
    }
    if (has_query && (query_u32(query, "inference_interval_ms", 0, 600000, &value) ||
                      query_u32(query, "inference_ms", 0, 600000, &value) ||
                      query_u32(query, "inf_ms", 0, 600000, &value))) {
        s_inference_interval_ms = value;
        save_setting_u32("inf_ms", s_inference_interval_ms);
    }
    if (has_query && (query_u32(query, "history_sample_interval_ms", 250, 600000, &value) ||
                      query_u32(query, "history_ms", 250, 600000, &value))) {
        s_history_sample_interval_ms = value;
        save_setting_u32("hist_ms", s_history_sample_interval_ms);
    }
    if (has_query && (query_u32(query, "jpeg_quality", 1, 100, &value) ||
                      query_u32(query, "jpeg_q", 1, 100, &value))) {
        s_jpeg_quality = value;
        save_setting_u32("jpeg_q", s_jpeg_quality);
        if (s_camera.valid) {
            set_camera_jpeg_quality(&s_camera, (int)s_jpeg_quality);
        }
    }

    char json[1200];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"recognition_method\":\"%s\",\"network_mode\":\"%s\","
             "\"rescue_ap\":%s,\"vision_enabled\":%s,\"history_enabled\":%s,\"recording_enabled\":%s,"
             "\"box_min_score\":%" PRIu32 ",\"stream_max_fps\":%" PRIu32 ","
             "\"inference_interval_ms\":%" PRIu32 ",\"history_sample_interval_ms\":%" PRIu32 ","
             "\"jpeg_quality\":%" PRIu32 ","
             "\"yolo_input_size\":%" PRIu32 ",\"yolo_model_loaded\":%s,"
             "\"yolo26_available\":%s,\"yolo11_available\":%s,\"coco_available\":%s,"
             "\"model_info\":{\"name\":\"%s\",\"bytes\":%" PRIu32 ",\"input_size\":%" PRIu32 ","
             "\"class_count\":%" PRIu32 ",\"max_detections\":%" PRIu32 ",\"nms_threshold\":%" PRIu32 "}}",
             recognition_method_name(s_recognition_method), network_mode_name(s_network_mode),
             s_rescue_ap_active ? "true" : "false",
             s_vision_enabled ? "true" : "false", s_history_enabled ? "true" : "false",
             s_recording_enabled ? "true" : "false",
             s_box_min_score, s_stream_max_fps, (unsigned long)s_inference_interval_ms,
             s_history_sample_interval_ms, s_jpeg_quality,
             (unsigned long)active_yolo_input_size(), active_yolo_available() ? "true" : "false",
             yolo26_espdl_available() ? "true" : "false",
             yolo11_espdl_available() ? "true" : "false",
             coco_espdl_available() ? "true" : "false",
             model_name_for_method(s_recognition_method), active_model_bytes(),
             model_input_size_for_method(s_recognition_method),
             model_class_count_for_method(s_recognition_method), (uint32_t)APP_MAX_DETECTIONS,
             (uint32_t)APP_YOLO_NMS_THRESHOLD_X100);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t history_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 20;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) == ESP_OK) {
        int parsed = atoi(limit_text);
        if (parsed > 0 && parsed <= 64) {
            limit = parsed;
        }
    }

    size_t cap = 768 + (size_t)limit * 2300;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no history buffer");
    }

    uint32_t count = 0;
    size_t off = 0;
    char *item = (char *)alloc_psram_buffer(4096);
    if (!item) {
        free(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no history item buffer");
    }

    off += snprintf(json + off, cap - off,
                    "{\"count\":%" PRIu32 ",\"saved\":%" PRIu32 ",\"queued\":%" PRIu32 ","
                    "\"dropped\":%" PRIu32 ",\"deleted\":%" PRIu32 ",\"sd_errors\":%" PRIu32 ","
                    "\"sd_mounted\":%s,\"storage_status\":\"%s\",\"records\":[",
                    s_history_count, s_history_saved, s_history_queued, s_history_dropped,
                    s_history_files_deleted, s_history_sd_errors,
                    s_sd_mounted ? "true" : "false", s_storage_status);

    if (s_history_lock && s_history_records) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        count = s_history_count < limit ? s_history_count : limit;
        for (uint32_t i = 0; i < count && off < cap; i++) {
            uint32_t idx = (s_history_head + CONFIG_APP_HISTORY_MAX_RECORDS - 1 - i) %
                           CONFIG_APP_HISTORY_MAX_RECORDS;
            history_record_to_json(item, 4096, &s_history_records[idx]);
            off += snprintf(json + off, cap - off, "%s%s", i ? "," : "", item);
        }
        xSemaphoreGive(s_history_lock);
    }

    snprintf(json + off, cap - off, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(item);
    free(json);
    return ret;
}

static void append_recent_jsonl_array(char *json, size_t cap, size_t *off,
                                      const char *path, uint32_t limit)
{
    if (!json || !off || !path || limit == 0 || *off >= cap) {
        if (json && off && *off < cap) {
            *off += snprintf(json + *off, cap - *off, "[]");
        }
        return;
    }

    FILE *file = fopen(path, "r");
    if (!file) {
        *off += snprintf(json + *off, cap - *off, "[]");
        return;
    }

    const size_t line_bytes = JSONL_TAIL_LINE_BYTES;
    char *lines = (char *)alloc_psram_buffer((size_t)limit * line_bytes);
    if (!lines) {
        fclose(file);
        *off += snprintf(json + *off, cap - *off, "[]");
        return;
    }
    memset(lines, 0, (size_t)limit * line_bytes);

    char *line = (char *)alloc_psram_buffer(line_bytes);
    if (!line) {
        free(lines);
        fclose(file);
        *off += snprintf(json + *off, cap - *off, "[]");
        return;
    }

    uint32_t total = 0;
    while (fgets(line, (int)line_bytes, file)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (line[0] != '{') {
            continue;
        }
        char *slot = lines + (size_t)(total % limit) * line_bytes;
        strlcpy(slot, line, line_bytes);
        total++;
    }

    *off += snprintf(json + *off, cap - *off, "[");
    uint32_t count = total < limit ? total : limit;
    uint32_t start = total > count ? total - count : 0;
    for (uint32_t i = 0; i < count && *off < cap; i++) {
        uint32_t idx = (start + i) % limit;
        const char *slot = lines + (size_t)idx * line_bytes;
        *off += snprintf(json + *off, cap - *off, "%s%s", i ? "," : "", slot);
    }
    *off += snprintf(json + *off, cap - *off, "]");

    free(line);
    free(lines);
    fclose(file);
}

static void append_recent_jsonl_wrapped_array(char *json, size_t cap, size_t *off,
                                              const char *path, uint32_t limit,
                                              const char *type, bool *need_comma)
{
    if (!json || !off || !path || !type || limit == 0 || *off >= cap) {
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
        if (line[0] != '{') {
            continue;
        }
        char *slot = lines + (size_t)(total % limit) * line_bytes;
        strlcpy(slot, line, line_bytes);
        total++;
    }
    fclose(file);

    uint32_t count = total < limit ? total : limit;
    uint32_t start = total > count ? total - count : 0;
    for (uint32_t i = 0; i < count && *off < cap; i++) {
        uint32_t idx = (start + i) % limit;
        const char *slot = lines + (size_t)idx * line_bytes;
        *off += snprintf(json + *off, cap - *off, "%s{\"type\":\"%s\",\"data\":%s}",
                         *need_comma ? "," : "", type, slot);
        *need_comma = true;
    }

    free(line);
    free(lines);
}

static esp_err_t timeline_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 50;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) == ESP_OK) {
        int parsed = atoi(limit_text);
        if (parsed > 0 && parsed <= 100) {
            limit = parsed;
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

    size_t off = 0;
    bool need_comma = false;
    off += snprintf(json + off, cap - off,
                    "{\"sd_mounted\":%s,\"storage_backend\":\"%s\",\"storage_status\":\"%s\","
                    "\"tf_ready\":%s,\"storage_acceptance_ok\":%s,\"index_version\":%" PRIu32
                    ",\"recording_enabled\":%s,"
                    "\"history_saved\":%" PRIu32 ",\"recording_segments\":%" PRIu32
                    ",\"summary_count\":%" PRIu32 ",\"timeline\":[",
                    s_sd_mounted ? "true" : "false", s_storage_backend, s_storage_status,
                    storage_tf_ready() ? "true" : "false",
                    storage_acceptance_ok() ? "true" : "false",
                    (uint32_t)APP_JSONL_INDEX_VERSION,
                    s_recording_enabled ? "true" : "false",
                    s_history_saved, s_recording_segments, s_recording_summary_count);

    if (s_history_lock && s_history_records) {
        xSemaphoreTake(s_history_lock, portMAX_DELAY);
        uint32_t count = s_history_count < limit ? s_history_count : limit;
        for (uint32_t i = 0; i < count && off < cap; i++) {
            uint32_t idx = (s_history_head + CONFIG_APP_HISTORY_MAX_RECORDS - 1 - i) %
                           CONFIG_APP_HISTORY_MAX_RECORDS;
            history_record_to_json(item, 4096, &s_history_records[idx]);
            off += snprintf(json + off, cap - off, "%s{\"type\":\"history\",\"data\":%s}",
                            need_comma ? "," : "", item);
            need_comma = true;
        }
        xSemaphoreGive(s_history_lock);
    }
    append_recent_jsonl_wrapped_array(json, cap, &off, RECORDING_INDEX_PATH, limit, "recording", &need_comma);
    append_recent_jsonl_wrapped_array(json, cap, &off, RECORDING_SUMMARY_PATH, limit, "summary", &need_comma);
    snprintf(json + off, cap - off, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(item);
    free(json);
    return ret;
}

static esp_err_t recordings_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 20;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) == ESP_OK) {
        int parsed = atoi(limit_text);
        if (parsed > 0 && parsed <= 100) {
            limit = parsed;
        }
    }

    size_t cap = 1536 + (size_t)limit * JSONL_TAIL_LINE_BYTES * 2U;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no recordings buffer");
    }

    size_t off = 0;
    off += snprintf(json + off, cap - off,
                    "{\"sd_mounted\":%s,\"storage_backend\":\"%s\",\"storage_status\":\"%s\","
                    "\"tf_ready\":%s,\"storage_acceptance_ok\":%s,\"index_version\":%" PRIu32 ","
                    "\"recording_enabled\":%s,\"segments\":%" PRIu32 ",\"frames\":%" PRIu32 ","
                    "\"queued\":%" PRIu32 ",\"dropped\":%" PRIu32 ",\"deleted\":%" PRIu32 ","
                    "\"sd_errors\":%" PRIu32 ",\"summary_count\":%" PRIu32 ",\"bytes\":%" PRIu64 ","
                    "\"current_uri\":\"%s\",\"current_frames\":%" PRIu32 ",\"current_bytes\":%" PRIu64 ","
                    "\"recordings\":",
                    s_sd_mounted ? "true" : "false", s_storage_backend, s_storage_status,
                    storage_tf_ready() ? "true" : "false",
                    storage_acceptance_ok() ? "true" : "false",
                    (uint32_t)APP_JSONL_INDEX_VERSION,
                    s_recording_enabled ? "true" : "false", s_recording_segments, s_recording_frames,
                    s_recording_queued, s_recording_dropped, s_recording_files_deleted,
                    s_recording_sd_errors, s_recording_summary_count, (uint64_t)s_recording_bytes,
                    s_recording_current_uri, s_recording_current_frames,
                    (uint64_t)s_recording_current_bytes);
    append_recent_jsonl_array(json, cap, &off, RECORDING_INDEX_PATH, limit);
    off += snprintf(json + off, cap - off, ",\"summaries\":");
    append_recent_jsonl_array(json, cap, &off, RECORDING_SUMMARY_PATH, limit);
    snprintf(json + off, cap - off, "}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t storage_files_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char limit_text[12] = {0};
    uint32_t limit = 200;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text)) == ESP_OK) {
        int parsed = atoi(limit_text);
        if (parsed > 0 && parsed <= 1000) {
            limit = (uint32_t)parsed;
        }
    }

    size_t cap = 256U + (size_t)limit * 192U;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "no storage files buffer");
    }

    size_t off = snprintf(json, cap,
                          "{\"ok\":true,\"sd_mounted\":%s,\"backend\":\"%s\",\"files\":[",
                          s_sd_mounted ? "true" : "false", s_storage_backend);
    uint32_t returned = 0;
    bool first = true;
    if (s_sd_mounted) {
        DIR *dir = opendir(RECORDING_DIR);
        if (dir) {
            struct dirent *entry;
            while (returned < limit && (entry = readdir(dir)) != NULL) {
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
                int written = snprintf(json + off, cap - off,
                                       "%s{\"name\":\"%s\",\"bytes\":%lld,\"part\":%s}",
                                       first ? "" : ",", name, (long long)st.st_size,
                                       has_suffix(name, ".part") ? "true" : "false");
                if (written < 0 || (size_t)written >= cap - off) {
                    break;
                }
                off += (size_t)written;
                first = false;
                returned++;
            }
            closedir(dir);
        }
    }
    snprintf(json + off, cap - off, "],\"returned\":%" PRIu32 "}", returned);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t http_socket_send_all(httpd_req_t *req, const char *buf, size_t len)
{
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd < 0 || !buf) {
        return ESP_FAIL;
    }

    size_t off = 0;
    while (off < len) {
        int sent = httpd_socket_send(req->handle, sockfd, buf + off, len - off, 0);
        if (sent <= 0) {
            return ESP_FAIL;
        }
        off += (size_t)sent;
    }
    return ESP_OK;
}

static esp_err_t send_file_response(httpd_req_t *req, const char *path, const char *type)
{
    if (s_storage_quiescing) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage is switching to USB ownership");
    }
    struct stat st = {0};
    if (stat(path, &st) != 0 || st.st_size < 0) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
    }
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "file not found");
    }
    __atomic_add_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);

    uint64_t start = 0;
    uint64_t end = st.st_size > 0 ? (uint64_t)st.st_size - 1U : 0;
    bool partial = false;
    char range[96] = {0};
    if (st.st_size > 0 &&
        httpd_req_get_hdr_value_str(req, "Range", range, sizeof(range)) == ESP_OK &&
        strncmp(range, "bytes=", 6) == 0) {
        char *spec = range + 6;
        char *dash = strchr(spec, '-');
        if (!dash || strchr(dash + 1, ',')) {
            close(fd);
            __atomic_sub_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);
            httpd_resp_set_status(req, "416 Range Not Satisfiable");
            return httpd_resp_sendstr(req, "bad range");
        }
        *dash = '\0';
        if (spec[0]) {
            start = strtoull(spec, NULL, 10);
            if (dash[1]) {
                end = strtoull(dash + 1, NULL, 10);
            }
        } else if (dash[1]) {
            uint64_t suffix = strtoull(dash + 1, NULL, 10);
            if (suffix > (uint64_t)st.st_size) {
                suffix = st.st_size;
            }
            start = (uint64_t)st.st_size - suffix;
        }
        if (start >= (uint64_t)st.st_size || end < start) {
            close(fd);
            __atomic_sub_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);
            httpd_resp_set_status(req, "416 Range Not Satisfiable");
            return httpd_resp_sendstr(req, "range outside file");
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
        __atomic_sub_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "seek failed");
    }

    uint8_t *buf = malloc(HTTP_SAFE_CHUNK_BYTES);
    if (!buf) {
        close(fd);
        __atomic_sub_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
    }
    esp_err_t ret = ESP_OK;
    uint64_t remaining = st.st_size > 0 ? end - start + 1U : 0;
    bool fixed_length = remaining <= (uint64_t)INT_MAX;
    if (fixed_length) {
        ret = httpd_resp_send(req, NULL, (ssize_t)remaining);
        if (ret == ESP_OK) {
            while (remaining > 0) {
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
    __atomic_sub_fetch(&s_file_download_clients, 1, __ATOMIC_ACQ_REL);

    if (ret == ESP_OK && !fixed_length) {
        ret = httpd_resp_send_chunk(req, NULL, 0);
    }
    return ret;
}

static esp_err_t history_file_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }
    return send_file_response(req, HISTORY_JSONL_PATH, "application/x-ndjson");
}

static esp_err_t snapshot_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    const char *name = req->uri + strlen(SNAPSHOT_URI_PREFIX);
    if (!is_safe_snapshot_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad snapshot name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", HISTORY_SNAPSHOT_DIR, name);
    return send_file_response(req, path, "image/jpeg");
}

static esp_err_t recording_async_handler(httpd_req_t *req)
{
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    const char *name = req->uri + strlen(RECORDING_URI_PREFIX);
    if (!is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    xSemaphoreTake(s_storage_lock, portMAX_DELAY);
    esp_err_t ret = send_file_response(req, path,
                                       has_suffix(name, ".avi") ?
                                       "video/x-msvideo" : STREAM_CONTENT_TYPE);
    xSemaphoreGive(s_storage_lock);
    return ret;
}

static esp_err_t recording_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }
    const char *name = req->uri + strlen(RECORDING_URI_PREFIX);
    if (!is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording name");
    }
    if (queue_async_request(req, recording_async_handler) != ESP_OK) {
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no download worker available");
    }
    return ESP_OK;
}

static esp_err_t recording_meta_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (!s_sd_mounted) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "TF card is not mounted");
    }

    const char *name = req->uri + strlen(RECORDING_META_URI_PREFIX);
    if (!is_safe_recording_meta_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad recording meta name");
    }

    char path[384];
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, name);
    return send_file_response(req, path, "application/x-ndjson");
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
    if (!name || !time_ms) {
        return false;
    }
    const char *mark = strstr(name, "_t");
    if (!mark) {
        return false;
    }
    *time_ms = atoll(mark + 2);
    return *time_ms > 0;
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
    char path[384];
    const char *media_suffixes[] = {"", ".part", ".part.corrupt"};
    for (size_t i = 0; i < sizeof(media_suffixes) / sizeof(media_suffixes[0]); i++) {
        snprintf(path, sizeof(path), "%s/%s%s", RECORDING_DIR, name, media_suffixes[i]);
        int ret = delete_path_if_file(path, freed_bytes);
        if (ret > 0) {
            deleted += ret;
        } else if (ret < 0 && failed) {
            *failed = true;
        }
    }

    char meta_name[96];
    meta_name_for_recording(name, meta_name, sizeof(meta_name));
    snprintf(path, sizeof(path), "%s/%s", RECORDING_DIR, meta_name);
    int ret = delete_path_if_file(path, freed_bytes);
    if (ret > 0) {
        deleted += ret;
    } else if (ret < 0 && failed) {
        *failed = true;
    }
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
    unlink(backup_path);
    if (rename(path, backup_path) != 0) {
        unlink(tmp_path);
        return false;
    }
    if (rename(tmp_path, path) != 0) {
        rename(backup_path, path);
        unlink(tmp_path);
        return false;
    }
    unlink(backup_path);
    return true;
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
    if (!line || !key) {
        return -1;
    }
    char needle[40];
    snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char *p = strstr(line, needle);
    if (!p) {
        return -1;
    }
    return atoll(p + strlen(needle));
}

static uint32_t filter_jsonl_time_range(const char *path, const char *tmp_path,
                                        const char *key, int64_t from_ms, int64_t to_ms)
{
    FILE *in = fopen(path, "r");
    if (!in) {
        return 0;
    }
    FILE *out = fopen(tmp_path, "w");
    if (!out) {
        fclose(in);
        return 0;
    }

    uint32_t removed = 0;
    char *line = (char *)alloc_psram_buffer(JSONL_TAIL_LINE_BYTES);
    if (!line) {
        fclose(in);
        fclose(out);
        unlink(tmp_path);
        return 0;
    }

    while (fgets(line, JSONL_TAIL_LINE_BYTES, in)) {
        int64_t t = json_line_i64(line, key);
        bool in_range = t >= from_ms && t <= to_ms;
        if (in_range) {
            removed++;
        } else {
            fputs(line, out);
        }
    }
    free(line);
    bool failed = ferror(in);
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
        return 0;
    }
    if (removed == 0) {
        unlink(tmp_path);
        return 0;
    }
    if (!replace_jsonl_file(path, tmp_path)) {
        return 0;
    }
    return removed;
}

static uint32_t delete_recordings_in_range(int64_t from_ms, int64_t to_ms, uint64_t *freed_bytes)
{
    DIR *dir = opendir(RECORDING_DIR);
    if (!dir) {
        return 0;
    }
    uint32_t deleted = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        int64_t t = 0;
        if (!is_safe_recording_name(entry->d_name) ||
            !recording_time_from_name(entry->d_name, &t) ||
            t < from_ms || t > to_ms) {
            continue;
        }
        bool delete_failed = false;
        int ret = delete_recording_files_by_name(entry->d_name, freed_bytes, &delete_failed);
        if (ret > 0) {
            deleted += (uint32_t)ret;
        }
        if (delete_failed) {
            ESP_LOGW(TAG, "Failed to delete all files for recording %s", entry->d_name);
        }
    }
    closedir(dir);
    return deleted;
}

static uint32_t delete_dir_whitelist(const char *dir_path, const char *suffix_a,
                                     const char *suffix_b, const char *suffix_c,
                                     uint64_t *freed_bytes)
{
    DIR *dir = opendir(dir_path);
    if (!dir) {
        return 0;
    }
    uint32_t deleted = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        bool suffix_ok = has_suffix(entry->d_name, suffix_a) ||
                         (suffix_b && has_suffix(entry->d_name, suffix_b)) ||
                         (suffix_c && has_suffix(entry->d_name, suffix_c));
        if (!is_safe_snapshot_name(entry->d_name) || !suffix_ok) {
            continue;
        }
        char path[384];
        snprintf(path, sizeof(path), "%s/%s", dir_path, entry->d_name);
        int ret = delete_path_if_file(path, freed_bytes);
        if (ret > 0) {
            deleted += (uint32_t)ret;
        }
    }
    closedir(dir);
    return deleted;
}

static esp_err_t recording_delete_handler(httpd_req_t *req)
{
    record_http_request();
    char query[320] = {0};
    char name[96] = {0};
    char paired_name[96] = {0};
    if (!query_confirm_delete(req, query, sizeof(query)) ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name and confirm=DELETE required");
    }
    if (httpd_query_key_value(query, "paired_name", paired_name, sizeof(paired_name)) == ESP_OK) {
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

    xSemaphoreTake(s_storage_lock, portMAX_DELAY);
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
    xSemaphoreGive(s_storage_lock);

    bool failed = file_failed || index_failed;
    bool not_found = !failed && deleted == 0 && removed_rows == 0;
    if (deleted > 0) {
        s_recording_files_deleted += (uint32_t)deleted;
    }
    update_sd_info();

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
    record_http_request();
    char query[192] = {0};
    char from_text[24] = {0};
    char to_text[24] = {0};
    if (!query_confirm_delete(req, query, sizeof(query)) ||
        httpd_query_key_value(query, "from_ms", from_text, sizeof(from_text)) != ESP_OK ||
        httpd_query_key_value(query, "to_ms", to_text, sizeof(to_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "from_ms, to_ms and confirm=DELETE required");
    }
    int64_t from_ms = atoll(from_text);
    int64_t to_ms = atoll(to_text);
    if (!s_sd_mounted || from_ms <= 0 || to_ms < from_ms) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad range or TF not mounted");
    }

    xSemaphoreTake(s_storage_lock, portMAX_DELAY);
    uint64_t freed = 0;
    uint32_t deleted = delete_recordings_in_range(from_ms, to_ms, &freed);
    uint32_t filtered = 0;
    filtered += filter_jsonl_time_range(RECORDING_INDEX_PATH, RECORDING_INDEX_TMP_PATH, "start_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(RECORDING_INDEX_OLD_PATH, RECORDING_INDEX_OLD_TMP_PATH, "start_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_TMP_PATH, "start_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(RECORDING_SUMMARY_OLD_PATH, RECORDING_SUMMARY_OLD_TMP_PATH, "start_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(HISTORY_JSONL_PATH, HISTORY_JSONL_TMP_PATH, "time_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(HISTORY_JSONL_OLD_PATH, HISTORY_JSONL_OLD_TMP_PATH, "time_ms", from_ms, to_ms);
    filtered += filter_jsonl_time_range(EVENT_INDEX_PATH, EVENT_INDEX_TMP_PATH, "time_ms", from_ms, to_ms);
    xSemaphoreGive(s_storage_lock);
    s_recording_files_deleted += deleted;
    update_sd_info();

    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"deleted_files\":%" PRIu32 ",\"filtered_records\":%" PRIu32
             ",\"freed_bytes\":%" PRIu64 "}",
             deleted, filtered, freed);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_records_delete_handler(httpd_req_t *req)
{
    record_http_request();
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

    xSemaphoreTake(s_storage_lock, portMAX_DELAY);
    uint64_t freed = 0;
    uint32_t deleted = 0;
    deleted += delete_dir_whitelist(RECORDING_DIR, ".avi", ".mjpg", ".jsonl", &freed);
    deleted += delete_dir_whitelist(RECORDING_DIR, ".part", NULL, NULL, &freed);
    deleted += delete_dir_whitelist(RECORDING_DIR, ".corrupt", NULL, NULL, &freed);
    deleted += delete_dir_whitelist(HISTORY_SNAPSHOT_DIR, ".jpg", ".jpeg", NULL, &freed);
    deleted += delete_dir_whitelist(DATASET_RUN_DIR, ".jsonl", ".json", NULL, &freed);
    const char *indexes[] = {
        HISTORY_JSONL_PATH, HISTORY_JSONL_OLD_PATH,
        RECORDING_INDEX_PATH, RECORDING_INDEX_OLD_PATH,
        RECORDING_SUMMARY_PATH, RECORDING_SUMMARY_OLD_PATH,
        EVENT_INDEX_PATH, SESSION_INDEX_PATH,
    };
    for (size_t i = 0; i < sizeof(indexes) / sizeof(indexes[0]); i++) {
        int ret = delete_path_if_file(indexes[i], &freed);
        if (ret > 0) {
            deleted += (uint32_t)ret;
        }
    }
    xSemaphoreGive(s_storage_lock);
    s_recording_files_deleted += deleted;
    update_sd_info();

    char json[192];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"deleted_files\":%" PRIu32 ",\"freed_bytes\":%" PRIu64 "}",
             deleted, freed);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_format_handler(httpd_req_t *req)
{
    record_http_request();
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

    if (!s_sd_mounted || !s_sd_card) {
        if (!s_storage_service_queue) {
            httpd_resp_set_status(req, "503 Service Unavailable");
            return httpd_resp_sendstr(req, "storage service unavailable");
        }
        storage_service_request_t svc = {
            .hold_ms = 2000,
            .format_if_failed = true,
            .reboot_after = true,
        };
        if (xQueueSend(s_storage_service_queue, &svc, 0) != pdTRUE) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req, "storage service busy");
        }

        char queued_json[384];
        snprintf(queued_json, sizeof(queued_json),
                 "{\"ok\":true,\"queued\":true,\"format_if_mount_failed\":true,"
                 "\"reboot_after\":true,\"note\":\"Wi-Fi will go offline, TF will be probed with format enabled, then the board will reboot\"}");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return http_send_cstr_chunked(req, queued_json);
    }

    /*
     * 格式化只能在底层已经识别到 SD 卡时执行。若当前是 ESP_ERR_TIMEOUT，
     * 说明 CMD/CLK/D0 等链路没有卡响应，格式化命令根本到不了卡上。
     */
    s_recording_enabled = false;
    s_history_enabled = false;
    vTaskDelay(pdMS_TO_TICKS(1200));

    esp_err_t ret = ESP_OK;
    if (s_sd_mounted && s_sd_card) {
        esp_vfs_fat_mount_config_t cfg = {
            .format_if_mount_failed = true,
            .max_files = 8,
            .allocation_unit_size = 16 * 1024,
            .use_one_fat = true,
        };
        ESP_LOGW(TAG, "Formatting mounted TF card at %s by explicit HTTP request", CONFIG_APP_SD_MOUNT_POINT);
        ret = esp_vfs_fat_sdcard_format_cfg(CONFIG_APP_SD_MOUNT_POINT, s_sd_card, &cfg);
        if (ret == ESP_OK) {
            s_sd_format_count++;
            storage_prepare_dirs_after_mount(s_sd_mount_mode[0] ? s_sd_mount_mode : "formatted");
        } else {
            record_sd_mount_error("format", ret);
        }
    } else {
        ret = ESP_ERR_INVALID_STATE;
    }

    char json[320];
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"formatted_or_mounted\":%s,\"sd_mounted\":%s,"
             "\"sd_mount_mode\":\"%s\",\"error\":\"%s\",\"error_code\":%d,"
             "\"hint\":\"%s\"}",
             ret == ESP_OK ? "true" : "false", ret == ESP_OK ? "true" : "false",
             s_sd_mounted ? "true" : "false", s_sd_mount_mode,
             ret == ESP_OK ? "" : esp_err_to_name(ret), (int)ret,
             ret == ESP_ERR_TIMEOUT ?
             "card did not answer; format is impossible until SDMMC pins/power/card insertion are fixed" :
             "format request finished");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t storage_remount_handler(httpd_req_t *req)
{
    record_http_request();
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

    storage_service_request_t svc = {
        .hold_ms = 2000,
        .format_if_failed = false,
        .reboot_after = true,
    };
    if (httpd_query_key_value(query, "hold_ms", hold_text, sizeof(hold_text)) == ESP_OK) {
        unsigned long hold = strtoul(hold_text, NULL, 10);
        if (hold >= 100 && hold <= 30000) {
            svc.hold_ms = (uint32_t)hold;
        }
    }

    if (xQueueSend(s_storage_service_queue, &svc, 0) != pdTRUE) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "storage service busy");
    }

    char json[320];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":true,\"hold_ms\":%" PRIu32
             ",\"reboot_after\":true,"
             "\"note\":\"Wi-Fi/HTTP will go offline for TF probing; the board will reboot to restore AP+STA\"}",
             svc.hold_ms);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t field_mode_start_handler(httpd_req_t *req)
{
    record_http_request();
    char query[64] = {0};
    char confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK ||
        strcmp(confirm, "FIELD") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "confirm=FIELD required");
    }
    if (s_app_mode == APP_MODE_FIELD || s_network_shutdown_for_idle ||
        (s_app_mode != APP_MODE_SERVER && s_app_mode != APP_MODE_EXPORT)) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "field mode already active or pending");
    }
    if (s_stream_clients > 0 || s_inference_worker_busy ||
        s_dataset_status.queued || s_dataset_status.running) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "stream or inference task is active");
    }
    s_field_mode_requested = true;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req, "{\"ok\":true,\"mode\":\"field_pending\",\"reboot_to_return\":\"server\"}");
}

static esp_err_t export_mode_start_handler(httpd_req_t *req)
{
    record_http_request();
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
    if (!s_eth_started && eth_init_runtime() != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Ethernet unavailable");
    }
    s_export_mode_requested = true;
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
                                             vision_result_t *vision,
                                             uint32_t *source_w, uint32_t *source_h)
{
    if (!jpeg || !jpeg_size || !vision || !source_w || !source_h) {
        free(jpeg);
        return ESP_ERR_INVALID_ARG;
    }

    validation_context_t ctx = {
        .sample = VALIDATION_SAMPLE_NONE,
        .method = RECOGNITION_METHOD_COCO,
        .box_min_score = 50,
        .jpeg_size = jpeg_size,
        .queued_ms = esp_timer_get_time() / 1000,
    };
    ctx.done = xSemaphoreCreateBinary();
    if (!ctx.done) {
        free(jpeg);
        return ESP_ERR_NO_MEM;
    }

    inference_job_t job = {
        .jpeg = jpeg,
        .jpeg_size = jpeg_size,
        .method = RECOGNITION_METHOD_COCO,
        .box_min_score = 50,
        .validation = true,
        .validation_sample = VALIDATION_SAMPLE_NONE,
        .validation_ctx = &ctx,
        .queued_ms = ctx.queued_ms,
    };
    fill_yolo_pending(&job.meta.vision, RECOGNITION_METHOD_COCO);

    if (!s_inference_queue || xQueueSend(s_inference_queue, &job, pdMS_TO_TICKS(5000)) != pdTRUE) {
        free(jpeg);
        vSemaphoreDelete(ctx.done);
        s_inference_queue_drops++;
        return ESP_ERR_TIMEOUT;
    }

    s_inference_jobs_queued++;
    if (xSemaphoreTake(ctx.done, pdMS_TO_TICKS(45000)) != pdTRUE) {
        vSemaphoreDelete(ctx.done);
        return ESP_ERR_TIMEOUT;
    }

    *vision = ctx.vision;
    *source_w = ctx.source_w;
    *source_h = ctx.source_h;
    esp_err_t err = ctx.err;
    vSemaphoreDelete(ctx.done);
    return err;
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
    return p ? p + strlen(pattern) : NULL;
}

static bool json_get_int64_field(const char *line, const char *key, int64_t *out)
{
    const char *p = json_find_number_value(line, key);
    if (!p || !out) {
        return false;
    }
    *out = strtoll(p, NULL, 10);
    return true;
}

static bool json_get_u32_field(const char *line, const char *key, uint32_t *out)
{
    const char *p = json_find_number_value(line, key);
    if (!p || !out) {
        return false;
    }
    *out = (uint32_t)strtoul(p, NULL, 10);
    return true;
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
        uint32_t v = (uint32_t)strtoul(p + strlen("\"score\":"), NULL, 10);
        if (v > max_score) {
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
        uint32_t idx = 0;
        if (json_get_u32_field(line, "frame_index", &idx) && idx == frame_index) {
            size_t len = strlen(line);
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
                line[--len] = '\0';
            }
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
        uint32_t idx = 0;
        if (json_get_u32_field(line, "index", &idx) && idx == frame_index) {
            size_t len = strlen(line);
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
                line[--len] = '\0';
            }
            found = true;
            break;
        }
    }
    fclose(file);
    return found;
}

static void json_escape_text(const char *src, char *dst, size_t dst_size)
{
    if (!dst || dst_size == 0) {
        return;
    }
    size_t off = 0;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    for (const char *p = src; *p && off + 1 < dst_size; p++) {
        unsigned char c = (unsigned char)*p;
        if ((c == '"' || c == '\\') && off + 2 < dst_size) {
            dst[off++] = '\\';
            dst[off++] = (char)c;
        } else if (c >= 0x20) {
            dst[off++] = (char)c;
        }
    }
    dst[off] = '\0';
}

typedef struct {
    char *json;
    size_t cap;
    size_t off;
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
    if (!ctx || !type || !line || line[0] != '{') {
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

    int n = snprintf(ctx->json + ctx->off, ctx->cap - ctx->off,
                     "%s{\"type\":\"%s\",\"data\":%s}",
                     ctx->need_comma ? "," : "", type, line);
    if (n < 0 || ctx->off + (size_t)n >= ctx->cap) {
        ctx->has_more = true;
        return true;
    }
    ctx->off += (size_t)n;
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

static esp_err_t search_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[384] = {0};
    char label[64] = {0};
    char type[16] = "all";
    char text[32] = {0};
    int64_t from_ms = 0;
    int64_t to_ms = 0;
    uint32_t min_score = 0;
    uint32_t limit = 50;
    uint32_t cursor = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "label", label, sizeof(label));
        if (httpd_query_key_value(query, "type", type, sizeof(type)) != ESP_OK) {
            strlcpy(type, "all", sizeof(type));
        }
        if (httpd_query_key_value(query, "from_ms", text, sizeof(text)) == ESP_OK) {
            from_ms = atoll(text);
        }
        text[0] = '\0';
        if (httpd_query_key_value(query, "to_ms", text, sizeof(text)) == ESP_OK) {
            to_ms = atoll(text);
        }
        text[0] = '\0';
        if (httpd_query_key_value(query, "min_score", text, sizeof(text)) == ESP_OK) {
            int parsed = atoi(text);
            if (parsed > 0) {
                min_score = (uint32_t)parsed;
            }
        }
        text[0] = '\0';
        if (httpd_query_key_value(query, "limit", text, sizeof(text)) == ESP_OK) {
            int parsed = atoi(text);
            if (parsed > 0) {
                limit = (uint32_t)parsed;
            }
        }
        text[0] = '\0';
        if (httpd_query_key_value(query, "cursor", text, sizeof(text)) == ESP_OK) {
            int parsed = atoi(text);
            if (parsed > 0) {
                cursor = (uint32_t)parsed;
            }
        }
    }
    if (limit == 0) {
        limit = 50;
    } else if (limit > 100) {
        limit = 100;
    }
    if (min_score > 100) {
        min_score = 100;
    }

    size_t cap = 2048U + (size_t)limit * (JSONL_TAIL_LINE_BYTES + 128U);
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no search buffer");
    }

    char safe_label[128];
    char safe_type[32];
    json_escape_text(label, safe_label, sizeof(safe_label));
    json_escape_text(type, safe_type, sizeof(safe_type));

    search_ctx_t ctx = {
        .json = json,
        .cap = cap,
        .off = 0,
        .label = label,
        .type_filter = type,
        .from_ms = from_ms,
        .to_ms = to_ms,
        .min_score = min_score,
        .limit = limit,
        .cursor = cursor,
    };

    ctx.off += snprintf(ctx.json + ctx.off, ctx.cap - ctx.off,
                        "{\"ok\":true,\"index_version\":%" PRIu32
                        ",\"storage_backend\":\"%s\",\"tf_ready\":%s,"
                        "\"storage_acceptance_ok\":%s,\"sd_mounted\":%s,"
                        "\"query\":{\"label\":\"%s\",\"type\":\"%s\","
                        "\"from_ms\":%" PRId64 ",\"to_ms\":%" PRId64
                        ",\"min_score\":%" PRIu32 ",\"limit\":%" PRIu32
                        ",\"cursor\":%" PRIu32 "},\"results\":[",
                        (uint32_t)APP_JSONL_INDEX_VERSION, s_storage_backend,
                        storage_tf_ready() ? "true" : "false",
                        storage_acceptance_ok() ? "true" : "false",
                        s_sd_mounted ? "true" : "false",
                        safe_label, safe_type, from_ms, to_ms,
                        min_score, limit, cursor);

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

    uint32_t next_cursor = cursor + ctx.returned;
    if (ctx.has_more) {
        next_cursor = cursor + ctx.returned;
    }
    snprintf(ctx.json + ctx.off, ctx.cap - ctx.off,
             "],\"returned\":%" PRIu32 ",\"matched_seen\":%" PRIu32
             ",\"next_cursor\":%" PRIu32 ",\"has_more\":%s}",
             ctx.returned, ctx.matched, next_cursor, ctx.has_more ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t recording_frame_svg_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[160] = {0};
    char name[96] = {0};
    char frame_text[16] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        !is_safe_recording_name(name)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recording name invalid or TF not mounted");
    }
    if (httpd_query_key_value(query, "frame", frame_text, sizeof(frame_text)) != ESP_OK) {
        httpd_query_key_value(query, "index", frame_text, sizeof(frame_text));
    }
    uint32_t frame_index = (uint32_t)atoi(frame_text);
    if (frame_index == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "frame index required");
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

static esp_err_t recording_manifest_get_handler(httpd_req_t *req)
{
    record_http_request();
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

static esp_err_t recording_annotated_get_handler(httpd_req_t *req)
{
    record_http_request();
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
    record_http_request();
    char query[224] = {0};
    char dataset[DATASET_NAME_MAX] = {0};
    char run_id[80] = {0};
    char index_text[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "run_id", run_id, sizeof(run_id)) != ESP_OK ||
        !is_safe_snapshot_name(run_id)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "run_id invalid");
    }
    httpd_query_key_value(query, "dataset", dataset, sizeof(dataset));
    if (!is_safe_dataset_name(dataset)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dataset invalid");
    }
    if (httpd_query_key_value(query, "index", index_text, sizeof(index_text)) != ESP_OK) {
        httpd_query_key_value(query, "frame", index_text, sizeof(index_text));
    }
    uint32_t frame_index = (uint32_t)atoi(index_text);
    if (frame_index == 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "index required");
    }

    dataset_frame_cache_t cached = {0};
    if (dataset_frame_cache_copy(run_id, dataset, frame_index, &cached)) {
        const uint8_t *jpeg_start = NULL;
        const uint8_t *jpeg_end = NULL;
        if (is_builtin_coco_video_dataset(dataset)) {
            if (!builtin_coco_video_frame_image(frame_index, &jpeg_start, &jpeg_end)) {
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
        strlcpy(status.dataset, req.dataset, sizeof(status.dataset));
        strlcpy(status.run_id, req.run_id, sizeof(status.run_id));
        bool builtin_coco_video = is_builtin_coco_video_dataset(status.dataset);
        if (builtin_coco_video) {
            uint32_t available = 1U + (BUILTIN_COCO_VIDEO_FRAMES - 1U) / status.stride;
            if (status.limit > available) {
                status.limit = available;
            }
        }
        bool persist_results = s_app_mode == APP_MODE_FIELD;
        status.started_ms = esp_timer_get_time() / 1000;
        if (persist_results) {
            snprintf(status.result_uri, sizeof(status.result_uri),
                     "/api/dataset/run/results?run_id=%s", status.run_id);
            snprintf(status.summary_uri, sizeof(status.summary_uri),
                     "/api/dataset/run/results?run_id=%s&type=summary", status.run_id);
        }
        dataset_frame_cache_clear();
        dataset_status_update(&status);

        if (!s_sd_mounted && !builtin_coco_video) {
            strlcpy(status.last_error, "TF card is not mounted", sizeof(status.last_error));
            status.running = false;
            status.done = true;
            status.finished_ms = esp_timer_get_time() / 1000;
            dataset_status_update(&status);
            continue;
        }

        char result_path[512];
        char summary_path[512];
        snprintf(result_path, sizeof(result_path), "%s/%s.jsonl", DATASET_RUN_DIR, status.run_id);
        snprintf(summary_path, sizeof(summary_path), "%s/%s_summary.json", DATASET_RUN_DIR, status.run_id);
        FILE *result = NULL;
        if (persist_results) {
            ensure_dir(DATASET_RUN_DIR);
            result = fopen(result_path, "w");
            if (!result) {
                strlcpy(status.last_error, "open dataset result failed", sizeof(status.last_error));
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
        if (!latencies || !sorted || !detections) {
            free(latencies);
            free(sorted);
            free(detections);
            if (result) {
                fclose(result);
            }
            strlcpy(status.last_error, "dataset run buffer alloc failed", sizeof(status.last_error));
            status.running = false;
            status.done = true;
            status.finished_ms = esp_timer_get_time() / 1000;
            dataset_status_update(&status);
            continue;
        }
        uint64_t sum_analysis = 0;
        for (uint32_t i = 0; i < status.limit; i++) {
            dataset_run_status_t latest;
            dataset_status_copy(&latest);
            if (latest.cancel) {
                strlcpy(status.last_error, "cancelled", sizeof(status.last_error));
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
            if (builtin_coco_video) {
                const uint8_t *jpeg_start = NULL;
                const uint8_t *jpeg_end = NULL;
                if (builtin_coco_video_frame_image(frame_index, &jpeg_start, &jpeg_end)) {
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
                    fprintf(result,
                            "{\"index_version\":%" PRIu32 ",\"index\":%" PRIu32
                            ",\"ok\":false,\"dataset\":\"%s\",\"file\":\"frames/frame_%05" PRIu32 ".jpg\","
                            "\"overlay_uri\":\"%s\",\"error\":\"read failed\"}\n",
                            (uint32_t)APP_JSONL_INDEX_VERSION, frame_index,
                            status.dataset, frame_index, overlay_uri);
                }
                if (i == 0) {
                    strlcpy(status.last_error, "first dataset frame missing or unreadable", sizeof(status.last_error));
                }
                break;
            }

            vision_result_t vision = {0};
            uint32_t source_w = 0;
            uint32_t source_h = 0;
            esp_err_t err = run_jpeg_on_inference_queue(jpeg, jpeg_size, &vision, &source_w, &source_h);
            detections_to_json(detections, 1280, &vision);
            bool ok = err == ESP_OK;
            if (ok) {
                status.ok_frames++;
                status.detection_total += vision.detection_count;
                if (vision.analysis_ms > 0 && status.processed < DATASET_RUN_LATENCY_CAP) {
                    latencies[status.processed] = (uint32_t)vision.analysis_ms;
                    sum_analysis += (uint32_t)vision.analysis_ms;
                    if ((uint32_t)vision.analysis_ms > status.max_analysis_ms) {
                        status.max_analysis_ms = (uint32_t)vision.analysis_ms;
                    }
                }
                for (uint32_t d = 0; d < vision.detection_count && d < APP_MAX_DETECTIONS; d++) {
                    label_count_add(status.labels, vision.detections[d].label);
                }
            } else {
                status.failed_frames++;
                strlcpy(status.last_error, esp_err_to_name(err), sizeof(status.last_error));
            }
            if (ok) {
                dataset_frame_cache_store(&status, frame_index, &vision, source_w, source_h);
            }

            if (result) {
                fprintf(result,
                    "{\"index_version\":%" PRIu32 ",\"index\":%" PRIu32
                    ",\"ok\":%s,\"dataset\":\"%s\",\"file\":\"frames/frame_%05" PRIu32 ".jpg\","
                    "\"overlay_uri\":\"%s\","
                    "\"source_w\":%" PRIu32 ",\"source_h\":%" PRIu32 ",\"jpeg_bytes\":%" PRIu32
                    ",\"model\":\"%s\",\"model_bytes\":%" PRIu32 ",\"input\":%" PRIu32
                    ",\"box_min_score\":%" PRIu32 ",\"best_score\":%" PRIu32
                    ",\"candidate_score\":%" PRIu32 ",\"raw_candidate_count\":%" PRIu32
                    ",\"inference_ms\":%" PRId64 ",\"analysis_ms\":%" PRId64
                    ",\"detection_count\":%" PRIu32 ",\"detections\":%s,\"error\":\"%s\"}\n",
                    (uint32_t)APP_JSONL_INDEX_VERSION, frame_index,
                    ok ? "true" : "false", status.dataset, frame_index,
                    overlay_uri, source_w, source_h, jpeg_size, COCO_MODEL_NAME,
                    (uint32_t)coco_espdl_model_bytes(), (uint32_t)COCO_INPUT_SIZE,
                    vision.box_min_score, vision.object_score,
                    vision.candidate_score, vision.raw_candidate_count,
                    vision.inference_ms, vision.analysis_ms,
                    vision.detection_count, detections, ok ? "" : esp_err_to_name(err));
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
            fclose(result);
        }
        free(latencies);
        free(sorted);
        free(detections);

        char labels[512];
        label_counts_to_json(labels, sizeof(labels), status.labels);
        FILE *summary = persist_results ? fopen(summary_path, "w") : NULL;
        if (summary) {
            fprintf(summary,
                    "{\"index_version\":%" PRIu32 ",\"run_id\":\"%s\",\"dataset\":\"%s\","
                    "\"overlay_endpoint\":\"%s\",\"processed\":%" PRIu32
                    ",\"ok_frames\":%" PRIu32 ",\"failed_frames\":%" PRIu32
                    ",\"detection_total\":%" PRIu32 ",\"avg_analysis_ms\":%" PRIu32
                    ",\"p95_analysis_ms\":%" PRIu32 ",\"max_analysis_ms\":%" PRIu32
                    ",\"model\":\"%s\",\"model_bytes\":%" PRIu32 ",\"labels\":%s,"
                    "\"started_ms\":%" PRId64 ",\"finished_ms\":%" PRId64 ",\"error\":\"%s\"}\n",
                    (uint32_t)APP_JSONL_INDEX_VERSION, status.run_id, status.dataset,
                    DATASET_FRAME_SVG_URI, status.processed,
                    status.ok_frames, status.failed_frames, status.detection_total,
                    status.avg_analysis_ms, status.p95_analysis_ms, status.max_analysis_ms,
                    COCO_MODEL_NAME, (uint32_t)coco_espdl_model_bytes(), labels,
                    status.started_ms, esp_timer_get_time() / 1000, status.last_error);
            fclose(summary);
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
    record_http_request();
    size_t cap = 2048;
    char *json = (char *)alloc_psram_buffer(cap);
    if (!json) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no dataset buffer");
    }

    size_t off = 0;
    off += snprintf(json + off, cap - off,
                    "{\"sd_mounted\":%s,\"storage_status\":\"%s\",\"datasets\":["
                    "{\"name\":\"%s\",\"frames\":%" PRIu32
                    ",\"source\":\"firmware\",\"embedded\":true}",
                    s_sd_mounted ? "true" : "false", s_storage_status,
                    BUILTIN_COCO_VIDEO_DATASET, (uint32_t)BUILTIN_COCO_VIDEO_FRAMES);
    if (s_sd_mounted) {
        DIR *dir = opendir(DATASET_ROOT_DIR);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL && off < cap) {
                if (!is_safe_dataset_name(entry->d_name) ||
                    is_builtin_coco_video_dataset(entry->d_name)) {
                    continue;
                }
                uint32_t frames = count_dataset_frames(entry->d_name);
                off += snprintf(json + off, cap - off,
                                ",{\"name\":\"%s\",\"frames\":%" PRIu32
                                ",\"path\":\"%s/%s\",\"source\":\"storage\",\"embedded\":false}",
                                entry->d_name, frames, DATASET_ROOT_DIR, entry->d_name);
            }
            closedir(dir);
        }
    }
    snprintf(json + off, cap - off, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = http_send_cstr_chunked(req, json);
    free(json);
    return ret;
}

static esp_err_t ensure_dataset_parent_dirs(const char *dataset, const char *relpath)
{
    char dataset_dir[512];
    snprintf(dataset_dir, sizeof(dataset_dir), "%s/%s", DATASET_ROOT_DIR, dataset);
    if (ensure_dir(DATASET_ROOT_DIR) != ESP_OK || ensure_dir(dataset_dir) != ESP_OK) {
        return ESP_FAIL;
    }
    if (strncmp(relpath, "frames/", 7) == 0) {
        char frames_dir[512];
        strlcpy(frames_dir, dataset_dir, sizeof(frames_dir));
        strlcat(frames_dir, "/frames", sizeof(frames_dir));
        return ensure_dir(frames_dir);
    }
    return ESP_OK;
}

static esp_err_t dataset_file_put_handler(httpd_req_t *req)
{
    record_http_request();
    if (s_storage_quiescing) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage is switching to USB ownership");
    }
    char query[256] = {0};
    char dataset[DATASET_NAME_MAX] = {0};
    char relpath[DATASET_PATH_MAX] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "dataset", dataset, sizeof(dataset)) != ESP_OK ||
        httpd_query_key_value(query, "path", relpath, sizeof(relpath)) != ESP_OK ||
        !is_safe_dataset_name(dataset) || !is_safe_dataset_relpath(relpath)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dataset/path invalid or TF not mounted");
    }
    for (char *p = relpath; *p; p++) {
        if (*p == '\\') {
            *p = '/';
        }
    }
    if (ensure_dataset_parent_dirs(dataset, relpath) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "mkdir failed");
    }

    char path[512];
    snprintf(path, sizeof(path), "%s/%s/%s", DATASET_ROOT_DIR, dataset, relpath);
    FILE *file = fopen(path, "wb");
    if (!file) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open upload file failed");
    }

    char buf[2048];
    int remaining = req->content_len;
    uint32_t written_total = 0;
    while (remaining > 0) {
        int recv_len = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? sizeof(buf) : remaining);
        if (recv_len <= 0) {
            fclose(file);
            unlink(path);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload receive failed");
        }
        size_t written = fwrite(buf, 1, (size_t)recv_len, file);
        if (written != (size_t)recv_len) {
            fclose(file);
            unlink(path);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload write failed");
        }
        written_total += (uint32_t)recv_len;
        remaining -= recv_len;
    }
    fclose(file);
    update_sd_info();

    char json[160];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"dataset\":\"%s\",\"path\":\"%s\",\"bytes\":%" PRIu32 "}",
             dataset, relpath, written_total);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t dataset_run_start_handler(httpd_req_t *req)
{
    record_http_request();
    if (export_mode_reject(req, "dataset run")) {
        return ESP_OK;
    }
    char query[160] = {0};
    char dataset[DATASET_NAME_MAX] = BUILTIN_COCO_VIDEO_DATASET;
    char limit_text[12] = {0};
    char stride_text[12] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "dataset", dataset, sizeof(dataset));
        httpd_query_key_value(query, "limit", limit_text, sizeof(limit_text));
        httpd_query_key_value(query, "stride", stride_text, sizeof(stride_text));
    }
    bool builtin_coco_video = is_builtin_coco_video_dataset(dataset);
    if ((!s_sd_mounted && !builtin_coco_video) || !is_safe_dataset_name(dataset)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "dataset invalid or TF not mounted");
    }
    dataset_run_request_t run = {0};
    strlcpy(run.dataset, dataset, sizeof(run.dataset));
    run.limit = limit_text[0] ? (uint32_t)atoi(limit_text) : CONFIG_APP_DATASET_RUN_MAX_FRAMES;
    run.stride = stride_text[0] ? (uint32_t)atoi(stride_text) : 1;
    if (run.limit == 0 || run.limit > CONFIG_APP_DATASET_RUN_MAX_FRAMES) {
        run.limit = CONFIG_APP_DATASET_RUN_MAX_FRAMES;
    }
    if (run.stride == 0 || run.stride > 1000) {
        run.stride = 1;
    }
    if (builtin_coco_video) {
        uint32_t available = 1U + (BUILTIN_COCO_VIDEO_FRAMES - 1U) / run.stride;
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
             "\"dataset\":\"%s\",\"run_id\":\"%s\",\"limit\":%" PRIu32
             ",\"stride\":%" PRIu32 "}",
             run.dataset, run.run_id, run.limit, run.stride);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static esp_err_t dataset_run_status_handler(httpd_req_t *req)
{
    record_http_request();
    dataset_run_status_t status;
    dataset_status_copy(&status);
    char labels[512];
    label_counts_to_json(labels, sizeof(labels), status.labels);
    const char *state = status.queued ? "queued" :
                        status.running ? "running" :
                        status.done ? "done" : "idle";
    char json[2200];
    snprintf(json, sizeof(json),
             "{\"index_version\":%" PRIu32 ",\"state\":\"%s\","
             "\"queued\":%s,\"running\":%s,\"done\":%s,"
             "\"dataset\":\"%s\",\"run_id\":\"%s\",\"overlay_endpoint\":\"%s\","
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
             status.dataset, status.run_id, DATASET_FRAME_SVG_URI,
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

static esp_err_t dataset_run_results_handler(httpd_req_t *req)
{
    record_http_request();
    char query[128] = {0};
    char run_id[80] = {0};
    char type[16] = {0};
    if (!s_sd_mounted ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "run_id", run_id, sizeof(run_id)) != ESP_OK ||
        !is_safe_snapshot_name(run_id)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "run_id invalid or TF not mounted");
    }
    httpd_query_key_value(query, "type", type, sizeof(type));

    char path[512];
    if (strcmp(type, "summary") == 0) {
        snprintf(path, sizeof(path), "%s/%s_summary.json", DATASET_RUN_DIR, run_id);
        return send_file_response(req, path, "application/json");
    }
    snprintf(path, sizeof(path), "%s/%s.jsonl", DATASET_RUN_DIR, run_id);
    return send_file_response(req, path, "application/x-ndjson");
}

static esp_err_t stream_async_handler(httpd_req_t *req)
{
    if (s_storage_quiescing) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "storage is switching to USB ownership");
    }
    /*
     * 每个 /stream 连接都会进入自己的 HTTP worker。这里不做摄像头采集，
     * 只按 stream_max_fps 复制最新 JPEG 并分块发送 MJPEG，因此两台设备同时访问时不会互相抢摄像头。
     */
    uint8_t *client_buf = alloc_psram_buffer(s_frame_capacity);
    if (!client_buf) {
        s_stream_errors++;
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "no stream buffer");
    }

    esp_err_t ret = ESP_OK;
    uint32_t last_seq = 0;
    int64_t last_send_ms = 0;
    const int64_t min_interval_ms = s_stream_max_fps > 0 ? 1000 / (int64_t)s_stream_max_fps : 0;
    char header[288];

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    s_stream_clients++;
    mark_network_activity();

    while (true) {
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
        ret = http_send_chunk_part(req, (const char *)client_buf, meta.size);
        if (ret != ESP_OK) {
            break;
        }

        last_seq = meta.seq;
        last_send_ms = now_ms;
        mark_network_activity();
        s_stream_frames_total++;
        s_stream_bytes_total += meta.size;
        update_stream_fps(now_ms);
    }

    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
    } else {
        s_stream_errors++;
    }

    if (s_stream_clients > 0) {
        s_stream_clients--;
    }
    mark_network_activity();
    free(client_buf);
    return ret;
}

static esp_err_t queue_async_request(httpd_req_t *req, async_req_handler_t handler)
{
    httpd_req_t *copy = NULL;
    esp_err_t ret = httpd_req_async_handler_begin(req, &copy);
    if (ret != ESP_OK) {
        return ret;
    }

    async_req_t async_req = {
        .req = copy,
        .handler = handler,
    };

    if (xSemaphoreTake(s_async_worker_ready, 0) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        return ESP_ERR_NOT_FOUND;
    }

    if (xQueueSend(s_async_req_queue, &async_req, pdMS_TO_TICKS(100)) != pdTRUE) {
        httpd_req_async_handler_complete(copy);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static esp_err_t stream_get_handler(httpd_req_t *req)
{
    record_http_request();
    if (export_mode_reject(req, "stream")) {
        return ESP_OK;
    }
    power_state_t state = s_power_state;
    if (state == POWER_STATE_STANDBY || state == POWER_STATE_STOPPING) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "camera is in standby");
    }

    if (queue_async_request(req, stream_async_handler) != ESP_OK) {
        httpd_resp_set_status(req, "503 Busy");
        return httpd_resp_sendstr(req, "no stream worker available");
    }

    return ESP_OK;
}

static esp_err_t power_get_handler(httpd_req_t *req)
{
    record_http_request();
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
    record_http_request();
    char query[96] = {0};
    char epoch_text[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "epoch_ms", epoch_text, sizeof(epoch_text)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "epoch_ms required");
    }

    char *end = NULL;
    uint64_t epoch_ms = strtoull(epoch_text, &end, 10);
    if (!end || *end != '\0' ||
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

static esp_err_t netmode_get_handler(httpd_req_t *req)
{
    record_http_request();
    char query[96] = {0};
    char mode_text[16] = {0};
    network_mode_t mode = s_network_mode;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "mode", mode_text, sizeof(mode_text));
    }

    if (strcmp(mode_text, "sta") == 0) {
        mode = NETWORK_MODE_STA;
    } else if (strcmp(mode_text, "softap") == 0 || strcmp(mode_text, "ap") == 0) {
        mode = NETWORK_MODE_SOFTAP;
    } else if (strcmp(mode_text, "apsta") == 0) {
        mode = NETWORK_MODE_APSTA;
    } else if (mode_text[0] != '\0') {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "supported mode: sta, softap, apsta");
    }

    if (s_netmode_queue && mode != s_network_mode) {
        if (xQueueOverwrite(s_netmode_queue, &mode) != pdTRUE) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "network command queue failed");
        }
    }

    char json[160];
    snprintf(json, sizeof(json), "{\"ok\":true,\"requested\":\"%s\",\"current\":\"%s\"}",
             network_mode_name(mode), network_mode_name(s_network_mode));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(req, json);
}

static void async_worker_task(void *arg)
{
    while (true) {
        xSemaphoreGive(s_async_worker_ready);

        async_req_t async_req;
        if (xQueueReceive(s_async_req_queue, &async_req, portMAX_DELAY) == pdTRUE) {
            async_req.handler(async_req.req);
            if (httpd_req_async_handler_complete(async_req.req) != ESP_OK) {
                ESP_LOGW(TAG, "async request complete failed");
            }
        }
    }
}

static void history_task(void *arg)
{
    int64_t last_mount_attempt_ms = -10000;
    while (true) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (CONFIG_APP_SD_ENABLE && s_storage_mount_allowed &&
            !s_sd_mounted && now_ms - last_mount_attempt_ms >= 10000) {
            last_mount_attempt_ms = now_ms;
            esp_err_t ret = storage_mount();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "TF card unavailable, retrying in 10s: %s", esp_err_to_name(ret));
            }
        }

        history_item_t item;
        if (xQueueReceive(s_history_queue, &item, pdMS_TO_TICKS(1000)) == pdTRUE) {
            history_store_item(&item);
            free(item.jpeg);
        }
    }
}

static void recording_task(void *arg)
{
    (void)arg;
    static recording_segment_t raw_segment;
    static recording_segment_t annotated_segment;

    while (true) {
        recording_item_t item = {0};
        if (xQueueReceive(s_recording_queue, &item, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (item.finalize) {
                recording_close_segment(&raw_segment);
                recording_close_segment(&annotated_segment);
                if (item.finalize_done) {
                    xSemaphoreGive(item.finalize_done);
                }
                continue;
            }
            if (s_app_mode == APP_MODE_FIELD && s_sd_mounted && s_recording_enabled) {
                recording_segment_t *segment = item.kind == RECORDING_KIND_ANNOTATED ?
                                               &annotated_segment : &raw_segment;
                recording_write_item(segment, &item);
            }
            free(item.jpeg);
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        recording_segment_t *segments[] = {&raw_segment, &annotated_segment};
        for (size_t i = 0; i < sizeof(segments) / sizeof(segments[0]); i++) {
            recording_segment_t *segment = segments[i];
            if (segment->writer &&
                (s_app_mode != APP_MODE_FIELD || !s_sd_mounted || !s_recording_enabled ||
                (CONFIG_APP_RECORDING_SEGMENT_MS > 0 &&
                 now_ms - segment->start_ms >= (int64_t)CONFIG_APP_RECORDING_SEGMENT_MS) ||
                (CONFIG_APP_SUMMARY_INTERVAL_MS > 0 &&
                 now_ms - segment->start_ms >= (int64_t)CONFIG_APP_SUMMARY_INTERVAL_MS))) {
                recording_close_segment(segment);
            }
        }
    }
}

static bool enrichment_should_cancel(void *arg)
{
    (void)arg;
    if (!CONFIG_APP_ENRICHMENT_ENABLE || s_storage_quiescing ||
        s_usb_export_requested || s_field_mode_requested || s_export_mode_requested ||
        s_app_mode != APP_MODE_SERVER ||
        !s_network_active || s_network_shutdown_for_idle ||
        !s_sd_mounted || s_power_state != POWER_STATE_STANDBY ||
        s_stream_clients > 0 ||
        __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) > 0 ||
        inference_worker_busy() ||
        (s_inference_queue && uxQueueMessagesWaiting(s_inference_queue) > 0) ||
        (s_history_queue && uxQueueMessagesWaiting(s_history_queue) > 0) ||
        (s_recording_queue && uxQueueMessagesWaiting(s_recording_queue) > 0)) {
        return true;
    }

    dataset_run_status_t dataset = {0};
    dataset_status_copy(&dataset);
    if (dataset.queued || dataset.running) {
        return true;
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t last_activity_ms = __atomic_load_n(
        &s_last_network_activity_ms, __ATOMIC_ACQUIRE);
    return last_activity_ms <= 0 ||
           now_ms - last_activity_ms < CONFIG_APP_ENRICHMENT_IDLE_MS;
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

static esp_err_t usb_mode_start_handler(httpd_req_t *req)
{
    record_http_request();
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
    if (s_app_mode == APP_MODE_USB_EXPORT || s_storage_quiescing) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "USB export already active or pending");
    }
    s_usb_export_requested = true;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return http_send_cstr_chunked(
        req,
        "{\"ok\":true,\"mode\":\"usb_export_pending\","
        "\"note\":\"camera and network will stop; safe eject and reboot are required\"}");
#endif
}

static esp_err_t storage_init_usb_sdmmc_card(void)
{
    if (s_usb_sd_card) {
        return ESP_OK;
    }

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_APP_SD_LDO_IO_ID,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_pwr_ctrl);
    ESP_RETURN_ON_ERROR(ret, TAG, "USB TF LDO init failed");
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_GOTO_ON_ERROR(storage_reset_card_power(), fail_ldo, TAG, "USB TF power reset failed");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = CONFIG_APP_USB_MSC_SD_FREQ_KHZ;
    host.unaligned_multi_block_rw_max_chunk_size = 8;
    host.pwr_ctrl_handle = s_sd_pwr_ctrl;

    bool host_initialized_here = false;
    ret = host.init();
    if (ret == ESP_OK) {
        host_initialized_here = true;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SDMMC host already initialized before USB handoff");
        ret = ESP_OK;
    }
    ESP_GOTO_ON_ERROR(ret, fail_ldo, TAG, "USB SDMMC host init failed");

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = CONFIG_APP_SD_PIN_CLK;
    slot_config.cmd = CONFIG_APP_SD_PIN_CMD;
    slot_config.d0 = CONFIG_APP_SD_PIN_D0;
    slot_config.d1 = CONFIG_APP_SD_PIN_D1;
    slot_config.d2 = CONFIG_APP_SD_PIN_D2;
    slot_config.d3 = CONFIG_APP_SD_PIN_D3;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    ret = sdmmc_host_init_slot(host.slot, &slot_config);
    ESP_GOTO_ON_ERROR(ret, fail_host, TAG, "USB SDMMC slot init failed");

    s_usb_sd_card = calloc(1, sizeof(*s_usb_sd_card));
    ESP_GOTO_ON_FALSE(s_usb_sd_card, ESP_ERR_NO_MEM, fail_host, TAG,
                      "USB SDMMC card allocation failed");
    ret = sdmmc_card_init(&host, s_usb_sd_card);
    ESP_GOTO_ON_ERROR(ret, fail_card, TAG, "USB SDMMC card init failed");

    ESP_LOGI(TAG, "USB TF ready: SDMMC 4-bit at %d kHz", CONFIG_APP_USB_MSC_SD_FREQ_KHZ);
    sdmmc_card_print_info(stdout, s_usb_sd_card);
    return ESP_OK;

fail_card:
    free(s_usb_sd_card);
    s_usb_sd_card = NULL;
fail_host:
    if (host_initialized_here) {
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
            host.deinit_p(host.slot);
        } else {
            host.deinit();
        }
    }
fail_ldo:
    if (s_sd_pwr_ctrl) {
        sd_pwr_ctrl_del_on_chip_ldo(s_sd_pwr_ctrl);
        s_sd_pwr_ctrl = NULL;
    }
    return ret;
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

static void stop_webserver(void)
{
    if (!s_server) {
        return;
    }

    if (s_mdns_started) {
#if CONFIG_APP_MDNS_ENABLE
        mdns_free();
#endif
        s_mdns_started = false;
        ESP_LOGI(TAG, "mDNS stopped");
    }

    httpd_handle_t server = s_server;
    s_server = NULL;
    esp_err_t ret = httpd_stop(server);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HTTP server stopped");
    } else {
        ESP_LOGW(TAG, "HTTP server stop failed: %s", esp_err_to_name(ret));
    }
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

static void start_webserver(void)
{
    if (s_server) {
        ESP_LOGI(TAG, "HTTP server already running");
        mdns_start_runtime();
        return;
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
    config.max_uri_handlers = 64;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_ERROR_CHECK(httpd_start(&s_server, &config));
    mdns_start_runtime();
    open_network_access_window("HTTP server ready");

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
    const httpd_uri_t validate_coke = {
        .uri = "/validate/coke.jpg",
        .method = HTTP_GET,
        .handler = validate_coke_jpg_handler,
    };
    const httpd_uri_t validate_sprite = {
        .uri = "/validate/sprite.jpg",
        .method = HTTP_GET,
        .handler = validate_sprite_jpg_handler,
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
    const httpd_uri_t validate_run = {
        .uri = "/api/validate/run",
        .method = HTTP_GET,
        .handler = validation_run_get_handler,
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
        .handler = vision_get_handler,
    };
    const httpd_uri_t recognition = {
        .uri = "/api/recognition",
        .method = HTTP_GET,
        .handler = recognition_get_handler,
    };
    const httpd_uri_t config_api = {
        .uri = "/api/config",
        .method = HTTP_GET,
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
    const httpd_uri_t storage_records_delete = {
        .uri = "/api/storage/records",
        .method = HTTP_DELETE,
        .handler = storage_records_delete_handler,
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
    const httpd_uri_t storage_remount = {
        .uri = "/api/storage/remount",
        .method = HTTP_POST,
        .handler = storage_remount_handler,
    };
    const httpd_uri_t field_mode_start = {
        .uri = "/api/mode/field",
        .method = HTTP_POST,
        .handler = field_mode_start_handler,
    };
    const httpd_uri_t export_mode_start = {
        .uri = "/api/mode/export",
        .method = HTTP_POST,
        .handler = export_mode_start_handler,
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
        .handler = power_get_handler,
    };
    const httpd_uri_t time_sync = {
        .uri = "/api/time/sync",
        .method = HTTP_POST,
        .handler = time_sync_post_handler,
    };
    const httpd_uri_t netmode = {
        .uri = "/api/netmode",
        .method = HTTP_GET,
        .handler = netmode_get_handler,
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

    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_coke));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_sprite));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_demo_01));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_demo_02));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_demo_03));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_demo_04));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_run));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &validate_overlay));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &search));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &healthz));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &frame));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &vision));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recognition));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &config_api));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &history));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &timeline));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &timeline_delete));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &history_file));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recordings));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording_frame_svg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording_manifest));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording_delete));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &storage_records_delete));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &storage_files));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &storage_format));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &storage_remount));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &field_mode_start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &export_mode_start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &usb_mode_start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &usb_mode_start_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &datasets));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_file));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_run_start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_run_start_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_run_status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_run_results));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &dataset_frame_svg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &power));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &time_sync));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &netmode));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &stream));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &snapshot));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording_annotated));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &recording_meta));
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

    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

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
                 WIFI_SSID);
    } else if (network_mode_has_sta(mode)) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                               WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                               pdFALSE,
                                               pdFALSE,
                                               pdMS_TO_TICKS(10000));
        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "connected to %s", WIFI_SSID);
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

static void wifi_shutdown_for_storage_window(void)
{
    stop_webserver();
    s_network_active = false;
    s_network_shutdown_for_idle = true;
    s_stream_clients = 0;
    s_ap_clients = 0;

    if (s_wifi_started) {
        esp_err_t ret = esp_wifi_stop();
        ESP_LOGI(TAG, "Wi-Fi stop for storage window: %s", esp_err_to_name(ret));
        s_wifi_started = false;
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    esp_err_t deinit_ret = esp_wifi_deinit();
    ESP_LOGI(TAG, "Wi-Fi deinit for storage window: %s", esp_err_to_name(deinit_ret));
    s_wifi_initialized = false;
    s_wifi_runtime_ready = false;
    strlcpy(s_ip_addr, "0.0.0.0", sizeof(s_ip_addr));
    strlcpy(s_sta_ip_addr, "0.0.0.0", sizeof(s_sta_ip_addr));
}

static void wifi_stop_for_export_mode(void)
{
    s_ap_clients = 0;
    if (s_wifi_started) {
        esp_err_t ret = esp_wifi_stop();
        ESP_LOGI(TAG, "Wi-Fi stop for export mode: %s", esp_err_to_name(ret));
        s_wifi_started = false;
    }
    s_wifi_runtime_ready = false;
    strlcpy(s_ip_addr, s_eth_ip_addr[0] ? s_eth_ip_addr : "0.0.0.0", sizeof(s_ip_addr));
    strlcpy(s_sta_ip_addr, "0.0.0.0", sizeof(s_sta_ip_addr));
}

static void storage_service_task(void *arg)
{
    storage_service_request_t req;
    while (true) {
        if (xQueueReceive(s_storage_service_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        s_storage_service_runs++;
        s_storage_service_last_mount_ok = false;
        s_storage_service_last_error_code = 0;
        strlcpy(s_storage_service_last_mode, "none", sizeof(s_storage_service_last_mode));

        /*
         * Give the HTTP handler time to send the queued response before Wi-Fi
         * disappears for the TF maintenance reboot.
         */
        vTaskDelay(pdMS_TO_TICKS(800));
        set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK, "closing HTTP and Wi-Fi");
        wifi_shutdown_for_storage_window();

        /*
         * Hosted deinit tears down the global SDMMC host shared by both slots.
         * Remove FAT and Slot 0 while that host is still valid.
         */
        storage_unmount("before Hosted maintenance shutdown");

        set_storage_service_state(STORAGE_SERVICE_HOSTED_DOWN, "deinitializing ESP-Hosted transport");
#if CONFIG_ESP_HOSTED_ENABLED
        esp_err_t hosted_ret = (esp_err_t)esp_hosted_deinit();
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
        s_hosted_sdmmc_host_active = false;
#endif
        if (hosted_ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_hosted_deinit for storage returned %s", esp_err_to_name(hosted_ret));
        }
        vTaskDelay(pdMS_TO_TICKS(300));
#endif

        set_storage_service_state(STORAGE_SERVICE_MOUNTING, "mounting TF while Hosted is down");
        bool old_format = s_sd_format_requested;
        bool old_flash_fallback = s_storage_flash_fallback_enabled;
        s_sd_format_requested = req.format_if_failed;
        s_storage_flash_fallback_enabled = false;
        esp_err_t mount_ret = storage_mount();
        s_storage_flash_fallback_enabled = old_flash_fallback;
        s_storage_service_last_error_code = mount_ret;
        if (mount_ret == ESP_OK) {
            s_storage_service_last_mount_ok = true;
            strlcpy(s_storage_service_last_mode, s_sd_mount_mode, sizeof(s_storage_service_last_mode));
            set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                                      "TF mounted via %s for %" PRIu32 " ms",
                                      s_sd_mount_mode, req.hold_ms);
            update_sd_info();
            vTaskDelay(pdMS_TO_TICKS(req.hold_ms));
        } else {
            set_storage_service_state(STORAGE_SERVICE_ERROR, "TF mount failed: %s", esp_err_to_name(mount_ret));
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        s_sd_format_requested = old_format;

        set_storage_service_state(STORAGE_SERVICE_UNMOUNTING, "releasing TF before network recovery");
        storage_unmount("Wi-Fi restore");

        if (req.reboot_after) {
            /*
             * ESP-Hosted transport and the TF socket both need a clean bus state.
             * A full reboot is currently the reliable way to bring the Wi-Fi
             * co-processor back after a maintenance storage window; hot re-init
             * can leave the co-processor transport in a bad state on this build.
             */
            set_storage_service_state(STORAGE_SERVICE_RESTORING_NETWORK,
                                      "maintenance done; rebooting to restore AP+STA");
            vTaskDelay(pdMS_TO_TICKS(800));
            esp_restart();
        }

        set_storage_service_state(STORAGE_SERVICE_RESTORING_NETWORK, "restarting ESP-Hosted and AP+STA");
        esp_err_t wifi_ret = wifi_reinit_after_storage_window();
        if (wifi_ret == ESP_OK) {
            start_webserver();
            s_network_shutdown_for_idle = false;
            s_network_active = true;
            mark_network_activity();
            set_storage_service_state(STORAGE_SERVICE_IDLE,
                                      "done; last TF mount %s via %s",
                                      s_storage_service_last_mount_ok ? "ok" : "failed",
                                      s_storage_service_last_mode);
        } else {
            s_network_active = false;
            set_storage_service_state(STORAGE_SERVICE_ERROR,
                                      "Wi-Fi restore failed: %s", esp_err_to_name(wifi_ret));
        }
    }
}

static esp_err_t wait_for_enrichment_idle(uint32_t timeout_ms)
{
    int64_t deadline_ms = esp_timer_get_time() / 1000 + timeout_ms;
    while (esp_timer_get_time() / 1000 < deadline_ms) {
        recording_enrichment_status_t enrichment = {0};
        recording_enrichment_get_status(&enrichment);
        if (!enrichment.running) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static void enter_offline_tf_capture_mode(void)
{
    ESP_LOGI(TAG, "entering offline TF capture mode after idle network window");
    s_storage_quiescing = true;
    set_storage_service_state(STORAGE_SERVICE_STOPPING_NETWORK,
                              "closing HTTP, Wi-Fi and Ethernet for field recording");
    wifi_shutdown_for_storage_window();
    eth_stop_runtime("field capture");
    if (wait_for_enrichment_idle(10000) != ESP_OK) {
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "enrichment did not stop; reboot required");
        ESP_LOGE(TAG, "FIELD transition aborted because enrichment did not stop");
        return;
    }
    coco_espdl_release_background();

    s_storage_quiescing = false;
    s_app_mode = APP_MODE_FIELD;
    s_recognition_method = RECOGNITION_METHOD_COCO;
    s_vision_enabled = true;
    s_history_enabled = false;
    s_recording_enabled = true;
    s_box_min_score = 50;
    s_inference_interval_ms = 0;
    s_jpeg_quality = 70;
    s_last_inference_ms = 0;
    s_last_recording_frame_ms = 0;

    set_storage_service_state(STORAGE_SERVICE_HOSTED_DOWN,
                              "idle timeout; deinitializing ESP-Hosted transport");
#if CONFIG_ESP_HOSTED_ENABLED
    bool tf_on_sdmmc = strcmp(s_storage_backend, "tf_sdmmc") == 0;
    esp_err_t hosted_ret = ESP_OK;
    if (tf_on_sdmmc) {
        ESP_LOGI(TAG, "keeping ESP-Hosted SDMMC host initialized because TF is mounted via SDMMC");
    } else {
        hosted_ret = (esp_err_t)esp_hosted_deinit();
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
        s_hosted_sdmmc_host_active = false;
#endif
    }
    if (hosted_ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_hosted_deinit for offline TF capture returned %s",
                 esp_err_to_name(hosted_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
#endif

    set_storage_service_state(STORAGE_SERVICE_MOUNTING,
                              "preparing TF for offline camera recording");
    s_sd_format_requested = false;
    s_storage_flash_fallback_enabled = false;
    esp_err_t mount_ret = ESP_OK;
    if (s_sd_mounted && s_sd_card && storage_acceptance_ok()) {
        /*
         * Reuse the healthy mount during the server-to-field transition.
         * This avoids a FatFS drive-slot leak seen when unmounting and
         * immediately mounting the same card, and preserves SDMMC bus state
         * when the TF socket is running in 4-bit mode.
         */
        char mount_mode[sizeof(s_sd_mount_mode)];
        strlcpy(mount_mode, s_sd_mount_mode, sizeof(mount_mode));
        mount_ret = storage_prepare_dirs_after_mount(mount_mode);
    } else {
        storage_unmount("server-to-field storage reset");
        mount_ret = storage_mount();
    }
    if (mount_ret == ESP_OK && !storage_acceptance_ok()) {
        mount_ret = ESP_ERR_INVALID_SIZE;
    }
    s_storage_service_last_error_code = mount_ret;

    if (mount_ret == ESP_OK) {
        s_storage_service_last_mount_ok = true;
        strlcpy(s_storage_service_last_mode, s_sd_mount_mode, sizeof(s_storage_service_last_mode));
        update_sd_info();
        set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                                  "offline TF capture active via %s; reboot to reopen AP+STA",
                                  s_sd_mount_mode);
        camera_cmd_t wake = CAMERA_CMD_WAKE;
        xQueueSend(s_camera_cmd_queue, &wake, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "offline TF capture active via %s; reboot to reopen AP+STA",
                 s_sd_mount_mode);
    } else {
        s_storage_service_last_mount_ok = false;
        set_storage_service_state(STORAGE_SERVICE_ERROR,
                                  "offline TF capture TF mount failed: %s; reboot to retry AP+STA",
                                  esp_err_to_name(mount_ret));
        ESP_LOGW(TAG, "offline TF capture TF mount failed: %s", esp_err_to_name(mount_ret));
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
    s_stream_clients = 0;
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
    start_webserver();
    s_network_active = true;
    s_network_shutdown_for_idle = false;
    mark_network_activity();
    open_network_access_window("export mode");
    set_storage_service_state(STORAGE_SERVICE_IDLE,
                              "export mode active; Ethernet HTTP download only");
}

static void usb_host_event_callback(bool connected, void *arg)
{
    (void)arg;
    if (connected && CONFIG_APP_USB_MSC_AUTO_EXPORT &&
        s_app_mode != APP_MODE_USB_EXPORT && !s_storage_quiescing) {
        ESP_LOGI(TAG, "USB1 host detected; queueing writable TF export");
        s_usb_export_requested = true;
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
    while (esp_timer_get_time() / 1000 < deadline_ms) {
        dataset_run_status_t dataset = {0};
        recording_enrichment_status_t enrichment = {0};
        dataset_status_copy(&dataset);
        recording_enrichment_get_status(&enrichment);
        bool camera_idle = s_power_state == POWER_STATE_STANDBY ||
                           s_power_state == POWER_STATE_ERROR;
        bool queues_idle = (!s_history_queue || uxQueueMessagesWaiting(s_history_queue) == 0) &&
                           (!s_recording_queue || uxQueueMessagesWaiting(s_recording_queue) == 0) &&
                           (!s_inference_queue || uxQueueMessagesWaiting(s_inference_queue) == 0);
        if (camera_idle && queues_idle && !inference_worker_busy() &&
            !dataset.queued && !dataset.running && !enrichment.running &&
            __atomic_load_n(&s_file_download_clients, __ATOMIC_ACQUIRE) == 0) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_ERR_TIMEOUT;
}

static void enter_usb_export_mode(void)
{
#if CONFIG_APP_USB_MSC_ENABLE
    ESP_LOGI(TAG, "entering writable USB TF export mode");
    s_storage_quiescing = true;
    s_storage_mount_allowed = false;
    s_vision_enabled = false;
    s_history_enabled = false;
    cancel_dataset_for_storage_handoff();

    camera_cmd_t standby = CAMERA_CMD_STANDBY;
    xQueueSend(s_camera_cmd_queue, &standby, pdMS_TO_TICKS(100));

    /* Let a manual HTTP request finish before its transport disappears. */
    vTaskDelay(pdMS_TO_TICKS(500));
    wifi_shutdown_for_storage_window();
    eth_stop_runtime("USB mass storage");

    esp_err_t ret = wait_for_usb_quiescence(8000);
    if (ret == ESP_OK) {
        ret = recording_finalize_sync(pdMS_TO_TICKS(5000));
    }
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "quiesce failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s; TF remains with application until reboot", s_usb_last_error);
        return;
    }
    coco_espdl_release_background();

    s_recording_enabled = false;
    s_stream_clients = 0;
    s_last_recording_frame_ms = 0;
    s_app_mode = APP_MODE_USB_EXPORT;
    s_network_active = false;
    s_network_shutdown_for_idle = true;

    storage_unmount("USB mass-storage handoff");

#if CONFIG_ESP_HOSTED_ENABLED
    esp_err_t hosted_ret = (esp_err_t)esp_hosted_deinit();
#if CONFIG_ESP_HOSTED_SDIO_HOST_INTERFACE
    s_hosted_sdmmc_host_active = false;
#endif
    if (hosted_ret != ESP_OK) {
        ESP_LOGW(TAG, "ESP-Hosted deinit for USB returned %s", esp_err_to_name(hosted_ret));
    }
    vTaskDelay(pdMS_TO_TICKS(300));
#endif

    ret = storage_init_usb_sdmmc_card();
    if (ret == ESP_OK) {
        ret = usb_msc_export_attach_sdmmc(s_usb_sd_card);
    }
    if (ret != ESP_OK) {
        snprintf(s_usb_last_error, sizeof(s_usb_last_error),
                 "TF attach failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "%s; reboot to restore application storage", s_usb_last_error);
        return;
    }

    s_usb_storage_ready = true;
    strlcpy(s_usb_last_error, "ok", sizeof(s_usb_last_error));
    set_storage_service_state(STORAGE_SERVICE_AVAILABLE,
                              "USB host owns writable TF; safe eject then reboot");
    ESP_LOGI(TAG, "USB writable TF ready; safe eject, wait 2 seconds, then reboot");
#endif
}

static void network_watchdog_tick(void)
{
    if (s_usb_export_requested) {
        s_usb_export_requested = false;
        enter_usb_export_mode();
        return;
    }
    if (!s_network_active || s_network_shutdown_for_idle) {
        return;
    }
    if (s_field_mode_requested) {
        s_field_mode_requested = false;
        ESP_LOGI(TAG, "manual FIELD_MODE request accepted");
        enter_offline_tf_capture_mode();
        return;
    }
    if (s_export_mode_requested) {
        s_export_mode_requested = false;
        ESP_LOGI(TAG, "manual EXPORT_MODE request accepted");
        enter_export_mode();
        return;
    }
#if CONFIG_APP_ETH_ENABLE
    if (s_eth_started) {
        return;
    }
#endif

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (s_network_boot_window_until_ms > 0 && now_ms < s_network_boot_window_until_ms) {
        return;
    }

    int64_t last_activity_ms = __atomic_load_n(
        &s_last_network_activity_ms, __ATOMIC_ACQUIRE);
    int64_t idle_ms = last_activity_ms > 0 ? now_ms - last_activity_ms : now_ms;
    if (s_ap_clients > 0 || s_stream_clients > 0 ||
        idle_ms < CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS) {
        return;
    }

    ESP_LOGI(TAG,
             "network idle shutdown: idle_ms=%" PRId64 ", ap_clients=%" PRIu32
             ", stream_clients=%" PRIu32,
             idle_ms, s_ap_clients, s_stream_clients);
    enter_offline_tf_capture_mode();
}

static void network_task(void *arg)
{
    /*
     * 网络任务独立于 HTTP handler：网页请求只把目标模式写入队列，真正切换
     * 在这个任务里完成。这样即使 Wi-Fi 重启耗时较长，HTTP handler 也能快速返回。
    */
    int64_t last_wifi_retry_ms = -10000;
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
            if (mode == s_network_mode) {
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
                    start_webserver();
                    ESP_LOGI(TAG, "Camera web server recovered: http://%s/", s_ip_addr);
                }
            }
            network_watchdog_tick();
            if (!s_storage_boot_probe_queued &&
                CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS > 0 &&
                now_ms >= CONFIG_APP_STORAGE_TIMESHARE_BOOT_PROBE_MS &&
                !s_sd_mounted &&
                s_storage_service_mode == STORAGE_SERVICE_IDLE &&
                s_ap_clients == 0 &&
                s_stream_clients == 0 &&
                s_storage_service_queue) {
                storage_service_request_t req = {
                    .hold_ms = 1500,
                    .format_if_failed = false,
                    .reboot_after = true,
                };
                if (xQueueSend(s_storage_service_queue, &req, 0) == pdTRUE) {
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
    s_network_boot_window_until_ms = 0;
    strlcpy(s_ap_ip_addr, CONFIG_APP_AP_STATIC_IP, sizeof(s_ap_ip_addr));
    ESP_LOGI(TAG,
             "Network access window=%d ms after HTTP starts, idle timeout=%d ms, fixed AP URL=http://%s/",
             CONFIG_APP_NETWORK_BOOT_WINDOW_MS, CONFIG_APP_NETWORK_IDLE_TIMEOUT_MS, s_ap_ip_addr);
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

    s_recording_queue = xQueueCreate(CONFIG_APP_RECORDING_QUEUE_DEPTH, sizeof(recording_item_t));
    ESP_ERROR_CHECK(s_recording_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_recording_finalize_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_recording_finalize_done ? ESP_OK : ESP_ERR_NO_MEM);
    recording_enrichment_init(CONFIG_APP_ENRICHMENT_ENABLE);

    s_inference_queue = xQueueCreate(1, sizeof(inference_job_t));
    ESP_ERROR_CHECK(s_inference_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_dataset_run_queue = xQueueCreate(1, sizeof(dataset_run_request_t));
    ESP_ERROR_CHECK(s_dataset_run_queue ? ESP_OK : ESP_ERR_NO_MEM);

    s_storage_service_queue = xQueueCreate(1, sizeof(storage_service_request_t));
    ESP_ERROR_CHECK(s_storage_service_queue ? ESP_OK : ESP_ERR_NO_MEM);

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
        start_webserver();
        ESP_LOGI(TAG, "Ethernet HTTP server active; DHCP pending, fallback=http://%s/",
                 CONFIG_APP_ETH_STATIC_FALLBACK_IP);
    }
#endif
    esp_err_t wifi_ret = wifi_init_runtime();
    if (wifi_ret == ESP_OK) {
        start_webserver();
        ESP_LOGI(TAG, "Camera web server started; AP=http://%s/ STA=%s ETH=%s",
                 s_ap_ip_addr, s_sta_ip_addr, s_eth_ip_addr);
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
                                        4096, NULL, 5, &s_network_task_handle);
    ESP_ERROR_CHECK(network_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);

    /*
     * The boot COCO self-test is deliberately last. It still warms and proves
     * the model, but no longer competes with TF recovery and Hosted startup.
     */
    BaseType_t selftest_ok = xTaskCreate(validation_selftest_task, "image_selftest",
                                         4096, NULL, CONFIG_APP_INFERENCE_TASK_PRIORITY - 1,
                                         &s_validation_selftest_task_handle);
    ESP_ERROR_CHECK(selftest_ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM);
}
