idf_component_register(
    SRCS
        "camera_web_main.c"
        "avi_mjpeg_writer.c"
        "yolo26_espdl_bridge.cpp"
        "yolo11_espdl_bridge.cpp"
        "coco_espdl_bridge.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp-dl
        esp_event
        esp_http_server
        esp_netif
        esp_system
        esp_timer
        esp_wifi
        esp_video
        example_video_common
        fatfs
        wear_levelling
        sdmmc
        esp_driver_sdmmc
        esp_driver_sdspi
        esp_driver_spi
        esp_driver_gpio
        vfs
        yolo26
        coco_detect
        nvs_flash
)

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
else()
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()

set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)

# 自训练 Coke/Sprite YOLO26n。
# 导出路线：best.pt -> raw-head ONNX -> ESP-PPQ/ESP-DL -> .espdl。
# 输出必须为 one2one_p3_box/cls、one2one_p4_box/cls、one2one_p5_box/cls。
set(YOLO26_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl")
target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO26_MODEL_FILE}" BINARY)

# 自训练 Coke/Sprite YOLO11n。
# 导出路线：best.pt -> box0/score0/box1/score1/box2/score2 ONNX -> ESP-DL。
# 板端 bridge 复用 ESP-DL 官方 yolo11PostProcessor 做 DFL 解码与 NMS。
set(YOLO11_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo11_coke_sprite_416_s8_p4.espdl")
target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO11_MODEL_FILE}" BINARY)

# 板端验证图片直接嵌入固件，避免依赖当前不可用的 TF 卡。
# Coke/Sprite 样例保留为历史对比 API；主验证页使用 demo_01~04 的 COCO classic 多目标图。
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/phone_gallery/images/coke_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/phone_gallery/images/sprite_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_02.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_03.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_04.jpg" BINARY)

# Compact real-video validation sequence. The board infers each frame and the
# phone page replays the generated annotated overlays.
set(COCO_VIDEO_DEMO_FRAMES
    frame_00001.jpg frame_00002.jpg frame_00003.jpg frame_00004.jpg
    frame_00005.jpg frame_00006.jpg frame_00007.jpg frame_00008.jpg
    frame_00009.jpg frame_00010.jpg frame_00011.jpg frame_00012.jpg
    frame_00013.jpg frame_00014.jpg frame_00015.jpg frame_00016.jpg
)
foreach(frame IN LISTS COCO_VIDEO_DEMO_FRAMES)
    target_add_binary_data(
        ${COMPONENT_LIB}
        "${CMAKE_CURRENT_LIST_DIR}/../test_assets/coco_video_demo/frames/${frame}"
        BINARY
    )
endforeach()
