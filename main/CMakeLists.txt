set(APP_SRCS
    "camera_web_main.c"
    "avi_mjpeg_writer.c"
    "json_writer.c"
    "json_validator.c"
    "recording_enrichment.c"
    "usb_msc_export.c"
    "tiny_cls_espdl_bridge.cpp"
    "fish31_espdl_bridge.cpp"
    "coco_espdl_bridge.cpp"
)

set(APP_PRIV_REQUIRES
    esp-dl
    esp_event
    esp_eth
    esp_http_server
    esp_netif
    esp_system
    esp_timer
    esp_wifi
    esp_video
    example_video_common
    fatfs
    mdns
    wear_levelling
    sdmmc
    esp_driver_sdmmc
    esp_driver_sdspi
    esp_driver_spi
    esp_driver_gpio
    vfs
    coco_detect
    nvs_flash
    esp_tinyusb
)

if(CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE)
    list(APPEND APP_SRCS
        "yolo26_espdl_bridge.cpp"
        "yolo11_espdl_bridge.cpp"
    )
    list(APPEND APP_PRIV_REQUIRES yolo26)
else()
    list(APPEND APP_SRCS "legacy_yolo_stubs.cpp")
endif()

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS "."
    PRIV_REQUIRES ${APP_PRIV_REQUIRES}
)

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
else()
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()

set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
include(${cmake_dir}/utilities.cmake)

if(CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE)
    # Legacy Coke/Sprite bridge source is retained for reproduction, but the
    # old model assets are not shipped in the v3.1.0 customer release.
    set(YOLO26_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo26_coke_sprite_raw_heads_416_allint8_p4.espdl")
    if(NOT EXISTS "${YOLO26_MODEL_FILE}")
        message(FATAL_ERROR "CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE is enabled, but ${YOLO26_MODEL_FILE} is not present. Restore the legacy asset or disable the legacy option.")
    endif()
    target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO26_MODEL_FILE}" BINARY)

    set(YOLO11_MODEL_FILE "${CMAKE_CURRENT_LIST_DIR}/../models/yolo11_coke_sprite_416_s8_p4.espdl")
    if(NOT EXISTS "${YOLO11_MODEL_FILE}")
        message(FATAL_ERROR "CONFIG_APP_ENABLE_LEGACY_COKE_SPRITE is enabled, but ${YOLO11_MODEL_FILE} is not present. Restore the legacy asset or disable the legacy option.")
    endif()
    target_add_aligned_binary_data(${COMPONENT_LIB} "${YOLO11_MODEL_FILE}" BINARY)
endif()

# TinyCNN Marine 192x192 6-class classifier.
set(TINY_CLS_MODEL_FILE "${CONFIG_APP_TINY_CLS_MODEL_FILE}")
if (NOT IS_ABSOLUTE "${TINY_CLS_MODEL_FILE}")
    get_filename_component(TINY_CLS_MODEL_FILE
        "${CMAKE_CURRENT_LIST_DIR}/${TINY_CLS_MODEL_FILE}" ABSOLUTE)
endif()
set(TINY_CLS_MODEL_EMBED_FILE "${CMAKE_CURRENT_BINARY_DIR}/tiny_cls_model.espdl")
configure_file("${TINY_CLS_MODEL_FILE}" "${TINY_CLS_MODEL_EMBED_FILE}" COPYONLY)
target_add_aligned_binary_data(${COMPONENT_LIB} "${TINY_CLS_MODEL_EMBED_FILE}" BINARY)

# Fish31 MobileNetV3-Small 0.75x 224x224 31-class classifier.
set(FISH31_MODEL_FILE "${CONFIG_APP_FISH31_MODEL_FILE}")
if (NOT IS_ABSOLUTE "${FISH31_MODEL_FILE}")
    get_filename_component(FISH31_MODEL_FILE
        "${CMAKE_CURRENT_LIST_DIR}/${FISH31_MODEL_FILE}" ABSOLUTE)
endif()
set(FISH31_MODEL_EMBED_FILE "${CMAKE_CURRENT_BINARY_DIR}/fish31_cls_model.espdl")
configure_file("${FISH31_MODEL_FILE}" "${FISH31_MODEL_EMBED_FILE}" COPYONLY)
target_add_aligned_binary_data(${COMPONENT_LIB} "${FISH31_MODEL_EMBED_FILE}" BINARY)

# Board validation images embedded directly into firmware.
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_02.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_03.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/video_frames_320/images/demo_04.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/tinycls_marine_demo/images/tiny_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/tinycls_marine_demo/images/tiny_02.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/tinycls_marine_demo/images/tiny_03.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/tinycls_marine_demo/images/tiny_04.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/fish31_validation/images/fish31_01.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/fish31_validation/images/fish31_02.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/fish31_validation/images/fish31_03.jpg" BINARY)
target_add_binary_data(${COMPONENT_LIB} "${CMAKE_CURRENT_LIST_DIR}/../test_assets/fish31_validation/images/fish31_04.jpg" BINARY)

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

set(TINYCLS_VIDEO_DEMO_FRAMES
    tiny_frame_00001.jpg tiny_frame_00002.jpg tiny_frame_00003.jpg tiny_frame_00004.jpg
    tiny_frame_00005.jpg tiny_frame_00006.jpg tiny_frame_00007.jpg tiny_frame_00008.jpg
    tiny_frame_00009.jpg tiny_frame_00010.jpg tiny_frame_00011.jpg tiny_frame_00012.jpg
    tiny_frame_00013.jpg tiny_frame_00014.jpg tiny_frame_00015.jpg tiny_frame_00016.jpg
)
foreach(frame IN LISTS TINYCLS_VIDEO_DEMO_FRAMES)
    target_add_binary_data(
        ${COMPONENT_LIB}
        "${CMAKE_CURRENT_LIST_DIR}/../test_assets/tinycls_marine_demo/frames/${frame}"
        BINARY
    )
endforeach()

set(FISH31_VIDEO_DEMO_FRAMES
    fish31_frame_00001.jpg fish31_frame_00002.jpg fish31_frame_00003.jpg fish31_frame_00004.jpg
    fish31_frame_00005.jpg fish31_frame_00006.jpg fish31_frame_00007.jpg fish31_frame_00008.jpg
    fish31_frame_00009.jpg fish31_frame_00010.jpg fish31_frame_00011.jpg fish31_frame_00012.jpg
    fish31_frame_00013.jpg fish31_frame_00014.jpg fish31_frame_00015.jpg fish31_frame_00016.jpg
)
foreach(frame IN LISTS FISH31_VIDEO_DEMO_FRAMES)
    target_add_binary_data(
        ${COMPONENT_LIB}
        "${CMAKE_CURRENT_LIST_DIR}/../test_assets/fish31_video_demo/frames/${frame}"
        BINARY
    )
endforeach()
