cmake_minimum_required(VERSION 3.22)

set(PROJECT_VER "3.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_buoy_vision_lab)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
execute_process(
    COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_LIST_DIR}/tools/apply_component_patches.py"
            "${CMAKE_CURRENT_LIST_DIR}"
    RESULT_VARIABLE component_patch_result
)
if(NOT component_patch_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply managed component patches")
endif()
