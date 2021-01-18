# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(${IDF_VERSION_MAJOR} LESS 4 OR ${IDF_VERSION_MINOR} LESS 3)
    message(FATAL_ERROR "ESP-IDF v${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR} is not supported! "
        "Please use v4.3 or newer.")
endif()

project(bridge)

# Workaround for Vendor class. ESP-IDF v4.3 doesn't support it directly through its tinyusb "additions" layer.
target_compile_definitions(tinyusb PUBLIC CFG_TUD_VENDOR=1)
target_compile_definitions(tinyusb PUBLIC CFG_TUD_VENDOR_RX_BUFSIZE=64)
target_compile_definitions(tinyusb PUBLIC CFG_TUD_VENDOR_TX_BUFSIZE=64)
