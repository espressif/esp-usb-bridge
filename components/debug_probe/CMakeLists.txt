set(DAP_sources
    "DAP/Source/DAP_vendor.c"
    "DAP/Source/DAP.c"
    "DAP/Source/JTAG_DP.c"
    "DAP/Source/SW_DP.c"
)

set(debug_probe_sources
    "swd.c"
    "esp_usb_jtag.c"
    "debug_gpio.c"
)

set(include_dirs
    "include"
    "DAP/Include"
    "DAP/Config"
)

set(priv_include_dirs
    "."
)

set(dependencies
    "esp_timer"
    "hal"
    "util"
    "esp_ringbuf"
)

# Starting from esp-idf v5.3, the GPIO driver is moved to a separate component
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND dependencies "esp_driver_gpio")
else()
    list(APPEND dependencies "driver")
endif()

idf_component_register(
    SRCS
        ${DAP_sources}
        ${debug_probe_sources}
    INCLUDE_DIRS
        ${include_dirs}
    PRIV_INCLUDE_DIRS
        ${priv_include_dirs}
    REQUIRES
        ${dependencies}
    LDFRAGMENTS
        "noflash.lf"
)
