set(sources
    "Source/DAP_vendor.c"
    "Source/DAP.c"
    "Source/JTAG_DP.c"
    "Source/SW_DP.c"
)

set(include_dirs
    "Include"
    "Config"
)

set(dependencies
    "esp_timer"
    "platform"
)

idf_component_register(
    SRCS
        ${sources}
    INCLUDE_DIRS
        ${include_dirs}
    REQUIRES
        ${dependencies}
)
