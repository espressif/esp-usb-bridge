set(dependencies "tinyusb" "esp_timer" "usb" "util" "serial_handler" "debug_probe" "esp_ringbuf")

# Starting from esp-idf v5.3, the GPIO driver is moved to a separate component
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND dependencies "esp_driver_gpio")
else()
    list(APPEND dependencies "driver")
endif()

idf_component_register(SRCS "main.c"
                            "msc.c"
                            "eub_vendord.c"
                            "serial_bridge.c"
                       REQUIRES ${dependencies}
                       LDFRAGMENTS "noflash.lf"
                       INCLUDE_DIRS "." "public_include")

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE public_include)

idf_build_get_property(project_ver PROJECT_VER)
if(project_ver)
    message(STATUS "Project version: '${project_ver}'")
    # Once a git tag is created, PROJECT_VER will look like v1.2-4-g567890a, assuming
    # that we use tags in `vX.Y` format. Extract X and Y from the tag:
    string(REGEX MATCH "^v([0-9]+)\\.([0-9]+)[^.]*$" match ${project_ver})
    if(match)
        set(project_ver_major ${CMAKE_MATCH_1})
        set(project_ver_minor ${CMAKE_MATCH_2})
        # extend project_ver_minor to two BCD digits
        string(LENGTH ${project_ver_minor} len)
        if(len EQUAL 1)
            set(project_ver_minor "0${project_ver_minor}")
        endif()
        set(bcd_device "0x${project_ver_major}${project_ver_minor}")
    else()
        message(STATUS "Failed to determine major and minor version from '${project_ver}'")
    endif()
endif()

# Otherwise, set the version to 0.01
if(NOT bcd_device)
    set(bcd_device "0x0001")
endif()
message(STATUS "bcdDevice value: ${bcd_device}")

target_compile_definitions(${COMPONENT_LIB} PRIVATE BCDDEVICE=${bcd_device})
