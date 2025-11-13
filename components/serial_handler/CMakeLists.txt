set(priv_requires esp-serial-flasher freertos esp_timer)

# Starting from esp-idf v5.3, the GPIO and UART drivers are moved to separate components
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND priv_requires "esp_driver_gpio" "esp_driver_uart")
else()
    list(APPEND priv_requires "driver")
endif()

idf_component_register(SRCS "serial_handler.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
