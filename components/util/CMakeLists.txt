set(dependencies freertos esp_system)

# Starting from esp-idf v5.3, the GPIO driver is moved to a separate component
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.3")
    list(APPEND dependencies "esp_driver_gpio")
else()
    list(APPEND dependencies "driver")
endif()

idf_component_register(SRCS "util.c"
                       INCLUDE_DIRS "include"
                       REQUIRES ${dependencies}
                      )
