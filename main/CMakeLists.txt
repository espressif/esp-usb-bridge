idf_component_register(SRCS "main.c"
                            "jtag.c"
                            "serial.c"
                            "msc.c"
                            "util.c"
                       REQUIRES "tinyusb" "driver" "esp_timer"
                       LDFRAGMENTS "noflash.lf"
                       INCLUDE_DIRS "." "public_include")

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE public_include)
