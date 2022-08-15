idf_component_register(SRCS "main.c"
                            "jtag.c"
                            "serial.c"
                            "msc.c"
                            "util.c"
                       LDFRAGMENTS "noflash.lf"
                       INCLUDE_DIRS .)
