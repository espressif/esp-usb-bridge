set(sources "esp_io.c")

set(include_dirs ".")

set(dependencies "driver")

idf_component_register(
    SRCS
        ${sources}
    INCLUDE_DIRS
        ${include_dirs}
	REQUIRES
		${dependencies}
)
