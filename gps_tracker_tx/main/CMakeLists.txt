idf_component_register(
    SRCS "main.c" "gps.c" "e220.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_common log
)