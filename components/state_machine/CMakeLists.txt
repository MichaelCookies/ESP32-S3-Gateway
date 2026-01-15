idf_component_register(
    SRCS "settings.c" "hw_recovery.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash aht20 ssd1306 common
)