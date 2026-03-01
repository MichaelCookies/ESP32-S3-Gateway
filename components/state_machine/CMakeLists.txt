idf_component_register(
    SRCS "settings.c" "hw_recovery.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash sa_hal common
)