idf_component_register(
    SRCS "settings.c" "hw_recovery.c" "smart_scene_engine.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash log sa_hal common
)