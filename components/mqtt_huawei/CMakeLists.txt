idf_component_register(
    SRCS "mqtt_huawei.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt json state_machine common
)