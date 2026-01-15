idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash state_machine ssd1306 aht20 mqtt_huawei wifi logs
)