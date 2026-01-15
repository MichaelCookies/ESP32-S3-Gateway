idf_component_register(
    SRCS "data_logger.c"
    INCLUDE_DIRS "."
    # 关键点！必须添加 esp_spiffs 依赖，否则找不到头文件
    REQUIRES spiffs logs
)