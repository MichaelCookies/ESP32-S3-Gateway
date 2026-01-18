idf_component_register(
    SRCS "data_logger.c"
    INCLUDE_DIRS "."
    REQUIRES spiffs logs wifi
)