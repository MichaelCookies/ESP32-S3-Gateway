set(component_srcs "ssd1306.c" "ssd1306_spi.c")

# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

if(idf_version VERSION_GREATER_EQUAL "5.2")
	if(CONFIG_LEGACY_DRIVER)
		list(APPEND component_srcs "ssd1306_i2c_legacy.c")
	else()
		list(APPEND component_srcs "ssd1306_i2c_new.c")
	endif()
else()
	list(APPEND component_srcs "ssd1306_i2c_legacy.c")
endif()

# idf_component_register(SRCS "${component_srcs}" PRIV_REQUIRES driver INCLUDE_DIRS ".")
idf_component_register(
    SRCS 
        "ssd1306.c" 
        "ssd1306_i2c_new.c" 
        "ssd1306_i2c_legacy.c" 
        "ssd1306_spi.c"
    INCLUDE_DIRS "."
    # v5.x 推荐指定具体的子驱动组件
    REQUIRES driver esp_driver_i2c esp_driver_spi
)