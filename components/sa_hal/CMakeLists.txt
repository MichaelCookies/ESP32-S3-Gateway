# 定义源文件列表
# 路径是相对于 CMakeLists.txt 所在的 sa_hal 根目录的
set(srcs
    "bus/sa_device_manager.c"
    "bus/sa_bus_i2c.c"
    "node/aht20/sa_node_aht20.c"
    "node/beep/sa_node_beep.c"
    "node/ssd1306/sa_node_oled.c"
    "mock/sa_mock_device.c"
)

# INCLUDE_DIRS
# 把所有包含 .h 的子目录都列在这里
set(include_dirs
    "bus"           # sa_hal.h, sa_device_manager.h
    "node/aht20"    # sa_node_aht20.h
    "node/beep"     # sa_node_beep.h
    "node/ssd1306"  # sa_node_oled.h, font8x8_basic.h
    "mock"          # sa_mock_device.h
)

# 注册组件
idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "${include_dirs}"
    REQUIRES 
        freertos     #Semaphores, Tasks
        driver       #GPIO, I2C, LEDC
        log          #ESP_LOGx
        esp_timer    #系统时间戳
        # esp_hw_support # 可选：如果有些底层硬件操作报错，可以打开这个
)