#ifndef HW_RECOVERY_H
#define HW_RECOVERY_H

#include "ssd1306.h"
#include "driver/i2c_master.h"

// 初始化句柄
void hw_recovery_init(SSD1306_t *oled, i2c_master_bus_handle_t *aht_bus, i2c_master_dev_handle_t *aht_dev);

// 启动低优先级的自愈守护任务
void hw_recovery_start_daemon(void);

// 手动触发传感器复位
void hw_recovery_trigger_sensor_reset(void);

#endif