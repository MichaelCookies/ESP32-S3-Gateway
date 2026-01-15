/*
 * hw_recovery.h
 * V4.2 - 修复参数类型冲突，支持句柄指针传递
 */

#ifndef HW_RECOVERY_H
#define HW_RECOVERY_H

#include <stdbool.h>
#include "driver/i2c_master.h"
#include "ssd1306.h"

/**
 * @brief 初始化自愈模块
 * @param oled_ptr 指向 OLED 设备的指针
 * @param aht_bus_ptr 指向 AHT20 总线句柄的指针（用于重连后更新句柄）
 * @param aht_dev_ptr 指向 AHT20 设备句柄的指针
 */
void hw_recovery_init(SSD1306_t *oled_ptr, 
                      i2c_master_bus_handle_t *aht_bus_ptr, 
                      i2c_master_dev_handle_t *aht_dev_ptr);

/**
 * @brief 硬件状态轮询自愈函数
 */
void hw_recovery_tick(void);

/**
 * @brief 外部手动触发一次硬复位
 */
void hw_recovery_trigger_sensor_reset(void);
void hw_recovery_trigger_oled_reset(void);

#endif // HW_RECOVERY_H