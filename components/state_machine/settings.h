/*
 * settings.h
 * V4.2
 * 职责：配置管理 + 运行时硬件状态监控
 * 新增：硬件在线状态管理接口
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include "app_types.h" 

//  硬件连接状态
typedef enum {
    HW_STATUS_UNKNOWN = 0,
    HW_STATUS_CONNECTED,    // 正常在线
    HW_STATUS_DISCONNECTED, // 离线/拔出
    HW_STATUS_ERROR         // 通信故障
} hw_status_t;

//  初始化
void settings_init(void);

//  单元测试
bool settings_run_self_test(void);

//  持久化配置
float settings_get_threshold(void);
void settings_set_threshold(float new_val);

bool settings_get_sensor_enabled(void);
void settings_set_sensor_enabled(bool enabled);

bool settings_get_display_enabled(void);
void settings_set_display_enabled(bool enabled);

bool settings_get_cloud_enabled(void);
void settings_set_cloud_enabled(bool enabled);

//  运行时状态
sys_mode_t settings_get_mode(void);
void settings_set_mode(sys_mode_t new_mode);

bool settings_get_alarm_state(void);
void settings_set_alarm_state(bool on);

bool settings_get_cloud_lock(void);
void settings_set_cloud_lock(bool locked);

//  硬件实时状态
hw_status_t settings_get_oled_hw_status(void);
void settings_set_oled_hw_status(hw_status_t status);

hw_status_t settings_get_sensor_hw_status(void);
void settings_set_sensor_hw_status(hw_status_t status);

#endif // SETTINGS_H