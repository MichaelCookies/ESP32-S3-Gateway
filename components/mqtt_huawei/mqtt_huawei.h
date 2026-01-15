#ifndef MQTT_HUAWEI_H
#define MQTT_HUAWEI_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_types.h"

//  初始化，传入系统总线句柄
void mqtt_huawei_init(QueueHandle_t bus_handle);

//  上报数据
void mqtt_huawei_publish_sensor(float temp, float humi);

//  生命周期控制
void mqtt_huawei_start(void);
void mqtt_huawei_stop(void);

//  上报系统状态，同步阈值和开关状态
void mqtt_huawei_report_state(float threshold, sys_mode_t mode, bool disp_on, bool sensor_on);
#endif // MQTT_HUAWEI_H