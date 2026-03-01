#ifndef MQTT_HUAWEI_H
#define MQTT_HUAWEI_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void mqtt_huawei_init(QueueHandle_t bus_handle);

void mqtt_huawei_publish_sensors(void);

void mqtt_huawei_start(void);
void mqtt_huawei_stop(void);

void mqtt_huawei_report_state(float threshold, sys_mode_t mode, bool disp_on, bool sensor_on);

#ifdef __cplusplus
}
#endif

#endif // MQTT_HUAWEI_H