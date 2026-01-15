#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

//  系统运行模式
typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL_ON,
    MODE_MANUAL_OFF
} sys_mode_t;

//  消息主类型 (必须确保每个值唯一)
typedef enum {
    MSG_TYPE_SENSOR_DATA,   //  传感器数据
    MSG_TYPE_SENSOR_HB,     //  传感器心跳
    MSG_TYPE_CLOUD_CMD,     //  云端指令
    MSG_TYPE_LOCAL_CMD,     //  本地局域网指令 (必须单独占一个坑位)
    MSG_TYPE_MQTT_EVT,      //  MQTT 事件
    MSG_TYPE_WIFI_STATUS,   //  WiFi 状态
    MSG_TYPE_I2C_ERROR      //  硬件故障
} app_msg_type_t;

//  云端 - 本地指令子类型
typedef enum {
    CMD_ALARM_CTRL,
    CMD_SET_THRESHOLD,
    CMD_SENSOR_SWITCH,
    CMD_DISPLAY_SWITCH,
    CMD_NET_RESET,          //  网络复位
    CMD_CLOUD_SWITCH        //  云端开关
} cloud_cmd_type_t;

//  MQTT 事件子类型
typedef enum {
    MQTT_EVT_CONNECTED,
    MQTT_EVT_DISCONNECTED
} mqtt_evt_type_t;

//  统一消息载体
typedef struct {
    app_msg_type_t type;
    union {
        struct {
            float temp;
            float humi;
        } sensor;

        struct {
            cloud_cmd_type_t cmd;
            float val_float;
            bool val_bool;
        } cloud;

        mqtt_evt_type_t mqtt_evt;
        int error_code;
    } payload;
} app_msg_t;

#endif