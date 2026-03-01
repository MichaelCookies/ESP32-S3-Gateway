#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL_ON,
    MODE_MANUAL_OFF,
    MODE_SILENT
} sys_mode_t;

typedef enum {
    CMD_ALARM_CTRL = 0,
    CMD_SET_THRESHOLD,
    CMD_SENSOR_SWITCH,
    CMD_DISPLAY_SWITCH,
    CMD_NET_RESET,
    CMD_CLOUD_SWITCH,
    CMD_ACTUATOR_CTRL  
} sys_cmd_t;

typedef enum {
    MSG_TYPE_WIFI_STATUS = 0,
    MSG_TYPE_MQTT_EVT,
    MSG_TYPE_LOCAL_CMD,
    MSG_TYPE_CLOUD_CMD,
    MSG_TYPE_SYS_TICK  
} msg_type_t;

typedef enum {
    MQTT_EVT_CONNECTED = 0,
    MQTT_EVT_DISCONNECTED
} mqtt_evt_type_t;

typedef struct {
    msg_type_t type;
    union {
        int error_code; 
        
        mqtt_evt_type_t mqtt_evt;
        
        struct {
            sys_cmd_t cmd;
            union {
                bool val_bool;
                float val_float;
            };
            char target_id[32]; 
        } cmd_data;
        
    } payload;
} app_msg_t;

#ifdef __cplusplus
}
#endif

#endif // APP_TYPES_H