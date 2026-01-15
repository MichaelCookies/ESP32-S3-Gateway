/*
 * lan_service.h
 * V4.2 - 增加远程日志重定向功能
 */

#ifndef LAN_SERVICE_H
#define LAN_SERVICE_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

//  协议定义

#define LAN_SERVICE_PORT    3333    
#define LAN_MAGIC_WORD      0x55AA  
#define LAN_PROTOCOL_VER    0x01    

typedef enum {
    UDP_TYPE_DISCOVERY      = 0x01, // 上位机搜索设备
    UDP_TYPE_CONTROL        = 0x02, // 上位机控制指令
    UDP_TYPE_REPORT         = 0x03, // 设备状态上报/ACK
    UDP_TYPE_TIME_SYNC      = 0x04, // 时间同步
    UDP_TYPE_SENSOR_BCAST   = 0x10, // 传感器数据广播
    UDP_TYPE_LOG            = 0x20  // 远程系统日志
} udp_msg_type_t;

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;      
    uint8_t  version;    
    uint8_t  type;       
    uint16_t sequence;   
    uint16_t data_len;   
} lan_packet_head_t;
#pragma pack(pop)

//  对外接口

esp_err_t lan_service_init(QueueHandle_t sys_bus_handle);
void lan_service_set_net_status(bool has_ip);
void lan_service_broadcast_sensor_data(float temp, float humi);

#endif // LAN_SERVICE_H