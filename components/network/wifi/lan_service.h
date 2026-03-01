#ifndef LAN_SERVICE_H
#define LAN_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LAN_SERVICE_PORT    3333    
#define LAN_MAGIC_WORD      0x55AA  
#define LAN_PROTOCOL_VER    0x01    

typedef enum {
    UDP_TYPE_DISCOVERY      = 0x01, 
    UDP_TYPE_CONTROL        = 0x02, 
    UDP_TYPE_REPORT         = 0x03, 
    UDP_TYPE_TIME_SYNC      = 0x04, 
    UDP_TYPE_SENSOR_BCAST   = 0x10, 
    UDP_TYPE_LOG            = 0x20  
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

esp_err_t lan_service_init(QueueHandle_t sys_bus_handle);
void lan_service_set_net_status(bool has_ip);
void lan_service_broadcast_data(void);

#ifdef __cplusplus
}
#endif

#endif // LAN_SERVICE_H