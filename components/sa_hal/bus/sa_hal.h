#ifndef SA_HAL_H
#define SA_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SA_VAL_TYPE_FLOAT = 0,
    SA_VAL_TYPE_BOOL,
    SA_VAL_TYPE_INT
} sa_val_type_t;

typedef enum {
    SA_NODE_SENSOR = 0,
    SA_NODE_ACTUATOR
} sa_node_type_t;

typedef union {
    float f_val;
    bool  b_val;
    int32_t i_val;
} sa_value_t;

// 前向声明
struct _sa_bus_t;
struct _sa_node_t;

// 完整定义 sa_bus_t，包含 probe 和 reset
typedef struct _sa_bus_t {
    const char *bus_name;
    SemaphoreHandle_t bus_mutex;
    
    esp_err_t (*init)(struct _sa_bus_t *self);
    esp_err_t (*read_bytes)(struct _sa_bus_t *self, uint8_t addr, uint8_t *data, size_t len);
    esp_err_t (*write_bytes)(struct _sa_bus_t *self, uint8_t addr, const uint8_t *data, size_t len);
    
    // 总线级探测与复位，用于硬件恢复模块
    esp_err_t (*probe)(struct _sa_bus_t *self, uint8_t addr, uint32_t timeout_ms);
    esp_err_t (*reset)(struct _sa_bus_t *self);
    
    esp_err_t (*deinit)(struct _sa_bus_t *self);
    
    void *user_ctx;
} sa_bus_t;

typedef struct {
    esp_err_t (*init)(struct _sa_node_t *node);
    esp_err_t (*read)(struct _sa_node_t *node, sa_value_t *out_val);
    esp_err_t (*write)(struct _sa_node_t *node, sa_value_t in_val);
    esp_err_t (*ioctl)(struct _sa_node_t *node, int cmd, void *args);
    esp_err_t (*deinit)(struct _sa_node_t *node);
} sa_node_ops_t;

typedef struct _sa_node_t {
    const char *id;
    sa_node_type_t type;
    sa_val_type_t val_type;
    
    sa_bus_t *bus;              // 关联的总线,可为 NULL，如 GPIO 设备
    sa_node_ops_t ops;
    void *user_ctx;
    
    uint32_t poll_interval_ms;
    TickType_t last_poll_tick;
    
    sa_value_t cached_val;
    bool is_online;
    uint32_t consecutive_errors; // 连续错误计数，用于故障判定
    
    SemaphoreHandle_t cache_mutex;
    
    struct _sa_node_t *next;
} sa_node_t;

#ifdef __cplusplus
}
#endif

#endif // SA_HAL_H