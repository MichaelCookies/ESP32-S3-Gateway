// sa_mock_device.c
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_random.h"
#include "sa_mock_device.h"

static const char *TAG = "SA_MOCK";

static esp_err_t mock_temp_read(sa_node_t *node, sa_value_t *out_val) {
    uint32_t random_val = esp_random();
    float noise = ((float)(random_val % 100) / 100.0f) * 2.0f - 1.0f; 
    out_val->f_val = 25.0f + noise; 
    return ESP_OK;
}

static esp_err_t mock_relay_write(sa_node_t *node, sa_value_t in_val) {
    bool state = in_val.b_val;
    ESP_LOGW(TAG, "MOCK RELAY [%s] is now %s", node->id, state ? "ON" : "OFF");
    return ESP_OK;
}

sa_node_t* sa_mock_temp_create(void) {
    sa_node_t *node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    if (!node) return NULL;
    
    node->id = "mock_temperature";
    node->type = SA_NODE_SENSOR;
    node->val_type = SA_VAL_TYPE_FLOAT;
    node->bus = NULL; 
    node->poll_interval_ms = 1000;
    node->ops.read = mock_temp_read;
    
    return node;
}

sa_node_t* sa_mock_relay_create(void) {
    sa_node_t *node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    if (!node) return NULL;
    
    node->id = "mock_main_relay";
    node->type = SA_NODE_ACTUATOR;
    node->val_type = SA_VAL_TYPE_BOOL;
    node->bus = NULL;
    node->ops.write = mock_relay_write;
    
    return node;
}

// Mock 光照传感器，模拟BH1750

static esp_err_t mock_lux_read(sa_node_t *node, sa_value_t *out_val) {
    // 产生 30.0 ~ 80.0 之间的随机数，故意跨越 50.0 的阈值来测试联动
    uint32_t random_val = esp_random();
    out_val->f_val = 30.0f + (float)(random_val % 500) / 10.0f;
    return ESP_OK;
}

sa_node_t* sa_mock_lux_create(void) {
    sa_node_t *node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    if (!node) return NULL;
    
    node->id = "light_intensity";          // 和Scene Engine里规则用到的ID一致，方便测试
    node->type = SA_NODE_SENSOR;
    node->val_type = SA_VAL_TYPE_FLOAT;
    node->bus = NULL;                      // Mock 设备不需要真总线
    node->poll_interval_ms = 1000;         // 1秒读一次
    node->ops.read = mock_lux_read;
    
    return node;
}


// Mock 气压传感器，模拟BMP280

static esp_err_t mock_pressure_read(sa_node_t *node, sa_value_t *out_val) {
    // 产生 1010.0 ~ 1015.0 hPa 的标准大气压波动
    uint32_t random_val = esp_random();
    out_val->f_val = 1010.0f + (float)(random_val % 50) / 10.0f;
    return ESP_OK;
}

sa_node_t* sa_mock_pressure_create(void) {
    sa_node_t *node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    if (!node) return NULL;
    
    node->id = "air_pressure";
    node->type = SA_NODE_SENSOR;
    node->val_type = SA_VAL_TYPE_FLOAT;
    node->bus = NULL; 
    node->poll_interval_ms = 2000;         // 2秒读一次
    node->ops.read = mock_pressure_read;
    
    return node;
}