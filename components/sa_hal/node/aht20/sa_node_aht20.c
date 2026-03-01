// sa_node_aht20.c
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "sa_node_aht20.h"

#define AHT20_ADDR 0x38

typedef struct {
    sa_bus_t *bus;
    float cached_temp;
    float cached_humi;
    TickType_t last_hw_read_tick;
    SemaphoreHandle_t hw_mutex;
} aht20_shared_ctx_t;

static esp_err_t aht20_hw_init(aht20_shared_ctx_t *ctx) {
    uint8_t status = 0;
    uint8_t cmd_status = 0x71;
    
    ctx->bus->write_bytes(ctx->bus, AHT20_ADDR, &cmd_status, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    ctx->bus->read_bytes(ctx->bus, AHT20_ADDR, &status, 1);
    
    if (!(status & 0x08)) {
        uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
        return ctx->bus->write_bytes(ctx->bus, AHT20_ADDR, init_cmd, 3);
    }
    return ESP_OK;
}

static esp_err_t aht20_sync_read(aht20_shared_ctx_t *ctx) {
    xSemaphoreTake(ctx->hw_mutex, portMAX_DELAY);
    
    TickType_t now = xTaskGetTickCount();
    if (now - ctx->last_hw_read_tick < pdMS_TO_TICKS(1500)) {
        xSemaphoreGive(ctx->hw_mutex);
        return ESP_OK;
    }

    uint8_t trigger_cmd[] = {0xAC, 0x33, 0x00};
    esp_err_t err = ctx->bus->write_bytes(ctx->bus, AHT20_ADDR, trigger_cmd, 3);
    if (err != ESP_OK) goto unlock;

    vTaskDelay(pdMS_TO_TICKS(80));

    uint8_t data[6];
    err = ctx->bus->read_bytes(ctx->bus, AHT20_ADDR, data, 6);
    if (err != ESP_OK) goto unlock;

    uint32_t humi_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    ctx->cached_humi = (float)humi_raw * 100.0f / 1048576.0f;
    ctx->cached_temp = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;
    ctx->last_hw_read_tick = now;

unlock:
    xSemaphoreGive(ctx->hw_mutex);
    return err;
}

static esp_err_t aht20_temp_read(sa_node_t *node, sa_value_t *out_val) {
    aht20_shared_ctx_t *ctx = (aht20_shared_ctx_t *)node->user_ctx;
    esp_err_t err = aht20_sync_read(ctx);
    if (err == ESP_OK) {
        out_val->f_val = ctx->cached_temp;
    }
    return err;
}

static esp_err_t aht20_humi_read(sa_node_t *node, sa_value_t *out_val) {
    aht20_shared_ctx_t *ctx = (aht20_shared_ctx_t *)node->user_ctx;
    esp_err_t err = aht20_sync_read(ctx);
    if (err == ESP_OK) {
        out_val->f_val = ctx->cached_humi;
    }
    return err;
}

esp_err_t sa_aht20_create_nodes(sa_bus_t *bus, sa_node_t **out_temp_node, sa_node_t **out_humi_node) {
    if (!bus || !out_temp_node || !out_humi_node) return ESP_ERR_INVALID_ARG;

    aht20_shared_ctx_t *ctx = (aht20_shared_ctx_t *)calloc(1, sizeof(aht20_shared_ctx_t));
    if (!ctx) return ESP_ERR_NO_MEM;
    
    ctx->bus = bus;
    ctx->hw_mutex = xSemaphoreCreateMutex();
    ctx->last_hw_read_tick = 0;

    aht20_hw_init(ctx);

    sa_node_t *temp_node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    temp_node->id = "room_temperature";
    temp_node->type = SA_NODE_SENSOR;
    temp_node->val_type = SA_VAL_TYPE_FLOAT;
    temp_node->bus = bus;
    temp_node->user_ctx = ctx;
    temp_node->poll_interval_ms = 2000;
    temp_node->ops.read = aht20_temp_read;

    sa_node_t *humi_node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    humi_node->id = "room_humidity";
    humi_node->type = SA_NODE_SENSOR;
    humi_node->val_type = SA_VAL_TYPE_FLOAT;
    humi_node->bus = bus;
    humi_node->user_ctx = ctx;
    humi_node->poll_interval_ms = 2000;
    humi_node->ops.read = aht20_humi_read;

    *out_temp_node = temp_node;
    *out_humi_node = humi_node;

    return ESP_OK;
}