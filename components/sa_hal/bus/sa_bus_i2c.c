#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "sa_bus_i2c.h"

#define MAX_CACHED_I2C_DEVICES 8
#define I2C_OP_TIMEOUT_MS     200

typedef struct {
    i2c_master_bus_handle_t idf_bus;
    struct {
        uint8_t addr;
        i2c_master_dev_handle_t dev_handle;
    } cached_devs[MAX_CACHED_I2C_DEVICES];
    uint8_t dev_count;
    SemaphoreHandle_t ctx_mutex;
} sa_i2c_ctx_t;

static i2c_master_dev_handle_t get_or_add_dev_handle(sa_i2c_ctx_t *ctx, uint8_t addr) {
    xSemaphoreTake(ctx->ctx_mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < ctx->dev_count; i++) {
        if (ctx->cached_devs[i].addr == addr) {
            xSemaphoreGive(ctx->ctx_mutex);
            return ctx->cached_devs[i].dev_handle;
        }
    }
    if (ctx->dev_count >= MAX_CACHED_I2C_DEVICES) {
        xSemaphoreGive(ctx->ctx_mutex);
        return NULL;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t new_handle;
    if (i2c_master_bus_add_device(ctx->idf_bus, &dev_cfg, &new_handle) == ESP_OK) {
        ctx->cached_devs[ctx->dev_count].addr = addr;
        ctx->cached_devs[ctx->dev_count].dev_handle = new_handle;
        ctx->dev_count++;
        xSemaphoreGive(ctx->ctx_mutex);
        return new_handle;
    }
    xSemaphoreGive(ctx->ctx_mutex);
    return NULL;
}

static esp_err_t i2c_bus_read(sa_bus_t *bus, uint8_t addr, uint8_t *data, size_t len) {
    if (!bus || !bus->user_ctx) return ESP_ERR_INVALID_ARG;
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    i2c_master_dev_handle_t dev = get_or_add_dev_handle(ctx, addr);
    if (!dev) return ESP_ERR_NOT_FOUND;
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_receive(dev, data, len, pdMS_TO_TICKS(I2C_OP_TIMEOUT_MS));
    xSemaphoreGive(bus->bus_mutex);
    return err;
}

static esp_err_t i2c_bus_write(sa_bus_t *bus, uint8_t addr, const uint8_t *data, size_t len) {
    if (!bus || !bus->user_ctx) return ESP_ERR_INVALID_ARG;
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    i2c_master_dev_handle_t dev = get_or_add_dev_handle(ctx, addr);
    if (!dev) return ESP_ERR_NOT_FOUND;
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    esp_err_t err = i2c_master_transmit(dev, data, len, pdMS_TO_TICKS(I2C_OP_TIMEOUT_MS));
    xSemaphoreGive(bus->bus_mutex);
    return err;
}

static esp_err_t i2c_bus_deinit(sa_bus_t *bus) {
    if (!bus) return ESP_OK;
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    if (ctx) {
        vSemaphoreDelete(ctx->ctx_mutex);
        free(ctx);
    }
    if (bus->bus_mutex) vSemaphoreDelete(bus->bus_mutex);
    free(bus);
    return ESP_OK;
}

sa_bus_t* sa_bus_i2c_create(i2c_master_bus_handle_t idf_bus_handle, const char *bus_name) {
    if (!idf_bus_handle) return NULL;
    sa_bus_t *bus = calloc(1, sizeof(sa_bus_t));
    sa_i2c_ctx_t *ctx = calloc(1, sizeof(sa_i2c_ctx_t));
    if (!bus || !ctx) { free(bus); free(ctx); return NULL; }
    ctx->idf_bus = idf_bus_handle;
    ctx->ctx_mutex = xSemaphoreCreateMutex();
    bus->bus_name = bus_name ? bus_name : "i2c_main";
    bus->bus_mutex = xSemaphoreCreateMutex();
    bus->user_ctx = ctx;
    bus->read_bytes = i2c_bus_read;
    bus->write_bytes = i2c_bus_write;
    bus->deinit = i2c_bus_deinit;
    return bus;
}