#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_idf_version.h"
#include "sa_bus_i2c.h"

#define TAG "SA_I2C_BUS"
#define MAX_CACHED_I2C_DEVICES 8
#define I2C_OP_TIMEOUT_MS     200

typedef struct {
    i2c_master_bus_handle_t idf_bus;
    struct {
        uint8_t addr;
        i2c_master_dev_handle_t dev_handle;
        bool in_use;           // 标记是否正在使用（用于 LRU 策略）
    } cached_devs[MAX_CACHED_I2C_DEVICES];
    uint8_t dev_count;
} sa_i2c_ctx_t;

// 内部辅助：清理所有缓存的设备句柄
static void clear_all_cached_devices(sa_i2c_ctx_t *ctx) {
    for (int i = 0; i < ctx->dev_count; i++) {
        if (ctx->cached_devs[i].dev_handle != NULL) {
            i2c_master_bus_rm_device(ctx->cached_devs[i].dev_handle);
            ctx->cached_devs[i].dev_handle = NULL;
            ctx->cached_devs[i].in_use = false;
        }
    }
    ctx->dev_count = 0;
    ESP_LOGI(TAG, "All cached I2C devices cleared");
}

// 获取或创建设备句柄
static i2c_master_dev_handle_t get_or_create_dev_handle(sa_i2c_ctx_t *ctx, uint8_t addr) {
    // 查找现有句柄
    for (uint8_t i = 0; i < ctx->dev_count; i++) {
        if (ctx->cached_devs[i].addr == addr) {
            ctx->cached_devs[i].in_use = true;
            return ctx->cached_devs[i].dev_handle;
        }
    }
    
    // 缓存未满，创建新句柄
    if (ctx->dev_count < MAX_CACHED_I2C_DEVICES) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 400000,  // 标准模式 400kHz
        };
        
        i2c_master_dev_handle_t new_handle;
        esp_err_t err = i2c_master_bus_add_device(ctx->idf_bus, &dev_cfg, &new_handle);
        
        if (err == ESP_OK) {
            ctx->cached_devs[ctx->dev_count].addr = addr;
            ctx->cached_devs[ctx->dev_count].dev_handle = new_handle;
            ctx->cached_devs[ctx->dev_count].in_use = true;
            ctx->dev_count++;
            ESP_LOGD(TAG, "Created new device handle for 0x%02X", addr);
            return new_handle;
        } else {
            ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", addr, esp_err_to_name(err));
            return NULL;
        }
    }
    
    // 缓存已满：清理所有句柄并重建（简单粗暴但有效）
    ESP_LOGW(TAG, "Device cache full, resetting cache...");
    clear_all_cached_devices(ctx);
    
    // 递归调用，现在缓存是空的
    return get_or_create_dev_handle(ctx, addr);
}

// 读操作（线程安全）
static esp_err_t i2c_bus_read(sa_bus_t *bus, uint8_t addr, uint8_t *data, size_t len) {
    if (!bus || !bus->user_ctx || !data || len == 0) return ESP_ERR_INVALID_ARG;
    
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    
    i2c_master_dev_handle_t dev = get_or_create_dev_handle(ctx, addr);
    if (!dev) {
        xSemaphoreGive(bus->bus_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t err = i2c_master_receive(dev, data, len, pdMS_TO_TICKS(I2C_OP_TIMEOUT_MS));
    
    xSemaphoreGive(bus->bus_mutex);
    
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C read from 0x%02X failed: %s", addr, esp_err_to_name(err));
    }
    return err;
}

// 写操作
static esp_err_t i2c_bus_write(sa_bus_t *bus, uint8_t addr, const uint8_t *data, size_t len) {
    if (!bus || !bus->user_ctx || !data || len == 0) return ESP_ERR_INVALID_ARG;
    
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    
    i2c_master_dev_handle_t dev = get_or_create_dev_handle(ctx, addr);
    if (!dev) {
        xSemaphoreGive(bus->bus_mutex);
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t err = i2c_master_transmit(dev, data, len, pdMS_TO_TICKS(I2C_OP_TIMEOUT_MS));
    
    xSemaphoreGive(bus->bus_mutex);
    
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "I2C write to 0x%02X failed: %s", addr, esp_err_to_name(err));
    }
    return err;
}

// 探测操作，避免直接调用 i2c_master_probe
static esp_err_t i2c_bus_probe(sa_bus_t *bus, uint8_t addr, uint32_t timeout_ms) {
    if (!bus || !bus->user_ctx) return ESP_ERR_INVALID_ARG;
    
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    
    // 使用 IDF 原生 probe，但在 mutex 保护下
    esp_err_t err = i2c_master_probe(ctx->idf_bus, addr, pdMS_TO_TICKS(timeout_ms));
    
    xSemaphoreGive(bus->bus_mutex);
    
    return err;
}

// 总线复位，用于从死锁中恢复
static esp_err_t i2c_bus_reset(sa_bus_t *bus) {
    if (!bus || !bus->user_ctx) return ESP_ERR_INVALID_ARG;
    
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    
    xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
    
    ESP_LOGW(TAG, "Performing I2C bus reset...");
    
    // 先清理所有设备句柄，避免 IDF 驱动内部状态混乱
    clear_all_cached_devices(ctx);
    
    // 执行总线复位
    esp_err_t err = ESP_OK;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    err = i2c_master_bus_reset(ctx->idf_bus);
#else
    // 对于旧版本，通过重新创建总线实现复位效果，但这里不做太多考虑
    ESP_LOGW(TAG, "IDF version < 5.2, bus reset not fully supported");
#endif

    xSemaphoreGive(bus->bus_mutex);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "I2C bus reset completed");
    } else {
        ESP_LOGE(TAG, "I2C bus reset failed: %s", esp_err_to_name(err));
    }
    return err;
}

// 销毁总线
static esp_err_t i2c_bus_deinit(sa_bus_t *bus) {
    if (!bus) return ESP_OK;
    
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)bus->user_ctx;
    if (ctx) {
        // 确保持有 mutex 再清理设备
        if (bus->bus_mutex) {
            xSemaphoreTake(bus->bus_mutex, portMAX_DELAY);
            clear_all_cached_devices(ctx);
            xSemaphoreGive(bus->bus_mutex);
            vSemaphoreDelete(bus->bus_mutex);
        }
        free(ctx);
    }
    free(bus);
    return ESP_OK;
}

// 创建 I2C 总线对象（对外接口）
sa_bus_t* sa_bus_i2c_create(i2c_master_bus_handle_t idf_bus_handle, const char *bus_name) {
    if (!idf_bus_handle) {
        ESP_LOGE(TAG, "Invalid IDF bus handle");
        return NULL;
    }
    
    sa_bus_t *bus = (sa_bus_t *)calloc(1, sizeof(sa_bus_t));
    sa_i2c_ctx_t *ctx = (sa_i2c_ctx_t *)calloc(1, sizeof(sa_i2c_ctx_t));
    
    if (!bus || !ctx) {
        ESP_LOGE(TAG, "Memory allocation failed");
        free(bus);
        free(ctx);
        return NULL;
    }
    
    ctx->idf_bus = idf_bus_handle;
    ctx->dev_count = 0;
    // in_use 已在 calloc 中初始化为 false
    
    bus->bus_name = bus_name ? bus_name : "i2c_main";
    bus->bus_mutex = xSemaphoreCreateMutex();
    if (!bus->bus_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(bus);
        free(ctx);
        return NULL;
    }
    
    bus->user_ctx = ctx;
    bus->read_bytes = i2c_bus_read;
    bus->write_bytes = i2c_bus_write;
    bus->probe = i2c_bus_probe;        // 注册探测接口
    bus->reset = i2c_bus_reset;        // 注册复位接口
    bus->deinit = i2c_bus_deinit;
    
    ESP_LOGI(TAG, "I2C bus '%s' created with HAL protection", bus->bus_name);
    return bus;
}