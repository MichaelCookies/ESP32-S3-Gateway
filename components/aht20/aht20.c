#include "aht20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "AHT20";

// 定义超时时间：100毫秒
// 如果 I2C 被 WiFi 中断打断了，等 100ms 就放弃，不要死等！
#define I2C_TIMEOUT_MS  100 

esp_err_t aht20_init(i2c_master_dev_handle_t dev_handle) {
    //  防止句柄无效导致 Crash
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle (NULL) during init");
        return ESP_ERR_INVALID_ARG;
    }

    vTaskDelay(pdMS_TO_TICKS(40));
    uint8_t status;
    uint8_t cmd_status = 0x71;
    
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &cmd_status, 1, &status, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err; 
    
    // 检查状态字 bit3，如果未置位则发送初始化
    if (!(status & 0x08)) {
        uint8_t init_cmd[] = {AHT20_CMD_INIT, 0x08, 0x00};
        return i2c_master_transmit(dev_handle, init_cmd, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }
    return ESP_OK;
}

esp_err_t aht20_read_data(i2c_master_dev_handle_t dev_handle, float *temp, float *humi) {
    //  防止句柄无效导致 Crash
    if (dev_handle == NULL) {
        //  静默失败或打印 Debug，防止刷屏，返回 Timeout 或 Invalid State 让上层处理
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t trigger_cmd[] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    uint8_t data[6];

    // 发送测量触发命令
    esp_err_t err = i2c_master_transmit(dev_handle, trigger_cmd, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C TX Failed (Wi-Fi interference?)");
        return err;
    }
    
    // 等待测量完成，datasheet要求 >75ms
    vTaskDelay(pdMS_TO_TICKS(80));

    // 读取数据
    err = i2c_master_receive(dev_handle, data, 6, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C RX Failed");
        return err;
    }

    // 解析数据
    uint32_t humi_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    *humi = (float)humi_raw * 100.0 / 1048576.0;
    *temp = (float)temp_raw * 200.0 / 1048576.0 - 50.0;

    return ESP_OK;
}