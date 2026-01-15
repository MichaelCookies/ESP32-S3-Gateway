#include "aht20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "AHT20";

//  定义超时时间：100毫秒
//  如果 I2C 被 WiFi 中断打断了，等 100ms 就放弃，不要死等！
#define I2C_TIMEOUT_MS  100 

esp_err_t aht20_init(i2c_master_dev_handle_t dev_handle) {
    vTaskDelay(pdMS_TO_TICKS(40));
    uint8_t status;
    uint8_t cmd_status = 0x71;
    
    //  把 -1 改为超时时间
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &cmd_status, 1, &status, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err; 
    
    if (!(status & 0x08)) {
        uint8_t init_cmd[] = {AHT20_CMD_INIT, 0x08, 0x00};
        //  把 -1 改为超时时间
        return i2c_master_transmit(dev_handle, init_cmd, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }
    return ESP_OK;
}

esp_err_t aht20_read_data(i2c_master_dev_handle_t dev_handle, float *temp, float *humi) {
    uint8_t trigger_cmd[] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    uint8_t data[6];

    //  超时机制
    esp_err_t err = i2c_master_transmit(dev_handle, trigger_cmd, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        //  I2C 失败是常事，打印个警告就行，千万别卡死
        ESP_LOGW(TAG, "I2C TX Failed (Wi-Fi interference?)");
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(80));

    //  超时机制
    err = i2c_master_receive(dev_handle, data, 6, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C RX Failed");
        return err;
    }

    uint32_t humi_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
    uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    *humi = (float)humi_raw * 100.0 / 1048576.0;
    *temp = (float)temp_raw * 200.0 / 1048576.0 - 50.0;

    return ESP_OK;
}