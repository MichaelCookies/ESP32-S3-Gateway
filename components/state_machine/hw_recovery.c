#include "hw_recovery.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "settings.h"

#define AHT20_ADDR         0x38
#define AHT20_CMD_INIT     0xBE
#define AHT20_CMD_RESET    0xBA 

static const char *TAG = "HW_RECOVERY";

static SSD1306_t *g_oled_dev = NULL;
static i2c_master_bus_handle_t *g_aht_bus_ptr = NULL; // 保存指针以获取 bus handle
static i2c_master_dev_handle_t *g_aht_dev_ptr = NULL; // 保存指针以操作设备

void hw_recovery_init(SSD1306_t *oled, i2c_master_bus_handle_t *aht_bus, i2c_master_dev_handle_t *aht_dev) {
    g_oled_dev = oled;
    g_aht_bus_ptr = aht_bus;
    g_aht_dev_ptr = aht_dev;
}


// OLED 恢复逻辑

static void perform_oled_reset_sequence(void) {
    ESP_LOGW(TAG, "Executing OLED Hard Reset Sequence...");
    if (g_oled_dev == NULL) return;
    
    // 强制恢复地址标记
    g_oled_dev->_address = 0x3C; 
    
    // 重新执行初始化序列
    ssd1306_init(g_oled_dev, 128, 64);
    
    if (g_oled_dev->_address != 0) {
        ESP_LOGI(TAG, "OLED Reset Successful.");
        settings_set_oled_hw_status(HW_STATUS_CONNECTED);
    } else {
        ESP_LOGE(TAG, "OLED Reset Failed.");
    }
}


//  AHT20 恢复逻辑

static void perform_aht20_reset_sequence(void) {
    if (!g_aht_dev_ptr || !*g_aht_dev_ptr) return;

    ESP_LOGW(TAG, "Executing AHT20 Soft Reset Sequence...");

    //  发送软复位命令 (0xBA)
    uint8_t reset_cmd = AHT20_CMD_RESET;
    esp_err_t err = i2c_master_transmit(*g_aht_dev_ptr, &reset_cmd, 1, pdMS_TO_TICKS(100));
    
    if (err == ESP_OK) {
        //  复位后需要等待 20ms+
        vTaskDelay(pdMS_TO_TICKS(40));
        
        //  发送初始化命令 (0xBE 0x08 0x00)
        uint8_t init_cmd[] = {AHT20_CMD_INIT, 0x08, 0x00};
        err = i2c_master_transmit(*g_aht_dev_ptr, init_cmd, 3, pdMS_TO_TICKS(100));
        
        if (err == ESP_OK) {
            //  等待初始化完成
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGI(TAG, "AHT20 Reset & Init Successful.");
            settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
        } else {
            ESP_LOGE(TAG, "AHT20 Init Cmd Failed: %d", err);
        }
    } else {
        ESP_LOGE(TAG, "AHT20 Reset Cmd Failed: %d", err);
    }
}


//  守护任务

static void recovery_daemon_task(void *pvParameters) {
    ESP_LOGI(TAG, "Recovery Daemon Started (Low Priority)");

    while (1) {
        // 慢速轮询，不占 CPU
        vTaskDelay(pdMS_TO_TICKS(5000));


        //  检查 OLED

        if (settings_get_oled_hw_status() != HW_STATUS_CONNECTED) {
            ESP_LOGD(TAG, "Checking OLED Health...");
            if (g_oled_dev && g_oled_dev->_i2c_bus_handle) {
                // 探针，地址 0x3C
                esp_err_t probe_res = i2c_master_probe(g_oled_dev->_i2c_bus_handle, 0x3C, 50);

                if (probe_res == ESP_OK) {
                    // 物理连接OK，可能是僵尸状态，执行软复位刷新配置
                    ESP_LOGW(TAG, "OLED Probe OK. Performing Soft Re-init...");
                    g_oled_dev->_address = 0x3C;
                    ssd1306_init(g_oled_dev, 128, 64);
                    if (g_oled_dev->_address != 0) {
                        settings_set_oled_hw_status(HW_STATUS_CONNECTED);
                        ESP_LOGI(TAG, "OLED Recovered.");
                    }
                } else {
                    // 物理连接断开，执行硬复位尝试
                    ESP_LOGW(TAG, "OLED Probe Failed. Hard Reset Sequence...");
                    perform_oled_reset_sequence();
                }
            }
        }

        //  检查 AHT20

        if (settings_get_sensor_hw_status() != HW_STATUS_CONNECTED) {
            ESP_LOGD(TAG, "Checking Sensor Health...");
            // 需要通过 Bus Handle 来 Probe，因为 Dev Handle 绑定了地址
            if (g_aht_bus_ptr && *g_aht_bus_ptr) {
                // 探针: AHT20 地址 0x38
                esp_err_t probe_res = i2c_master_probe(*g_aht_bus_ptr, AHT20_ADDR, 50);
                
                if (probe_res == ESP_OK) {
                    ESP_LOGW(TAG, "Sensor Probe OK. Performing Protocol Reset...");
                    perform_aht20_reset_sequence();
                } else {
                    ESP_LOGW(TAG, "Sensor Probe Failed (Wire broken?). Retrying Reset anyway...");
                    perform_aht20_reset_sequence();
                }
            }
        }
    }
}

void hw_recovery_start_daemon(void) {
    // 栈大小 6KB，防止 I2C 错误日志撑爆
    xTaskCreate(recovery_daemon_task, "RecovDaemon", 6144, NULL, 1, NULL);
}

void hw_recovery_trigger_sensor_reset(void) {
    // 仅仅标记为 ERROR，让守护进程在后台处理
    // 这样不会阻塞 Sensor 读取任务
    ESP_LOGW(TAG, "Triggering Sensor Recovery (Flagged ERROR)");
    settings_set_sensor_hw_status(HW_STATUS_ERROR);
}