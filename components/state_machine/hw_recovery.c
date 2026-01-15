#include "hw_recovery.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "settings.h"
#include "aht20.h"

static const char *TAG = "HW_RECOVERY";

static SSD1306_t *p_oled = NULL;
static i2c_master_bus_handle_t *p_aht_bus = NULL; // 改为指针以直接修改 main 里的句柄
static i2c_master_dev_handle_t *p_aht_dev = NULL;

static uint32_t g_oled_next_retry = 0;
static uint32_t g_sensor_next_retry = 0;
#define RECOVERY_COOLDOWN_MS 20000 

//  物理层强力清理

static void force_release_i2c_pins(int sda, int scl) {
    ESP_LOGD(TAG, "Force resetting pins SDA:%d SCL:%d", sda, scl);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void i2c_bus_recovery_sequence(int sda, int scl) {
    ESP_LOGW(TAG, "Starting Hardware Recovery Pulse on SDA:%d SCL:%d", sda, scl);
    
    //  强制重置引脚状态，清除之前的驱动配置
    force_release_i2c_pins(sda, scl);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl)
    };
    gpio_config(&io_conf);

    //  9个脉冲
    for (int i = 0; i < 9; i++) {
        gpio_set_level(scl, 0); vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(scl, 1); vTaskDelay(pdMS_TO_TICKS(5));
    }

    //  STOP 信号
    gpio_set_level(scl, 1);
    gpio_set_level(sda, 0); vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(sda, 1); vTaskDelay(pdMS_TO_TICKS(5));

    //  再次重置引脚，准备让 I2C 驱动重新接管
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    vTaskDelay(pdMS_TO_TICKS(50));
}

//  硬复位具体实现

static void perform_oled_reset(void) {
    ESP_LOGW(TAG, "Executing OLED Hard Reset...");
    
    //  严格按顺序销毁
    if (p_oled->_i2c_dev_handle) {
        i2c_master_bus_rm_device(p_oled->_i2c_dev_handle);
        p_oled->_i2c_dev_handle = NULL;
    }
    if (p_oled->_i2c_bus_handle) {
        i2c_del_master_bus(p_oled->_i2c_bus_handle);
        p_oled->_i2c_bus_handle = NULL;
    }

    //  物理拨动
    i2c_bus_recovery_sequence(CONFIG_SDA_GPIO, CONFIG_SCL_GPIO);

    //  重新加载，使用 i2c_master_init 重新给 oled_dev 赋值
    i2c_master_init(p_oled, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(p_oled, 128, 64);

    if (p_oled->_address != 0) {
        settings_set_oled_hw_status(HW_STATUS_CONNECTED);
        ssd1306_clear_screen(p_oled, false);
        ESP_LOGI(TAG, "OLED Recovery Successful.");
    } else {
        g_oled_next_retry = xTaskGetTickCount() * portTICK_PERIOD_MS + RECOVERY_COOLDOWN_MS;
        ESP_LOGE(TAG, "OLED Recovery Failed.");
    }
}

static void perform_sensor_reset(void) {
    ESP_LOGW(TAG, "Executing Sensor Hard Reset...");
    
    //  销毁逻辑
    if (*p_aht_dev) {
        i2c_master_bus_rm_device(*p_aht_dev);
        *p_aht_dev = NULL;
    }
    if (*p_aht_bus) {
        i2c_del_master_bus(*p_aht_bus);
        *p_aht_bus = NULL;
    }

    //  物理拨动
    i2c_bus_recovery_sequence(CONFIG_AHT_SDA_GPIO, CONFIG_AHT_SCL_GPIO);

    //  重新建立
    i2c_master_bus_config_t cfg = {
        .i2c_port = CONFIG_AHT_I2C_PORT_NUM,
        .sda_io_num = CONFIG_AHT_SDA_GPIO, .scl_io_num = CONFIG_AHT_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true
    };
    
    //  通过指针直接修改 main.c 里的句柄变量
    if (i2c_new_master_bus(&cfg, p_aht_bus) == ESP_OK) {
        i2c_device_config_t dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = AHT20_ADDR, .scl_speed_hz = 100000 };
        if (i2c_master_bus_add_device(*p_aht_bus, &dev_cfg, p_aht_dev) == ESP_OK) {
            if (aht20_init(*p_aht_dev) == ESP_OK) {
                settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
                ESP_LOGI(TAG, "Sensor Recovery Successful.");
                return;
            }
        }
    }
    g_sensor_next_retry = xTaskGetTickCount() * portTICK_PERIOD_MS + RECOVERY_COOLDOWN_MS;
    ESP_LOGE(TAG, "Sensor Recovery Failed.");
}

//  公开接口

void hw_recovery_init(SSD1306_t *oled_ptr, 
                      i2c_master_bus_handle_t *aht_bus_ptr, 
                      i2c_master_dev_handle_t *aht_dev_ptr) {
    p_oled = oled_ptr;
    p_aht_bus = aht_bus_ptr; // 改为接收地址
    p_aht_dev = aht_dev_ptr;
}

void hw_recovery_tick(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    //  OLED 恢复
    if (settings_get_oled_hw_status() != HW_STATUS_CONNECTED && now > g_oled_next_retry) {
        perform_oled_reset();
    }

    //  Sensor 恢复
    if (settings_get_sensor_hw_status() != HW_STATUS_CONNECTED && now > g_sensor_next_retry) {
        perform_sensor_reset();
    }
}

void hw_recovery_trigger_sensor_reset(void) { settings_set_sensor_hw_status(HW_STATUS_DISCONNECTED); }
void hw_recovery_trigger_oled_reset(void) { settings_set_oled_hw_status(HW_STATUS_DISCONNECTED); }
