#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "ssd1306.h"

#define TAG "SSD1306"

#if CONFIG_I2C_PORT_0
#define I2C_NUM I2C_NUM_0
#elif CONFIG_I2C_PORT_1
#define I2C_NUM I2C_NUM_1
#else
#define I2C_NUM I2C_NUM_0 
#endif

#define I2C_MASTER_FREQ_HZ 400000   
#define I2C_TICK_TIMEOUT   pdMS_TO_TICKS(1000) 

#define SSD1306_MAX_WIDTH 128

void i2c_master_init(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset)
{
    ESP_LOGI(TAG, "Initializing SSD1306 I2C (50kHz Split-Mode)...");

    if (reset >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << reset),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(reset, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(reset, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t i2c_bus_handle;
    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    // 允许复用总线
    if (err != ESP_OK && i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C Bus Init Error: %d", err);
        dev->_address = 0;
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    i2c_master_dev_handle_t i2c_dev_handle;
    err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C Dev Add Error: %d", err);
        dev->_address = 0;
        return;
    }

    dev->_address = I2C_ADDRESS;
    dev->_flip = false;
    dev->_i2c_num = I2C_NUM;
    dev->_i2c_bus_handle = i2c_bus_handle;
    dev->_i2c_dev_handle = i2c_dev_handle;
}

void i2c_device_add(SSD1306_t * dev, i2c_port_t i2c_num, int16_t reset, uint16_t i2c_address)
{
    if (dev->_i2c_bus_handle == NULL) return;

    if (reset >= 0) {
        gpio_reset_pin(reset);
        gpio_set_direction(reset, GPIO_MODE_OUTPUT);
        gpio_set_level(reset, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(reset, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    i2c_master_dev_handle_t i2c_dev_handle;
    if (i2c_master_bus_add_device(dev->_i2c_bus_handle, &dev_cfg, &i2c_dev_handle) == ESP_OK) {
        dev->_address = i2c_address;
        dev->_flip = false;
        dev->_i2c_num = i2c_num;
        dev->_i2c_dev_handle = i2c_dev_handle;
    } else {
        dev->_address = 0;
    }
}

void i2c_init(SSD1306_t * dev, int width, int height) {
    if (dev->_address == 0) return;
    dev->_width = width;
    dev->_height = height;
    dev->_pages = (height == 32) ? 4 : 8;

    uint8_t out_buf[32]; 
    int idx = 0;
    out_buf[idx++] = OLED_CONTROL_BYTE_CMD_STREAM;
    out_buf[idx++] = OLED_CMD_DISPLAY_OFF;
    out_buf[idx++] = OLED_CMD_SET_MUX_RATIO;
    out_buf[idx++] = (dev->_height == 64) ? 0x3F : 0x1F;
    out_buf[idx++] = OLED_CMD_SET_DISPLAY_OFFSET;
    out_buf[idx++] = 0x00;
    out_buf[idx++] = OLED_CMD_SET_DISPLAY_START_LINE;
    out_buf[idx++] = dev->_flip ? OLED_CMD_SET_SEGMENT_REMAP_0 : OLED_CMD_SET_SEGMENT_REMAP_1;
    out_buf[idx++] = OLED_CMD_SET_COM_SCAN_MODE;
    out_buf[idx++] = OLED_CMD_SET_DISPLAY_CLK_DIV;
    out_buf[idx++] = 0x80;
    out_buf[idx++] = OLED_CMD_SET_COM_PIN_MAP;
    out_buf[idx++] = (dev->_height == 64) ? 0x12 : 0x02;
    out_buf[idx++] = OLED_CMD_SET_CONTRAST;
    out_buf[idx++] = 0xFF;
    out_buf[idx++] = OLED_CMD_DISPLAY_RAM;
    out_buf[idx++] = OLED_CMD_SET_VCOMH_DESELCT;
    out_buf[idx++] = 0x40;
    out_buf[idx++] = OLED_CMD_SET_MEMORY_ADDR_MODE;
    out_buf[idx++] = OLED_CMD_SET_PAGE_ADDR_MODE;
    out_buf[idx++] = 0x00;
    out_buf[idx++] = 0x10;
    out_buf[idx++] = OLED_CMD_SET_CHARGE_PUMP;
    out_buf[idx++] = 0x14;
    out_buf[idx++] = OLED_CMD_DEACTIVE_SCROLL;
    out_buf[idx++] = OLED_CMD_DISPLAY_NORMAL;
    out_buf[idx++] = OLED_CMD_DISPLAY_ON;

    esp_err_t res = i2c_master_transmit(dev->_i2c_dev_handle, out_buf, idx, I2C_TICK_TIMEOUT);
    if (res != ESP_OK) dev->_address = 0; 
}

//	分包发送

void i2c_display_image(SSD1306_t * dev, int page, int seg, const uint8_t * images, int width) {
    if (dev->_address == 0) return; 
    if (width > SSD1306_MAX_WIDTH) width = SSD1306_MAX_WIDTH;

    int _seg = seg + CONFIG_OFFSETX;
    uint8_t columLow = _seg & 0x0F;
    uint8_t columHigh = (_seg >> 4) & 0x0F;
    int _page = dev->_flip ? (dev->_pages - page) - 1 : page;

    //	发送地址指令
    uint8_t cmd_buf[4];
    cmd_buf[0] = OLED_CONTROL_BYTE_CMD_STREAM;
    cmd_buf[1] = (0x00 + columLow);
    cmd_buf[2] = (0x10 + columHigh);
    cmd_buf[3] = 0xB0 | _page;

    if (i2c_master_transmit(dev->_i2c_dev_handle, cmd_buf, 4, I2C_TICK_TIMEOUT) != ESP_OK) {
        dev->_address = 0;
        return;
    }

    //	发送图像数据，拆成 16 字节的小包发送
    #define CHUNK_SIZE 16
    uint8_t data_buf[CHUNK_SIZE + 1]; 
    data_buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    int remaining = width;
    int offset = 0;

    while (remaining > 0) {
        int tx_len = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        
        //	复制这一小段数据
        memcpy(&data_buf[1], &images[offset], tx_len);
        
        //	发送这一小段
        if (i2c_master_transmit(dev->_i2c_dev_handle, data_buf, tx_len + 1, I2C_TICK_TIMEOUT) != ESP_OK) {
            ESP_LOGE(TAG, "Chunk TX Fail");
            dev->_address = 0; //	只要有一包挂了，就标记离线
            return;
        }

        remaining -= tx_len;
        offset += tx_len;
    }
}

void i2c_contrast(SSD1306_t * dev, int contrast) {
    if (dev->_address == 0) return;
    uint8_t out_buf[3] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, (uint8_t)contrast};
    i2c_master_transmit(dev->_i2c_dev_handle, out_buf, 3, I2C_TICK_TIMEOUT);
}

void i2c_hardware_scroll(SSD1306_t * dev, ssd1306_scroll_type_t scroll) {
    (void)dev; (void)scroll; 
}