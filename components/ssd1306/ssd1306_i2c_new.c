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

#define I2C_MASTER_FREQ_HZ 100000	// 弱网或不稳定硬件建议用 100kHz
//	统一使用快速超时，绝不阻塞主任务
#define I2C_FAST_TIMEOUT  pdMS_TO_TICKS(50) 

void i2c_master_init(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset)
{
	ESP_LOGI(TAG, "New i2c driver is used");
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM,
		.scl_io_num = scl,
		.sda_io_num = sda,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t i2c_bus_handle;
    //	移除错误检查，交给逻辑处理
	esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
    if (err != ESP_OK) return;

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = I2C_ADDRESS,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
	i2c_master_dev_handle_t i2c_dev_handle;
	err = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) return;

	if (reset >= 0) {
		gpio_reset_pin(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(pdMS_TO_TICKS(50));
		gpio_set_level(reset, 1);
	}

	dev->_address = I2C_ADDRESS;
	dev->_flip = false;
	dev->_i2c_num = I2C_NUM;
	dev->_i2c_bus_handle = i2c_bus_handle;
	dev->_i2c_dev_handle = i2c_dev_handle;
}

void i2c_device_add(SSD1306_t * dev, i2c_port_t i2c_num, int16_t reset, uint16_t i2c_address)
{
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = i2c_address,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
	i2c_master_dev_handle_t i2c_dev_handle;
	i2c_master_bus_add_device(dev->_i2c_bus_handle, &dev_cfg, &i2c_dev_handle);

	if (reset >= 0) {
		gpio_reset_pin(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(pdMS_TO_TICKS(50));
		gpio_set_level(reset, 1);
	}

	dev->_address = i2c_address;
	dev->_flip = false;
	dev->_i2c_num = i2c_num;
	dev->_i2c_dev_handle = i2c_dev_handle;
}

void i2c_init(SSD1306_t * dev, int width, int height) {
	dev->_width = width;
	dev->_height = height;
	dev->_pages = 8;
	if (dev->_height == 32) dev->_pages = 4;
	
	uint8_t out_buf[27];
	int out_index = 0;
	out_buf[out_index++] = OLED_CONTROL_BYTE_CMD_STREAM;
	out_buf[out_index++] = OLED_CMD_DISPLAY_OFF;
	out_buf[out_index++] = OLED_CMD_SET_MUX_RATIO;
	if (dev->_height == 64) out_buf[out_index++] = 0x3F;
	if (dev->_height == 32) out_buf[out_index++] = 0x1F;
	out_buf[out_index++] = OLED_CMD_SET_DISPLAY_OFFSET;
	out_buf[out_index++] = 0x00;
	out_buf[out_index++] = OLED_CMD_SET_DISPLAY_START_LINE;
	if (dev->_flip) out_buf[out_index++] = OLED_CMD_SET_SEGMENT_REMAP_0;
	else out_buf[out_index++] = OLED_CMD_SET_SEGMENT_REMAP_1;
	out_buf[out_index++] = OLED_CMD_SET_COM_SCAN_MODE;
	out_buf[out_index++] = OLED_CMD_SET_DISPLAY_CLK_DIV;
	out_buf[out_index++] = 0x80;
	out_buf[out_index++] = OLED_CMD_SET_COM_PIN_MAP;
	if (dev->_height == 64) out_buf[out_index++] = 0x12;
	if (dev->_height == 32) out_buf[out_index++] = 0x02;
	out_buf[out_index++] = OLED_CMD_SET_CONTRAST;
	out_buf[out_index++] = 0xFF;
	out_buf[out_index++] = OLED_CMD_DISPLAY_RAM;
	out_buf[out_index++] = OLED_CMD_SET_VCOMH_DESELCT;
	out_buf[out_index++] = 0x40;
	out_buf[out_index++] = OLED_CMD_SET_MEMORY_ADDR_MODE;
	out_buf[out_index++] = OLED_CMD_SET_PAGE_ADDR_MODE;
	out_buf[out_index++] = 0x00;
	out_buf[out_index++] = 0x10;
	out_buf[out_index++] = OLED_CMD_SET_CHARGE_PUMP;
	out_buf[out_index++] = 0x14;
	out_buf[out_index++] = OLED_CMD_DEACTIVE_SCROLL;
	out_buf[out_index++] = OLED_CMD_DISPLAY_NORMAL;
	out_buf[out_index++] = OLED_CMD_DISPLAY_ON;

	esp_err_t res = i2c_master_transmit(dev->_i2c_dev_handle, out_buf, out_index, I2C_FAST_TIMEOUT);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "OLED Init Failed: %d", res);
        dev->_address = 0;	// 标记不在线
    } else {
        ESP_LOGI(TAG, "OLED Ready.");
    }
}

void i2c_display_image(SSD1306_t * dev, int page, int seg, const uint8_t * images, int width) {
	if (dev->_address == 0) return; //	硬件不在线，直接返回

	int _seg = seg + CONFIG_OFFSETX;
	uint8_t columLow = _seg & 0x0F;
	uint8_t columHigh = (_seg >> 4) & 0x0F;
	int _page = dev->_flip ? (dev->_pages - page) - 1 : page;

	uint8_t *out_buf = malloc(width + 4);
	if (out_buf == NULL) return;

	int out_index = 0;
	out_buf[out_index++] = OLED_CONTROL_BYTE_CMD_STREAM;
	out_buf[out_index++] = (0x00 + columLow);
	out_buf[out_index++] = (0x10 + columHigh);
	out_buf[out_index++] = 0xB0 | _page;

    //	使用快速超时
	esp_err_t res = i2c_master_transmit(dev->_i2c_dev_handle, out_buf, out_index, I2C_FAST_TIMEOUT);
	if (res == ESP_OK) {
        out_buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
        memcpy(&out_buf[1], images, width);
        res = i2c_master_transmit(dev->_i2c_dev_handle, out_buf, width + 1, I2C_FAST_TIMEOUT);
    }
    
	if (res != ESP_OK) {
        ESP_LOGE(TAG, "Display error, marking offline");
        dev->_address = 0; //	通讯失败立刻标记离线，交给恢复模块处理
    }
	free(out_buf);
}

// i2c_contrast 和 i2c_hardware_scroll 同理，全部使用 I2C_FAST_TIMEOUT
void i2c_contrast(SSD1306_t * dev, int contrast) {
	if (dev->_address == 0) return;
	uint8_t out_buf[3] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, (uint8_t)contrast};
	if (i2c_master_transmit(dev->_i2c_dev_handle, out_buf, 3, I2C_FAST_TIMEOUT) != ESP_OK) {
        dev->_address = 0;
    }
}

void i2c_hardware_scroll(SSD1306_t * dev, ssd1306_scroll_type_t scroll) {
	//	SCROLL_STOP 不执行任何操作
}