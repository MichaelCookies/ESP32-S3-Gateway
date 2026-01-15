#ifndef AHT20_H
#define AHT20_H

#include "driver/i2c_master.h"

#define AHT20_ADDR         0x38  // AHT20固定地址
#define AHT20_CMD_INIT     0xBE  // 初始化指令
#define AHT20_CMD_TRIGGER  0xAC  // 触发测量指令
#define AHT20_CMD_RESET    0xBA  // 软复位指令

//  初始化函数：传入I2C设备句柄
esp_err_t aht20_init(i2c_master_dev_handle_t dev_handle);

//  读取函数：传入句柄，返回温湿度
esp_err_t aht20_read_data(i2c_master_dev_handle_t dev_handle, float *temp, float *humi);

#endif // AHT20_H