#ifndef SA_BUS_I2C_H
#define SA_BUS_I2C_H

#include "sa_hal.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 I2C 总线 HAL 对象
 * 
 * @param idf_bus_handle ESP-IDF 原生 I2C 总线句柄
 * @param bus_name 总线名称（用于日志）
 * @return sa_bus_t* HAL 总线对象，失败返回 NULL
 */
sa_bus_t* sa_bus_i2c_create(i2c_master_bus_handle_t idf_bus_handle, const char *bus_name);

/**
 * @brief 销毁 I2C 总线 HAL 对象
 */
void sa_bus_i2c_destroy(sa_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif // SA_BUS_I2C_H