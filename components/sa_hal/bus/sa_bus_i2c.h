// sa_bus_i2c.h
#ifndef SA_BUS_I2C_H
#define SA_BUS_I2C_H

#include "sa_hal.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

sa_bus_t* sa_bus_i2c_create(i2c_master_bus_handle_t idf_bus_handle, const char *bus_name);

void sa_bus_i2c_destroy(sa_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif // SA_BUS_I2C_H