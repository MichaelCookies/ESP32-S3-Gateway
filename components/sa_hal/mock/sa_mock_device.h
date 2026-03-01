// sa_mock_device.h
#ifndef SA_MOCK_DEVICE_H
#define SA_MOCK_DEVICE_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

sa_node_t* sa_mock_temp_create(void);

sa_node_t* sa_mock_relay_create(void);

#ifdef __cplusplus
}
#endif

#endif // SA_MOCK_DEVICE_H