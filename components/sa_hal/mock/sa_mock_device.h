#ifndef SA_MOCK_DEVICE_H
#define SA_MOCK_DEVICE_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

sa_node_t* sa_mock_temp_create(void);
sa_node_t* sa_mock_relay_create(void);

// 光照与气压 Mock 设备
sa_node_t* sa_mock_lux_create(void);
sa_node_t* sa_mock_pressure_create(void);

#ifdef __cplusplus
}
#endif

#endif // SA_MOCK_DEVICE_H