#ifndef WIFI_STATION_H
#define WIFI_STATION_H

#include "esp_err.h"

// 简单的初始化函数
void wifi_init_sta(void);

// 重连函数
void wifi_force_reset(void);

#endif