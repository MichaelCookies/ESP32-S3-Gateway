#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void settings_init(void);

float settings_get_threshold(void);
void settings_set_threshold(float new_val);

bool settings_get_sensor_enabled(void);
void settings_set_sensor_enabled(bool enabled);

bool settings_get_display_enabled(void);
void settings_set_display_enabled(bool enabled);

bool settings_get_cloud_enabled(void);
void settings_set_cloud_enabled(bool enabled);

sys_mode_t settings_get_mode(void);
void settings_set_mode(sys_mode_t new_mode);

bool settings_get_alarm_state(void);
void settings_set_alarm_state(bool on);

bool settings_get_cloud_lock(void);
void settings_set_cloud_lock(bool locked);

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_H