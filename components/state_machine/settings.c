#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "settings.h"

static const char *TAG = "SETTINGS";

#define NVS_NAMESPACE  "storage"
#define KEY_THRESHOLD  "th_temp"
#define KEY_SENSOR_SW  "sw_sens"
#define KEY_DISPLAY_SW "sw_disp"
#define KEY_CLOUD_SW   "sw_cloud"

typedef struct {
    float alarm_threshold;
    bool is_sensor_enabled;
    bool is_display_enabled;
    bool is_cloud_enabled;

    sys_mode_t sys_mode;
    bool is_alarm_ringing;
    bool is_cloud_locked;
} app_runtime_t;

static app_runtime_t g_runtime = {
    .alarm_threshold = 25.0f,
    .is_sensor_enabled = true,
    .is_display_enabled = true,
    .is_cloud_enabled = true,
    .sys_mode = MODE_AUTO,
    .is_alarm_ringing = false,
    .is_cloud_locked = false
};

static SemaphoreHandle_t g_mutex = NULL;

static esp_err_t nvs_write_blob(const char* key, const void* data, size_t len) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(my_handle, key, data, len);
        if (err == ESP_OK) {
            err = nvs_commit(my_handle);
        }
        nvs_close(my_handle);
    }
    return err;
}

static esp_err_t nvs_read_blob(const char* key, void* out_data, size_t len) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t read_len = len;
        err = nvs_get_blob(my_handle, key, out_data, &read_len);
        nvs_close(my_handle);
    }
    return err;
}

static void load_all_settings_from_nvs(void) {
    if (nvs_read_blob(KEY_THRESHOLD, &g_runtime.alarm_threshold, sizeof(float)) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded Threshold: %.1f", g_runtime.alarm_threshold);
    }
    if (nvs_read_blob(KEY_SENSOR_SW, &g_runtime.is_sensor_enabled, sizeof(bool)) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded Sensor SW: %d", g_runtime.is_sensor_enabled);
    }
    if (nvs_read_blob(KEY_DISPLAY_SW, &g_runtime.is_display_enabled, sizeof(bool)) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded Display SW: %d", g_runtime.is_display_enabled);
    }
    if (nvs_read_blob(KEY_CLOUD_SW, &g_runtime.is_cloud_enabled, sizeof(bool)) == ESP_OK) {
        ESP_LOGI(TAG, "Loaded Cloud SW: %d", g_runtime.is_cloud_enabled);
    }
}

void settings_init(void) {
    if (g_mutex == NULL) {
        g_mutex = xSemaphoreCreateMutex();
        if (!g_mutex) abort();
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    load_all_settings_from_nvs();
    xSemaphoreGive(g_mutex);
}

float settings_get_threshold(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    float val = g_runtime.alarm_threshold;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_threshold(float new_val) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.alarm_threshold = new_val;
    xSemaphoreGive(g_mutex);
    nvs_write_blob(KEY_THRESHOLD, &new_val, sizeof(float));
}

bool settings_get_sensor_enabled(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool val = g_runtime.is_sensor_enabled;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_sensor_enabled(bool enabled) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.is_sensor_enabled = enabled;
    xSemaphoreGive(g_mutex);
    nvs_write_blob(KEY_SENSOR_SW, &enabled, sizeof(bool));
}

bool settings_get_display_enabled(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool val = g_runtime.is_display_enabled;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_display_enabled(bool enabled) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.is_display_enabled = enabled;
    xSemaphoreGive(g_mutex);
    nvs_write_blob(KEY_DISPLAY_SW, &enabled, sizeof(bool));
}

bool settings_get_cloud_enabled(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool val = g_runtime.is_cloud_enabled;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_cloud_enabled(bool enabled) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.is_cloud_enabled = enabled;
    xSemaphoreGive(g_mutex);
    nvs_write_blob(KEY_CLOUD_SW, &enabled, sizeof(bool));
}

sys_mode_t settings_get_mode(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    sys_mode_t val = g_runtime.sys_mode;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_mode(sys_mode_t new_mode) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.sys_mode = new_mode;
    xSemaphoreGive(g_mutex);
}

bool settings_get_alarm_state(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool val = g_runtime.is_alarm_ringing;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_alarm_state(bool on) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.is_alarm_ringing = on;
    xSemaphoreGive(g_mutex);
}

bool settings_get_cloud_lock(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    bool val = g_runtime.is_cloud_locked;
    xSemaphoreGive(g_mutex);
    return val;
}

void settings_set_cloud_lock(bool locked) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.is_cloud_locked = locked;
    xSemaphoreGive(g_mutex);
}