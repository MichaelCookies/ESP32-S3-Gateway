/*
 * settings.c
 * V4.2
 * 职责：集中管理配置与硬件状态
 */

#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "SETTINGS";

#define NVS_NAMESPACE  "storage"
#define KEY_THRESHOLD  "th_temp"
#define KEY_SENSOR_SW  "sw_sens"
#define KEY_DISPLAY_SW "sw_disp"
#define KEY_CLOUD_SW   "sw_cloud"

//  私有数据结构
typedef struct {
    //  持久化配置
    float alarm_threshold;
    bool is_sensor_enabled;
    bool is_display_enabled;
    bool is_cloud_enabled;

    //  运行时逻辑状态
    sys_mode_t sys_mode;
    bool is_alarm_ringing;
    bool is_cloud_locked;

    //  硬件物理状态
    hw_status_t oled_status;
    hw_status_t sensor_status;
} app_runtime_t;

//  默认值
static app_runtime_t g_runtime = {
    .alarm_threshold = 25.0f,
    .is_sensor_enabled = true,
    .is_display_enabled = true,
    .is_cloud_enabled = true,

    .sys_mode = MODE_AUTO,
    .is_alarm_ringing = false,
    .is_cloud_locked = false,

    //  默认硬件状态为未知，等待 main 探测
    .oled_status = HW_STATUS_UNKNOWN,
    .sensor_status = HW_STATUS_UNKNOWN
};

static SemaphoreHandle_t g_mutex = NULL;

//  NVS 内部辅助

static esp_err_t nvs_write_blob(const char* key, void* data, size_t len) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(my_handle, key, data, len);
        if (err == ESP_OK) err = nvs_commit(my_handle);
        nvs_close(my_handle);
    }
    return err;
}

static esp_err_t nvs_read_blob(const char* key, void* out_data, size_t len) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        err = nvs_get_blob(my_handle, key, out_data, &len);
        nvs_close(my_handle);
    }
    return err;
}

static void load_all_settings_from_nvs(void) {
    if (nvs_read_blob(KEY_THRESHOLD, &g_runtime.alarm_threshold, sizeof(float)) == ESP_OK)
        ESP_LOGI(TAG, "Loaded Threshold: %.1f", g_runtime.alarm_threshold);
    if (nvs_read_blob(KEY_SENSOR_SW, &g_runtime.is_sensor_enabled, sizeof(bool)) == ESP_OK)
        ESP_LOGI(TAG, "Loaded Sensor SW: %d", g_runtime.is_sensor_enabled);
    if (nvs_read_blob(KEY_DISPLAY_SW, &g_runtime.is_display_enabled, sizeof(bool)) == ESP_OK)
        ESP_LOGI(TAG, "Loaded Display SW: %d", g_runtime.is_display_enabled);
    if (nvs_read_blob(KEY_CLOUD_SW, &g_runtime.is_cloud_enabled, sizeof(bool)) == ESP_OK)
        ESP_LOGI(TAG, "Loaded Cloud SW: %d", g_runtime.is_cloud_enabled);
}

//  初始化

void settings_init(void) {
    g_mutex = xSemaphoreCreateMutex();
    if (!g_mutex) abort();

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

//  配置存取接口

//  阈值
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

//  传感器开关
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

//  显示开关
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

//  云端开关
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

//  状态存取接口

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

//  硬件状态接口

hw_status_t settings_get_oled_hw_status(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    hw_status_t val = g_runtime.oled_status;
    xSemaphoreGive(g_mutex);
    return val;
}
void settings_set_oled_hw_status(hw_status_t status) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.oled_status = status;
    xSemaphoreGive(g_mutex);
}

hw_status_t settings_get_sensor_hw_status(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    hw_status_t val = g_runtime.sensor_status;
    xSemaphoreGive(g_mutex);
    return val;
}
void settings_set_sensor_hw_status(hw_status_t status) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_runtime.sensor_status = status;
    xSemaphoreGive(g_mutex);
}

//  单元测试
bool settings_run_self_test(void) {
    ESP_LOGW(TAG, "Running Self-Test...");
    float backup_th = settings_get_threshold();
    float test_val = 88.8f;
    settings_set_threshold(test_val);
    
    if (fabs(settings_get_threshold() - test_val) > 0.01f) return false;
    
    float nvs_val = 0.0f;
    if (nvs_read_blob(KEY_THRESHOLD, &nvs_val, sizeof(float)) != ESP_OK) return false;
    if (fabs(nvs_val - test_val) > 0.01f) return false;

    settings_set_threshold(backup_th);
    ESP_LOGI(TAG, "Self-Test PASSED");
    return true;
}