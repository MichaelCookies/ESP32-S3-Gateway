#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_logger.h"

static const char *TAG = "LOGGER";

#define PARTITION_LABEL "storage"
#define MOUNT_POINT     "/spiffs"
#define LOG_FILE        "/spiffs/sensor_log.jsonl"
#define TEMP_FILE       "/spiffs/sensor_log.tmp"

#define MAX_FILE_SIZE   (100 * 1024) 

static TaskHandle_t s_sync_task_handle = NULL;

bool data_logger_init(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = MOUNT_POINT,
        .partition_label = PARTITION_LABEL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS Init Failed: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

void data_logger_write(const char *json_payload) {
    if (!json_payload) return;

    struct stat st;
    if (stat(LOG_FILE, &st) == 0 && st.st_size >= MAX_FILE_SIZE) {
        ESP_LOGW(TAG, "Log full, dropping data.");
        return;
    }

    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) return;
    fprintf(f, "%s\n", json_payload);
    fclose(f);
}

bool data_logger_has_data(void) {
    struct stat st;
    if (stat(TEMP_FILE, &st) == 0 && st.st_size > 0) return true;
    if (stat(LOG_FILE, &st) == 0 && st.st_size > 0) return true;
    return false;
}

static void task_sync_worker(void *pvParameters) {
    logger_sync_cb_t send_cb = (logger_sync_cb_t)pvParameters;
    struct stat st;
    bool has_temp = (stat(TEMP_FILE, &st) == 0 && st.st_size > 0);

    if (!has_temp) {
        if (stat(LOG_FILE, &st) == 0 && st.st_size > 0) {
            if (rename(LOG_FILE, TEMP_FILE) != 0) {
                s_sync_task_handle = NULL;
                vTaskDelete(NULL);
                return;
            }
        } else {
            s_sync_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    FILE* f = fopen(TEMP_FILE, "r");
    if (f == NULL) {
        s_sync_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    char line[256];
    while (fgets(line, sizeof(line), f) != NULL) {
        size_t len = strlen(line);
        if (len > 0 && line[len-1] == '\n') line[len-1] = '\0';
        
        if (send_cb && strlen(line) > 0) {
            send_cb(line);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    fclose(f);
    unlink(TEMP_FILE);
    
    s_sync_task_handle = NULL;
    vTaskDelete(NULL);
}

void data_logger_sync_data(logger_sync_cb_t send_cb) {
    if (s_sync_task_handle != NULL || !data_logger_has_data()) return;
    xTaskCreate(task_sync_worker, "LogSync", 4096, (void*)send_cb, 4, &s_sync_task_handle);
}