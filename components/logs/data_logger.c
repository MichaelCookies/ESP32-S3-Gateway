#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lan_service.h"
#include "sa_device_manager.h"
#include "data_logger.h"

static const char *TAG = "LOGGER";

#define PARTITION_LABEL "storage"
#define MOUNT_POINT     "/spiffs"
#define LOG_FILE        "/spiffs/sensor_log.csv"
#define TEMP_FILE       "/spiffs/sensor_log.tmp"
#define MAX_FILE_SIZE   (100 * 1024) 

static TaskHandle_t s_sync_task_handle = NULL;

bool data_logger_init(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS...");
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
    size_t total = 0, used = 0;
    esp_spiffs_info(PARTITION_LABEL, &total, &used);
    ESP_LOGI(TAG, "Filesystem Ready. Used: %d / %d", used, total);
    return true;
}

void data_logger_save_snapshot(void) {
    struct stat st;
    if (stat(LOG_FILE, &st) == 0 && st.st_size >= MAX_FILE_SIZE) {
        ESP_LOGW(TAG, "Log full, dropping data.");
        return;
    }

    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) return;

    fprintf(f, "%lu,", (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));

    sa_mgr_list_lock();
    sa_node_t *curr = sa_mgr_get_list_head();
    bool first = true;
    while (curr) {
        if (curr->type == SA_NODE_SENSOR) {
            sa_value_t val;
            bool online;
            if (sa_mgr_read_node_cache(curr, &val, &online) == ESP_OK && online) {
                if (!first) fprintf(f, ",");
                if (curr->val_type == SA_VAL_TYPE_FLOAT) {
                    fprintf(f, "%.2f", val.f_val);
                } else if (curr->val_type == SA_VAL_TYPE_INT) {
                    fprintf(f, "%d", (int)val.i_val);
                } else if (curr->val_type == SA_VAL_TYPE_BOOL) {
                    fprintf(f, "%d", val.b_val ? 1 : 0);
                }
                first = false;
            }
        }
        curr = curr->next;
    }
    sa_mgr_list_unlock();

    fprintf(f, "\n");
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
    ESP_LOGI(TAG, "Sync Task Started (Priority 4)");

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

    char line[128];
    int count = 0;

    while (fgets(line, sizeof(line), f) != NULL) {
        line[strcspn(line, "\r\n")] = 0; 
        if (strlen(line) > 0 && send_cb) {
            send_cb(line);
            count++;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    fclose(f);
    unlink(TEMP_FILE);
    
    ESP_LOGI(TAG, "Sync finished. Uploaded %d records.", count);
    s_sync_task_handle = NULL;
    vTaskDelete(NULL);
}

void data_logger_sync_data(logger_sync_cb_t send_cb) {
    if (s_sync_task_handle != NULL) return;
    if (!data_logger_has_data()) return;
    xTaskCreate(task_sync_worker, "LogSync", 4096, (void*)send_cb, 4, &s_sync_task_handle);
}