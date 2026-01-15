/*
 * data_logger.c
 * V4.2 弱网高可靠性版
 */

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"   //  它能解决同步阻塞导致的重启
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "data_logger.h"

static const char *TAG = "LOGGER";

//  必须与 partitions.csv 中的标签一致！！
#define PARTITION_LABEL "storage"
#define MOUNT_POINT     "/spiffs"
#define LOG_FILE        "/spiffs/sensor_log.csv"
#define TEMP_FILE       "/spiffs/sensor_log.tmp"

//  保护措施：限制日志文件大小，目前是100KB
#define MAX_FILE_SIZE   (100 * 1024) 

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
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition in map");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(PARTITION_LABEL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get partition info");
    } else {
        ESP_LOGI(TAG, "Filesystem initialized. Size: %d, Used: %d", total, used);
    }

    return true;
}

void data_logger_write(float temp, float humi) {
    //  容量安全检查
    struct stat st;
    if (stat(LOG_FILE, &st) == 0) {
        if (st.st_size >= MAX_FILE_SIZE) {
            ESP_LOGW(TAG, "Log file reached limit, skipping write to protect flash");
            return;
        }
    }

    //  追加方式打开文件
    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Cannot open log file for writing");
        return;
    }

    //  存储 CSV 格式
    fprintf(f, "%.2f,%.2f\n", temp, humi);
    fclose(f);
}

bool data_logger_has_data(void) {
    struct stat st;
    //  如果文件存在且大小大于0
    if (stat(LOG_FILE, &st) == 0) {
        return (st.st_size > 0);
    }
    return false;
}

void data_logger_sync_data(logger_sync_cb_t send_cb) {
    if (!data_logger_has_data()) return;

    ESP_LOGI(TAG, "Found offline records. Starting synchronization...");

    //  健壮性检查：如果上次同步异常中断，清理掉旧的临时文件
    struct stat st;
    if (stat(TEMP_FILE, &st) == 0) {
        ESP_LOGW(TAG, "Cleaning up leftover temp file...");
        unlink(TEMP_FILE);
    }

    //  重命名日志文件，确保同步期间 SensorTask 可以向新文件写入，不造成死锁
    if (rename(LOG_FILE, TEMP_FILE) != 0) {
        ESP_LOGE(TAG, "Critical: Failed to rename log file for sync");
        return;
    }

    FILE* f = fopen(TEMP_FILE, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Critical: Failed to open temp file");
        return;
    }

    char line[64];
    int count = 0;

    //  逐行处理数据
    while (fgets(line, sizeof(line), f) != NULL) {
        // 在循环内部喂狗。如果离线数据高达上千条，这里的同步会耗时数秒，不喂狗系统会崩溃
        esp_task_wdt_reset();

        float t, h;
        if (sscanf(line, "%f,%f", &t, &h) == 2) {
            if (send_cb) {
                send_cb(t, h); // 执行真正的发送动作，如调用 MQTT Publish
                count++;
                
                // 流量整形：每发一条稍微停顿，防止撑爆 MQTT 发送缓冲区或触发云端流控
                vTaskDelay(pdMS_TO_TICKS(50)); 
            }
        }
    }

    fclose(f);

    // 同步结束后删除临时备份，彻底释放 Flash 空间
    if (unlink(TEMP_FILE) == 0) {
        ESP_LOGI(TAG, "Sync complete. %d records uploaded and cleared.", count);
    } else {
        ESP_LOGE(TAG, "Failed to delete temp file after sync");
    }
}
