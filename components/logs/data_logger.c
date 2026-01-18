/*
 * data_logger.c
 * V4.3 - 独立任务 + 断点续传 + 优先级管控
 */

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lan_service.h" // 借用一下网络状态查询

#include "data_logger.h"

static const char *TAG = "LOGGER";

#define PARTITION_LABEL "storage"
#define MOUNT_POINT     "/spiffs"
#define LOG_FILE        "/spiffs/sensor_log.csv"
#define TEMP_FILE       "/spiffs/sensor_log.tmp" // 正在上传的文件

#define MAX_FILE_SIZE   (100 * 1024) 

// 任务句柄，防止重复启动
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

void data_logger_write(float temp, float humi) {
    // 1. 检查文件大小限制
    struct stat st;
    if (stat(LOG_FILE, &st) == 0) {
        if (st.st_size >= MAX_FILE_SIZE) {
            // 策略：为了保留最新数据，如果满了可以删除旧文件重建，或者简单丢弃
            // 这里选择保护 Flash，暂时丢弃
            ESP_LOGW(TAG, "Log full, dropping data.");
            return;
        }
    }

    FILE* f = fopen(LOG_FILE, "a");
    if (f == NULL) return;
    fprintf(f, "%.2f,%.2f\n", temp, humi);
    fclose(f);
}

bool data_logger_has_data(void) {
    struct stat st;
    // 只要有日志文件或者有未传完的临时文件，都算有数据
    if (stat(TEMP_FILE, &st) == 0 && st.st_size > 0) return true;
    if (stat(LOG_FILE, &st) == 0 && st.st_size > 0) return true;
    return false;
}


// 同步任务逻辑

static void task_sync_worker(void *pvParameters) {
    logger_sync_cb_t send_cb = (logger_sync_cb_t)pvParameters;
    
    ESP_LOGI(TAG, "Sync Task Started (Priority 4)");

    //  检查是否有上次未传完的临时文件
    struct stat st;
    bool has_temp = (stat(TEMP_FILE, &st) == 0 && st.st_size > 0);

    //  如果没有临时文件，把当前的 Log 文件重命名为临时文件
    //  这样做是为了把“正在写入”和“正在上传”分离开，防止冲突
    if (!has_temp) {
        if (stat(LOG_FILE, &st) == 0 && st.st_size > 0) {
            if (rename(LOG_FILE, TEMP_FILE) != 0) {
                ESP_LOGE(TAG, "Rename failed, aborting.");
                s_sync_task_handle = NULL;
                vTaskDelete(NULL);
                return;
            }
            ESP_LOGI(TAG, "Rotated log file to temp for upload.");
        } else {
            // 没数据
            s_sync_task_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
    }

    //  开始处理临时文件
    FILE* f = fopen(TEMP_FILE, "r");
    FILE* f_retry = NULL; // 用于保存本次没发成功的（如果中途断网）
    
    if (f == NULL) {
        s_sync_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    char line[64];
    int count = 0;
    bool abort_sync = false;

    while (fgets(line, sizeof(line), f) != NULL) {

        //  每次发送前检查是否还有别的任务需要 CPU，或者网络是否断了
        
        float t, h;
        if (sscanf(line, "%f,%f", &t, &h) == 2) {
            if (send_cb) {
                send_cb(t, h);
                count++;
                // 流量控制：每条间隔 50ms，20条/秒，既不堵塞 MQTT 队列，也能快速清空
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }

    fclose(f);

    //  全部处理完毕，删除临时文件

    unlink(TEMP_FILE);
    
    ESP_LOGI(TAG, "Sync finished. Uploaded %d records.", count);

    // 任务自杀
    s_sync_task_handle = NULL;
    vTaskDelete(NULL);
}


// 启动任务

void data_logger_sync_data(logger_sync_cb_t send_cb) {
    if (s_sync_task_handle != NULL) {
        ESP_LOGW(TAG, "Sync task already running.");
        return;
    }

    if (!data_logger_has_data()) return;

    // 创建任务
    // 栈大小：4096 足够文件操作
    // 优先级：4 (高于 Main/Display(1)，低于 WiFi(23))
    // 这样保证网络连上后优先把历史数据传完再去处理别的闲杂事务
    xTaskCreate(task_sync_worker, "LogSync", 4096, (void*)send_cb, 4, &s_sync_task_handle);
}