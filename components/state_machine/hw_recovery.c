#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hw_recovery.h"
#include "sa_device_manager.h"

static const char *TAG = "HW_RECOVERY";

static void recovery_daemon_task(void *pvParameters) {
    ESP_LOGI(TAG, "SA-HAL Recovery Daemon Started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        sa_mgr_list_lock();
        sa_node_t *curr = sa_mgr_get_list_head();
        
        while (curr != NULL) {
            sa_value_t val;
            bool is_online = true;
            sa_mgr_read_node_cache(curr, &val, &is_online);

            if (!is_online && curr->ops.init != NULL) {
                ESP_LOGW(TAG, "Node [%s] is offline. Attempting recovery...", curr->id);
                esp_err_t err = curr->ops.init(curr);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Node [%s] recovered successfully.", curr->id);
                } else {
                    ESP_LOGE(TAG, "Node [%s] recovery failed: %d", curr->id, err);
                }
            }
            curr = curr->next;
        }
        
        sa_mgr_list_unlock();
    }
}

void hw_recovery_start_daemon(void) {
    xTaskCreate(recovery_daemon_task, "RecovDaemon", 4096, NULL, 1, NULL);
}