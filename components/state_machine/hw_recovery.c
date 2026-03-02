#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw_recovery.h"
#include "sa_device_manager.h"
#include "sdkconfig.h"

static const char *TAG = "HW_RECOVERY";

static sa_bus_t *g_main_bus = NULL;
static TaskHandle_t g_recovery_task_handle = NULL;
static recovery_stats_t g_stats = {0};

static bool try_recover_node(sa_node_t *node) {
    if (!node || !node->ops.init) return false;
    
    ESP_LOGI(TAG, "Attempting to recover offline node '%s'...", node->id);
    
    esp_err_t err = node->ops.init(node);
    sa_value_t dummy = {0};
    
    if (err == ESP_OK) {
        sa_mgr_update_node_cache(node, dummy, true);
        ESP_LOGI(TAG, "Node '%s' recovered successfully!", node->id);
        g_stats.recovery_success++;
        return true;
    } else {
        ESP_LOGD(TAG, "Node '%s' recovery failed (still disconnected).", node->id);
        // 复用 SA 架构接口累加连续错误，避免死锁无法触发硬件复位
        sa_mgr_update_node_cache(node, dummy, false);
        return false;
    }
}

static void recovery_daemon_task(void *pvParameters) {
    ESP_LOGI(TAG, "Recovery daemon started. Monitoring SA-HAL node states.");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_HW_RECOVERY_CHECK_PERIOD_MS));
        
        sa_node_t *offline_nodes[8] = {NULL};
        int offline_count = 0;
        int total_consecutive_errors = 0;
        
        sa_mgr_list_lock();
        sa_node_t *curr = sa_mgr_get_list_head();
        while (curr) {
            if (!curr->is_online && curr->ops.init != NULL) {
                if (offline_count < 8) {
                    offline_nodes[offline_count++] = curr;
                }
            }
            total_consecutive_errors += curr->consecutive_errors;
            curr = curr->next;
        }
        sa_mgr_list_unlock();

        if (total_consecutive_errors > 20 && g_main_bus && g_main_bus->reset) {
            ESP_LOGE(TAG, "Too many node errors (%d). Triggering physical I2C Bus Reset!", total_consecutive_errors);
            g_stats.reset_count++;
            g_main_bus->reset(g_main_bus);
            
            sa_mgr_list_lock();
            curr = sa_mgr_get_list_head();
            while(curr) { 
                if (!curr->is_online) {
                    curr->consecutive_errors = 0; 
                }
                curr = curr->next; 
            }
            sa_mgr_list_unlock();
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        for (int i = 0; i < offline_count; i++) {
            try_recover_node(offline_nodes[i]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

bool hw_recovery_init(sa_bus_t *main_bus) {
    g_main_bus = main_bus;
    ESP_LOGI(TAG, "Hardware recovery module initialized");
    return true;
}

void hw_recovery_start_daemon(void) {
    if (g_recovery_task_handle != NULL) return;
    
    xTaskCreate(
        recovery_daemon_task,
        "recov_daemon",
        4096,
        NULL,
        1,
        &g_recovery_task_handle
    );
}

void hw_recovery_trigger_node_reset(const char *node_id) {
    if (!node_id) return;
    sa_node_t *node = sa_mgr_find_node_by_id(node_id);
    if (node) {
        ESP_LOGI(TAG, "Manual recovery triggered for '%s'", node_id);
        sa_value_t dummy = {0};
        sa_mgr_update_node_cache(node, dummy, false);
    }
}

void hw_recovery_get_stats(recovery_stats_t *out_stats) {
    if (out_stats) {
        memcpy(out_stats, &g_stats, sizeof(recovery_stats_t));
    }
}