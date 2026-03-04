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
    
    ESP_LOGI(TAG, "Attempting to recover offline node '%s' (Attempt %d/5)...", 
             node->id, node->failed_recovery_count + 1);
    
    esp_err_t err = node->ops.init(node);
    
    if (err == ESP_OK) {
        sa_value_t dummy = {0};
        sa_mgr_update_node_cache(node, dummy, true);
        
        // 恢复成功，清零熔断计数器
        node->failed_recovery_count = 0;
        
        ESP_LOGI(TAG, "Node '%s' recovered successfully!", node->id);
        g_stats.recovery_success++;
        return true;
    } else {
        node->failed_recovery_count++;
        ESP_LOGD(TAG, "Node '%s' recovery failed.", node->id);
        return false;
    }
}

static void recovery_daemon_task(void *pvParameters) {
    ESP_LOGI(TAG, "Recovery daemon started. Monitoring SA-HAL node states.");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_HW_RECOVERY_CHECK_PERIOD_MS));
        
        sa_mgr_list_lock();
        sa_node_t *curr = sa_mgr_get_list_head();
        
        while (curr) {
            // 只处理离线且具备初始化能力的节点
            if (!curr->is_online && curr->ops.init != NULL) {
                
                // 尝试超过 5 次放弃
                if (curr->failed_recovery_count >= 5) {
                    if (curr->failed_recovery_count == 5) {
                        ESP_LOGE(TAG, "FATAL: Node '%s' hardware permanently offline. Please force reboot.", curr->id);
                        // 为了防止疯狂刷屏，把计数器锁死在 6
                        curr->failed_recovery_count = 6; 
                    
                        // 如果 OLED 活着，可以在这里调用 sa_mgr_display_text 提示用户重启，暂时不实现
                    }
                } else {
                    // 没有熔断，继续尝试救活它
                    try_recover_node(curr);
                }
            }
            curr = curr->next;
        }
        sa_mgr_list_unlock();
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