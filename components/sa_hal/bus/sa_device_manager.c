#include <string.h>
#include "esp_log.h"
#include "sa_device_manager.h"
#include "sa_node_oled.h"

static const char *TAG = "SA_MGR";

static sa_node_t *s_node_list_head = NULL;
static SemaphoreHandle_t s_list_mutex = NULL;

esp_err_t sa_mgr_init(void) {
    if (s_list_mutex == NULL) {
        s_list_mutex = xSemaphoreCreateMutex();
        if (s_list_mutex == NULL) return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t sa_mgr_register_node(sa_node_t *node) {
    if (node == NULL || node->id == NULL || s_list_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_list_mutex, portMAX_DELAY);

    sa_node_t *curr = s_node_list_head;
    while (curr != NULL) {
        if (strcmp(curr->id, node->id) == 0) {
            xSemaphoreGive(s_list_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        curr = curr->next;
    }

    if (node->cache_mutex == NULL) {
        node->cache_mutex = xSemaphoreCreateMutex();
    }

    memset(&node->cached_val, 0, sizeof(sa_value_t));
    esp_err_t init_err = ESP_OK;

    if (node->ops.init != NULL) {
        init_err = node->ops.init(node);
        if (init_err != ESP_OK) {
            ESP_LOGW(TAG, "Node '%s' init failed (%s). Marked as offline.", node->id, esp_err_to_name(init_err));
        }
    }

    node->is_online = (init_err == ESP_OK);
    node->consecutive_errors = 0;
    
    node->next = s_node_list_head;
    s_node_list_head = node;

    xSemaphoreGive(s_list_mutex);
    
    ESP_LOGI(TAG, "Registered node: %s [%s]", node->id, node->is_online ? "ONLINE" : "OFFLINE");
    return ESP_OK;
}

esp_err_t sa_mgr_unregister_node(const char *id) {
    if (id == NULL || s_list_mutex == NULL) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(s_list_mutex, portMAX_DELAY);
    sa_node_t *curr = s_node_list_head;
    sa_node_t *prev = NULL;

    while (curr != NULL) {
        if (strcmp(curr->id, id) == 0) {
            if (prev == NULL) s_node_list_head = curr->next;
            else prev->next = curr->next;

            if (curr->ops.deinit != NULL) curr->ops.deinit(curr);
            if (curr->cache_mutex != NULL) vSemaphoreDelete(curr->cache_mutex);

            xSemaphoreGive(s_list_mutex);
            return ESP_OK;
        }
        prev = curr;
        curr = curr->next;
    }
    xSemaphoreGive(s_list_mutex);
    return ESP_ERR_NOT_FOUND;
}

sa_node_t* sa_mgr_find_node_by_id(const char *id) {
    if (id == NULL || s_list_mutex == NULL) return NULL;
    
    sa_node_t *found = NULL;
    xSemaphoreTake(s_list_mutex, portMAX_DELAY);
    sa_node_t *curr = s_node_list_head;
    while (curr != NULL) {
        if (strcmp(curr->id, id) == 0) {
            found = curr;
            break;
        }
        curr = curr->next;
    }
    xSemaphoreGive(s_list_mutex);
    return found;
}

sa_node_t* sa_mgr_get_list_head(void) { return s_node_list_head; }
void sa_mgr_list_lock(void) { if (s_list_mutex) xSemaphoreTake(s_list_mutex, portMAX_DELAY); }
void sa_mgr_list_unlock(void) { if (s_list_mutex) xSemaphoreGive(s_list_mutex); }

esp_err_t sa_mgr_update_node_cache(sa_node_t *node, sa_value_t val, bool is_online) {
    if (node == NULL || node->cache_mutex == NULL) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(node->cache_mutex, portMAX_DELAY);
    node->cached_val = val;
    node->is_online = is_online;
    
    if (!is_online) {
        node->consecutive_errors++;
    } else {
        node->consecutive_errors = 0;
    }
    
    xSemaphoreGive(node->cache_mutex);
    return ESP_OK;
}

esp_err_t sa_mgr_read_node_cache(sa_node_t *node, sa_value_t *out_val, bool *out_online) {
    if (node == NULL || out_val == NULL || node->cache_mutex == NULL) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(node->cache_mutex, portMAX_DELAY);
    *out_val = node->cached_val;
    if (out_online != NULL) *out_online = node->is_online;
    xSemaphoreGive(node->cache_mutex);
    return ESP_OK;
}

esp_err_t sa_mgr_display_text(const char *node_id, uint8_t page, uint8_t col, const char *text, bool inverse) {
    sa_node_t *node = sa_mgr_find_node_by_id(node_id);
    if (!node || !node->is_online || !node->ops.ioctl) {
        return ESP_ERR_NOT_FOUND;
    }

    sa_oled_text_args_t args = {
        .page = page,
        .col = col,
        .text = text,
        .inverse = inverse
    };

    esp_err_t err = node->ops.ioctl(node, SA_OLED_CMD_DRAW_TEXT, &args);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Actuator '%s' failed to write (%s), kicking offline.", node_id, esp_err_to_name(err));
        sa_value_t dummy = {0};
        sa_mgr_update_node_cache(node, dummy, false);
    }
    
    return err;
}

esp_err_t sa_mgr_display_clear(const char *node_id) {
    sa_node_t *node = sa_mgr_find_node_by_id(node_id);
    if (!node || !node->is_online || !node->ops.ioctl) {
        return ESP_ERR_NOT_FOUND;
    }
    
    esp_err_t err = node->ops.ioctl(node, SA_OLED_CMD_CLEAR, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Actuator '%s' failed to clear screen, kicking offline.", node_id);
        sa_value_t dummy = {0};
        sa_mgr_update_node_cache(node, dummy, false);
    }
    return err;
}