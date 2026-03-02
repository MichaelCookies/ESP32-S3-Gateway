#ifndef SA_DEVICE_MANAGER_H
#define SA_DEVICE_MANAGER_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 核心管理器接口

esp_err_t sa_mgr_init(void);
esp_err_t sa_mgr_register_node(sa_node_t *node);
esp_err_t sa_mgr_unregister_node(const char *id);
sa_node_t* sa_mgr_find_node_by_id(const char *id);

sa_node_t* sa_mgr_get_list_head(void);
void sa_mgr_list_lock(void);
void sa_mgr_list_unlock(void);


// 节点缓存管理

esp_err_t sa_mgr_update_node_cache(sa_node_t *node, sa_value_t val, bool is_online);
esp_err_t sa_mgr_read_node_cache(sa_node_t *node, sa_value_t *out_val, bool *out_online);

// 设备功能包装
/**
 * @brief 在显示设备上打印文本，自动屏蔽底层的 ioctl 和驱动结构体
 * 
 * @param node_id  设备ID (如 "oled_display")
 * @param page     行/页 (对于 128x64 OLED 通常为 0-7)
 * @param col      列起点 (0-127)
 * @param text     要显示的字符串
 * @param inverse  是否反色显示
 * @return esp_err_t 
 */
esp_err_t sa_mgr_display_text(const char *node_id, uint8_t page, uint8_t col, const char *text, bool inverse);

/**
 * @brief 清空显示设备屏幕
 */
esp_err_t sa_mgr_display_clear(const char *node_id);

#ifdef __cplusplus
}
#endif

#endif // SA_DEVICE_MANAGER_H