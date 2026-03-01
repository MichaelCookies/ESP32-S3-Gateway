#ifndef SA_DEVICE_MANAGER_H
#define SA_DEVICE_MANAGER_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sa_mgr_init(void);

esp_err_t sa_mgr_register_node(sa_node_t *node);

esp_err_t sa_mgr_unregister_node(const char *id);

sa_node_t* sa_mgr_find_node_by_id(const char *id);

sa_node_t* sa_mgr_get_list_head(void);

void sa_mgr_list_lock(void);

void sa_mgr_list_unlock(void);

esp_err_t sa_mgr_update_node_cache(sa_node_t *node, sa_value_t val, bool is_online);

esp_err_t sa_mgr_read_node_cache(sa_node_t *node, sa_value_t *out_val, bool *out_online);

#ifdef __cplusplus
}
#endif

#endif // SA_DEVICE_MANAGER_H