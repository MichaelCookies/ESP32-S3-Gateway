// sa_node_aht20.h
#ifndef SA_NODE_AHT20_H
#define SA_NODE_AHT20_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t sa_aht20_create_nodes(sa_bus_t *bus, sa_node_t **out_temp_node, sa_node_t **out_humi_node);

#ifdef __cplusplus
}
#endif

#endif // SA_NODE_AHT20_H