#ifndef HW_RECOVERY_H
#define HW_RECOVERY_H

#include <stdint.h>
#include <stdbool.h>
#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t probe_attempts;
    uint32_t reset_count;
    uint32_t recovery_success;
} recovery_stats_t;

bool hw_recovery_init(sa_bus_t *main_bus);
void hw_recovery_start_daemon(void);
void hw_recovery_trigger_node_reset(const char *node_id);
void hw_recovery_get_stats(recovery_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif // HW_RECOVERY_H