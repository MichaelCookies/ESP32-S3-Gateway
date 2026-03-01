#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*logger_sync_cb_t)(const char *json_payload);

bool data_logger_init(void);

void data_logger_write(const char *json_payload);

bool data_logger_has_data(void);

void data_logger_sync_data(logger_sync_cb_t send_cb);

#ifdef __cplusplus
}
#endif

#endif // DATA_LOGGER_H