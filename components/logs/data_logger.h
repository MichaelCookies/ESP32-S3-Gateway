#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdbool.h>

typedef void (*logger_sync_cb_t)(const char *csv_record);

bool data_logger_init(void);
void data_logger_save_snapshot(void);
bool data_logger_has_data(void);
void data_logger_sync_data(logger_sync_cb_t send_cb);

#endif