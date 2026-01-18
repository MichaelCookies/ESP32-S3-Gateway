#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdbool.h>

//  回调函数，传入温度、湿度，返回发送是否成功
//  为了实现断点续传，回调函数最好能返回发送结果，实际逻辑中我们会检查网络状态
typedef void (*logger_sync_cb_t)(float, float);

bool data_logger_init(void);
void data_logger_write(float temp, float humi);
bool data_logger_has_data(void);

// 启动同步，非阻塞式后台任务
void data_logger_sync_data(logger_sync_cb_t send_cb);

#endif