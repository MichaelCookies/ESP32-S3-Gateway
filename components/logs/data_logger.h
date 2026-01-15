/*
 * data_logger.h
 * 职责：负责离线数据的持久化存储与读取。
 * 底层：基于 SPIFFS 文件系统。
 * 特性：
 * - 自动挂载/格式化分区。
 * - 支持数据追加写入。
 * - 采用“写时重命名”策略，防止同步与采样冲突。
 * - 支持回调式数据同步（解耦通讯层）。
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <stdbool.h>

/**
 * @brief 定义同步回调函数的类型
 * @param t 温度数值
 * @param h 湿度数值
 */
typedef void (*logger_sync_cb_t)(float t, float h);

/**
 * @brief 初始化日志模块 (挂载 SPIFFS 文件系统)
 * @return true 成功, false 失败
 */
bool data_logger_init(void);

/**
 * @brief 写入一条温湿度数据
 * @param temp 温度
 * @param humi 湿度
 */
void data_logger_write(float temp, float humi);

/**
 * @brief 检查是否存在离线日志文件
 * @return true 有数据, false 空
 */
bool data_logger_has_data(void);

/**
 * @brief 执行数据同步（从 Flash 上传至云端）
 * 
 * 逻辑：
 * - 处理残留的临时文件。
 * - 将 sensor_log.csv 重命名为 sensor_log.tmp（确保同步期间传感器写入不冲突）。
 * - 逐行解析数据并通过回调发送。
 * - 循环内部自动执行 esp_task_wdt_reset() 防止看门狗超时。
 * - 同步完成后删除临时文件。
 * @param send_cb 指向发送函数的指针（例如：on_offline_sync）
 */
void data_logger_sync_data(logger_sync_cb_t send_cb);

#endif // DATA_LOGGER_H