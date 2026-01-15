/*
 * lan_service.c
 * V4.2
 * 职责：局域网双向通信、系统日志重定向、设备发现
 * 优化：
 * - 增加非阻塞日志保护，防止弱网环境下 sendto 导致的系统卡顿。
 * - 补全全部分支解析逻辑。
 * - 增加 WiFi 状态联动，仅在有 IP 时执行网络操作。
 */

#include <string.h>
#include <stdarg.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include "sdkconfig.h"

#include "lan_service.h"
#include "app_types.h"
#include "settings.h"

static const char *TAG = "LAN_SVC";

//  模块私有变量
static int g_sock = -1;
static QueueHandle_t g_internal_bus_handle = NULL;
static bool g_is_logging_active = false; // 递归锁标记
static bool g_wifi_has_ip = false;       // 内部记录 WiFi 状态

#ifdef CONFIG_UDP_TARGET_IP
    #define BROADCAST_IP CONFIG_UDP_TARGET_IP
#else
    #define BROADCAST_IP "255.255.255.255"
#endif

//  内部辅助函数

/**
 * @brief 基础数据包发送
 * @note 内部处理对齐和头部封装
 */
static void send_raw_packet(struct sockaddr_in *dest_addr, uint8_t type, uint16_t seq, const char *payload, uint16_t len) {
    if (g_sock < 0 || !g_wifi_has_ip) return;

    uint16_t total_len = sizeof(lan_packet_head_t) + len;
    uint8_t *tx_buffer = malloc(total_len);
    if (!tx_buffer) return;

    lan_packet_head_t *head = (lan_packet_head_t *)tx_buffer;
    head->magic = LAN_MAGIC_WORD;
    head->version = LAN_PROTOCOL_VER;
    head->type = type;
    head->sequence = seq;
    head->data_len = len;

    if (len > 0 && payload != NULL) {
        memcpy(tx_buffer + sizeof(lan_packet_head_t), payload, len);
    }

    //  使用 MSG_DONTWAIT 防止在底层缓冲区满时阻塞调用任务
    sendto(g_sock, tx_buffer, total_len, MSG_DONTWAIT, (struct sockaddr *)dest_addr, sizeof(struct sockaddr_in));
    
    free(tx_buffer);
}

/**
 * @brief 系统日志重定向回调
 */
static int udp_log_vprintf(const char *fmt, va_list l) {
    //  无论如何，先输出到物理串口
    int ret = vprintf(fmt, l);

    //  检查条件：Socket 已开、WiFi 已连、未在日志处理递归中
    if (g_sock < 0 || !g_wifi_has_ip || g_is_logging_active) {
        return ret;
    }

    //  递归锁保护，防止打印日志的行为本身触发新的日志发送导致死循环
    g_is_logging_active = true;

    char buf[512];
    int len = vsnprintf(buf, sizeof(buf), fmt, l);

    if (len > 0) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(LAN_SERVICE_PORT);
        dest_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);

        send_raw_packet(&dest_addr, UDP_TYPE_LOG, 0, buf, (uint16_t)len);
    }

    g_is_logging_active = false;
    return ret;
}

/**
 * @brief 发送系统详细状态（响应 get_status 指令）
 */
static void send_system_status_json(struct sockaddr_in *dest_addr, uint16_t seq) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t free_heap = esp_get_free_heap_size();
    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "sys_status");
    cJSON_AddNumberToObject(root, "rev", chip_info.revision);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    cJSON_AddStringToObject(root, "build", __DATE__ " " __TIME__);
    cJSON_AddNumberToObject(root, "uptime", uptime_ms);
    cJSON_AddNumberToObject(root, "heap", free_heap);
    
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(root, "mac", mac_str);

    char *json_out = cJSON_PrintUnformatted(root);
    if (json_out) {
        send_raw_packet(dest_addr, UDP_TYPE_REPORT, seq, json_out, strlen(json_out));
        free(json_out);
    }
    cJSON_Delete(root);
}

/**
 * @brief 处理设备发现请求
 */
static void handle_discovery(struct sockaddr_in *source_addr, uint16_t seq) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "{\"id\":\"ESP32_GW\",\"res\":\"alive\"}");
    send_raw_packet(source_addr, UDP_TYPE_REPORT, seq, buffer, strlen(buffer));
}

/**
 * @brief 处理局域网下发的控制指令
 */
static void handle_control_command(struct sockaddr_in *source_addr, uint16_t seq, char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;

    cJSON *item_cmd = cJSON_GetObjectItem(root, "cmd");
    cJSON *item_val = cJSON_GetObjectItem(root, "val");

    if (cJSON_IsString(item_cmd)) {
        ESP_LOGI(TAG, "LAN Command: %s", item_cmd->valuestring);

        //  特殊指令
        //获取系统状态，不进总线直接回包
        if (strcmp(item_cmd->valuestring, "get_status") == 0) {
            send_system_status_json(source_addr, seq);
        } 
        //  业务指令
        //  转发至系统总线
        else {
            app_msg_t msg;
            msg.type = MSG_TYPE_LOCAL_CMD; 
            bool valid = true;

            if (strcmp(item_cmd->valuestring, "alarm") == 0) {
                msg.payload.cloud.cmd = CMD_ALARM_CTRL;
                msg.payload.cloud.val_bool = cJSON_IsTrue(item_val);
            } else if (strcmp(item_cmd->valuestring, "threshold") == 0) {
                msg.payload.cloud.cmd = CMD_SET_THRESHOLD;
                msg.payload.cloud.val_float = (float)item_val->valuedouble;
            } else if (strcmp(item_cmd->valuestring, "sensor_sw") == 0) {
                msg.payload.cloud.cmd = CMD_SENSOR_SWITCH;
                msg.payload.cloud.val_bool = cJSON_IsTrue(item_val);
            } else if (strcmp(item_cmd->valuestring, "disp_sw") == 0) {
                msg.payload.cloud.cmd = CMD_DISPLAY_SWITCH;
                msg.payload.cloud.val_bool = cJSON_IsTrue(item_val);
            } else if (strcmp(item_cmd->valuestring, "net_reset") == 0) {
                msg.payload.cloud.cmd = CMD_NET_RESET;
            } else if (strcmp(item_cmd->valuestring, "cloud_sw") == 0) {
                msg.payload.cloud.cmd = CMD_CLOUD_SWITCH;
                msg.payload.cloud.val_bool = cJSON_IsTrue(item_val);
            } else {
                valid = false;
            }

            if (valid && g_internal_bus_handle) {
                xQueueSend(g_internal_bus_handle, &msg, 0);
                send_raw_packet(source_addr, UDP_TYPE_REPORT, seq, "{\"res\":\"ok\"}", 11);
            }
        }
    }
    cJSON_Delete(root);
}

//  任务逻辑

static void lan_server_task(void *pvParameters) {
    char rx_buffer[1024];
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(LAN_SERVICE_PORT);

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        vTaskDelete(NULL);
        return;
    }
    
    int broadcast = 1;
    setsockopt(g_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    if (bind(g_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
        close(g_sock);
        g_sock = -1;
        vTaskDelete(NULL);
        return;
    }

    //  只有 Socket 成功打开后才允许开启日志重定向
    esp_log_set_vprintf(udp_log_vprintf);
    ESP_LOGI(TAG, "Local Service listening on port %d", LAN_SERVICE_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(g_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len >= sizeof(lan_packet_head_t)) {
            lan_packet_head_t *head = (lan_packet_head_t *)rx_buffer;
            
            //  验证魔数和协议版本
            if (head->magic == LAN_MAGIC_WORD && head->version == LAN_PROTOCOL_VER) {
                switch (head->type) {
                    case UDP_TYPE_DISCOVERY:
                        handle_discovery(&source_addr, head->sequence);
                        break;
                    case UDP_TYPE_CONTROL:
                        //  确保字符串结尾
                        if (sizeof(lan_packet_head_t) + head->data_len < sizeof(rx_buffer)) {
                            rx_buffer[sizeof(lan_packet_head_t) + head->data_len] = '\0';
                            handle_control_command(&source_addr, head->sequence, (char *)(rx_buffer + sizeof(lan_packet_head_t)));
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//  公开接口

/**
 * @brief 设置 WiFi 在线状态
 * @note 此函数应由 main 在监听到 IP 事件时调用
 */
void lan_service_set_net_status(bool has_ip) {
    g_wifi_has_ip = has_ip;
}

void lan_service_broadcast_sensor_data(float temp, float humi) {
    if (!g_wifi_has_ip) return;

    char json[64];
    snprintf(json, sizeof(json), "{\"t\":%.2f,\"h\":%.2f}", temp, humi);
    
    struct sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(LAN_SERVICE_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);
    
    send_raw_packet(&dest_addr, UDP_TYPE_SENSOR_BCAST, 0, json, strlen(json));
}

esp_err_t lan_service_init(QueueHandle_t sys_bus_handle) {
    if (sys_bus_handle == NULL) return ESP_ERR_INVALID_ARG;
    g_internal_bus_handle = sys_bus_handle;
    
    //  默认开启日志拦截，但内部会检查 WiFi 状态
    xTaskCreate(lan_server_task, "lan_svc_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}
