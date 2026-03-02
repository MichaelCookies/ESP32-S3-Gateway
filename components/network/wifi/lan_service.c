#include <string.h>
#include <stdarg.h>
#include <math.h>
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
#include "cJSON.h"

#include "lan_service.h"
#include "app_types.h"
#include "sa_device_manager.h"

static const char *TAG = "LAN_SVC";

static int g_sock = -1;
static QueueHandle_t g_internal_bus_handle = NULL;
static bool g_is_logging_active = false; 
static bool g_wifi_has_ip = false;       

#ifdef CONFIG_UDP_TARGET_IP
    #define BROADCAST_IP CONFIG_UDP_TARGET_IP
#else
    #define BROADCAST_IP "255.255.255.255"
#endif

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

    sendto(g_sock, tx_buffer, total_len, MSG_DONTWAIT, (struct sockaddr *)dest_addr, sizeof(struct sockaddr_in));
    free(tx_buffer);
}

static void attach_base_info(cJSON *root) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(root, "mac", mac_str);
    cJSON_AddStringToObject(root, "name", "Virtual Lab Gateway");
}

static int udp_log_vprintf(const char *fmt, va_list l) {
    // 必须优先 copy，因为 vprintf 消费后 va_list 会失效
    va_list l_copy;
    va_copy(l_copy, l);
    
    // 正常终端打印
    int ret = vprintf(fmt, l);

    // 在ISR中绝对不能执行发包和 malloc。如果没有网络，或者正处于重入状态，直接退出。
    if (xPortInIsrContext() || g_sock < 0 || !g_wifi_has_ip || g_is_logging_active) {
        va_end(l_copy);
        return ret;
    }

    g_is_logging_active = true;
    
    // 把 512 字节从栈移到堆。这样彻底解放所有调用 ESP_LOG 的任务
    char *buf = (char *)malloc(512);
    if (buf) {
        int len = vsnprintf(buf, 512, fmt, l_copy);
        
        if (len > 0) {
            struct sockaddr_in dest_addr;
            memset(&dest_addr, 0, sizeof(dest_addr)); 
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(LAN_SERVICE_PORT);
            dest_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);
            send_raw_packet(&dest_addr, UDP_TYPE_LOG, 0, buf, (uint16_t)len);
        }
        free(buf); // 用完释放
    }
    
    va_end(l_copy);
    g_is_logging_active = false;
    return ret;
}

static void send_system_status_json(struct sockaddr_in *dest_addr, uint16_t seq) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t free_heap = esp_get_free_heap_size();

    cJSON *root = cJSON_CreateObject();
    attach_base_info(root);
    
    cJSON_AddStringToObject(root, "type", "sys_status");
    cJSON_AddNumberToObject(root, "rev", chip_info.revision);
    cJSON_AddNumberToObject(root, "cores", chip_info.cores);
    cJSON_AddStringToObject(root, "build", __DATE__ " " __TIME__);
    cJSON_AddNumberToObject(root, "uptime", uptime_ms);
    cJSON_AddNumberToObject(root, "heap", free_heap);

    cJSON *act_obj = cJSON_CreateObject();
    sa_mgr_list_lock();
    sa_node_t *curr = sa_mgr_get_list_head();
    while (curr) {
        if (curr->type == SA_NODE_ACTUATOR) {
            sa_value_t val; bool online;
            if (sa_mgr_read_node_cache(curr, &val, &online) == ESP_OK) {
                cJSON_AddBoolToObject(act_obj, curr->id, val.b_val);
            }
        }
        curr = curr->next;
    }
    sa_mgr_list_unlock();
    cJSON_AddItemToObject(root, "actuators", act_obj);

    char *json_out = cJSON_PrintUnformatted(root);
    if (json_out) {
        send_raw_packet(dest_addr, UDP_TYPE_REPORT, seq, json_out, strlen(json_out));
        free(json_out);
    }
    cJSON_Delete(root);
}

static void handle_discovery(struct sockaddr_in *source_addr, uint16_t seq) {
    cJSON *root = cJSON_CreateObject();
    attach_base_info(root);
    cJSON_AddStringToObject(root, "res", "alive");

    char *json_out = cJSON_PrintUnformatted(root);
    if (json_out) {
        send_raw_packet(source_addr, UDP_TYPE_REPORT, seq, json_out, strlen(json_out));
        free(json_out);
    }
    cJSON_Delete(root);
}

static void handle_control_command(struct sockaddr_in *source_addr, uint16_t seq, char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return;

    cJSON *item_cmd = cJSON_GetObjectItem(root, "cmd");
    cJSON *item_val = cJSON_GetObjectItem(root, "val");

    if (cJSON_IsString(item_cmd)) {
        if (strcmp(item_cmd->valuestring, "get_status") == 0) {
            send_system_status_json(source_addr, seq);
        } else {
            app_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.type = MSG_TYPE_LOCAL_CMD; 
            bool valid = true;

            if (strcmp(item_cmd->valuestring, "alarm") == 0) {
                msg.payload.cmd_data.cmd = CMD_ALARM_CTRL;
                msg.payload.cmd_data.val_bool = cJSON_IsTrue(item_val);
            } else if (strcmp(item_cmd->valuestring, "threshold") == 0) {
                msg.payload.cmd_data.cmd = CMD_SET_THRESHOLD;
                msg.payload.cmd_data.val_float = (float)item_val->valuedouble;
            } else if (strcmp(item_cmd->valuestring, "sensor_sw") == 0) {
                msg.payload.cmd_data.cmd = CMD_SENSOR_SWITCH;
                msg.payload.cmd_data.val_bool = cJSON_IsTrue(item_val);
            } else if (strcmp(item_cmd->valuestring, "net_reset") == 0) {
                msg.payload.cmd_data.cmd = CMD_NET_RESET;
            } else {
                msg.payload.cmd_data.cmd = CMD_ACTUATOR_CTRL;
                strncpy(msg.payload.cmd_data.target_id, item_cmd->valuestring, sizeof(msg.payload.cmd_data.target_id) - 1);
                msg.payload.cmd_data.val_bool = cJSON_IsTrue(item_val) || (cJSON_IsNumber(item_val) && item_val->valueint > 0);
            }

            if (valid && g_internal_bus_handle) {
                xQueueSend(g_internal_bus_handle, &msg, 0);
                
                cJSON *ack = cJSON_CreateObject();
                attach_base_info(ack);
                cJSON_AddStringToObject(ack, "res", "ok");
                char *ack_str = cJSON_PrintUnformatted(ack);
                if (ack_str) {
                    send_raw_packet(source_addr, UDP_TYPE_REPORT, seq, ack_str, strlen(ack_str));
                    free(ack_str);
                }
                cJSON_Delete(ack);
            }
        }
    }
    cJSON_Delete(root);
}

static void lan_server_task(void *pvParameters) {
    char rx_buffer[1024];
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr)); 

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

    ESP_LOGI(TAG, "LAN Service listening on port %d", LAN_SERVICE_PORT);
    esp_log_set_vprintf(udp_log_vprintf);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(g_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len >= sizeof(lan_packet_head_t)) {
            lan_packet_head_t *head = (lan_packet_head_t *)rx_buffer;
            
            if (head->magic == LAN_MAGIC_WORD && head->version == LAN_PROTOCOL_VER) {
                switch (head->type) {
                    case UDP_TYPE_DISCOVERY:
                        handle_discovery(&source_addr, head->sequence);
                        break;
                    case UDP_TYPE_CONTROL:
                        if (sizeof(lan_packet_head_t) + head->data_len <= sizeof(rx_buffer)) {
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

void lan_service_set_net_status(bool has_ip) {
    g_wifi_has_ip = has_ip;
}

void lan_service_broadcast_data(void) {
    if (!g_wifi_has_ip) return;

    cJSON *root = cJSON_CreateObject();
    attach_base_info(root);

    cJSON *data_obj = cJSON_CreateObject();
    
    sa_mgr_list_lock();
    sa_node_t *curr = sa_mgr_get_list_head();
    bool has_data = false;

    while (curr) {
        sa_value_t val; bool is_online;
        if (sa_mgr_read_node_cache(curr, &val, &is_online) == ESP_OK && is_online) {
            // 【关键修复】只向数据看板推送 SENSOR，不要把继电器推上去当传感器显示！
            if (curr->type == SA_NODE_SENSOR) {
                if (curr->val_type == SA_VAL_TYPE_FLOAT) {
                    // 【关键修复】截断到1位小数，防止 Qt UI 太丑
                    float clean_f = roundf(val.f_val * 10.0f) / 10.0f;
                    cJSON_AddNumberToObject(data_obj, curr->id, clean_f);
                    has_data = true;
                } else if (curr->val_type == SA_VAL_TYPE_BOOL) {
                    cJSON_AddBoolToObject(data_obj, curr->id, val.b_val);
                    has_data = true;
                } else if (curr->val_type == SA_VAL_TYPE_INT) {
                    cJSON_AddNumberToObject(data_obj, curr->id, val.i_val);
                    has_data = true;
                }
            }
        }
        curr = curr->next;
    }
    sa_mgr_list_unlock();

    if (!has_data) {
        cJSON_Delete(root);
        return; 
    }

    cJSON_AddItemToObject(root, "data", data_obj);

    char *json_out = cJSON_PrintUnformatted(root);
    if (json_out) {
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr)); // 【关键修复】清零 sin_zero 填充区，防止 Windows 防火墙丢包！
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(LAN_SERVICE_PORT);
        dest_addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);
        
        ESP_LOGI(TAG, "Broadcasting SA-HAL Payload: %s", json_out);

        send_raw_packet(&dest_addr, UDP_TYPE_SENSOR_BCAST, 0, json_out, strlen(json_out));
        free(json_out);
    }
    cJSON_Delete(root);
}

esp_err_t lan_service_init(QueueHandle_t sys_bus_handle) {
    if (sys_bus_handle == NULL) return ESP_ERR_INVALID_ARG;
    g_internal_bus_handle = sys_bus_handle;
    
    xTaskCreate(lan_server_task, "lan_svc_task", 6144, NULL, 5, NULL);
    return ESP_OK;
}