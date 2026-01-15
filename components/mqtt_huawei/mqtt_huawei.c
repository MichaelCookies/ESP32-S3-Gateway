/*
 * mqtt_huawei.c
 * 职责：华为云 IoTDA 协议处理
 * V4.2
 * - 增加 start/stop 接口，彻底解决断网后 DNS 解析死锁问题。
 * - 集成 settings 模块，支持云端功能的开启/关闭。
 * - 完整的指令解析（含网络复位、云端开关）。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "mqtt_huawei.h"
#include "app_types.h"
#include "settings.h"   //  引入配置，检查云端开关

static const char *TAG = "HUAWEI_MQTT";

//  全局客户端句柄，初始化为 NULL
static esp_mqtt_client_handle_t client = NULL;
//  系统总线句柄
static QueueHandle_t g_system_bus = NULL;

//  宏定义
#define SERVICE_ID         "Sensor_Service"
#define PROPERTY_TEMP      "Temperature"
#define PROPERTY_HUMI      "Humidity"

//  状态同步属性名
#define PROP_THRESH        "Current_Threshold"
#define PROP_MODE          "System_Mode"
#define PROP_DISP          "Display_Status"
#define PROP_SENSOR        "Sensor_Status"
#define PROP_CLOUD_SW      "Cloud_Switch"

//  Topic 定义
#define TOPIC_CMD_SUB       "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/commands/#"
#define TOPIC_CMD_RESP_FMT  "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/commands/response/request_id=%.*s"
#define TOPIC_PROP_REPORT  "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/properties/report"

//  辅助函数

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
}

//  发送同步命令响应
static void send_command_response(esp_mqtt_client_handle_t client, const char *request_id, int id_len) {
    if (client == NULL) return;
    
    char topic[128];
    snprintf(topic, sizeof(topic), TOPIC_CMD_RESP_FMT, id_len, request_id);
    const char *payload = "{\"result_code\":0,\"response_name\":\"COMMAND_RESPONSE\",\"paras\":{\"result\":\"success\"}}";
    
    esp_mqtt_client_publish(client, topic, payload, 0, 0, 0);
    ESP_LOGI(TAG, "Sent Response to request_id: %.*s", id_len, request_id);
}

//  核心：数据解析

static void handle_mqtt_data(esp_mqtt_event_handle_t event) {
    //  提取 request_id
    const char *p_req = strstr(event->topic, "request_id=");
    const char *request_id = NULL;
    int request_id_len = 0;
    
    if (p_req) {
        request_id = p_req + 11;
        request_id_len = event->topic + event->topic_len - request_id;
    }

    //  解析 JSON Payload
    cJSON *root = cJSON_ParseWithLength(event->data, event->data_len);
    if (root) {
        cJSON *cmd_name = cJSON_GetObjectItem(root, "command_name");
        cJSON *paras = cJSON_GetObjectItem(root, "paras");
        
        if (cJSON_IsString(cmd_name) && cJSON_IsObject(paras)) {
            ESP_LOGI(TAG, "Recv Command: %s", cmd_name->valuestring);
            
            app_msg_t msg;
            msg.type = MSG_TYPE_CLOUD_CMD;
            bool send_to_bus = false;

            //  报警控制
            if (strcmp(cmd_name->valuestring, "Control_Alarm") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cloud.cmd = CMD_ALARM_CTRL;
                    msg.payload.cloud.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            }
            //  设置阈值
            else if (strcmp(cmd_name->valuestring, "Set_Threshold") == 0) {
                cJSON *val = cJSON_GetObjectItem(paras, "value");
                float fval = 0.0f;
                bool valid = false;
                //  兼容 String、Number
                if (cJSON_IsString(val)) { fval = (float)atof(val->valuestring); valid = true; }
                else if (cJSON_IsNumber(val)) { fval = (float)val->valuedouble; valid = true; }
                
                if (valid) {
                    msg.payload.cloud.cmd = CMD_SET_THRESHOLD;
                    msg.payload.cloud.val_float = fval;
                    send_to_bus = true;
                }
            }
            //  传感器开关
            else if (strcmp(cmd_name->valuestring, "Control_Sensor") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cloud.cmd = CMD_SENSOR_SWITCH;
                    msg.payload.cloud.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            }
            //  显示器开关
            else if (strcmp(cmd_name->valuestring, "Control_Display") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cloud.cmd = CMD_DISPLAY_SWITCH;
                    msg.payload.cloud.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            }
            //  网络复位
            else if (strcmp(cmd_name->valuestring, "Reset_Network") == 0) {
                //  这里不需要参数，直接复位就行
                msg.payload.cloud.cmd = CMD_NET_RESET;
                send_to_bus = true;
            }
            //  云端连接开关
            else if (strcmp(cmd_name->valuestring, "Control_Cloud") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cloud.cmd = CMD_CLOUD_SWITCH;
                    msg.payload.cloud.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            }

            //  发送给 Controller
            if (send_to_bus && g_system_bus) {
                xQueueSend(g_system_bus, &msg, 0);
            }
        }
        cJSON_Delete(root);
    }

    //  发送同步应答
    // 注意：如果是 CMD_NET_RESET 或 关闭 Cloud则可能发不出去，尽力而为
    if (request_id && client) {
        send_command_response(client, request_id, request_id_len);
    }
}

//  MQTT 事件回调

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    app_msg_t msg;
    bool send_evt = false;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to Server!");
        //  订阅命令
        esp_mqtt_client_subscribe(client, TOPIC_CMD_SUB, 1);
        
        //  通知总线：已连接，然后Controller会触发状态同步
        msg.type = MSG_TYPE_MQTT_EVT;
        msg.payload.mqtt_evt = MQTT_EVT_CONNECTED;
        send_evt = true;
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from Server");

        //  通知总线：已断开，UI显示离线
        msg.type = MSG_TYPE_MQTT_EVT;
        msg.payload.mqtt_evt = MQTT_EVT_DISCONNECTED;
        send_evt = true;
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Data Received");
        handle_mqtt_data(event);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "Connection Error");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("ESP-TLS", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("SOCK", event->error_handle->esp_transport_sock_errno);
        }
        break;
        
    default:
        break;
    }

    if (send_evt && g_system_bus) {
        xQueueSend(g_system_bus, &msg, 0);
    }
}

//  生命周期管理

//  停止并销毁客户端
//  用于断网时清理资源，防止 DNS 缓存导致的重连死循环
void mqtt_huawei_stop(void) {
    if (client != NULL) {
        ESP_LOGW(TAG, "Stopping MQTT Client...");
        // 停止
        esp_mqtt_client_stop(client);
        // 销毁
        // 这是关键，释放 TCP/IP 栈资源，防止 DNS 解析死锁
        esp_mqtt_client_destroy(client);
        client = NULL;
        ESP_LOGW(TAG, "MQTT Client Destroyed.");
    }
}

//  创建并启动客户端
//  每次连接网络后调用，确保是一个全新的干净连接
void mqtt_huawei_start(void) {
    //  防止重复启动
    if (client != NULL) {
        ESP_LOGW(TAG, "Client already running, skip start.");
        return;
    }

    //  检查云端总开关
    //  如果用户设置了“禁止上云”，则直接返回，不发起连接
    if (!settings_get_cloud_enabled()) {
        ESP_LOGW(TAG, "Cloud is Disabled by Settings. MQTT Skipped.");
        return;
    }

    //  配置参数
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_HWC_MQTT_BROKER_URL,
        .credentials.client_id = CONFIG_HWC_CLIENT_ID,
        .credentials.username = CONFIG_HWC_DEVICE_ID,
        .credentials.authentication.password = CONFIG_HWC_MQTT_PASSWORD,
        .session.keepalive = 60,
        .network.disable_auto_reconnect = false, // 允许自动重连 (在连接保持期间)
        //  这里的自动重连处理的是 TCP 层的抖动，如果 WiFi 彻底断了，这里就销毁了
    };

    //  创建 & 启动
    ESP_LOGI(TAG, "Starting MQTT Client...");
    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to init MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

//  初始化函数
//  仅保存总线句柄
void mqtt_huawei_init(QueueHandle_t bus_handle) {
    g_system_bus = bus_handle;

}

//  业务接口

void mqtt_huawei_publish_sensor(float temp, float humi) {
    if (client == NULL) return; // 没连上就不发

    char payload[256];
    snprintf(payload, sizeof(payload), 
             "{\"services\":[{\"service_id\":\"%s\",\"properties\":{\"%s\":%.2f,\"%s\":%.2f}}]}",
             SERVICE_ID, PROPERTY_TEMP, temp, PROPERTY_HUMI, humi);
    
    esp_mqtt_client_publish(client, TOPIC_PROP_REPORT, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Report Sensor Data");
}

void mqtt_huawei_report_state(float threshold, sys_mode_t mode, bool disp_on, bool sensor_on) {
    if (client == NULL) return; // 没连上就不发

    char payload[512];
    
    const char* mode_str = "AUTO";
    if (mode == MODE_MANUAL_ON) mode_str = "MANUAL_ON";
    else if (mode == MODE_MANUAL_OFF) mode_str = "MANUAL_OFF";

    //  获取云端开关状态
    bool cloud_sw = settings_get_cloud_enabled();

    snprintf(payload, sizeof(payload), 
             "{\"services\":[{\"service_id\":\"%s\",\"properties\":{"
             "\"%s\":%.1f,"
             "\"%s\":\"%s\","
             "\"%s\":\"%s\","
             "\"%s\":\"%s\","
             "\"%s\":\"%s\""
             "}}]}",
             SERVICE_ID,
             PROP_THRESH, threshold,
             PROP_MODE,   mode_str,
             PROP_DISP,   disp_on ? "ON" : "OFF",
             PROP_SENSOR, sensor_on ? "ON" : "OFF",
             PROP_CLOUD_SW, cloud_sw ? "ON" : "OFF" // 上报云端开关状态
             );

    esp_mqtt_client_publish(client, TOPIC_PROP_REPORT, payload, 0, 0, 0);
    ESP_LOGI(TAG, "Report Full State");
}