#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "mqtt_huawei.h"
#include "app_types.h"
#include "settings.h"
#include "sa_device_manager.h"

static const char *TAG = "HUAWEI_MQTT";

static esp_mqtt_client_handle_t client = NULL;
static QueueHandle_t g_system_bus = NULL;

#define SERVICE_ID         "Sensor_Service"
#define PROP_THRESH        "Current_Threshold"
#define PROP_MODE          "System_Mode"
#define PROP_DISP          "Display_Status"
#define PROP_SENSOR        "Sensor_Status"
#define PROP_CLOUD_SW      "Cloud_Switch"

#define TOPIC_CMD_SUB       "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/commands/#"
#define TOPIC_CMD_RESP_FMT  "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/commands/response/request_id=%.*s"
#define TOPIC_PROP_REPORT   "$oc/devices/" CONFIG_HWC_DEVICE_ID "/sys/properties/report"

static void log_error_if_nonzero(const char *message, int error_code) {
    if (error_code != 0) ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
}

static void send_command_response(esp_mqtt_client_handle_t client_handle, const char *request_id, int id_len) {
    if (client_handle == NULL) return;
    
    char topic[128];
    snprintf(topic, sizeof(topic), TOPIC_CMD_RESP_FMT, id_len, request_id);
    const char *payload = "{\"result_code\":0,\"response_name\":\"COMMAND_RESPONSE\",\"paras\":{\"result\":\"success\"}}";
    
    esp_mqtt_client_publish(client_handle, topic, payload, 0, 0, 0);
}

static void handle_mqtt_data(esp_mqtt_event_handle_t event) {
    const char *p_req = strstr(event->topic, "request_id=");
    const char *request_id = NULL;
    int request_id_len = 0;
    
    if (p_req) {
        request_id = p_req + 11;
        request_id_len = event->topic + event->topic_len - request_id;
    }

    cJSON *root = cJSON_ParseWithLength(event->data, event->data_len);
    if (root) {
        cJSON *cmd_name = cJSON_GetObjectItem(root, "command_name");
        cJSON *paras = cJSON_GetObjectItem(root, "paras");
        
        if (cJSON_IsString(cmd_name) && cJSON_IsObject(paras)) {
            app_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            msg.type = MSG_TYPE_CLOUD_CMD;
            bool send_to_bus = false;

            if (strcmp(cmd_name->valuestring, "Control_Alarm") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cmd_data.cmd = CMD_ALARM_CTRL;
                    msg.payload.cmd_data.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            } else if (strcmp(cmd_name->valuestring, "Set_Threshold") == 0) {
                cJSON *val = cJSON_GetObjectItem(paras, "value");
                if (cJSON_IsNumber(val)) { 
                    msg.payload.cmd_data.cmd = CMD_SET_THRESHOLD;
                    msg.payload.cmd_data.val_float = (float)val->valuedouble;
                    send_to_bus = true; 
                }
            } else if (strcmp(cmd_name->valuestring, "Control_Sensor") == 0) {
                cJSON *status = cJSON_GetObjectItem(paras, "status");
                if (cJSON_IsString(status)) {
                    msg.payload.cmd_data.cmd = CMD_SENSOR_SWITCH;
                    msg.payload.cmd_data.val_bool = (strcmp(status->valuestring, "ON") == 0);
                    send_to_bus = true;
                }
            } else if (strcmp(cmd_name->valuestring, "Control_Actuator") == 0) {
                cJSON *target = cJSON_GetObjectItem(paras, "target_id");
                cJSON *val = cJSON_GetObjectItem(paras, "value");
                if (cJSON_IsString(target) && (cJSON_IsBool(val) || cJSON_IsNumber(val))) {
                    msg.payload.cmd_data.cmd = CMD_ACTUATOR_CTRL;
                    strncpy(msg.payload.cmd_data.target_id, target->valuestring, sizeof(msg.payload.cmd_data.target_id) - 1);
                    msg.payload.cmd_data.val_bool = cJSON_IsTrue(val) || (cJSON_IsNumber(val) && val->valueint > 0);
                    send_to_bus = true;
                }
            }

            if (send_to_bus && g_system_bus) {
                xQueueSend(g_system_bus, &msg, 0);
            }
        }
        cJSON_Delete(root);
    }

    if (request_id && client) {
        send_command_response(client, request_id, request_id_len);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    app_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    bool send_evt = false;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            esp_mqtt_client_subscribe(client, TOPIC_CMD_SUB, 1);
            msg.type = MSG_TYPE_MQTT_EVT;
            msg.payload.mqtt_evt = MQTT_EVT_CONNECTED;
            send_evt = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            msg.type = MSG_TYPE_MQTT_EVT;
            msg.payload.mqtt_evt = MQTT_EVT_DISCONNECTED;
            send_evt = true;
            break;
        case MQTT_EVENT_DATA:
            handle_mqtt_data(event);
            break;
        case MQTT_EVENT_ERROR:
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

void mqtt_huawei_stop(void) {
    if (client != NULL) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
        client = NULL;
    }
}

void mqtt_huawei_start(void) {
    if (client != NULL || !settings_get_cloud_enabled()) return;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_HWC_MQTT_BROKER_URL,
        .credentials.client_id = CONFIG_HWC_CLIENT_ID,
        .credentials.username = CONFIG_HWC_DEVICE_ID,
        .credentials.authentication.password = CONFIG_HWC_MQTT_PASSWORD,
        .session.keepalive = 60,
        .network.disable_auto_reconnect = false,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    if (client) {
        esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
        esp_mqtt_client_start(client);
    }
}

void mqtt_huawei_init(QueueHandle_t bus_handle) {
    g_system_bus = bus_handle;
}

void mqtt_huawei_publish_sensors(void) {
    if (client == NULL) return;

    cJSON *root = cJSON_CreateObject();
    cJSON *services_arr = cJSON_CreateArray();
    cJSON *service_obj = cJSON_CreateObject();
    cJSON *properties_obj = cJSON_CreateObject();

    sa_mgr_list_lock();
    sa_node_t *curr = sa_mgr_get_list_head();
    bool has_data = false;

    while (curr) {
        sa_value_t val;
        bool is_online;
        if (sa_mgr_read_node_cache(curr, &val, &is_online) == ESP_OK && is_online) {
            if (curr->val_type == SA_VAL_TYPE_FLOAT) {
                cJSON_AddNumberToObject(properties_obj, curr->id, val.f_val);
                has_data = true;
            } else if (curr->val_type == SA_VAL_TYPE_BOOL) {
                cJSON_AddBoolToObject(properties_obj, curr->id, val.b_val);
                has_data = true;
            } else if (curr->val_type == SA_VAL_TYPE_INT) {
                cJSON_AddNumberToObject(properties_obj, curr->id, val.i_val);
                has_data = true;
            }
        }
        curr = curr->next;
    }
    sa_mgr_list_unlock();

    if (!has_data) {
        cJSON_Delete(root);
        return; 
    }

    cJSON_AddStringToObject(service_obj, "service_id", SERVICE_ID);
    cJSON_AddItemToObject(service_obj, "properties", properties_obj);
    cJSON_AddItemToArray(services_arr, service_obj);
    cJSON_AddItemToObject(root, "services", services_arr);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, TOPIC_PROP_REPORT, payload, 0, 1, 0);
        free(payload);
    }
    cJSON_Delete(root);
}

void mqtt_huawei_report_state(float threshold, sys_mode_t mode, bool disp_on, bool sensor_on) {
    if (client == NULL) return;

    const char* mode_str = (mode == MODE_MANUAL_ON) ? "MANUAL_ON" : 
                           (mode == MODE_MANUAL_OFF) ? "MANUAL_OFF" : "AUTO";

    cJSON *root = cJSON_CreateObject();
    cJSON *services_arr = cJSON_CreateArray();
    cJSON *service_obj = cJSON_CreateObject();
    cJSON *properties_obj = cJSON_CreateObject();

    cJSON_AddNumberToObject(properties_obj, PROP_THRESH, threshold);
    cJSON_AddStringToObject(properties_obj, PROP_MODE, mode_str);
    cJSON_AddStringToObject(properties_obj, PROP_DISP, disp_on ? "ON" : "OFF");
    cJSON_AddStringToObject(properties_obj, PROP_SENSOR, sensor_on ? "ON" : "OFF");
    cJSON_AddStringToObject(properties_obj, PROP_CLOUD_SW, settings_get_cloud_enabled() ? "ON" : "OFF");

    cJSON_AddStringToObject(service_obj, "service_id", SERVICE_ID);
    cJSON_AddItemToObject(service_obj, "properties", properties_obj);
    cJSON_AddItemToArray(services_arr, service_obj);
    cJSON_AddItemToObject(root, "services", services_arr);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, TOPIC_PROP_REPORT, payload, 0, 0, 0);
        free(payload);
    }
    cJSON_Delete(root);
}