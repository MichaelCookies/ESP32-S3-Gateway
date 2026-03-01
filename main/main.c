#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "driver/i2c_master.h"

#include "app_types.h"
#include "settings.h"
#include "data_logger.h"
#include "hw_recovery.h"
#include "wifi_station.h"
#include "lan_service.h"
#include "mqtt_huawei.h"

// SA-HAL 框架包含
#include "sa_hal.h"
#include "sa_device_manager.h"
#include "sa_bus_i2c.h"
#include "sa_node_aht20.h"
#include "sa_node_beep.h"
#include "sa_node_oled.h"
#include "sa_mock_device.h"

static const char *TAG = "MAIN_CTRL";

// 请确保这俩引脚是接在 AHT20/OLED 上的！
#define I2C_MASTER_SCL_IO 1
#define I2C_MASTER_SDA_IO 2
#define I2C_MASTER_NUM    1

static QueueHandle_t g_system_bus = NULL;
static bool g_wifi_connected = false;
static bool g_mqtt_connected = false;

static void on_offline_data_sync(const char *payload) {
    ESP_LOGI(TAG, "Syncing offline data to cloud...");
}

// 统一的传感器轮询调度任务 (完美利用 SA-HAL 多态)
static void task_sensor_poll(void *pvParameters) {
    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool data_updated = false;

        sa_mgr_list_lock();
        sa_node_t *curr = sa_mgr_get_list_head();
        while (curr) {
            // 只轮询 Sensor 节点
            if (curr->type == SA_NODE_SENSOR) {
                if (now - curr->last_poll_tick >= pdMS_TO_TICKS(curr->poll_interval_ms)) {
                    sa_value_t val;
                    esp_err_t err = ESP_FAIL;
                    
                    // 防御性编程：确保该节点实现了 read 接口
                    if (curr->ops.read != NULL) {
                        err = curr->ops.read(curr, &val);
                    }
                    
                    // 更新缓存 (成功则 is_online=true, 失败则 is_online=false)
                    sa_mgr_update_node_cache(curr, val, (err == ESP_OK));
                    curr->last_poll_tick = now;
                    
                    if (err == ESP_OK) data_updated = true;
                }
            }
            curr = curr->next;
        }
        sa_mgr_list_unlock();

        // 只要有任何一个传感器成功更新了数据，就触发一次全局广播
        if (data_updated) {
            if (g_wifi_connected) {
                lan_service_broadcast_data();
                if (g_mqtt_connected && settings_get_cloud_enabled()) {
                    mqtt_huawei_publish_sensors();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 滴答步长
    }
}

static void task_display_refresh(void *pvParameters) {
    while (1) {
        if (!settings_get_display_enabled()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        sa_node_t *oled = sa_mgr_find_node_by_id("oled_display");
        if (oled && oled->ops.ioctl) {
            sa_value_t dummy; bool is_online;
            sa_mgr_read_node_cache(oled, &dummy, &is_online);
            
            if (is_online) {
                oled->ops.ioctl(oled, SA_OLED_CMD_CLEAR, NULL);

                sa_node_t *temp = sa_mgr_find_node_by_id("room_temperature");
                sa_node_t *humi = sa_mgr_find_node_by_id("room_humidity");
                
                char buf[32];
                if (temp) {
                    sa_value_t t_val; bool t_ok;
                    sa_mgr_read_node_cache(temp, &t_val, &t_ok);
                    snprintf(buf, sizeof(buf), "T: %.1f C %s", t_val.f_val, t_ok?"":"[ERR]");
                    sa_oled_text_args_t arg0 = {.page = 2, .col = 0, .text = buf, .inverse = false};
                    oled->ops.ioctl(oled, SA_OLED_CMD_DRAW_TEXT, &arg0);
                }

                if (humi) {
                    sa_value_t h_val; bool h_ok;
                    sa_mgr_read_node_cache(humi, &h_val, &h_ok);
                    snprintf(buf, sizeof(buf), "H: %.1f %% %s", h_val.f_val, h_ok?"":"[ERR]");
                    sa_oled_text_args_t arg1 = {.page = 4, .col = 0, .text = buf, .inverse = false};
                    oled->ops.ioctl(oled, SA_OLED_CMD_DRAW_TEXT, &arg1);
                }

                snprintf(buf, sizeof(buf), "NET: %s", g_wifi_connected ? "OK" : "DISC");
                sa_oled_text_args_t arg2 = {.page = 0, .col = 0, .text = buf, .inverse = !g_wifi_connected};
                oled->ops.ioctl(oled, SA_OLED_CMD_DRAW_TEXT, &arg2);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void process_system_logic(app_msg_t *msg) {
    switch (msg->type) {
        case MSG_TYPE_CLOUD_CMD:
        case MSG_TYPE_LOCAL_CMD:
            // 【控制反转核心体现】：直接根据 ID 找节点并驱动，消灭所有 if-else!
            if (msg->payload.cmd_data.cmd == CMD_ACTUATOR_CTRL) {
                sa_node_t *node = sa_mgr_find_node_by_id(msg->payload.cmd_data.target_id);
                if (node && node->type == SA_NODE_ACTUATOR && node->ops.write) {
                    sa_value_t val = {.b_val = msg->payload.cmd_data.val_bool};
                    esp_err_t err = node->ops.write(node, val);
                    sa_mgr_update_node_cache(node, val, (err == ESP_OK));
                    ESP_LOGI(TAG, "Hardware Actuator [%s] set to %s", node->id, val.b_val ? "ON" : "OFF");
                } else {
                    ESP_LOGW(TAG, "Command target [%s] not found or is not actuator", msg->payload.cmd_data.target_id);
                }
            } else if (msg->payload.cmd_data.cmd == CMD_SET_THRESHOLD) {
                settings_set_threshold(msg->payload.cmd_data.val_float);
            } else if (msg->payload.cmd_data.cmd == CMD_NET_RESET) {
                esp_wifi_disconnect();
            }
            break;

        case MSG_TYPE_WIFI_STATUS:
            g_wifi_connected = (msg->payload.error_code == 1);
            lan_service_set_net_status(g_wifi_connected);
            if (g_wifi_connected) mqtt_huawei_start();
            else mqtt_huawei_stop();
            break;

        case MSG_TYPE_MQTT_EVT:
            g_mqtt_connected = (msg->payload.mqtt_evt == MQTT_EVT_CONNECTED);
            if (g_mqtt_connected) {
                ESP_LOGI(TAG, "Cloud Services Connected! Synchronizing state...");
                mqtt_huawei_report_state(settings_get_threshold(), settings_get_mode(), 
                                         settings_get_display_enabled(), settings_get_sensor_enabled());
                if (data_logger_has_data()) {
                    data_logger_sync_data(on_offline_data_sync);
                }
            } else {
                ESP_LOGW(TAG, "Cloud Services Disconnected.");
            }
            break;

        default:
            break;
    }
}

static void system_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    app_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        msg.type = MSG_TYPE_WIFI_STATUS; msg.payload.error_code = 1;
        if(g_system_bus) xQueueSend(g_system_bus, &msg, 0);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        msg.type = MSG_TYPE_WIFI_STATUS; msg.payload.error_code = 0;
        if(g_system_bus) xQueueSend(g_system_bus, &msg, 0);
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    settings_init();
    data_logger_init();
    sa_mgr_init();

    // 1. 实例化主 I2C 总线 (适配 ESP-IDF V5.x 规范)
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // 2. 将 IDF 原生总线包装为 SA-HAL 虚拟总线
    sa_bus_t *i2c_bus = sa_bus_i2c_create(bus_handle, "main_i2c_bus");

    // 3. 动态注册真实硬件节点
    sa_node_t *temp_node, *humi_node;
    if (sa_aht20_create_nodes(i2c_bus, &temp_node, &humi_node) == ESP_OK) {
        sa_mgr_register_node(temp_node);
        sa_mgr_register_node(humi_node);
    }
    sa_mgr_register_node(sa_node_oled_create(i2c_bus));
    sa_mgr_register_node(sa_node_beep_create());

    // 4. 动态注册模拟测试节点
    sa_mgr_register_node(sa_mock_temp_create());
    sa_mgr_register_node(sa_mock_relay_create());

    g_system_bus = xQueueCreate(20, sizeof(app_msg_t));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, system_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, system_event_handler, NULL, NULL));

    hw_recovery_start_daemon();
    mqtt_huawei_init(g_system_bus);
    lan_service_init(g_system_bus);
    wifi_init_sta();

    // 提升任务栈大小，防止 cJSON 引发的 Stack Overflow
    xTaskCreate(task_sensor_poll, "SensorPoll", 6144, NULL, 5, NULL);
    xTaskCreate(task_display_refresh, "Display", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "Gateway System Started (SA-HAL V5.0 Arch)");
    ESP_LOGI(TAG, "Waiting for network connection...");
    ESP_LOGI(TAG, "===============================================");

    app_msg_t msg;
    while (1) {
        if (xQueueReceive(g_system_bus, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            process_system_logic(&msg);
        }
    }
}