#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"

#include "app_types.h"
#include "settings.h"
#include "data_logger.h"
#include "wifi_station.h"
#include "lan_service.h"
#include "mqtt_huawei.h"
#include "hw_recovery.h"

#include "sa_hal.h"
#include "sa_bus_i2c.h"
#include "sa_device_manager.h"
#include "sa_mock_device.h"
#include "smart_scene_engine.h"

#if CONFIG_NODE_ENABLE_AHT20
#include "sa_node_aht20.h"
#endif

#if CONFIG_NODE_ENABLE_OLED
#include "sa_node_oled.h"
#endif

#if CONFIG_NODE_ENABLE_BEEP
#include "sa_node_beep.h"
#endif

static const char *TAG = "MAIN";

static QueueHandle_t g_system_bus = NULL;
static sa_bus_t *g_disp_i2c_bus = NULL;
static sa_bus_t *g_sensor_i2c_bus = NULL;

static volatile bool g_wifi_connected = false;
static volatile bool g_mqtt_connected = false;

static void task_sensor_poll(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool data_updated = false;

        sa_node_t *sensors[8] = {NULL};
        int count = 0;
        
        sa_mgr_list_lock();
        sa_node_t *curr = sa_mgr_get_list_head();
        while (curr && count < 8) {
            if (curr->type == SA_NODE_SENSOR && curr->is_online) {
                if (now - curr->last_poll_tick >= pdMS_TO_TICKS(curr->poll_interval_ms)) {
                    sensors[count++] = curr;
                }
            }
            curr = curr->next;
        }
        sa_mgr_list_unlock();

        for (int i = 0; i < count; i++) {
            sa_node_t *node = sensors[i];
            sa_value_t val;
            
            if (node->ops.read) {
                esp_err_t err = node->ops.read(node, &val);
                sa_mgr_update_node_cache(node, val, (err == ESP_OK));
                node->last_poll_tick = now;
                if (err == ESP_OK) data_updated = true;
            }
        }

        if (data_updated && g_wifi_connected) {
            lan_service_broadcast_data();
            if (g_mqtt_connected) mqtt_huawei_publish_sensors();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void task_display_refresh(void *pvParameters) {
    (void)pvParameters;
    char temp_str[32];
    char line_buf[32];
    
    sa_mgr_display_clear("oled_display");

    while (1) {
        if (!settings_get_display_enabled()) {
            sa_mgr_display_clear("oled_display");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        sa_node_t *oled = sa_mgr_find_node_by_id("oled_display");
        if (!oled || !oled->is_online) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        snprintf(temp_str, sizeof(temp_str), "NET:%s", 
                 g_wifi_connected ? (g_mqtt_connected ? "CLOUD" : "LAN") : "DISC");
        snprintf(line_buf, sizeof(line_buf), "%-16s", temp_str);
        sa_mgr_display_text("oled_display", 0, 0, line_buf, !g_wifi_connected);

        sa_node_t *temp = sa_mgr_find_node_by_id("room_temperature");
        if (temp && temp->is_online) {
            sa_value_t t_val; bool t_ok;
            sa_mgr_read_node_cache(temp, &t_val, &t_ok);
            snprintf(temp_str, sizeof(temp_str), "T:%.1fC %s", t_val.f_val, t_ok ? "" : "ERR");
            snprintf(line_buf, sizeof(line_buf), "%-16s", temp_str);
            sa_mgr_display_text("oled_display", 2, 0, line_buf, false);
        } else {
            snprintf(line_buf, sizeof(line_buf), "%-16s", "T:--C");
            sa_mgr_display_text("oled_display", 2, 0, line_buf, true);
        }

        sa_node_t *humi = sa_mgr_find_node_by_id("room_humidity");
        if (humi && humi->is_online) {
            sa_value_t h_val; bool h_ok;
            sa_mgr_read_node_cache(humi, &h_val, &h_ok);
            snprintf(temp_str, sizeof(temp_str), "H:%.1f%% %s", h_val.f_val, h_ok ? "" : "ERR");
            snprintf(line_buf, sizeof(line_buf), "%-16s", temp_str);
            sa_mgr_display_text("oled_display", 4, 0, line_buf, false);
        } else {
            snprintf(line_buf, sizeof(line_buf), "%-16s", "H:--%");
            sa_mgr_display_text("oled_display", 4, 0, line_buf, true);
        }

        snprintf(temp_str, sizeof(temp_str), "SYS:%s", 
                 g_mqtt_connected ? "OK" : (g_wifi_connected ? "WIFI" : "INIT"));
        snprintf(line_buf, sizeof(line_buf), "%-16s", temp_str);
        sa_mgr_display_text("oled_display", 6, 0, line_buf, false);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void system_event_handler(void* arg, esp_event_base_t event_base, 
                                int32_t event_id, void* event_data) {
    (void)arg;
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connected = true;
        lan_service_set_net_status(true);
        ESP_LOGI(TAG, "Network is UP. Starting Cloud Services...");
        mqtt_huawei_start();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        g_mqtt_connected = false;
        lan_service_set_net_status(false);
        ESP_LOGW(TAG, "Network is DOWN. Stopping Cloud Services...");
        mqtt_huawei_stop();
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    settings_init();
    data_logger_init();
    sa_mgr_init();

    i2c_master_bus_config_t disp_i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = CONFIG_Display_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_Display_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t disp_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&disp_i2c_cfg, &disp_bus_handle));
    g_disp_i2c_bus = sa_bus_i2c_create(disp_bus_handle, "disp_i2c_bus");

    i2c_master_bus_config_t sensor_i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = CONFIG_SENSOR_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_SENSOR_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t sensor_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&sensor_i2c_cfg, &sensor_bus_handle));
    g_sensor_i2c_bus = sa_bus_i2c_create(sensor_bus_handle, "sensor_i2c_bus");

#if CONFIG_NODE_ENABLE_AHT20
    sa_node_t *temp_node, *humi_node;
    if (sa_aht20_create_nodes(g_sensor_i2c_bus, &temp_node, &humi_node) == ESP_OK) {
        sa_mgr_register_node(temp_node);
        sa_mgr_register_node(humi_node);
    }
#endif

#if CONFIG_NODE_ENABLE_OLED
    sa_node_t *oled_node = sa_node_oled_create(g_disp_i2c_bus);
    if (oled_node) {
        sa_mgr_register_node(oled_node);
    }
#endif

#if CONFIG_NODE_ENABLE_BEEP
    sa_node_t *beep_node = sa_node_beep_create();
    if (beep_node) {
        sa_mgr_register_node(beep_node);
    }
#endif

#if CONFIG_HW_RECOVERY_ENABLE
    if (hw_recovery_init(g_disp_i2c_bus)) {
        hw_recovery_start_daemon();
    }
#endif

    sa_mgr_register_node(sa_mock_relay_create());
    sa_mgr_register_node(sa_mock_lux_create());
    sa_mgr_register_node(sa_mock_pressure_create());

    g_system_bus = xQueueCreate(20, sizeof(app_msg_t));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, system_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, system_event_handler, NULL, NULL));
    
    mqtt_huawei_init(g_system_bus);
    lan_service_init(g_system_bus);
    wifi_init_sta();

    xTaskCreate(task_sensor_poll, "SensorPoll", 6144, NULL, 5, NULL);
    xTaskCreate(task_display_refresh, "Display", 4096, NULL, 2, NULL);
    smart_scene_engine_start();

    ESP_LOGI(TAG, "System architecture initialized successfully. Multi-Bus active.");

    app_msg_t msg;
    while (1) {
        if (xQueueReceive(g_system_bus, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (msg.type == MSG_TYPE_MQTT_EVT) {
                g_mqtt_connected = (msg.payload.mqtt_evt == MQTT_EVT_CONNECTED);
                if (g_mqtt_connected) {
                    ESP_LOGI(TAG, "Huawei Cloud MQTT Connected successfully!");
                    mqtt_huawei_report_state(
                        settings_get_threshold(), 
                        settings_get_mode(), 
                        settings_get_display_enabled(), 
                        settings_get_sensor_enabled()
                    );
                } else {
                    ESP_LOGW(TAG, "Huawei Cloud MQTT Disconnected!");
                }
            } else if (msg.type == MSG_TYPE_LOCAL_CMD) {
                switch (msg.payload.cmd_data.cmd) {
                    case CMD_CLOUD_SWITCH:
                        settings_set_cloud_enabled(msg.payload.cmd_data.val_bool);
                        ESP_LOGI(TAG, "Cloud switch toggled: %s", msg.payload.cmd_data.val_bool ? "ON" : "OFF");
                        if (msg.payload.cmd_data.val_bool) {
                            if (g_wifi_connected) mqtt_huawei_start();
                        } else {
                            mqtt_huawei_stop();
                            g_mqtt_connected = false;
                        }
                        break;
                    case CMD_NET_RESET:
                        ESP_LOGW(TAG, "Manual Network Reset Triggered!");
                        wifi_force_reset();
                        break;
                    case CMD_SET_THRESHOLD:
                        settings_set_threshold(msg.payload.cmd_data.val_float);
                        break;
                    case CMD_ACTUATOR_CTRL:
                        safe_set_actuator(msg.payload.cmd_data.target_id, msg.payload.cmd_data.val_bool);
                        break;
                    default:
                        break;
                }
            } else if (msg.type == MSG_TYPE_CLOUD_CMD) {
                ESP_LOGI(TAG, "Received command from Huawei Cloud!");
                if (msg.payload.cmd_data.cmd == CMD_SET_THRESHOLD) {
                    settings_set_threshold(msg.payload.cmd_data.val_float);
                }
            }
        }
    }
}