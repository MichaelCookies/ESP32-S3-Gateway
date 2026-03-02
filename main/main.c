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

#if CONFIG_NODE_ENABLE_AHT20
#include "sa_node_aht20.h"
#endif

#if CONFIG_NODE_ENABLE_OLED
#include "sa_node_oled.h"
#endif

#if CONFIG_NODE_ENABLE_BEEP
#include "sa_node_beep.h"
#endif

#include "sa_mock_device.h"

static const char *TAG = "MAIN";

static QueueHandle_t g_system_bus = NULL;
static sa_bus_t *g_i2c_bus = NULL;

static volatile bool g_wifi_connected = false;
static volatile bool g_mqtt_connected = false;

// 业务任务 1：传感器轮询

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

// 业务任务 2：OLED 显示

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

        // 检查 OLED 是否存活。如果刚才被拔掉，它会被踢下线。
        sa_node_t *oled = sa_mgr_find_node_by_id("oled_display");
        if (!oled || !oled->is_online) {
            // 如果屏幕离线，不要发任何 I2C 数据，静静等待 recovery daemon 将其救活
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
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        g_mqtt_connected = false;
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    settings_init();
    data_logger_init();
    sa_mgr_init();

    // 初始化 I2C 驱动
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CONFIG_MAIN_I2C_PORT_NUM,
        .scl_io_num = CONFIG_MAIN_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_MAIN_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // 包装为 SA-HAL 总线
    g_i2c_bus = sa_bus_i2c_create(bus_handle, "main_i2c_bus");
    if (!g_i2c_bus) {
        ESP_LOGE(TAG, "Failed to create HAL bus");
        return;
    }

    // 按 Kconfig 静态装载硬件节点
#if CONFIG_NODE_ENABLE_AHT20
    sa_node_t *temp_node, *humi_node;
    if (sa_aht20_create_nodes(g_i2c_bus, &temp_node, &humi_node) == ESP_OK) {
        sa_mgr_register_node(temp_node);
        sa_mgr_register_node(humi_node);
    }
#endif

#if CONFIG_NODE_ENABLE_OLED
    sa_node_t *oled_node = sa_node_oled_create(g_i2c_bus);
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

    // 注册无条件依赖的基础测试节点
    sa_mgr_register_node(sa_mock_temp_create());
    sa_mgr_register_node(sa_mock_relay_create());

    // 按 Kconfig 启动恢复守护
#if CONFIG_HW_RECOVERY_ENABLE
    if (hw_recovery_init(g_i2c_bus)) {
        hw_recovery_start_daemon();
    }
#endif

    // 网络和系统服务
    g_system_bus = xQueueCreate(20, sizeof(app_msg_t));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, system_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, system_event_handler, NULL, NULL));
    
    mqtt_huawei_init(g_system_bus);
    lan_service_init(g_system_bus);
    wifi_init_sta();

    // 启动业务任务
    xTaskCreate(task_sensor_poll, "SensorPoll", 6144, NULL, 5, NULL);
    xTaskCreate(task_display_refresh, "Display", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "System architecture initialized successfully.");

    // 主消息循环
    app_msg_t msg;
    while (1) {
        if (xQueueReceive(g_system_bus, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (msg.type == MSG_TYPE_MQTT_EVT) {
                g_mqtt_connected = (msg.payload.mqtt_evt == MQTT_EVT_CONNECTED);
            }
        }
    }
}