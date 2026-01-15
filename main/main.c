#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

//  引入业务模块
#include "app_types.h"
#include "settings.h"
#include "ssd1306.h"
#include "aht20.h"
#include "wifi_station.h"
#include "mqtt_huawei.h"
#include "data_logger.h"
#include "lan_service.h"
#include "hw_recovery.h" // 故障恢复模块

static const char *TAG = "MAIN_CTRL";

//  类型定义
typedef enum {
    UI_STATE_OFF = 0,
    UI_STATE_SENSOR_OFF,
    UI_STATE_WIFI_LOST,
    UI_STATE_ALARM,
    UI_STATE_NORMAL,
    UI_STATE_INFO
} ui_state_t;

typedef struct {
    float temp;
    float humi;
} sensor_event_t;

//  全局资源句柄
static SSD1306_t oled_dev;
static i2c_master_dev_handle_t aht20_handle = NULL;
static i2c_master_bus_handle_t aht_bus_handle = NULL;
//  统一使用 g_system_bus
static QueueHandle_t g_system_bus = NULL; 

static float g_last_temp = 0.0f;
static float g_last_humi = 0.0f;
static bool  g_wifi_connected = false;
static bool  g_mqtt_connected = false;
static ui_state_t g_last_ui_state = UI_STATE_OFF;

//  辅助业务逻辑

static void on_offline_data_sync(float t, float h) {
    if (g_mqtt_connected) mqtt_huawei_publish_sensor(t, h);
}

static void sync_cloud_state(void) {
    if (g_mqtt_connected && settings_get_cloud_enabled()) {
        mqtt_huawei_report_state(settings_get_threshold(), settings_get_mode(),
                               settings_get_display_enabled(), settings_get_sensor_enabled());
    }
}

static void system_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    app_msg_t msg;
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        msg.type = MSG_TYPE_WIFI_STATUS; msg.payload.error_code = 1;
        if(g_system_bus) xQueueSend(g_system_bus, &msg, 0);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        msg.type = MSG_TYPE_WIFI_STATUS; msg.payload.error_code = 0;
        if(g_system_bus) xQueueSend(g_system_bus, &msg, 0);
    }
}

//  UI 渲染层

static void update_display_smart(float temp, float humi, bool alarm_on, sys_mode_t mode) {
    if (settings_get_oled_hw_status() != HW_STATUS_CONNECTED) return;
    if (!settings_get_display_enabled()) {
        if (g_last_ui_state != UI_STATE_OFF) { ssd1306_clear_screen(&oled_dev, false); g_last_ui_state = UI_STATE_OFF; }
        return;
    }

    ui_state_t current_state;
    if (settings_get_sensor_hw_status() != HW_STATUS_CONNECTED) current_state = UI_STATE_INFO;
    else if (!settings_get_sensor_enabled()) current_state = UI_STATE_SENSOR_OFF;
    else if (!g_wifi_connected) current_state = UI_STATE_WIFI_LOST;
    else if (alarm_on) current_state = UI_STATE_ALARM;
    else current_state = UI_STATE_NORMAL;

    if (current_state != g_last_ui_state || g_last_ui_state == UI_STATE_OFF) ssd1306_clear_screen(&oled_dev, false);
    g_last_ui_state = current_state;

    char line_buf[17];
    switch (current_state) {
        case UI_STATE_INFO: ssd1306_display_text(&oled_dev, 3, " SENSOR ERROR!  ", 16, true); break;
        case UI_STATE_SENSOR_OFF: ssd1306_display_text(&oled_dev, 3, " Sensor Stopped ", 16, true); break;
        case UI_STATE_WIFI_LOST: ssd1306_display_text(&oled_dev, 0, " Network Lost!  ", 16, true); memset(line_buf, ' ', 16); line_buf[16] = 0; snprintf(line_buf, 16, "Last T: %.1f C", temp); ssd1306_display_text(&oled_dev, 6, line_buf, 16, false); break;
        case UI_STATE_ALARM: ssd1306_display_text(&oled_dev, 3, "!!! WARNING !!!", 15, true); memset(line_buf, ' ', 16); line_buf[16] = 0; snprintf(line_buf, 16, "Temp: %.1f C", temp); ssd1306_display_text(&oled_dev, 5, line_buf, 16, false); break;
        case UI_STATE_NORMAL: default:
            memset(line_buf, ' ', 16); line_buf[16] = 0;
            if (!settings_get_cloud_enabled()) snprintf(line_buf, 16, "Mode: LAN ONLY");
            else snprintf(line_buf, 16, "Mode: %s", mode==MODE_AUTO?"AUTO":mode==MODE_MANUAL_ON?"MAN ON":"SILENT");
            ssd1306_display_text(&oled_dev, 0, line_buf, 16, false);
            memset(line_buf, ' ', 16); line_buf[16] = 0; snprintf(line_buf, 16, "T: %.1f C", temp); ssd1306_display_text(&oled_dev, 2, line_buf, 16, false);
            memset(line_buf, ' ', 16); line_buf[16] = 0; snprintf(line_buf, 16, "H: %.1f %%", humi); ssd1306_display_text(&oled_dev, 4, line_buf, 16, false);
            break;
    }
}

//  控制中心逻辑

static void process_system_logic(app_msg_t *msg) {
    switch (msg->type) {
        case MSG_TYPE_SENSOR_DATA: {
            g_last_temp = msg->payload.sensor.temp; g_last_humi = msg->payload.sensor.humi;
            settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
            float threshold = settings_get_threshold(); sys_mode_t mode = settings_get_mode(); bool trigger_alarm = false;
            if (mode == MODE_MANUAL_ON) trigger_alarm = true;
            else if (mode == MODE_MANUAL_OFF) { if (g_last_temp < (threshold - 0.5f)) { settings_set_mode(MODE_AUTO); sync_cloud_state(); } }
            else { if (g_last_temp > threshold) trigger_alarm = true; else if (g_last_temp < (threshold - 0.5f)) trigger_alarm = false; else trigger_alarm = settings_get_alarm_state(); }
            settings_set_alarm_state(trigger_alarm);
            update_display_smart(g_last_temp, g_last_humi, trigger_alarm, mode);
            if (g_wifi_connected) lan_service_broadcast_sensor_data(g_last_temp, g_last_humi);
            if (g_wifi_connected && g_mqtt_connected && settings_get_cloud_enabled()) mqtt_huawei_publish_sensor(g_last_temp, g_last_humi);
            else data_logger_write(g_last_temp, g_last_humi);
            break;
        }
        case MSG_TYPE_SENSOR_HB: update_display_smart(g_last_temp, g_last_humi, false, settings_get_mode()); break;
        case MSG_TYPE_LOCAL_CMD: msg->type = MSG_TYPE_CLOUD_CMD; process_system_logic(msg); break;
        case MSG_TYPE_CLOUD_CMD: {
            bool ns = true;
            switch (msg->payload.cloud.cmd) {
                case CMD_ALARM_CTRL: settings_set_mode(msg->payload.cloud.val_bool?MODE_MANUAL_ON:MODE_MANUAL_OFF); break;
                case CMD_SET_THRESHOLD: settings_set_threshold(msg->payload.cloud.val_float); break;
                case CMD_SENSOR_SWITCH: settings_set_sensor_enabled(msg->payload.cloud.val_bool); break;
                case CMD_DISPLAY_SWITCH: settings_set_display_enabled(msg->payload.cloud.val_bool); break;
                case CMD_NET_RESET: esp_wifi_disconnect(); ns = false; break;
                case CMD_CLOUD_SWITCH: settings_set_cloud_enabled(msg->payload.cloud.val_bool); if (msg->payload.cloud.val_bool) mqtt_huawei_start(); else { mqtt_huawei_stop(); g_mqtt_connected = false; } break;
                default: ns = false; break;
            }
            if (ns) sync_cloud_state();
            update_display_smart(g_last_temp, g_last_humi, settings_get_alarm_state(), settings_get_mode());
            break;
        }
        case MSG_TYPE_WIFI_STATUS:
            if (msg->payload.error_code == 1) { g_wifi_connected = true; lan_service_set_net_status(true); mqtt_huawei_start(); }
            else { g_wifi_connected = false; g_mqtt_connected = false; lan_service_set_net_status(false); mqtt_huawei_stop(); update_display_smart(g_last_temp, g_last_humi, settings_get_alarm_state(), settings_get_mode()); }
            break;
        case MSG_TYPE_MQTT_EVT:
            if (msg->payload.mqtt_evt == MQTT_EVT_CONNECTED) { g_mqtt_connected = true; sync_cloud_state(); if (data_logger_has_data()) data_logger_sync_data(on_offline_data_sync); }
            else g_mqtt_connected = false;
            break;
        case MSG_TYPE_I2C_ERROR:
            settings_set_sensor_hw_status(HW_STATUS_DISCONNECTED);
            update_display_smart(g_last_temp, g_last_humi, false, settings_get_mode());
            break;
    }
}

//  传感器工作任务

void task_sensor_read(void *pvParameters) {
    QueueHandle_t bus = (QueueHandle_t)pvParameters;
    app_msg_t msg;
    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();

        if (!settings_get_sensor_enabled()) {
            msg.type = MSG_TYPE_SENSOR_HB; xQueueSend(bus, &msg, 0);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // 仅在硬件正常连接时读取
        if (settings_get_sensor_hw_status() == HW_STATUS_CONNECTED) {
            esp_err_t res = aht20_read_data(aht20_handle, &msg.payload.sensor.temp, &msg.payload.sensor.humi);
            if (res == ESP_OK) {
                msg.type = MSG_TYPE_SENSOR_DATA;
                xQueueSend(bus, &msg, 0);
            } else {
                ESP_LOGE(TAG, "AHT20 Read Fail. Triggering Recovery...");
                hw_recovery_trigger_sensor_reset(); 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

//  入口函数

void app_main(void) {
    //  系统核心配置与文件系统
    settings_init();
    data_logger_init();

    //  初始化网络栈，必须早于任何回调注册
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //  必须在这里初始化总线句柄，确保 loop 不崩
    g_system_bus = xQueueCreate(20, sizeof(app_msg_t));
    if (g_system_bus == NULL) esp_restart();

    //  看门狗配置
    esp_task_wdt_deinit();
    esp_task_wdt_config_t twdt_config = { .timeout_ms = 5000, .idle_core_mask = 3, .trigger_panic = true };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL); // 将 app_main 加入喂狗名单

    //  OLED初次加载
    i2c_master_init(&oled_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&oled_dev, 128, 64);
    if (oled_dev._address != 0) settings_set_oled_hw_status(HW_STATUS_CONNECTED);
    else settings_set_oled_hw_status(HW_STATUS_DISCONNECTED);

    //  AHT20硬件驱动初次加载 (AHT20)
    i2c_master_bus_config_t aht_bus_cfg = { .i2c_port = CONFIG_AHT_I2C_PORT_NUM, .sda_io_num = CONFIG_AHT_SDA_GPIO, .scl_io_num = CONFIG_AHT_SCL_GPIO, .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true };
    if (i2c_new_master_bus(&aht_bus_cfg, &aht_bus_handle) == ESP_OK) {
        i2c_device_config_t aht_dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = AHT20_ADDR, .scl_speed_hz = 100000 };
        i2c_master_bus_add_device(aht_bus_handle, &aht_dev_cfg, &aht20_handle);
        if (aht20_init(aht20_handle) == ESP_OK) settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
        else settings_set_sensor_hw_status(HW_STATUS_DISCONNECTED);
    }

    //  初始化故障自愈模块
    hw_recovery_init(&oled_dev, &aht_bus_handle, &aht20_handle);

    //  注册系统事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, system_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, system_event_handler, NULL, NULL));

    //  启动服务
    mqtt_huawei_init(g_system_bus);
    lan_service_init(g_system_bus);
    wifi_init_sta();

    //  启动 Worker 任务
    xTaskCreate(task_sensor_read, "Sensor", 4096, (void*)g_system_bus, 5, NULL);

    ESP_LOGI(TAG, "Gateway System Started.");
    
    app_msg_t msg;


    //  main.c -> 主循环
    while (1) {
    esp_task_wdt_reset();
    hw_recovery_tick();

    // 只要 WiFi 连着，即使没有新传感器消息，每秒也播报一次最后已知的温湿度
    // 这样上位机界面就不会显示离线
    static uint32_t last_broadcast = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (g_wifi_connected && (now - last_broadcast > 1000)) {
        lan_service_broadcast_sensor_data(g_last_temp, g_last_humi);
        last_broadcast = now;
    }

    if (xQueueReceive(g_system_bus, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
        process_system_logic(&msg);
    }
}
}