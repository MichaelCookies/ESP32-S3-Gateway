#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 
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
#include "hw_recovery.h"

static const char *TAG = "MAIN_CTRL";

//  UI 数据结构
typedef struct {
    float temp;
    float humi;
    bool  wifi_connected;
    bool  alarm_on;
    sys_mode_t mode;
    bool  sensor_ok;
} ui_context_t;

//  UI 状态机
typedef enum {
    UI_STATE_OFF = 0,
    UI_STATE_SENSOR_OFF,
    UI_STATE_WIFI_LOST,
    UI_STATE_ALARM,
    UI_STATE_NORMAL,
    UI_STATE_INFO
} ui_state_t;

//  全局资源句柄
static SSD1306_t oled_dev;
static i2c_master_dev_handle_t aht20_handle = NULL;
static i2c_master_bus_handle_t aht_bus_handle = NULL;
static QueueHandle_t g_system_bus = NULL; 

//  互斥锁与共享数据
static SemaphoreHandle_t g_ui_mutex = NULL;
static ui_context_t g_ui_data = {0};

//  全局状态变量，供 process_system_logic 和 app_main 共享
static float g_last_temp = 0.0f;
static float g_last_humi = 0.0f;
static bool  g_wifi_connected = false;
static bool  g_mqtt_connected = false;

// 辅助函数

static void on_offline_data_sync(float t, float h) {
    if (g_mqtt_connected) mqtt_huawei_publish_sensor(t, h);
}

static void sync_cloud_state(void) {
    if (g_mqtt_connected && settings_get_cloud_enabled()) {
        mqtt_huawei_report_state(settings_get_threshold(), settings_get_mode(),
                               settings_get_display_enabled(), settings_get_sensor_enabled());
    }
}

// 主任务调用此函数更新 UI 数据
static void update_ui_context(float t, float h, bool wifi, bool alarm, sys_mode_t m, bool sensor) {
    if (xSemaphoreTake(g_ui_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_ui_data.temp = t;
        g_ui_data.humi = h;
        g_ui_data.wifi_connected = wifi;
        g_ui_data.alarm_on = alarm;
        g_ui_data.mode = m;
        g_ui_data.sensor_ok = sensor;
        xSemaphoreGive(g_ui_mutex);
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


// 独立显示任务

void task_display_refresh(void *pvParameters) {
    static ui_state_t last_state = UI_STATE_OFF;
    char line_buf[17];
    ui_context_t local_ctx;

    ESP_LOGI("DISPLAY", "Task Started.");

    while (1) {
        //  获取开关状态
        if (!settings_get_display_enabled()) {
            if (last_state != UI_STATE_OFF) {
                if (oled_dev._address != 0) ssd1306_clear_screen(&oled_dev, false);
                last_state = UI_STATE_OFF;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        //  检查硬件状态
        if (oled_dev._address == 0 || settings_get_oled_hw_status() != HW_STATUS_CONNECTED) {
            if (settings_get_oled_hw_status() == HW_STATUS_CONNECTED && oled_dev._address == 0) {
                 ESP_LOGW("DISPLAY", "I2C Error. Marking for Recovery.");
                 settings_set_oled_hw_status(HW_STATUS_ERROR);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        //  从主任务获取最新数据
        if (xSemaphoreTake(g_ui_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(&local_ctx, &g_ui_data, sizeof(ui_context_t));
            xSemaphoreGive(g_ui_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        //  计算状态
        ui_state_t current_state;
        if (!local_ctx.sensor_ok) current_state = UI_STATE_INFO;
        else if (!settings_get_sensor_enabled()) current_state = UI_STATE_SENSOR_OFF;
        else if (!local_ctx.wifi_connected) current_state = UI_STATE_WIFI_LOST;
        else if (local_ctx.alarm_on) current_state = UI_STATE_ALARM;
        else current_state = UI_STATE_NORMAL;

        //  清屏逻辑
        if (current_state != last_state || last_state == UI_STATE_OFF) {
             ssd1306_clear_screen(&oled_dev, false);
        }
        last_state = current_state;

        //  绘图
        switch (current_state) {
            case UI_STATE_INFO: ssd1306_display_text(&oled_dev, 3, " SENSOR ERROR!  ", 16, true); break;
            case UI_STATE_SENSOR_OFF: ssd1306_display_text(&oled_dev, 3, " Sensor Stopped ", 16, true); break;
            case UI_STATE_WIFI_LOST: 
                ssd1306_display_text(&oled_dev, 0, " Network Lost!  ", 16, true); 
                memset(line_buf, ' ', 16); line_buf[16] = 0; 
                snprintf(line_buf, 16, "Last T: %.1f C", local_ctx.temp); 
                ssd1306_display_text(&oled_dev, 6, line_buf, 16, false); 
                break;
            case UI_STATE_ALARM: 
                ssd1306_display_text(&oled_dev, 3, "!!! WARNING !!!", 15, true); 
                memset(line_buf, ' ', 16); line_buf[16] = 0; 
                snprintf(line_buf, 16, "Temp: %.1f C", local_ctx.temp); 
                ssd1306_display_text(&oled_dev, 5, line_buf, 16, false); 
                break;
            case UI_STATE_NORMAL: default:
                memset(line_buf, ' ', 16); line_buf[16] = 0;
                if (!settings_get_cloud_enabled()) snprintf(line_buf, 16, "Mode: LAN ONLY");
                else snprintf(line_buf, 16, "Mode: %s", local_ctx.mode==MODE_AUTO?"AUTO":local_ctx.mode==MODE_MANUAL_ON?"MAN ON":"SILENT");
                ssd1306_display_text(&oled_dev, 0, line_buf, 16, false);
                
                memset(line_buf, ' ', 16); line_buf[16] = 0; 
                snprintf(line_buf, 16, "T: %.1f C", local_ctx.temp); 
                ssd1306_display_text(&oled_dev, 2, line_buf, 16, false);
                
                memset(line_buf, ' ', 16); line_buf[16] = 0; 
                snprintf(line_buf, 16, "H: %.1f %%", local_ctx.humi); 
                ssd1306_display_text(&oled_dev, 4, line_buf, 16, false);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// 主业务逻辑

static void process_system_logic(app_msg_t *msg) {
    switch (msg->type) {
        case MSG_TYPE_SENSOR_DATA: {
            // 更新全局变量
            g_last_temp = msg->payload.sensor.temp; 
            g_last_humi = msg->payload.sensor.humi;
            settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
            
            float threshold = settings_get_threshold(); 
            sys_mode_t mode = settings_get_mode(); 
            bool trigger_alarm = false;
            
            if (mode == MODE_MANUAL_ON) trigger_alarm = true;
            else if (mode == MODE_MANUAL_OFF) { if (g_last_temp < (threshold - 0.5f)) { settings_set_mode(MODE_AUTO); sync_cloud_state(); } }
            else { if (g_last_temp > threshold) trigger_alarm = true; else if (g_last_temp < (threshold - 0.5f)) trigger_alarm = false; else trigger_alarm = settings_get_alarm_state(); }
            
            settings_set_alarm_state(trigger_alarm);
            
            // 通知 UI 任务
            update_ui_context(g_last_temp, g_last_humi, g_wifi_connected, trigger_alarm, mode, true);

            if (g_wifi_connected) lan_service_broadcast_sensor_data(g_last_temp, g_last_humi);
            if (g_wifi_connected && g_mqtt_connected && settings_get_cloud_enabled()) mqtt_huawei_publish_sensor(g_last_temp, g_last_humi);
            else data_logger_write(g_last_temp, g_last_humi);
            break;
        }
        case MSG_TYPE_SENSOR_HB: 
            update_ui_context(g_last_temp, g_last_humi, g_wifi_connected, false, settings_get_mode(), true);
            break;
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
            update_ui_context(g_last_temp, g_last_humi, g_wifi_connected, settings_get_alarm_state(), settings_get_mode(), true);
            break;
        }
        case MSG_TYPE_WIFI_STATUS:
            g_wifi_connected = (msg->payload.error_code == 1);
            if (g_wifi_connected) { lan_service_set_net_status(true); mqtt_huawei_start(); }
            else { g_mqtt_connected = false; lan_service_set_net_status(false); mqtt_huawei_stop(); }
            update_ui_context(g_last_temp, g_last_humi, g_wifi_connected, settings_get_alarm_state(), settings_get_mode(), true);
            break;
        case MSG_TYPE_MQTT_EVT:
            if (msg->payload.mqtt_evt == MQTT_EVT_CONNECTED) { 
                g_mqtt_connected = true; 
                sync_cloud_state(); 
            // 这一步现在是瞬间完成的，不会卡死主循环
            if (data_logger_has_data()) {
                data_logger_sync_data(on_offline_data_sync); 
            }
        }
            break;
        case MSG_TYPE_I2C_ERROR:
            // 忽略
            break;
    }
}

static void system_event_handler_wrapper(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    system_event_handler(arg, event_base, event_id, event_data);
}

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

        if (aht20_handle && settings_get_sensor_hw_status() == HW_STATUS_CONNECTED) {
            esp_err_t res = aht20_read_data(aht20_handle, &msg.payload.sensor.temp, &msg.payload.sensor.humi);
            if (res == ESP_OK) {
                msg.type = MSG_TYPE_SENSOR_DATA;
                xQueueSend(bus, &msg, 0);
            } else {
                hw_recovery_trigger_sensor_reset(); 
            }
        } else {
             msg.type = MSG_TYPE_SENSOR_HB; xQueueSend(bus, &msg, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void) {
    settings_init();
    data_logger_init();
    g_ui_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    g_system_bus = xQueueCreate(20, sizeof(app_msg_t));
    if (g_system_bus == NULL) esp_restart();

    esp_task_wdt_deinit();
    esp_task_wdt_config_t twdt_config = { .timeout_ms = 5000, .idle_core_mask = 3, .trigger_panic = true };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL); 

    i2c_master_init(&oled_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&oled_dev, 128, 64);
    if (oled_dev._address != 0) settings_set_oled_hw_status(HW_STATUS_CONNECTED);
    else settings_set_oled_hw_status(HW_STATUS_DISCONNECTED);

    i2c_master_bus_config_t aht_bus_cfg = { 
        .i2c_port = CONFIG_AHT_I2C_PORT_NUM, 
        .sda_io_num = CONFIG_AHT_SDA_GPIO, 
        .scl_io_num = CONFIG_AHT_SCL_GPIO, 
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .glitch_ignore_cnt = 7, 
        .flags.enable_internal_pullup = true 
    };

    if (i2c_new_master_bus(&aht_bus_cfg, &aht_bus_handle) == ESP_OK) {
        i2c_device_config_t aht_dev_cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = AHT20_ADDR, .scl_speed_hz = 100000 };
        i2c_master_bus_add_device(aht_bus_handle, &aht_dev_cfg, &aht20_handle);
        if (aht20_init(aht20_handle) == ESP_OK) settings_set_sensor_hw_status(HW_STATUS_CONNECTED);
        else settings_set_sensor_hw_status(HW_STATUS_DISCONNECTED);
    } else {
        aht20_handle = NULL;
        settings_set_sensor_hw_status(HW_STATUS_ERROR);
    }

    hw_recovery_init(&oled_dev, &aht_bus_handle, &aht20_handle);
    hw_recovery_start_daemon();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, system_event_handler_wrapper, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, system_event_handler_wrapper, NULL, NULL));

    mqtt_huawei_init(g_system_bus);
    lan_service_init(g_system_bus);
    wifi_init_sta();

    xTaskCreate(task_sensor_read, "Sensor", 6144, (void*)g_system_bus, 5, NULL);
    xTaskCreate(task_display_refresh, "Display", 6144, NULL, 1, NULL);

    ESP_LOGI(TAG, "Gateway System Started.");
    
    app_msg_t msg;

    while (1) {
        esp_task_wdt_reset();
        
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