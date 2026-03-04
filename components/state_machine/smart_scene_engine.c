#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "smart_scene_engine.h"
#include "sa_device_manager.h"
#include "settings.h"
#include "app_types.h"

static const char *TAG = "SCENE_ENG";
static TaskHandle_t s_engine_task_handle = NULL;


// 数据快照结构体，扩展新传感器就在这里加

typedef struct {
    float temp;
    bool  temp_valid;
    
    float humi;
    bool  humi_valid;

    // 为未来预留空间
    // float lux;
    // bool  lux_valid;
    // float pressure;

} scene_env_snapshot_t;


// 辅助执行器

void safe_set_actuator(const char* node_id, bool target_state) {
    sa_node_t *node = sa_mgr_find_node_by_id(node_id);
    if (node && node->is_online && node->ops.write) {
        sa_value_t current_val;
        bool is_online;
        sa_mgr_read_node_cache(node, &current_val, &is_online);
        
        if (is_online && current_val.b_val != target_state) {
            sa_value_t new_val = { .b_val = target_state };
            node->ops.write(node, new_val);
            sa_mgr_update_node_cache(node, new_val, true);
            ESP_LOGI(TAG, "Actuator [%s] -> %s", node_id, target_state ? "ON" : "OFF");
        }
    }
}


// 输入收集

static void gather_environment_inputs(scene_env_snapshot_t *env) {
    sa_node_t *node;
    sa_value_t val;

    node = sa_mgr_find_node_by_id("room_temperature");
    if (node) {
        sa_mgr_read_node_cache(node, &val, &env->temp_valid);
        if (env->temp_valid) env->temp = val.f_val;
    }

    node = sa_mgr_find_node_by_id("room_humidity");
    if (node) {
        sa_mgr_read_node_cache(node, &val, &env->humi_valid);
        if (env->humi_valid) env->humi = val.f_val;
    }
    
    // 未来可以考虑加入 node = sa_mgr_find_node_by_id("light_intensity"); ...
}

// 自动规则评估

static void evaluate_auto_rules(const scene_env_snapshot_t *env) {
    float alarm_threshold = settings_get_threshold();

    // 规则 1：高温报警
    if (env->temp_valid) {
        if (env->temp > alarm_threshold) {
            safe_set_actuator("buzzer_alarm", true);
            settings_set_alarm_state(true);
        } else if (env->temp < (alarm_threshold - 0.5f)) {
            safe_set_actuator("buzzer_alarm", false);
            settings_set_alarm_state(false);
        }
    } else {
        safe_set_actuator("buzzer_alarm", false);
    }

    // 规则 2：光敏自动开灯规则
    // if (env->lux_valid && env->lux < 50.0f) { ... }
}


// 手动模式执行器

static void execute_manual_mode(sys_mode_t mode) {
    if (mode == MODE_MANUAL_ON) {
        safe_set_actuator("mock_main_relay", true);
    } 
    else if (mode == MODE_MANUAL_OFF || mode == MODE_SILENT) {
        safe_set_actuator("mock_main_relay", false);
        safe_set_actuator("buzzer_alarm", false);
        settings_set_alarm_state(false);
    }
}


// 引擎主循环

static void scene_engine_task(void *pvParameters) {
    ESP_LOGI(TAG, "Smart Scene Engine Started. IPO Model Active.");
    scene_env_snapshot_t env = {0};

    while (1) {
        sys_mode_t current_mode = settings_get_mode();
        
        // Input
        gather_environment_inputs(&env);

        // Process & Output
        if (current_mode == MODE_AUTO) {
            evaluate_auto_rules(&env);
        } else {
            execute_manual_mode(current_mode);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void smart_scene_engine_start(void) {
    if (s_engine_task_handle == NULL) {
        xTaskCreate(scene_engine_task, "SceneEngine", 4096, NULL, 4, &s_engine_task_handle);
    }
}