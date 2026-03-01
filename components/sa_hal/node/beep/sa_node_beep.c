#include <stdlib.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sa_node_beep.h"

static const char *TAG = "SA_BEEP";

#define BEEP_PIN CONFIG_BEEP_GPIO_PIN

#ifdef CONFIG_BEEP_TYPE_ACTIVE
    #ifdef CONFIG_BEEP_ACTIVE_HIGH
        #define BEEP_LEVEL_ON  1
        #define BEEP_LEVEL_OFF 0
    #else
        #define BEEP_LEVEL_ON  0
        #define BEEP_LEVEL_OFF 1
    #endif
#else
    #define BEEP_LEDC_TIMER       LEDC_TIMER_0
    #define BEEP_LEDC_MODE        LEDC_LOW_SPEED_MODE
    #define BEEP_LEDC_CHANNEL     LEDC_CHANNEL_0
    #define BEEP_LEDC_DUTY_RES    LEDC_TIMER_13_BIT
    #define BEEP_LEDC_FREQ_HZ     CONFIG_BEEP_PWM_FREQ_HZ
    #define BEEP_DUTY_50_PERCENT  (4096) 
    #define BEEP_DUTY_OFF         (0)
#endif

typedef struct {
    bool current_state;
    SemaphoreHandle_t hw_mutex;
} beep_ctx_t;

static esp_err_t beep_init(sa_node_t *node) {
    beep_ctx_t *ctx = (beep_ctx_t *)node->user_ctx;
    if (!ctx) return ESP_ERR_INVALID_STATE;

    esp_err_t err = ESP_OK;

#ifdef CONFIG_BEEP_TYPE_ACTIVE
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BEEP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    err = gpio_config(&io_conf);
    if (err == ESP_OK) {
        gpio_set_level(BEEP_PIN, BEEP_LEVEL_OFF);
    }
#else
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = BEEP_LEDC_MODE,
        .timer_num        = BEEP_LEDC_TIMER,
        .duty_resolution  = BEEP_LEDC_DUTY_RES,
        .freq_hz          = BEEP_LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = BEEP_LEDC_MODE,
        .channel        = BEEP_LEDC_CHANNEL,
        .timer_sel      = BEEP_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BEEP_PIN,
        .duty           = BEEP_DUTY_OFF,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel);
#endif

    if (err == ESP_OK) {
        ctx->current_state = false;
        ESP_LOGI(TAG, "Beep hardware initialized (Pin: %d)", BEEP_PIN);
    }
    return err;
}

static esp_err_t beep_write(sa_node_t *node, sa_value_t in_val) {
    beep_ctx_t *ctx = (beep_ctx_t *)node->user_ctx;
    if (!ctx) return ESP_ERR_INVALID_STATE;

    bool target_state = in_val.b_val;

    xSemaphoreTake(ctx->hw_mutex, portMAX_DELAY);

    if (ctx->current_state == target_state) {
        xSemaphoreGive(ctx->hw_mutex);
        return ESP_OK;
    }

#ifdef CONFIG_BEEP_TYPE_ACTIVE
    gpio_set_level(BEEP_PIN, target_state ? BEEP_LEVEL_ON : BEEP_LEVEL_OFF);
#else
    uint32_t duty = target_state ? BEEP_DUTY_50_PERCENT : BEEP_DUTY_OFF;
    ledc_set_duty(BEEP_LEDC_MODE, BEEP_LEDC_CHANNEL, duty);
    ledc_update_duty(BEEP_LEDC_MODE, BEEP_LEDC_CHANNEL);
#endif

    ctx->current_state = target_state;
    
    xSemaphoreGive(ctx->hw_mutex);

    return ESP_OK;
}

static esp_err_t beep_deinit(sa_node_t *node) {
    beep_ctx_t *ctx = (beep_ctx_t *)node->user_ctx;
    if (ctx) {
        xSemaphoreTake(ctx->hw_mutex, portMAX_DELAY);
#ifdef CONFIG_BEEP_TYPE_ACTIVE
        gpio_set_level(BEEP_PIN, BEEP_LEVEL_OFF);
        gpio_reset_pin(BEEP_PIN);
#else
        ledc_stop(BEEP_LEDC_MODE, BEEP_LEDC_CHANNEL, BEEP_DUTY_OFF);
#endif
        xSemaphoreGive(ctx->hw_mutex);
        vSemaphoreDelete(ctx->hw_mutex);
        free(ctx);
    }
    free(node);
    return ESP_OK;
}

sa_node_t* sa_node_beep_create(void) {
    sa_node_t *node = (sa_node_t *)calloc(1, sizeof(sa_node_t));
    if (!node) return NULL;

    beep_ctx_t *ctx = (beep_ctx_t *)calloc(1, sizeof(beep_ctx_t));
    if (!ctx) {
        free(node);
        return NULL;
    }

    ctx->hw_mutex = xSemaphoreCreateMutex();
    ctx->current_state = false;

    node->id = "buzzer_alarm";
    node->type = SA_NODE_ACTUATOR;
    node->val_type = SA_VAL_TYPE_BOOL;
    
    node->bus = NULL; 
    node->user_ctx = ctx;

    node->ops.init = beep_init;
    node->ops.write = beep_write;
    node->ops.deinit = beep_deinit;

    return node;
}