#include <string.h>
#include "esp_log.h"
#include "sa_node_oled.h"
#include "font8x8_basic.h"

static const char *TAG = "SA_OLED";
#define OLED_I2C_ADDR 0x3C

typedef struct {
    sa_bus_t *bus;
    bool is_on;
    SemaphoreHandle_t draw_mutex;
} oled_ctx_t;

static esp_err_t oled_cmd(sa_bus_t *bus, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    return bus->write_bytes(bus, OLED_I2C_ADDR, buf, 2);
}

static esp_err_t oled_data_long(sa_bus_t *bus, const uint8_t *data, size_t len) {
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    esp_err_t err = bus->write_bytes(bus, OLED_I2C_ADDR, buf, len + 1);
    free(buf);
    return err;
}

static esp_err_t oled_set_window(sa_bus_t *bus, uint8_t p_start, uint8_t p_end, uint8_t c_start, uint8_t c_end) {
    esp_err_t err;
    err = oled_cmd(bus, 0x22); if (err != ESP_OK) return err;
    err = oled_cmd(bus, p_start); if (err != ESP_OK) return err;
    err = oled_cmd(bus, p_end); if (err != ESP_OK) return err;
    err = oled_cmd(bus, 0x21); if (err != ESP_OK) return err;
    err = oled_cmd(bus, c_start); if (err != ESP_OK) return err;
    err = oled_cmd(bus, c_end);
    return err;
}

static esp_err_t oled_clear_screen(sa_bus_t *bus) {
    esp_err_t err = oled_set_window(bus, 0, 7, 0, 127);
    if (err != ESP_OK) return err;
    
    uint8_t line[128] = {0};
    for (int i = 0; i < 8; i++) {
        err = oled_data_long(bus, line, 128);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t oled_init(sa_node_t *node) {
    oled_ctx_t *ctx = (oled_ctx_t *)node->user_ctx;
    sa_bus_t *bus = ctx->bus;
    vTaskDelay(pdMS_TO_TICKS(100));

    static const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40, 
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 
        0x00, 0xD5, 0x80, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 
        0x20, 0x8D, 0x14, 0xAF
    };

    xSemaphoreTake(ctx->draw_mutex, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    
    for (int i = 0; i < sizeof(init_cmds); i++) {
        err = oled_cmd(bus, init_cmds[i]);
        if (err != ESP_OK) break;
    }
    
    if (err == ESP_OK) {
        err = oled_clear_screen(bus);
    }
    
    ctx->is_on = (err == ESP_OK);
    xSemaphoreGive(ctx->draw_mutex);
    
    if (err == ESP_OK) ESP_LOGI(TAG, "OLED Display node initialized successfully");
    return err;
}

static esp_err_t oled_ioctl(sa_node_t *node, int cmd, void *args) {
    oled_ctx_t *ctx = (oled_ctx_t *)node->user_ctx;
    esp_err_t err = ESP_OK;
    
    xSemaphoreTake(ctx->draw_mutex, portMAX_DELAY);
    if (cmd == SA_OLED_CMD_CLEAR) {
        err = oled_clear_screen(ctx->bus);
    }
    else if (cmd == SA_OLED_CMD_DRAW_TEXT) {
        sa_oled_text_args_t *t = (sa_oled_text_args_t *)args;
        const char *p = t->text;
        uint8_t cur_col = t->col;
        while (*p && cur_col <= 120) {
            uint8_t char_data[8];
            memcpy(char_data, font8x8_basic_tr[(uint8_t)*p], 8);
            if (t->inverse) for (int i = 0; i < 8; i++) char_data[i] = ~char_data[i];
            
            err = oled_set_window(ctx->bus, t->page, t->page, cur_col, cur_col + 7);
            if (err != ESP_OK) break;
            
            err = oled_data_long(ctx->bus, char_data, 8);
            if (err != ESP_OK) break;
            
            cur_col += 8; p++;
        }
    }
    xSemaphoreGive(ctx->draw_mutex);
    return err;
}

sa_node_t* sa_node_oled_create(sa_bus_t *bus) {
    sa_node_t *node = calloc(1, sizeof(sa_node_t));
    oled_ctx_t *ctx = calloc(1, sizeof(oled_ctx_t));
    ctx->bus = bus;
    ctx->draw_mutex = xSemaphoreCreateMutex();
    node->id = "oled_display";
    node->type = SA_NODE_ACTUATOR;
    node->val_type = SA_VAL_TYPE_BOOL;
    node->user_ctx = ctx;
    node->ops.init = oled_init;
    node->ops.ioctl = oled_ioctl;
    node->ops.write = NULL;
    return node;
}