#ifndef SA_NODE_OLED_H
#define SA_NODE_OLED_H

#include "sa_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// OLED 专属 IOCTL 命令字
typedef enum {
    SA_OLED_CMD_CLEAR = 0x01,
    SA_OLED_CMD_DRAW_TEXT = 0x02,
    SA_OLED_CMD_SET_CONTRAST = 0x03
} sa_oled_ioctl_cmd_t;

// Draw Text 的参数结构体
typedef struct {
    uint8_t page;       // 0-7 (128x64 屏幕分为 8 个 Page/行)
    uint8_t col;        // 0-127 (列)
    const char *text;   // 字符串
    bool inverse;       // 是否反色显示
} sa_oled_text_args_t;

// 创建 OLED 节点
sa_node_t* sa_node_oled_create(sa_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif // SA_NODE_OLED_H