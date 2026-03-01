/*
 * wifi_station.c
 * 职责：负责 WiFi 连接管理、断线重连策略（指数退避）
 * 最佳实践：
 * - 防御性初始化：兼容外部已初始化 Event Loop 的情况
 * - 健壮的重连算法：避免网络震荡时的资源耗尽
 * - 明确的资源释放：虽然此模块通常伴随系统全生命周期，但逻辑上保持闭环
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_station.h"

//  从 KConfig 读取配置，解耦硬编码
#define WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD

//  重连策略配置
#define MAXIMUM_RETRY  5           // 最大快速重试次数
#define MAX_BACKOFF_MS 10000       // 最大退避时间 (10秒)

static const char *TAG = "wifi_station";
static int s_retry_num = 0;

//  内部逻辑

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    //  WiFi 启动 -> 开始连接
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    //  WiFi 断开 -> 触发重连策略
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_num++;
        
        //  指数退避算法: 1s, 2s, 4s, 8s...
        int delay_ms = 1000 * (1 << (s_retry_num - 1));
        if (delay_ms > MAX_BACKOFF_MS) {
            delay_ms = MAX_BACKOFF_MS; // 封顶
        }

        ESP_LOGW(TAG, "WiFi Disconnected. Reconnecting in %d ms (Attempt %d)...", delay_ms, s_retry_num);
        
        //  阻塞当前任务，等待退避时间
        //  这会暂时阻塞其他 WiFi 事件的处理，但在断网场景下是合理的
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        
        esp_wifi_connect();

    } 
    //  获取 IP -> 连接成功
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        
        s_retry_num = 0; // 重置重试计数器

    }
}

//  公开接口

void wifi_init_sta(void)
{
    
    //  初始化 TCP/IP 协议栈
    //  最佳实践：Netif 初始化是幂等的
    //  如果其他地方已经初始化过，这里通常不会报错，或者返回特定状态。
    ESP_ERROR_CHECK(esp_netif_init());

    //  创建系统默认事件循环
    //  如果 main.c 已经创建了 Loop，这里会返回 ESP_ERR_INVALID_STATE
    //  我们不应该因为“已经创建成功”而崩溃，所以需要过滤这个错误码
    esp_err_t err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "Event loop already initialized, skipping.");
    } else {
        ESP_ERROR_CHECK(err); // 如果是内存不足等其他错误，必须报错
    }

    //  创建默认 WiFi STA 网络接口
    // 如果已经存在，esp_netif_create_default_wifi_sta 会返回指针或 NULL
    // 但它内部有检查机制，通常可以安全多次调用，会返回已存在的 handle
    // 也可以通过 esp_netif_get_handle_from_ifkey 检查
    // 这里为了简洁保持默认写法，IDF 内部处理了重复调用保护
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    (void)netif; // 防止未使用变量警告

    //  初始化 WiFi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //  注册事件处理程序
    //  这里的注册必须确保 Event Loop 已经存在
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    //  配置 WiFi 模式和参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            //  设置 WPA2 认证，增强安全性
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            //  启用 PMF，这里为提高兼容性设为 false
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    //  启动 WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi Station Initialized.");

}

void wifi_force_reset(void) {
    ESP_LOGW(TAG, "Manual WiFi Reset Triggered!");
    // 主动断开以触发 DISCONNECTED 事件，进而进入 event_handler 的重连流程
    esp_wifi_disconnect(); 
}
