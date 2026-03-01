set(srcs
    "mqtt_huawei/mqtt_huawei.c"
    "wifi/lan_service.c"
    "wifi/wifi_station.c"
)


set(include_dirs
    "mqtt_huawei"
    "wifi"
)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES mqtt esp_wifi esp_event nvs_flash lwip esp_netif sa_hal state_machine common json)