idf_component_register(
    SRCS "lan_service.c" "wifi_station.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_wifi esp_netif esp_timer lwip driver json common state_machine
)