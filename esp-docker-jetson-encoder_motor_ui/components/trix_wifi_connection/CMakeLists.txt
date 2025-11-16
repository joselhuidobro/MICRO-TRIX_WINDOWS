idf_component_register(
    SRCS           "trix_wifi_connection.c"
    INCLUDE_DIRS   "include"
    REQUIRES       esp_netif esp_wifi esp_event nvs_flash
    PRIV_REQUIRES  esp_timer driver
)

