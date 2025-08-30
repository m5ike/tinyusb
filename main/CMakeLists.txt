idf_component_register(
    SRCS
        "main.cpp"
        "tcp_server.c"
        "usbip.cpp"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
         esp_timer esp_event esp_netif lwip esp_wifi nvs_flash driver
    esp_tinyusb usb esp_http_server
    usb-host
)