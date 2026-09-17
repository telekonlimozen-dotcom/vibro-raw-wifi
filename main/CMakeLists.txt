idf_component_register(
    SRCS "main.c" "lsm6ds3trc.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        driver
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        lwip
    PRIV_REQUIRES
        esp_driver_usb_serial_jtag
        esp_hw_support
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE "FIRMWARE_VERSION=\"0.4.1-rawwifi\"")
