set(SOURCES
    main.c
    ble_transport.c
    control.c
    ros_node.c
    status_led.c
    am32.c
)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_gpio esp_driver_mcpwm esp_driver_ledc esp_adc esp_driver_uart
)
