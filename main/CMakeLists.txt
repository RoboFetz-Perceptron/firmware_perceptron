set(SOURCES
    main.c
    ble_transport.c
    robot_control.c
)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_driver_gpio esp_driver_mcpwm esp_driver_ledc esp_adc
)