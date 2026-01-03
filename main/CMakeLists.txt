set(SOURCES
    main.c
    ble_transport.c
)

idf_component_register(SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash
)