#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

typedef struct {
    bool connected;
    uint16_t conn_id;
    uint16_t mtu_size;
} ble_transport_ctx_t;

bool microros_ble_init(ble_transport_ctx_t *ctx);

bool ble_transport_open(struct uxrCustomTransport *transport);

bool ble_transport_close(struct uxrCustomTransport *transport);

size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif // BLE_TRANSPORT_H
