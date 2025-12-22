#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <uxr/client/transport.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief BLE transport context for micro-ROS
 *
 * This structure holds the BLE connection state and buffers
 * for the custom transport implementation.
 */
typedef struct {
    bool connected;
    uint16_t conn_id;
    uint16_t mtu_size;
    uint8_t *rx_buffer;
    size_t rx_buffer_size;
    size_t rx_data_len;
} ble_transport_ctx_t;

/**
 * @brief Initialize BLE transport context
 *
 * @param ctx Pointer to BLE transport context
 * @return true if initialization succeeds, false otherwise
 */
bool ble_transport_init(ble_transport_ctx_t *ctx);

/**
 * @brief Open the BLE transport
 *
 * This function is called by micro-ROS to open the custom transport.
 * It initializes the BLE stack and starts advertising.
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @return true if the transport is successfully opened, false otherwise
 */
bool ble_transport_open(struct uxrCustomTransport *transport);

/**
 * @brief Close the BLE transport
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @return true if the transport is successfully closed, false otherwise
 */
bool ble_transport_close(struct uxrCustomTransport *transport);

/**
 * @brief Write data to the BLE transport
 *
 * This function sends data over BLE using GATT notifications.
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @param buf Buffer containing data to write
 * @param len Length of data to write
 * @param err Pointer to store error code (if any)
 * @return Number of bytes written
 */
size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

/**
 * @brief Read data from the BLE transport
 *
 * This function reads data received over BLE through GATT write characteristics.
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @param buf Buffer to store read data
 * @param len Maximum length to read
 * @param timeout Timeout in milliseconds
 * @param err Pointer to store error code (if any)
 * @return Number of bytes read
 */
size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif // BLE_TRANSPORT_H
