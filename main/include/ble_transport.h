/**
 * @file ble_transport.h
 * @brief BLE Custom Transport for micro-ROS using ESP-IDF NimBLE Stack
 *
 * ============================================================================
 * USAGE GUIDE
 * ============================================================================
 *
 * 1. Initialize BLE (once at startup):
 *
 *      ble_transport_ctx_t ble_ctx;
 *      if (!microros_ble_init(&ble_ctx)) {
 *          // Handle error
 *      }
 *
 * 2. Register custom transport with micro-ROS:
 *
 *      rmw_uros_set_custom_transport(
 *          true,                   // framing = true (HDLC for stream-oriented BLE)
 *          &ble_ctx,               // User context (passed to callbacks)
 *          ble_transport_open,     // Open callback
 *          ble_transport_close,    // Close callback
 *          ble_transport_write,    // Write callback
 *          ble_transport_read      // Read callback
 *      );
 *
 * 3. Start micro-ROS agent on PC with BLE support:
 *
 *      ros2 run micro_ros_agent micro_ros_agent ble --dev <BLE_MAC_ADDRESS>
 *
 * ============================================================================
 * CONFIGURATION (via menuconfig)
 * ============================================================================
 * See Kconfig.projbuild for available options:
 *   - MICROROS_BLE_DEVICE_NAME: BLE advertised name
 *   - MICROROS_BLE_RX_BUFFER_SIZE: RX buffer size per packet
 *   - MICROROS_BLE_RX_QUEUE_SIZE: Number of packets to buffer
 *   - MICROROS_BLE_TX_CHUNK_DELAY_MS: Delay between TX chunks
 *   - MICROROS_BLE_ADV_INTERVAL_MIN/MAX: Advertising intervals
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

/**
 * @brief BLE transport context
 *
 * This struct tracks the BLE connection state and is updated by the
 * BLE stack callbacks. You can read these values to monitor connection status.
 */
typedef struct {
    bool connected;    /**< true when BLE link established */
    uint16_t conn_id;  /**< BLE connection handle */
    uint16_t mtu_size; /**< Negotiated MTU (affects TX chunk size) */
} ble_transport_ctx_t;

/**
 * @brief Initialize and start BLE transport
 *
 * Call this ONCE at startup, BEFORE rmw_uros_set_custom_transport().
 * This function:
 *   - Initializes NVS (required by NimBLE)
 *   - Creates RX stream buffer
 *   - Initializes NimBLE stack and GATT services
 *   - Starts BLE advertising
 *
 * @param ctx Pointer to context struct (updated with connection state)
 * @return true on success, false on failure
 */
bool microros_ble_init(ble_transport_ctx_t *ctx);

/**
 * @brief Open transport callback for micro-ROS
 * @note No-op since BLE is initialized separately via microros_ble_init()
 */
bool ble_transport_open(struct uxrCustomTransport *transport);

/**
 * @brief Close transport callback for micro-ROS
 * @note No-op - BLE remains running
 */
bool ble_transport_close(struct uxrCustomTransport *transport);

/**
 * @brief Write data to agent via BLE notifications
 *
 * Data is sent in MTU-sized chunks with small delays between chunks
 * to prevent overwhelming the BLE stack.
 *
 * @param transport Unused (context via global)
 * @param buf Data to send
 * @param len Number of bytes
 * @param err Set to 1 on error
 * @return Bytes successfully sent
 */
size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

/**
 * @brief Read data from agent via BLE
 *
 * Blocks until data available or timeout expires.
 *
 * @param transport Unused
 * @param buf Buffer to store data
 * @param len Max bytes to read
 * @param timeout Timeout in ms (0 = non-blocking)
 * @param err Set to 1 on error
 * @return Bytes read
 */
size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif /* BLE_TRANSPORT_H */
