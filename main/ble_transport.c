/**
 * @file ble_transport.c
 * @brief BLE Custom Transport for micro-ROS using ESP-IDF NimBLE Stack
 *
 * ============================================================================
 * OVERVIEW
 * ============================================================================
 * This file implements a Bluetooth Low Energy (BLE) transport layer that allows
 * micro-ROS on an ESP32 to communicate with a micro-ROS Agent running on a PC.
 *
 * The communication uses the Nordic UART Service (NUS), a widely-supported
 * BLE profile that emulates a serial port over BLE:
 *   - RX Characteristic: Agent writes data TO the ESP32 (we receive)
 *   - TX Characteristic: ESP32 sends data TO the Agent via notifications
 *
 * ============================================================================
 * DATA FLOW DIAGRAM
 * ============================================================================
 *
 *   +----------------+          BLE           +------------------+
 *   |  micro-ROS     |  <---- NUS Service --> |  micro-ROS Agent |
 *   |  (ESP32)       |                        |  (PC/Linux)      |
 *   +----------------+                        +------------------+
 *
 *   RECEIVE (Agent → ESP32):
 *     Agent write → BLE stack → gatt_cb() → StreamBuffer → ble_transport_read()
 *
 *   SEND (ESP32 → Agent):
 *     ble_transport_write() → ble_gatts_notify_custom() → BLE stack → Agent
 *
 * ============================================================================
 * micro-ROS CUSTOM TRANSPORT API
 * ============================================================================
 * micro-ROS requires 4 callback functions for custom transports:
 *   1. open()  - Initialize transport (no-op here, BLE init done separately)
 *   2. close() - Cleanup transport (no-op, keep BLE running)
 *   3. write() - Send data to agent
 *   4. read()  - Receive data from agent
 *
 * Register via: rmw_uros_set_custom_transport(true, &ctx, open, close, write, read)
 *
 * The 'framing=true' parameter tells micro-ROS to use HDLC framing (0x7E delimiters)
 * because BLE is stream-oriented and needs explicit message boundaries.
 */

#include "include/ble_transport.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <nvs_flash.h>
#include <string.h>

/* NimBLE stack headers */
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BLE"

/* ============================================================================
 * SECTION 1: NORDIC UART SERVICE (NUS) UUIDs
 * ============================================================================
 * Standard UUIDs for NUS. Bytes are REVERSED (little-endian) for NimBLE.
 *
 * Human-readable format: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
 * - Service:  6e400001-...
 * - RX Char:  6e400002-... (Agent writes TO ESP32 here)
 * - TX Char:  6e400003-... (ESP32 notifies Agent here)
 */
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* ============================================================================
 * SECTION 2: GLOBAL STATE VARIABLES
 * ============================================================================
 * These track connection state and are accessed from BLE callbacks + micro-ROS task.
 */
static ble_transport_ctx_t *g_ctx = NULL;                /* User context (connection info) */
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE; /* BLE connection handle */
static uint16_t g_tx_handle;                             /* TX characteristic handle (set by NimBLE) */
static uint8_t g_addr_type;                              /* Our BLE address type */
static bool g_notify_enabled = false;                    /* True when agent subscribed to TX */

/* ============================================================================
 * SECTION 3: RX STREAM BUFFER
 * ============================================================================
 * FreeRTOS StreamBuffer is perfect for BLE RX because:
 * - Agent sends data in MTU-sized chunks (typically 20-500 bytes)
 * - StreamBuffer accumulates all chunks into a contiguous byte stream
 * - Single-producer (BLE callback) / single-consumer (micro-ROS) is safe
 * - Built-in blocking read with timeout
 *
 * Size calculation: Based on Kconfig values for flexibility
 */
static StreamBufferHandle_t g_rx_stream = NULL;
#define RX_BUFFER_SIZE (CONFIG_MICROROS_BLE_RX_BUFFER_SIZE * CONFIG_MICROROS_BLE_RX_QUEUE_SIZE)

/* Forward declarations */
static void start_advertise(void);
static int gap_event_cb(struct ble_gap_event *event, void *arg);

/* ============================================================================
 * SECTION 4: GATT SERVER CALLBACK
 * ============================================================================
 * Called by NimBLE when the agent writes to the RX characteristic.
 * This is the entry point for all incoming data.
 */
static int gatt_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn;
    (void)attr;
    (void)arg; /* Unused parameters */

    /* Only handle write operations */
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || !ctxt->om || !g_rx_stream)
        return 0;

    /* Get data length from mbuf (NimBLE's buffer chain) */
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0)
        return 0;

    /* Copy mbuf to flat buffer, then to stream.
     * Stack buffer is safe here since BLE MTU is always < 512 bytes. */
    uint8_t buf[512];
    uint16_t copy_len = len > sizeof(buf) ? sizeof(buf) : len;

    if (ble_hs_mbuf_to_flat(ctxt->om, buf, copy_len, NULL) == 0) {
        /* Non-blocking send - drop data if buffer full (better than blocking BLE) */
        xStreamBufferSend(g_rx_stream, buf, copy_len, 0);
    }

    return 0;
}

/* ============================================================================
 * SECTION 5: GATT SERVICE DEFINITION
 * ============================================================================
 * Static definition of the NUS service for NimBLE's GATT database.
 */
static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    /* RX: Agent writes here */
                    .uuid = &nus_rx_uuid.u,
                    .access_cb = gatt_cb,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                },
                {
                    /* TX: We notify agent here */
                    .uuid = &nus_tx_uuid.u,
                    .access_cb = gatt_cb,
                    .val_handle = &g_tx_handle, /* NimBLE fills this handle */
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                },
                {0} /* Array terminator */
            },
    },
    {0} /* Array terminator */
};

/* ============================================================================
 * SECTION 6: BLE ADVERTISING
 * ============================================================================
 * Start advertising so the agent can discover and connect to us.
 */
static void start_advertise(void) {
    /* Advertising data (in advertisement packets) */
    struct ble_hs_adv_fields adv = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (uint8_t *)CONFIG_MICROROS_BLE_DEVICE_NAME,
        .name_len = strlen(CONFIG_MICROROS_BLE_DEVICE_NAME),
        .name_is_complete = 1,
    };
    ble_gap_adv_set_fields(&adv);

    /* Scan response data (sent when scanner requests more info) */
    struct ble_hs_adv_fields rsp = {
        .uuids128 = (ble_uuid128_t *)&nus_svc_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    ble_gap_adv_rsp_set_fields(&rsp);

    /* Advertising parameters */
    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND, /* Connectable */
        .disc_mode = BLE_GAP_DISC_MODE_GEN, /* General discoverable */
        .itvl_min = CONFIG_MICROROS_BLE_ADV_INTERVAL_MIN,
        .itvl_max = CONFIG_MICROROS_BLE_ADV_INTERVAL_MAX,
    };
    ble_gap_adv_start(g_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event_cb, NULL);

    ESP_LOGI(TAG, "Advertising '%s'", CONFIG_MICROROS_BLE_DEVICE_NAME);
}

/* ============================================================================
 * SECTION 7: GAP EVENT HANDLER
 * ============================================================================
 * Handles BLE connection lifecycle: connect, disconnect, MTU exchange, etc.
 */
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            /* Connected successfully */
            g_conn_handle = event->connect.conn_handle;
            if (g_ctx) {
                g_ctx->connected = true;
                g_ctx->conn_id = g_conn_handle;
            }
            /* Request larger MTU (default 23 bytes is tiny) */
            ble_gattc_exchange_mtu(g_conn_handle, NULL, NULL);
            ESP_LOGI(TAG, "Connected");
        } else {
            start_advertise(); /* Connection failed, retry */
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Reset state and restart advertising */
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notify_enabled = false;
        if (g_ctx)
            g_ctx->connected = false;
        if (g_rx_stream)
            xStreamBufferReset(g_rx_stream); /* Clear stale data */
        start_advertise();
        ESP_LOGI(TAG, "Disconnected");
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        /* Agent subscribed/unsubscribed to TX notifications */
        if (event->subscribe.attr_handle == g_tx_handle)
            g_notify_enabled = event->subscribe.cur_notify;
        break;

    case BLE_GAP_EVENT_MTU:
        /* MTU negotiated - important for chunking in write() */
        if (g_ctx)
            g_ctx->mtu_size = event->mtu.value;
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertise(); /* Restart if advertising stopped */
        break;
    }
    return 0;
}

/* ============================================================================
 * SECTION 8: NIMBLE CALLBACKS AND TASK
 * ============================================================================ */
static void on_sync(void) {
    /* BLE stack ready - determine address type and start advertising */
    ble_hs_id_infer_auto(0, &g_addr_type);
    start_advertise();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "BLE reset, reason=%d", reason); }

static void ble_task(void *param) {
    (void)param;
    nimble_port_run(); /* Runs until nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ============================================================================
 * SECTION 9: PUBLIC API - INITIALIZATION
 * ============================================================================
 * Call this ONCE at startup, BEFORE rmw_uros_set_custom_transport().
 */
bool microros_ble_init(ble_transport_ctx_t *ctx) {
    if (!ctx)
        return false;

    /* Initialize context with defaults */
    ctx->connected = false;
    ctx->conn_id = 0;
    ctx->mtu_size = 23; /* BLE default */
    g_ctx = ctx;

    /* NVS is required by NimBLE for storing bonding info */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed");
        return false;
    }

    /* Create stream buffer with trigger level of 1 (wake on any data) */
    g_rx_stream = xStreamBufferCreate(RX_BUFFER_SIZE, 1);
    if (!g_rx_stream) {
        ESP_LOGE(TAG, "Stream buffer failed");
        return false;
    }

    /* Initialize NimBLE stack */
    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed");
        vStreamBufferDelete(g_rx_stream);
        g_rx_stream = NULL;
        return false;
    }

    /* Configure NimBLE callbacks */
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(gatt_services) != 0 || ble_gatts_add_svcs(gatt_services) != 0) {
        ESP_LOGE(TAG, "GATT init failed");
        nimble_port_deinit();
        vStreamBufferDelete(g_rx_stream);
        g_rx_stream = NULL;
        return false;
    }

    /* Set device name and start BLE task */
    ble_svc_gap_device_name_set(CONFIG_MICROROS_BLE_DEVICE_NAME);
    nimble_port_freertos_init(ble_task);

    ESP_LOGI(TAG, "Initialized");
    return true;
}

/* ============================================================================
 * SECTION 10: micro-ROS TRANSPORT CALLBACKS
 * ============================================================================
 * These 4 functions are passed to rmw_uros_set_custom_transport().
 */

/** Open transport (no-op - BLE already initialized) */
bool ble_transport_open(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

/** Close transport (no-op - keep BLE running) */
bool ble_transport_close(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

/**
 * Write data to agent via BLE notifications.
 *
 * Chunks data based on MTU (MTU - 3 bytes for ATT header).
 * Small delay between chunks prevents BLE stack overflow.
 */
size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)transport;

    /* Must be connected and agent must have enabled notifications */
    if (!buf || len == 0 || !g_ctx || !g_ctx->connected || !g_notify_enabled) {
        if (err)
            *err = 1;
        return 0;
    }

    /* Chunk size: MTU minus 3-byte ATT header, minimum 20 */
    size_t chunk = g_ctx->mtu_size > 3 ? g_ctx->mtu_size - 3 : 20;
    size_t sent = 0;

    while (sent < len) {
        size_t to_send = (len - sent) > chunk ? chunk : (len - sent);

        /* Create mbuf and send notification */
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buf + sent, to_send);
        if (!om || ble_gatts_notify_custom(g_conn_handle, g_tx_handle, om) != 0) {
            if (err)
                *err = 1;
            return sent;
        }

        sent += to_send;
        if (sent < len)
            vTaskDelay(pdMS_TO_TICKS(CONFIG_MICROROS_BLE_TX_CHUNK_DELAY_MS));
    }

    return sent;
}

/**
 * Read data from agent via BLE.
 *
 * Blocks until data available or timeout. StreamBuffer handles
 * accumulation of multiple BLE packets into a contiguous stream.
 */
size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    (void)transport;

    if (!buf || len == 0 || !g_rx_stream) {
        if (err)
            *err = 1;
        return 0;
    }

    /* StreamBuffer handles blocking, timeout, and byte accumulation */
    return xStreamBufferReceive(g_rx_stream, buf, len, timeout > 0 ? pdMS_TO_TICKS(timeout) : 0);
}
