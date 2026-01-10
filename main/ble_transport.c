#include "include/ble_transport.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <nvs_flash.h>
#include <string.h>

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define TAG "BLE"

static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static ble_transport_ctx_t *g_ctx = NULL;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_tx_handle;
static uint8_t g_addr_type;
static bool g_notify_enabled = false;

static StreamBufferHandle_t g_rx_stream = NULL;
#define RX_BUFFER_SIZE (CONFIG_MICROROS_BLE_RX_BUFFER_SIZE * CONFIG_MICROROS_BLE_RX_QUEUE_SIZE)

static void start_advertise(void);
static int gap_event_cb(struct ble_gap_event *event, void *arg);

static int gatt_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn;
    (void)attr;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || !ctxt->om || !g_rx_stream)
        return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0)
        return 0;

    uint8_t buf[512];
    uint16_t copy_len = len > sizeof(buf) ? sizeof(buf) : len;

    if (ble_hs_mbuf_to_flat(ctxt->om, buf, copy_len, NULL) == 0) {
        xStreamBufferSend(g_rx_stream, buf, copy_len, 0);
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_services[] = {{
                                                            .type = BLE_GATT_SVC_TYPE_PRIMARY,
                                                            .uuid = &nus_svc_uuid.u,
                                                            .characteristics = (struct ble_gatt_chr_def[]){{
                                                                                                               .uuid = &nus_rx_uuid.u,
                                                                                                               .access_cb = gatt_cb,
                                                                                                               .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                                                                           },
                                                                                                           {
                                                                                                               .uuid = &nus_tx_uuid.u,
                                                                                                               .access_cb = gatt_cb,
                                                                                                               .val_handle = &g_tx_handle,
                                                                                                               .flags = BLE_GATT_CHR_F_NOTIFY,
                                                                                                           },
                                                                                                           {0}},
                                                        },
                                                        {0}};

static void start_advertise(void) {
    struct ble_hs_adv_fields adv = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (uint8_t *)CONFIG_MICROROS_BLE_DEVICE_NAME,
        .name_len = strlen(CONFIG_MICROROS_BLE_DEVICE_NAME),
        .name_is_complete = 1,
    };
    ble_gap_adv_set_fields(&adv);

    struct ble_hs_adv_fields rsp = {
        .uuids128 = (ble_uuid128_t *)&nus_svc_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = CONFIG_MICROROS_BLE_ADV_INTERVAL_MIN,
        .itvl_max = CONFIG_MICROROS_BLE_ADV_INTERVAL_MAX,
    };
    ble_gap_adv_start(g_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event_cb, NULL);

    ESP_LOGI(TAG, "Advertising '%s'", CONFIG_MICROROS_BLE_DEVICE_NAME);
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            if (g_ctx) {
                g_ctx->connected = true;
                g_ctx->conn_id = g_conn_handle;
            }

            // Request MTU exchange for larger packets
            ble_gattc_exchange_mtu(g_conn_handle, NULL, NULL);

            // Request faster connection interval (7.5ms - 15ms) for higher throughput
            struct ble_gap_upd_params conn_params = {
                .itvl_min = 6,              // 7.5ms  (units of 1.25ms)
                .itvl_max = 12,             // 15ms
                .latency = 0,               // No slave latency for max responsiveness
                .supervision_timeout = 400, // 4 seconds
            };
            ble_gap_update_params(g_conn_handle, &conn_params);

            // Enable Data Length Extension for larger link layer packets
            ble_gap_set_data_len(g_conn_handle, 251, 2120);

            ESP_LOGI(TAG, "Connected - requesting fast CI and DLE");
        } else {
            start_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_notify_enabled = false;
        if (g_ctx)
            g_ctx->connected = false;
        if (g_rx_stream)
            xStreamBufferReset(g_rx_stream);
        start_advertise();
        ESP_LOGI(TAG, "Disconnected");
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == g_tx_handle)
            g_notify_enabled = event->subscribe.cur_notify;
        break;

    case BLE_GAP_EVENT_MTU:
        if (g_ctx)
            g_ctx->mtu_size = event->mtu.value;
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertise();
        break;
    }
    return 0;
}

static void on_sync(void) {
    ble_hs_id_infer_auto(0, &g_addr_type);
    start_advertise();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "BLE reset, reason=%d", reason); }

static void ble_task(void *param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool microros_ble_init(ble_transport_ctx_t *ctx) {
    if (!ctx)
        return false;

    ctx->connected = false;
    ctx->conn_id = 0;
    ctx->mtu_size = 23;
    g_ctx = ctx;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed");
        return false;
    }

    g_rx_stream = xStreamBufferCreate(RX_BUFFER_SIZE, 1);
    if (!g_rx_stream) {
        ESP_LOGE(TAG, "Stream buffer failed");
        return false;
    }

    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed");
        vStreamBufferDelete(g_rx_stream);
        g_rx_stream = NULL;
        return false;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(gatt_services) != 0 || ble_gatts_add_svcs(gatt_services) != 0) {
        ESP_LOGE(TAG, "GATT init failed");
        nimble_port_deinit();
        vStreamBufferDelete(g_rx_stream);
        g_rx_stream = NULL;
        return false;
    }

    ble_svc_gap_device_name_set(CONFIG_MICROROS_BLE_DEVICE_NAME);
    nimble_port_freertos_init(ble_task);

    ESP_LOGI(TAG, "Initialized");
    return true;
}

bool ble_transport_open(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

bool ble_transport_close(struct uxrCustomTransport *transport) {
    (void)transport;
    return true;
}

size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)transport;

    if (!buf || len == 0 || !g_ctx || !g_ctx->connected || !g_notify_enabled) {
        if (err)
            *err = 1;
        return 0;
    }

    size_t chunk = g_ctx->mtu_size > 3 ? g_ctx->mtu_size - 3 : 20;
    size_t sent = 0;

    while (sent < len) {
        size_t to_send = (len - sent) > chunk ? chunk : (len - sent);

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

size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    (void)transport;

    if (!buf || len == 0 || !g_rx_stream) {
        if (err)
            *err = 1;
        return 0;
    }

    return xStreamBufferReceive(g_rx_stream, buf, len, timeout > 0 ? pdMS_TO_TICKS(timeout) : 0);
}
