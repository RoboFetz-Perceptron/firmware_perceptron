#include "ble_transport.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gatts_api.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <string.h>

#define TAG "BLE_TRANSPORT"

// BLE configuration
#define BLE_DEVICE_NAME "microROS_BLE"
#define BLE_MTU_SIZE 512

// GATT Service UUID for micro-ROS (custom 128-bit UUID)
// Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service compatible)
static const uint8_t SERVICE_UUID[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};

// RX Characteristic UUID (write from client to ESP32)
// 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t RX_CHAR_UUID[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};

// TX Characteristic UUID (notify from ESP32 to client)
// 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t TX_CHAR_UUID[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

// GATT handles
static uint16_t g_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_service_handle = 0;
static uint16_t g_rx_char_handle = 0;
static uint16_t g_tx_char_handle = 0;

// Global transport context
static ble_transport_ctx_t *g_transport_ctx = NULL;

// Queue for received data
static QueueHandle_t rx_queue = NULL;
#define RX_QUEUE_SIZE 10
#define RX_BUFFER_SIZE 512

typedef struct {
    uint8_t data[RX_BUFFER_SIZE];
    size_t len;
} ble_rx_packet_t;

// Advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t *)SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising data set, starting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Advertising started successfully");
        } else {
            ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        }
        break;
    default:
        break;
    }
}

// GATTS event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT server registered, app_id: %d", param->reg.app_id);
        g_gatts_if = gatts_if;

        // Set device name
        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);

        // Configure advertising data
        esp_ble_gap_config_adv_data(&adv_data);

        // Create GATT service
        esp_ble_gatts_create_service(gatts_if, (esp_gatt_srvc_id_t *)&(esp_gatt_srvc_id_t){
                                                    .is_primary = true,
                                                    .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}}}},
                                      10);
        memcpy(((esp_gatt_srvc_id_t *)&(esp_gatt_srvc_id_t){
                    .is_primary = true,
                    .id = {.inst_id = 0, .uuid = {.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}}}}
                    ->id.uuid.uuid.uuid128,
                SERVICE_UUID, 16);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle: %d", param->create.service_handle);
        g_service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(g_service_handle);

        // Add RX characteristic (write)
        esp_ble_gatts_add_char(g_service_handle,
                                (esp_bt_uuid_t *)&(esp_bt_uuid_t){.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}},
                                ESP_GATT_PERM_WRITE,
                                ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                                NULL, NULL);
        memcpy(((esp_bt_uuid_t *)&(esp_bt_uuid_t){.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}})->uuid.uuid128, RX_CHAR_UUID, 16);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "Characteristic added, handle: %d", param->add_char.attr_handle);
        if (g_rx_char_handle == 0) {
            g_rx_char_handle = param->add_char.attr_handle;
            // Add TX characteristic (notify)
            esp_ble_gatts_add_char(g_service_handle,
                                    (esp_bt_uuid_t *)&(esp_bt_uuid_t){.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}},
                                    ESP_GATT_PERM_READ,
                                    ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                    NULL, NULL);
            memcpy(((esp_bt_uuid_t *)&(esp_bt_uuid_t){.len = ESP_UUID_LEN_128, .uuid = {.uuid128 = {0}}})->uuid.uuid128, TX_CHAR_UUID, 16);
        } else {
            g_tx_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "BLE GATT service setup complete");
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
        if (g_transport_ctx) {
            g_transport_ctx->connected = true;
            g_transport_ctx->conn_id = param->connect.conn_id;
        }
        // Update connection parameters for better throughput
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.min_int = 0x10; // 20ms
        conn_params.max_int = 0x20; // 40ms
        conn_params.latency = 0;
        conn_params.timeout = 400; // 4s
        esp_ble_gap_update_conn_params(&conn_params);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected");
        if (g_transport_ctx) {
            g_transport_ctx->connected = false;
        }
        // Restart advertising
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == g_rx_char_handle) {
            // Data received from client - put into queue
            ESP_LOGD(TAG, "RX data: %d bytes", param->write.len);
            if (rx_queue) {
                ble_rx_packet_t packet;
                packet.len = param->write.len > RX_BUFFER_SIZE ? RX_BUFFER_SIZE : param->write.len;
                memcpy(packet.data, param->write.value, packet.len);
                xQueueSend(rx_queue, &packet, 0);
            }
        }
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU set to: %d", param->mtu.mtu);
        if (g_transport_ctx) {
            g_transport_ctx->mtu_size = param->mtu.mtu;
        }
        break;

    default:
        break;
    }
}

bool ble_transport_init(ble_transport_ctx_t *ctx) {
    if (!ctx) {
        return false;
    }

    memset(ctx, 0, sizeof(ble_transport_ctx_t));
    ctx->mtu_size = 23; // Default MTU size
    ctx->connected = false;

    return true;
}

bool ble_transport_open(struct uxrCustomTransport *transport) {
    if (!transport || !transport->args) {
        ESP_LOGE(TAG, "Invalid transport or args");
        return false;
    }

    g_transport_ctx = (ble_transport_ctx_t *)transport->args;

    // Create RX queue
    rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(ble_rx_packet_t));
    if (!rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return false;
    }

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Register callbacks
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);

    // Register GATT application
    esp_ble_gatts_app_register(0);

    // Set MTU
    esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);

    ESP_LOGI(TAG, "BLE transport opened successfully");
    return true;
}

bool ble_transport_close(struct uxrCustomTransport *transport) {
    if (rx_queue) {
        vQueueDelete(rx_queue);
        rx_queue = NULL;
    }

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    ESP_LOGI(TAG, "BLE transport closed");
    return true;
}

size_t ble_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
    if (!transport || !buf || len == 0) {
        if (err)
            *err = 1;
        return 0;
    }

    ble_transport_ctx_t *ctx = (ble_transport_ctx_t *)transport->args;
    if (!ctx || !ctx->connected) {
        if (err)
            *err = 1;
        return 0;
    }

    // Send notification with the data
    // Split into MTU-sized chunks if necessary
    size_t bytes_sent = 0;
    size_t chunk_size = ctx->mtu_size - 3; // 3 bytes overhead for ATT protocol

    while (bytes_sent < len) {
        size_t to_send = (len - bytes_sent) > chunk_size ? chunk_size : (len - bytes_sent);

        esp_err_t ret = esp_ble_gatts_send_indicate(g_gatts_if, ctx->conn_id, g_tx_char_handle,
                                                     to_send, (uint8_t *)(buf + bytes_sent), false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Send indicate failed: %s", esp_err_to_name(ret));
            if (err)
                *err = 1;
            return bytes_sent;
        }

        bytes_sent += to_send;

        // Small delay between packets to avoid overwhelming the BLE stack
        if (bytes_sent < len) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    return bytes_sent;
}

size_t ble_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
    if (!transport || !buf || len == 0) {
        if (err)
            *err = 1;
        return 0;
    }

    if (!rx_queue) {
        if (err)
            *err = 1;
        return 0;
    }

    ble_rx_packet_t packet;
    TickType_t ticks_to_wait = timeout > 0 ? pdMS_TO_TICKS(timeout) : 0;

    if (xQueueReceive(rx_queue, &packet, ticks_to_wait) == pdTRUE) {
        size_t copy_len = packet.len > len ? len : packet.len;
        memcpy(buf, packet.data, copy_len);
        return copy_len;
    }

    return 0;
}
