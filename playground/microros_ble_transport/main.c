#include "ble_transport.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#define TAG "MAIN"

// micro-ROS objects
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

// BLE transport context
static ble_transport_ctx_t ble_ctx;

// Publisher task
static void publisher_task(void *arg) {
    int32_t count = 0;

    while (1) {
        // Wait for BLE connection
        if (!ble_ctx.connected) {
            ESP_LOGI(TAG, "Waiting for BLE connection...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Publish message
        msg.data = count++;
        rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

        if (ret == RCL_RET_OK) {
            ESP_LOGI(TAG, "Published: %ld", msg.data);
        } else {
            ESP_LOGW(TAG, "Failed to publish: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "micro-ROS BLE Integer Publisher Example");

    // Initialize BLE transport context
    if (!ble_transport_init(&ble_ctx)) {
        ESP_LOGE(TAG, "Failed to initialize BLE transport context");
        return;
    }

    // Set custom transport
    rmw_uros_set_custom_transport(
        false, // framing disabled (we're using stream-oriented mode)
        &ble_ctx,
        ble_transport_open,
        ble_transport_close,
        ble_transport_write,
        ble_transport_read);

    ESP_LOGI(TAG, "Custom BLE transport configured");

    // Wait for BLE to initialize and client to connect
    ESP_LOGI(TAG, "Starting BLE advertising...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();

    // Create init options
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to initialize support: %d", ret);
        return;
    }

    // Create node
    ret = rclc_node_init_default(&node, "ble_int_publisher", "", &support);
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create node: %d", ret);
        return;
    }

    // Create publisher
    ret = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "ble_counter");

    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG, "Failed to create publisher: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "micro-ROS initialized successfully");

    // Create publisher task
    xTaskCreate(publisher_task, "publisher_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Publisher task started");
}
