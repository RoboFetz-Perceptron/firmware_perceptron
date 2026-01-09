#include <string.h>
#include <sys/time.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <benchmark_msgs/msg/roundtrip.h>

#include "include/ble_transport.h"

#define TAG "MICROROS_LED"
#define LED_GPIO 8

#define NODE_NAME "esp32c6_led"
#define NODE_NAMESPACE "rc_car"

#define BENCHMARK_REQUEST_TOPIC CONFIG_MICROROS_BENCHMARK_REQUEST_TOPIC
#define BENCHMARK_RESPONSE_TOPIC CONFIG_MICROROS_BENCHMARK_RESPONSE_TOPIC

#define PARAM_NAMESPACE "led_params"
#define PARAM_BRIGHTNESS "led_bright"
#define PARAM_RED "led_red"
#define PARAM_GREEN "led_green"
#define PARAM_BLUE "led_blue"

#define DEFAULT_BRIGHTNESS 64
#define DEFAULT_RED 0
#define DEFAULT_GREEN 0
#define DEFAULT_BLUE 64

#define SPIN_TIMEOUT_MS 100
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_RESYNC_PERIOD_MS 30000

// Parameter server handles + 1 for benchmark subscription
#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1)

#define RCCHECK(fn)                                                                                                                                                                                                        \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t rc = fn;                                                                                                                                                                                                 \
        if (rc != RCL_RET_OK) {                                                                                                                                                                                            \
            ESP_LOGE(TAG, "FAILED: %s = %d at %s:%d", #fn, (int)rc, __FILE__, __LINE__);                                                                                                                                   \
            return;                                                                                                                                                                                                        \
        } else {                                                                                                                                                                                                           \
            ESP_LOGI(TAG, "OK: %s", #fn);                                                                                                                                                                                  \
        }                                                                                                                                                                                                                  \
    } while (0)

typedef struct {
    int32_t brightness;
    int32_t red;
    int32_t green;
    int32_t blue;
} led_params_t;

static led_params_t s_params = {
    .brightness = DEFAULT_BRIGHTNESS,
    .red = DEFAULT_RED,
    .green = DEFAULT_GREEN,
    .blue = DEFAULT_BLUE,
};

static led_strip_handle_t s_led = NULL;
static bool s_time_synced = false;
static int64_t s_last_sync_time = 0;
static ble_transport_ctx_t s_ble_ctx;

static rcl_allocator_t s_allocator;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_executor;
static rclc_parameter_server_t s_param_server;

// Benchmark roundtrip
static rcl_subscription_t s_benchmark_sub;
static rcl_publisher_t s_benchmark_pub;
static benchmark_msgs__msg__Roundtrip s_benchmark_msg;

static esp_err_t nvs_load_i32(const char *key, int32_t *value) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_OK) {
        err = nvs_get_i32(h, key, value);
        nvs_close(h);
    }
    return err;
}

static esp_err_t nvs_save_i32(const char *key, int32_t value) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_i32(h, key, value);
        if (err == ESP_OK)
            err = nvs_commit(h);
        nvs_close(h);
    }
    return err;
}

static esp_err_t params_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    int32_t val;
    if (nvs_load_i32(PARAM_BRIGHTNESS, &val) == ESP_OK)
        s_params.brightness = val;
    if (nvs_load_i32(PARAM_RED, &val) == ESP_OK)
        s_params.red = val;
    if (nvs_load_i32(PARAM_GREEN, &val) == ESP_OK)
        s_params.green = val;
    if (nvs_load_i32(PARAM_BLUE, &val) == ESP_OK)
        s_params.blue = val;

    ESP_LOGI(TAG, "Params: brightness=%ld R=%ld G=%ld B=%ld", s_params.brightness, s_params.red, s_params.green, s_params.blue);
    return ESP_OK;
}

static bool led_init(void) {
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
    };
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_led) != ESP_OK) {
        return false;
    }
    led_strip_clear(s_led);
    return true;
}

static void led_update(void) {
    if (!s_led)
        return;

    uint8_t r = (s_params.red * s_params.brightness) / 255;
    uint8_t g = (s_params.green * s_params.brightness) / 255;
    uint8_t b = (s_params.blue * s_params.brightness) / 255;

    if (r > 0 || g > 0 || b > 0) {
        led_strip_set_pixel(s_led, 0, r, g, b);
    } else {
        led_strip_clear(s_led);
    }
    led_strip_refresh(s_led);
}

static void led_set_blink(bool on) {
    if (!s_led)
        return;
    if (on) {
        led_strip_set_pixel(s_led, 0, 32, 32, 32);
    } else {
        led_strip_clear(s_led);
    }
    led_strip_refresh(s_led);
}

static bool time_sync_with_agent(void) {
    if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Time sync session failed");
        return false;
    }

    if (!rmw_uros_epoch_synchronized()) {
        ESP_LOGW(TAG, "Epoch not synchronized");
        return false;
    }

    int64_t epoch_nanos = rmw_uros_epoch_nanos();

    struct timeval tv;
    tv.tv_sec = epoch_nanos / 1000000000LL;
    tv.tv_usec = (epoch_nanos % 1000000000LL) / 1000LL;

    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return false;
    }

    s_time_synced = true;
    s_last_sync_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Time synced: %lld.%06ld", (long long)tv.tv_sec, tv.tv_usec);
    return true;
}

static bool on_param_changed(const Parameter *old_param, const Parameter *new_param, void *ctx) {
    (void)ctx;

    if (old_param == NULL && new_param == NULL) {
        ESP_LOGE(TAG, "Callback error: both params NULL");
        return false;
    }

    if (old_param == NULL) {
        ESP_LOGI(TAG, "Creating param: %s", new_param->name.data);
        return true;
    }

    if (new_param == NULL) {
        ESP_LOGI(TAG, "Deleting param: %s", old_param->name.data);
        return false; // We do not allow deletion
    }

    if (new_param->value.type != RCLC_PARAMETER_INT) {
        ESP_LOGW(TAG, "Param %s: wrong type", new_param->name.data);
        return false;
    }

    const char *name = new_param->name.data;
    int32_t val = (int32_t)new_param->value.integer_value;

    if (val < 0)
        val = 0;
    if (val > 255)
        val = 255;

    esp_err_t err = ESP_OK;
    if (strcmp(name, PARAM_BRIGHTNESS) == 0) {
        s_params.brightness = val;
        err = nvs_save_i32(PARAM_BRIGHTNESS, val);
    } else if (strcmp(name, PARAM_RED) == 0) {
        s_params.red = val;
        err = nvs_save_i32(PARAM_RED, val);
    } else if (strcmp(name, PARAM_GREEN) == 0) {
        s_params.green = val;
        err = nvs_save_i32(PARAM_GREEN, val);
    } else if (strcmp(name, PARAM_BLUE) == 0) {
        s_params.blue = val;
        err = nvs_save_i32(PARAM_BLUE, val);
    } else {
        ESP_LOGW(TAG, "Unknown param: %s", name);
        return false;
    }

    ESP_LOGI(TAG, "Param %s = %ld (NVS: %s)", name, val, err == ESP_OK ? "saved" : "FAILED");
    led_update();
    return err == ESP_OK;
}

static void benchmark_callback(const void *msg_in) {
    const benchmark_msgs__msg__Roundtrip *req = (const benchmark_msgs__msg__Roundtrip *)msg_in;

    // Copy the incoming message
    s_benchmark_msg.seq = req->seq;
    s_benchmark_msg.stamp_host_send = req->stamp_host_send;

    // Get synchronized time for firmware receive timestamp
    int64_t time_ns = rmw_uros_epoch_nanos();
    s_benchmark_msg.stamp_firmware_recv.sec = (int32_t)(time_ns / 1000000000LL);
    s_benchmark_msg.stamp_firmware_recv.nanosec = (uint32_t)(time_ns % 1000000000LL);

    // Copy payload if present
    if (req->payload.size > 0 && req->payload.data != NULL) {
        if (s_benchmark_msg.payload.capacity < req->payload.size) {
            // Reallocate if needed
            if (s_benchmark_msg.payload.data != NULL) {
                s_allocator.deallocate(s_benchmark_msg.payload.data, s_allocator.state);
            }
            s_benchmark_msg.payload.data = s_allocator.allocate(req->payload.size, s_allocator.state);
            s_benchmark_msg.payload.capacity = req->payload.size;
        }
        if (s_benchmark_msg.payload.data != NULL) {
            memcpy(s_benchmark_msg.payload.data, req->payload.data, req->payload.size);
            s_benchmark_msg.payload.size = req->payload.size;
        }
    } else {
        s_benchmark_msg.payload.size = 0;
    }

    // Echo back immediately
    rcl_ret_t rc = rcl_publish(&s_benchmark_pub, &s_benchmark_msg, NULL);
    if (rc != RCL_RET_OK) {
        ESP_LOGW(TAG, "Benchmark publish failed: %d", (int)rc);
    }
}

static void microros_init(void) {
    ESP_LOGI(TAG, "=== micro-ROS Init Start ===");

    RCCHECK(rclc_support_init(&s_support, 0, NULL, &s_allocator));

    s_node = rcl_get_zero_initialized_node();
    ESP_LOGI(TAG, "Creating node: /%s/%s", NODE_NAMESPACE, NODE_NAME);
    RCCHECK(rclc_node_init_default(&s_node, NODE_NAME, NODE_NAMESPACE, &s_support));

    ESP_LOGI(TAG, "Initializing parameter server...");
    RCCHECK(rclc_parameter_server_init_default(&s_param_server, &s_node));

    ESP_LOGI(TAG, "Initializing executor with %d handles...", EXECUTOR_HANDLES);
    s_executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&s_executor, &s_support.context, EXECUTOR_HANDLES, &s_allocator));

    RCCHECK(rclc_executor_add_parameter_server(&s_executor, &s_param_server, on_param_changed));

    ESP_LOGI(TAG, "Adding parameters...");
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_BRIGHTNESS, RCLC_PARAMETER_INT));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_RED, RCLC_PARAMETER_INT));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_GREEN, RCLC_PARAMETER_INT));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_BLUE, RCLC_PARAMETER_INT));

    ESP_LOGI(TAG, "Setting parameter values...");
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_BRIGHTNESS, s_params.brightness));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_RED, s_params.red));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_GREEN, s_params.green));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_BLUE, s_params.blue));

    // Initialize benchmark publisher (best effort for lower latency)
    ESP_LOGI(TAG, "Initializing benchmark publisher...");
    s_benchmark_pub = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(&s_benchmark_pub, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(benchmark_msgs, msg, Roundtrip), BENCHMARK_RESPONSE_TOPIC));

    // Initialize benchmark subscriber (best effort for lower latency)
    ESP_LOGI(TAG, "Initializing benchmark subscriber...");
    s_benchmark_sub = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(&s_benchmark_sub, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(benchmark_msgs, msg, Roundtrip), BENCHMARK_REQUEST_TOPIC));

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_benchmark_sub, &s_benchmark_msg, &benchmark_callback, ON_NEW_DATA));

    // Initialize message memory
    benchmark_msgs__msg__Roundtrip__init(&s_benchmark_msg);

    ESP_LOGI(TAG, "=== micro-ROS Init Complete ===");
    ESP_LOGI(TAG, "Node: /%s/%s", NODE_NAMESPACE, NODE_NAME);
    ESP_LOGI(TAG, "Benchmark: %s -> %s", BENCHMARK_REQUEST_TOPIC, BENCHMARK_RESPONSE_TOPIC);
    ESP_LOGI(TAG, "Try: ros2 param list /%s/%s", NODE_NAMESPACE, NODE_NAME);
}

static void microros_task(void *arg) {
    (void)arg;

    ESP_LOGI(TAG, "Waiting for BLE connection...");
    while (!s_ble_ctx.connected)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "BLE connected (MTU=%d)", s_ble_ctx.mtu_size);
    vTaskDelay(pdMS_TO_TICKS(1000));

    s_allocator = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(true, &s_ble_ctx, ble_transport_open, ble_transport_close, ble_transport_write, ble_transport_read);

    ESP_LOGI(TAG, "Waiting for micro-ROS agent...");
    while (rmw_uros_ping_agent(5000, 1) != RMW_RET_OK) {
        if (!s_ble_ctx.connected) {
            while (!s_ble_ctx.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent connected!");

    vTaskDelay(pdMS_TO_TICKS(500));
    microros_init();

    ESP_LOGI(TAG, "Synchronizing time...");
    for (int i = 0; i < 3 && !s_time_synced; i++) {
        if (time_sync_with_agent())
            break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!s_time_synced) {
        ESP_LOGW(TAG, "Time sync failed, using monotonic time");
    }

    ESP_LOGI(TAG, "Entering main loop...");
    while (true) {
        if (!s_ble_ctx.connected) {
            ESP_LOGW(TAG, "BLE disconnected - waiting for reconnect...");
            s_time_synced = false; // Invalidate time on disconnect
            while (!s_ble_ctx.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
            vTaskDelay(pdMS_TO_TICKS(500));
            // Re-sync after reconnect
            time_sync_with_agent();
        }

        // Periodic time re-sync
        int64_t now = esp_timer_get_time();
        if (s_time_synced && (now - s_last_sync_time) > (TIME_RESYNC_PERIOD_MS * 1000LL)) {
            time_sync_with_agent();
        }

        rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "micro-ROS BLE Parameter Server");
    ESP_LOGI(TAG, "Node: /%s/%s", NODE_NAMESPACE, NODE_NAME);
    ESP_LOGI(TAG, "======================================");

    if (params_init() != ESP_OK) {
        ESP_LOGE(TAG, "Params init failed");
        return;
    }

    if (!led_init()) {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }

    for (int i = 0; i < 3; i++) {
        led_set_blink(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set_blink(false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    led_update();

    if (!microros_ble_init(&s_ble_ctx)) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    xTaskCreate(microros_task, "microros", 16384, NULL, 5, NULL);
}
