/**
 * @file main.c
 * @brief micro-ROS BLE LED Controller with Parameter Server
 *
 * Controls the onboard WS2812 LED on ESP32-C6 (GPIO 8) via micro-ROS over BLE.
 * Includes persistent NVS parameter storage and custom message publishing.
 *
 * ROS2 Topics:
 *   - /led_control (std_msgs/Bool)     - Control LED on/off
 *   - /led_state   (std_msgs/Bool)     - Current LED state feedback
 *   - /robot_status (std_msgs/Bool)    - Periodic status (best effort)
 *
 * ROS2 Parameters (persistent in NVS):
 *   - led_bright  (int32) - LED brightness 0-255
 *   - led_red     (int32) - Red channel 0-255
 *   - led_green   (int32) - Green channel 0-255
 *   - led_blue    (int32) - Blue channel 0-255
 *
 * Usage:
 *   ros2 param set /esp32c6_led led_red 255
 *   ros2 topic pub --once /led_control std_msgs/Bool "{data: true}"
 */

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <sys/time.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>

#include "include/ble_transport.h"

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define TAG "MICROROS_LED"
#define LED_GPIO 8

/* NVS parameter keys (max 15 chars!) */
#define PARAM_NAMESPACE "led_params"
#define PARAM_BRIGHTNESS "led_bright"
#define PARAM_RED "led_red"
#define PARAM_GREEN "led_green"
#define PARAM_BLUE "led_blue"

/* Default LED values */
#define DEFAULT_BRIGHTNESS 64
#define DEFAULT_RED 0
#define DEFAULT_GREEN 0
#define DEFAULT_BLUE 64

/* Timing */
#define STATUS_PERIOD_MS 1000
#define SPIN_TIMEOUT_MS 10
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_RESYNC_PERIOD_MS 30000 /* Re-sync every 30 seconds */

/* Executor handle count: subscription + timer + parameter server */
#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2)

/* Error check macro */
#define RCCHECK(fn)                                                                                                                                                                                                        \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t rc = fn;                                                                                                                                                                                                 \
        if (rc != RCL_RET_OK) {                                                                                                                                                                                            \
            ESP_LOGE(TAG, "Error %d at %s:%d", (int)rc, __FILE__, __LINE__);                                                                                                                                               \
            return;                                                                                                                                                                                                        \
        }                                                                                                                                                                                                                  \
    } while (0)

/* ==========================================================================
 * State
 * ========================================================================== */

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
static bool s_led_on = false;
static bool s_time_synced = false;
static int64_t s_last_sync_time = 0;
static ble_transport_ctx_t s_ble_ctx;

/* micro-ROS objects */
static rcl_allocator_t s_allocator;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_executor;
static rclc_parameter_server_t s_param_server;

static rcl_subscription_t s_led_sub;
static std_msgs__msg__Bool s_led_msg;

static rcl_publisher_t s_state_pub;
static std_msgs__msg__Bool s_state_msg;

static rcl_timer_t s_status_timer;
static rcl_publisher_t s_status_pub;
static std_msgs__msg__Bool s_status_msg;

/* ==========================================================================
 * NVS Parameter Storage
 * ========================================================================== */

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

/* ==========================================================================
 * LED Control
 * ========================================================================== */

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
    if (s_led_on) {
        uint8_t r = (s_params.red * s_params.brightness) / 255;
        uint8_t g = (s_params.green * s_params.brightness) / 255;
        uint8_t b = (s_params.blue * s_params.brightness) / 255;
        led_strip_set_pixel(s_led, 0, r, g, b);
    } else {
        led_strip_clear(s_led);
    }
    led_strip_refresh(s_led);
}

static void led_set(bool on) {
    s_led_on = on;
    led_update();
}

/* ==========================================================================
 * Time Synchronization
 * ========================================================================== */

/**
 * @brief Synchronize system time with micro-ROS agent
 *
 * Uses rmw_uros_sync_session() to get NTP-like time from the agent,
 * then sets the ESP32 system clock via settimeofday().
 *
 * @return true if sync successful
 */
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

/**
 * @brief Get current ROS2-compatible timestamp
 *
 * If time is synced, returns wall clock time from gettimeofday().
 * Otherwise returns monotonic time from esp_timer (for message stamps).
 *
 * @param sec Output: seconds
 * @param nanosec Output: nanoseconds
 */
static void get_timestamp(int32_t *sec, uint32_t *nanosec) {
    if (s_time_synced) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        *sec = (int32_t)tv.tv_sec;
        *nanosec = (uint32_t)(tv.tv_usec * 1000);
    } else {
        /* Fallback to monotonic time */
        int64_t us = esp_timer_get_time();
        *sec = (int32_t)(us / 1000000);
        *nanosec = (uint32_t)((us % 1000000) * 1000);
    }
}

/* ==========================================================================
 * ROS Callbacks
 * ========================================================================== */

static bool on_param_changed(const rcl_interfaces__msg__Parameter *old, const rcl_interfaces__msg__Parameter *new, void *ctx) {
    (void)old;
    (void)ctx;
    if (!new || new->value.type != RCLC_PARAMETER_INT)
        return false;

    const char *name = new->name.data;
    int32_t val = (int32_t) new->value.integer_value;
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
        return false;
    }

    ESP_LOGI(TAG, "%s=%ld (saved=%s)", name, val, err == ESP_OK ? "yes" : "no");
    led_update();
    return err == ESP_OK;
}

static void on_led_control(const void *msg) {
    led_set(((const std_msgs__msg__Bool *)msg)->data);
    s_state_msg.data = s_led_on;
    rcl_publish(&s_state_pub, &s_state_msg, NULL);
    ESP_LOGI(TAG, "LED %s", s_led_on ? "ON" : "OFF");
}

static void on_status_timer(rcl_timer_t *timer, int64_t last) {
    (void)timer;
    (void)last;
    s_status_msg.data = s_led_on;
    rcl_publish(&s_status_pub, &s_status_msg, NULL);
}

/* ==========================================================================
 * micro-ROS Setup
 * ========================================================================== */

static void microros_init(void) {
    RCCHECK(rclc_support_init(&s_support, 0, NULL, &s_allocator));
    RCCHECK(rclc_node_init_default(&s_node, "esp32c6_led", "", &s_support));

    /* Subscription: LED control */
    RCCHECK(rclc_subscription_init_default(&s_led_sub, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/led_control"));

    /* Publisher: LED state (reliable) */
    RCCHECK(rclc_publisher_init_default(&s_state_pub, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/led_state"));

    /* Publisher: Status (best effort for telemetry) */
    RCCHECK(rclc_publisher_init_best_effort(&s_status_pub, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/robot_status"));

    /* Timer: periodic status */
    RCCHECK(rclc_timer_init_default2(&s_status_timer, &s_support, RCL_MS_TO_NS(STATUS_PERIOD_MS), on_status_timer, true));

    /* Parameter server */
    RCCHECK(rclc_parameter_server_init_default(&s_param_server, &s_node));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_BRIGHTNESS, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_BRIGHTNESS, s_params.brightness));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_RED, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_RED, s_params.red));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_GREEN, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_GREEN, s_params.green));
    RCCHECK(rclc_add_parameter(&s_param_server, PARAM_BLUE, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_param_server, PARAM_BLUE, s_params.blue));

    /* Executor */
    s_executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&s_executor, &s_support.context, EXECUTOR_HANDLES, &s_allocator));
    RCCHECK(rclc_executor_add_subscription(&s_executor, &s_led_sub, &s_led_msg, on_led_control, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&s_executor, &s_status_timer));
    RCCHECK(rclc_executor_add_parameter_server(&s_executor, &s_param_server, on_param_changed));

    ESP_LOGI(TAG, "micro-ROS initialized");
}

/* ==========================================================================
 * Main Task
 * ========================================================================== */

static void microros_task(void *arg) {
    (void)arg;

    /* Wait for BLE */
    ESP_LOGI(TAG, "Waiting for BLE...");
    while (!s_ble_ctx.connected)
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "BLE connected (MTU=%d)", s_ble_ctx.mtu_size);
    vTaskDelay(pdMS_TO_TICKS(1000));

    s_allocator = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(true, &s_ble_ctx, ble_transport_open, ble_transport_close, ble_transport_write, ble_transport_read);

    /* Wait for agent */
    ESP_LOGI(TAG, "Waiting for agent...");
    while (rmw_uros_ping_agent(5000, 1) != RMW_RET_OK) {
        if (!s_ble_ctx.connected) {
            while (!s_ble_ctx.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Agent connected");

    /* Initial time synchronization */
    ESP_LOGI(TAG, "Synchronizing time...");
    for (int i = 0; i < 3 && !s_time_synced; i++) {
        if (time_sync_with_agent())
            break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (!s_time_synced) {
        ESP_LOGW(TAG, "Time sync failed, using monotonic time");
    }

    microros_init();

    ESP_LOGI(TAG, "Ready - /led_control, /led_state, /robot_status");
    ESP_LOGI(TAG, "Params: %s, %s, %s, %s", PARAM_BRIGHTNESS, PARAM_RED, PARAM_GREEN, PARAM_BLUE);

    /* Main loop */
    while (true) {
        if (!s_ble_ctx.connected) {
            ESP_LOGW(TAG, "BLE disconnected");
            s_time_synced = false; /* Invalidate time on disconnect */
            while (!s_ble_ctx.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
            vTaskDelay(pdMS_TO_TICKS(500));
            /* Re-sync after reconnect */
            time_sync_with_agent();
        }

        /* Periodic time re-sync */
        int64_t now = esp_timer_get_time();
        if (s_time_synced && (now - s_last_sync_time) > (TIME_RESYNC_PERIOD_MS * 1000LL)) {
            time_sync_with_agent();
        }

        rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ==========================================================================
 * Entry Point
 * ========================================================================== */

void app_main(void) {
    ESP_LOGI(TAG, "micro-ROS BLE LED Controller");

    if (params_init() != ESP_OK) {
        ESP_LOGE(TAG, "Params init failed");
        return;
    }

    if (!led_init()) {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }

    /* Boot blink */
    for (int i = 0; i < 3; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_set(false);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    if (!microros_ble_init(&s_ble_ctx)) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    xTaskCreate(microros_task, "microros", 16384, NULL, 5, NULL);
}
