#include <string.h>
#include <sys/time.h>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <led_strip.h>
#include <nvs_flash.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <perceptron_msgs/msg/rc_car_command.h>
#include <std_msgs/msg/bool.h>

#include "include/ble_transport.h"
#include "include/motor_control.h"

#define TAG "RCCAR"
#define LED_GPIO 8

#define NODE_NAME "esp32c6_rccar"
#define NODE_NAMESPACE "rc_car"
#define TOPIC_CMD "cmd"
#define TOPIC_ESTOP "estop"

#define PARAM_MAX_SPEED "max_speed"
#define DEFAULT_MAX_SPEED 100

#define SPIN_TIMEOUT_MS 100
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_RESYNC_PERIOD_MS 30000
#define CONTROLLER_PERIOD_MS 10

// param server handles + cmd sub + estop sub
#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2)

#define RCCHECK(fn)                                                            \
    do {                                                                       \
        rcl_ret_t rc = fn;                                                     \
        if (rc != RCL_RET_OK) {                                                \
            ESP_LOGE(TAG, "FAIL %s rc=%d [%s:%d]", #fn, (int)rc,               \
                     __FILE__, __LINE__);                                       \
            return;                                                            \
        }                                                                      \
    } while (0)

typedef enum {
    LED_BOOT,
    LED_BLE_WAITING,
    LED_AGENT_WAITING,
    LED_CONNECTED,
    LED_ESTOP,
    LED_ERROR,
    LED_OFF,
} led_status_t;

static led_strip_handle_t s_led;
static ble_transport_ctx_t s_ble;
static QueueHandle_t s_cmd_queue;
static int32_t s_max_speed = DEFAULT_MAX_SPEED;
static bool s_time_synced;
static int64_t s_last_sync;

static rcl_allocator_t s_alloc;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_exec;
static rclc_parameter_server_t s_params;
static rcl_subscription_t s_sub_cmd;
static rcl_subscription_t s_sub_estop;
static perceptron_msgs__msg__RcCarCommand s_msg_cmd;
static std_msgs__msg__Bool s_msg_estop;

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
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_led) != ESP_OK)
        return false;
    led_strip_clear(s_led);
    return true;
}

static void led_status_set(led_status_t s) {
    if (!s_led)
        return;
    switch (s) {
    case LED_BOOT:          led_strip_set_pixel(s_led, 0, 32, 32, 32); break;
    case LED_BLE_WAITING:   led_strip_set_pixel(s_led, 0, 0, 0, 40);  break;
    case LED_AGENT_WAITING: led_strip_set_pixel(s_led, 0, 0, 0, 15);  break;
    case LED_CONNECTED:     led_strip_set_pixel(s_led, 0, 0, 40, 0);  break;
    case LED_ESTOP:         led_strip_set_pixel(s_led, 0, 40, 0, 0);  break;
    case LED_ERROR:         led_strip_set_pixel(s_led, 0, 40, 0, 0);  break;
    case LED_OFF:           led_strip_clear(s_led);                    break;
    }
    led_strip_refresh(s_led);
}

static bool time_sync(void) {
    if (rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS) != RMW_RET_OK)
        return false;
    if (!rmw_uros_epoch_synchronized())
        return false;

    int64_t ns = rmw_uros_epoch_nanos();
    struct timeval tv = {
        .tv_sec = ns / 1000000000LL,
        .tv_usec = (ns % 1000000000LL) / 1000LL,
    };
    if (settimeofday(&tv, NULL) != 0)
        return false;

    s_time_synced = true;
    s_last_sync = esp_timer_get_time();
    return true;
}

static void cb_cmd(const void *msg) {
    xQueueOverwrite(s_cmd_queue, msg);
}

static void cb_estop(const void *msg) {
    const std_msgs__msg__Bool *estop = msg;
    if (estop->data) {
        motor_control_set_enabled(false);
        led_status_set(LED_ESTOP);
        ESP_LOGW(TAG, "E-STOP activated");
    } else {
        motor_control_set_enabled(true);
        led_status_set(LED_CONNECTED);
        ESP_LOGI(TAG, "E-STOP released");
    }
}

static bool on_param_changed(const Parameter *old_p, const Parameter *new_p,
                             void *ctx) {
    (void)ctx;
    if (!old_p && !new_p) return false;
    if (!old_p) return true;
    if (!new_p) return false;
    if (strcmp(new_p->name.data, PARAM_MAX_SPEED) == 0 &&
        new_p->value.type == RCLC_PARAMETER_INT) {
        s_max_speed = (int32_t)new_p->value.integer_value;
        ESP_LOGI(TAG, "%s = %ld", PARAM_MAX_SPEED, s_max_speed);
        return true;
    }
    return false;
}

static void controller_task(void *arg) {
    (void)arg;
    perceptron_msgs__msg__RcCarCommand cmd = {0};

    while (true) {
        xQueuePeek(s_cmd_queue, &cmd, pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
        motor_control_update(&cmd);
        vTaskDelay(pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
    }
}

static void microros_init(void) {
    RCCHECK(rclc_support_init(&s_support, 0, NULL, &s_alloc));

    s_node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&s_node, NODE_NAME, NODE_NAMESPACE,
                                   &s_support));

    RCCHECK(rclc_parameter_server_init_default(&s_params, &s_node));

    s_exec = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&s_exec, &s_support.context, EXECUTOR_HANDLES,
                               &s_alloc));
    RCCHECK(rclc_executor_add_parameter_server(&s_exec, &s_params,
                                               on_param_changed));

    RCCHECK(rclc_add_parameter(&s_params, PARAM_MAX_SPEED,
                               RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_params, PARAM_MAX_SPEED, s_max_speed));

    s_sub_cmd = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(
        &s_sub_cmd, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(perceptron_msgs, msg, RcCarCommand),
        TOPIC_CMD));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_cmd, &s_msg_cmd,
                                           cb_cmd, ON_NEW_DATA));

    s_sub_estop = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(
        &s_sub_estop, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_ESTOP));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_estop, &s_msg_estop,
                                           cb_estop, ON_NEW_DATA));

    ESP_LOGI(TAG, "micro-ROS ready: /%s/%s", NODE_NAMESPACE, NODE_NAME);
}

static void microros_task(void *arg) {
    (void)arg;

    led_status_set(LED_BLE_WAITING);
    while (!s_ble.connected)
        vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(1000));

    s_alloc = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(true, &s_ble, ble_transport_open,
                                  ble_transport_close, ble_transport_write,
                                  ble_transport_read);

    led_status_set(LED_AGENT_WAITING);
    while (rmw_uros_ping_agent(5000, 1) != RMW_RET_OK) {
        if (!s_ble.connected) {
            led_status_set(LED_BLE_WAITING);
            while (!s_ble.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
            led_status_set(LED_AGENT_WAITING);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    microros_init();

    for (int i = 0; i < 3 && !s_time_synced; i++) {
        if (time_sync()) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    motor_control_set_enabled(true);
    led_status_set(LED_CONNECTED);

    while (true) {
        if (!s_ble.connected) {
            motor_control_set_enabled(false);
            led_status_set(LED_ERROR);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }

        int64_t now = esp_timer_get_time();
        if (s_time_synced &&
            (now - s_last_sync) > (TIME_RESYNC_PERIOD_MS * 1000LL))
            time_sync();

        rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!led_init()) {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }
    for (int i = 0; i < 3; i++) {
        led_status_set(LED_BOOT);
        vTaskDelay(pdMS_TO_TICKS(150));
        led_status_set(LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    if (motor_control_init() != ESP_OK) {
        led_status_set(LED_ERROR);
        return;
    }

    s_cmd_queue = xQueueCreate(1, sizeof(perceptron_msgs__msg__RcCarCommand));
    if (!s_cmd_queue) {
        led_status_set(LED_ERROR);
        return;
    }

    if (!microros_ble_init(&s_ble)) {
        led_status_set(LED_ERROR);
        return;
    }

    xTaskCreate(microros_task, "microros", 16384, NULL, 5, NULL);
    xTaskCreate(controller_task, "controller", 4096, NULL, 4, NULL);
}
