#include <stdarg.h>
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

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8.h>

#include "include/ble_transport.h"
#include "include/robot_control.h"

#define TAG "RCCAR"
#define LED_GPIO 8

#define NODE_NAME "esp32c6_rccar"
#define NODE_NAMESPACE "rc_car"
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_WEAPON "weapon_duty"
#define TOPIC_FLIPPED "is_flipped"
#define TOPIC_ESTOP "estop"
#define TOPIC_ROSLOG "roslog"
#define TOPIC_BATTERY "battery_voltage"

#define BATTERY_PUB_PERIOD_MS 5000

#define PARAM_MOTOR1_REVERSED "motor1_reversed"
#define PARAM_MOTOR2_REVERSED "motor2_reversed"
#define PARAM_MOTOR3_REVERSED "motor3_reversed"

#define SPIN_TIMEOUT_MS 100
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_RESYNC_PERIOD_MS 30000
#define CONTROLLER_PERIOD_MS 10
#define CMD_VEL_TIMEOUT_MS 500
#define LOG_MSG_MAX_LEN 192
#define LOG_QUEUE_DEPTH 8

#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 4)

#define RCCHECK(fn)                                                            \
    do {                                                                       \
        rcl_ret_t rc = fn;                                                     \
        if (rc != RCL_RET_OK) {                                                \
            ESP_LOGE(TAG, "FAIL %s rc=%d [%s:%d]", #fn, (int)rc,               \
                     __FILE__, __LINE__);                                       \
            return false;                                                      \
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
static bool s_time_synced;
static int64_t s_last_sync;

static QueueHandle_t s_q_cmd_vel;
static QueueHandle_t s_q_weapon;
static QueueHandle_t s_q_flipped;
static QueueHandle_t s_q_estop;
static QueueHandle_t s_q_log;

static rcl_allocator_t s_alloc;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_exec;
static rclc_parameter_server_t s_params;

static rcl_publisher_t s_pub_log;
static std_msgs__msg__String s_msg_log;
static char s_log_pub_buf[LOG_MSG_MAX_LEN];

static rcl_publisher_t s_pub_battery;
static std_msgs__msg__Float32 s_msg_battery;

static rcl_subscription_t s_sub_cmd_vel;
static rcl_subscription_t s_sub_weapon;
static rcl_subscription_t s_sub_flipped;
static rcl_subscription_t s_sub_estop;

static geometry_msgs__msg__Twist s_msg_cmd_vel;
static std_msgs__msg__UInt8 s_msg_weapon;
static std_msgs__msg__Bool s_msg_flipped;
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

static void roslog(const char *fmt, ...) {
    char buf[LOG_MSG_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "%s", buf);

    if (s_q_log)
        xQueueSendToBack(s_q_log, buf, 0);
}

static void roslog_drain(void) {
    char buf[LOG_MSG_MAX_LEN];
    while (xQueueReceive(s_q_log, buf, 0) == pdTRUE) {
        s_msg_log.data.size = strlen(buf);
        memcpy(s_log_pub_buf, buf, s_msg_log.data.size + 1);
        rcl_publish(&s_pub_log, &s_msg_log, NULL);
    }
}

static void cb_cmd_vel(const void *msg) {
    xQueueOverwrite(s_q_cmd_vel, msg);
}

static void cb_weapon(const void *msg) {
    xQueueOverwrite(s_q_weapon, msg);
}

static void cb_flipped(const void *msg) {
    xQueueOverwrite(s_q_flipped, msg);
}

static void cb_estop(const void *msg) {
    xQueueOverwrite(s_q_estop, msg);
}

static const char *s_motor_param_names[] = {
    PARAM_MOTOR1_REVERSED, PARAM_MOTOR2_REVERSED, PARAM_MOTOR3_REVERSED,
};

static bool on_param_changed(const Parameter *old_p, const Parameter *new_p,
                             void *ctx) {
    (void)ctx;
    if (!old_p && !new_p) return false;
    if (!old_p) return true;
    if (!new_p) return false;
    if (new_p->value.type == RCLC_PARAMETER_BOOL) {
        for (int i = 0; i < 3; i++) {
            if (strcmp(new_p->name.data, s_motor_param_names[i]) == 0) {
                robot_set_reversed(i, new_p->value.bool_value);
                roslog("Motor %d reversed=%d", i + 1,
                       new_p->value.bool_value);
                return true;
            }
        }
    }
    return false;
}

static void controller_task(void *arg) {
    (void)arg;
    geometry_msgs__msg__Twist twist = {0};
    geometry_msgs__msg__Twist zero_twist = {0};
    std_msgs__msg__UInt8 weapon = {0};
    std_msgs__msg__Bool flipped = {0};
    std_msgs__msg__Bool estop = {0};
    bool estop_active = true;
    int64_t last_cmd_vel_time = 0;

    while (true) {
        // cmd_vel: consume from queue, track timeout
        if (xQueueReceive(s_q_cmd_vel, &twist, 0) == pdTRUE)
            last_cmd_vel_time = esp_timer_get_time();

        bool cmd_vel_timed_out =
            (esp_timer_get_time() - last_cmd_vel_time) >
            (CMD_VEL_TIMEOUT_MS * 1000LL);

        xQueuePeek(s_q_weapon, &weapon, 0);
        xQueuePeek(s_q_flipped, &flipped, 0);

        if (xQueuePeek(s_q_estop, &estop, 0) == pdTRUE) {
            if (estop.data && !estop_active) {
                robot_set_enabled(false);
                led_status_set(LED_ESTOP);
                roslog("E-STOP activated");
                estop_active = true;
            } else if (!estop.data && estop_active) {
                robot_set_enabled(true);
                led_status_set(LED_CONNECTED);
                roslog("E-STOP released");
                estop_active = false;
            }
        }

        if (!estop_active) {
            if (cmd_vel_timed_out)
                robot_update(&zero_twist, 0, flipped.data);
            else
                robot_update(&twist, weapon.data, flipped.data);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
    }
}

static bool microros_init(void) {
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

    for (int i = 0; i < 3; i++) {
        RCCHECK(rclc_add_parameter(&s_params, s_motor_param_names[i],
                                   RCLC_PARAMETER_BOOL));
        RCCHECK(rclc_parameter_set_bool(&s_params, s_motor_param_names[i],
                                        false));
    }

    s_sub_cmd_vel = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(
        &s_sub_cmd_vel, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_CMD_VEL));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_cmd_vel,
                                           &s_msg_cmd_vel, cb_cmd_vel,
                                           ON_NEW_DATA));

    s_sub_weapon = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(
        &s_sub_weapon, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        TOPIC_WEAPON));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_weapon,
                                           &s_msg_weapon, cb_weapon,
                                           ON_NEW_DATA));

    s_sub_flipped = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(
        &s_sub_flipped, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_FLIPPED));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_flipped,
                                           &s_msg_flipped, cb_flipped,
                                           ON_NEW_DATA));

    s_sub_estop = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(
        &s_sub_estop, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        TOPIC_ESTOP));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_estop,
                                           &s_msg_estop, cb_estop,
                                           ON_NEW_DATA));

    s_pub_log = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(
        &s_pub_log, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        TOPIC_ROSLOG));
    s_msg_log.data.data = s_log_pub_buf;
    s_msg_log.data.size = 0;
    s_msg_log.data.capacity = sizeof(s_log_pub_buf);

    s_pub_battery = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(
        &s_pub_battery, &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        TOPIC_BATTERY));

    ESP_LOGI(TAG, "micro-ROS ready: /%s/%s", NODE_NAMESPACE, NODE_NAME);
    return true;
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
    if (!microros_init()) {
        ESP_LOGE(TAG, "micro-ROS init failed");
        led_status_set(LED_ERROR);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    for (int i = 0; i < 3 && !s_time_synced; i++) {
        if (time_sync()) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    led_status_set(LED_ESTOP);
    roslog("Connected, waiting for estop release");

    int64_t last_battery_pub = 0;

    while (true) {
        if (!s_ble.connected) {
            roslog("BLE disconnected, restarting");
            roslog_drain();
            robot_set_enabled(false);
            led_status_set(LED_ERROR);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }

        int64_t now = esp_timer_get_time();
        if (s_time_synced &&
            (now - s_last_sync) > (TIME_RESYNC_PERIOD_MS * 1000LL))
            time_sync();

        if ((now - last_battery_pub) > (BATTERY_PUB_PERIOD_MS * 1000LL)) {
            s_msg_battery.data = robot_read_battery_voltage();
            rcl_publish(&s_pub_battery, &s_msg_battery, NULL);
            last_battery_pub = now;
        }

        rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        roslog_drain();
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

    if (robot_init() != ESP_OK) {
        led_status_set(LED_ERROR);
        return;
    }

    s_q_cmd_vel = xQueueCreate(1, sizeof(geometry_msgs__msg__Twist));
    s_q_weapon = xQueueCreate(1, sizeof(std_msgs__msg__UInt8));
    s_q_flipped = xQueueCreate(1, sizeof(std_msgs__msg__Bool));
    s_q_estop = xQueueCreate(1, sizeof(std_msgs__msg__Bool));
    s_q_log = xQueueCreate(LOG_QUEUE_DEPTH, LOG_MSG_MAX_LEN);
    if (!s_q_cmd_vel || !s_q_weapon || !s_q_flipped || !s_q_estop ||
        !s_q_log) {
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
