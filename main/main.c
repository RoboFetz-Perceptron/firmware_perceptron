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
#include <nvs.h>
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
#include <std_srvs/srv/trigger.h>

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
#define SRV_CALIBRATE "calibrate_esc"

#define BATTERY_PUB_PERIOD_MS 5000

#define PARAM_MOTOR1_REVERSED "motor1_reversed"
#define PARAM_MOTOR2_REVERSED "motor2_reversed"
#define PARAM_MOTOR3_REVERSED "motor3_reversed"
#define PARAM_WPN_PULSE_MIN "wpn_pulse_min"
#define PARAM_WPN_PULSE_MAX "wpn_pulse_max"
#define NVS_NAMESPACE "robot_params"

#define SPIN_TIMEOUT_MS 100
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_RESYNC_PERIOD_MS 30000
#define CONTROLLER_PERIOD_MS 10
#define CMD_VEL_TIMEOUT_MS 500
#define WEAPON_ARM_DELAY_MS 3000
#define CAL_POWER_OFF_MS 2000
#define CAL_MAX_HOLD_MS 5000
#define CAL_MIN_HOLD_MS 5000
#define LOG_MSG_MAX_LEN 192
#define LOG_QUEUE_DEPTH 8

#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 5)

#define RCCHECK(fn)                                                                                                                                                                                                        \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t rc = fn;                                                                                                                                                                                                 \
        if (rc != RCL_RET_OK) {                                                                                                                                                                                            \
            ESP_LOGE(TAG, "FAIL %s rc=%d [%s:%d]", #fn, (int)rc, __FILE__, __LINE__);                                                                                                                                      \
            return false;                                                                                                                                                                                                  \
        }                                                                                                                                                                                                                  \
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
static QueueHandle_t s_q_calibrate;

typedef enum {
    CAL_IDLE = 0,
    CAL_POWER_OFF,
    CAL_POWER_ON_MAX,
    CAL_SEND_MIN,
} cal_state_t;

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

static rcl_service_t s_srv_calibrate;
static std_srvs__srv__Trigger_Request s_req_calibrate;
static std_srvs__srv__Trigger_Response s_res_calibrate;
static char s_cal_msg_buf[48];

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
    case LED_BOOT:
        led_strip_set_pixel(s_led, 0, 32, 32, 32);
        break;
    case LED_BLE_WAITING:
        led_strip_set_pixel(s_led, 0, 0, 0, 40);
        break;
    case LED_AGENT_WAITING:
        led_strip_set_pixel(s_led, 0, 0, 0, 15);
        break;
    case LED_CONNECTED:
        led_strip_set_pixel(s_led, 0, 0, 40, 0);
        break;
    case LED_ESTOP:
        led_strip_set_pixel(s_led, 0, 40, 20, 0);
        break;
    case LED_ERROR:
        led_strip_set_pixel(s_led, 0, 40, 0, 0);
        break;
    case LED_OFF:
        led_strip_clear(s_led);
        break;
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
        rcl_ret_t rc = rcl_publish(&s_pub_log, &s_msg_log, NULL);
        if (rc != RCL_RET_OK)
            ESP_LOGW(TAG, "roslog publish failed: %d", (int)rc);
    }
}

static void cb_cmd_vel(const void *msg) { xQueueOverwrite(s_q_cmd_vel, msg); }

static void cb_weapon(const void *msg) { xQueueOverwrite(s_q_weapon, msg); }

static void cb_flipped(const void *msg) { xQueueOverwrite(s_q_flipped, msg); }

static void cb_estop(const void *msg) { xQueueOverwrite(s_q_estop, msg); }

static void cb_calibrate(const void *req, void *res) {
    (void)req;
    std_srvs__srv__Trigger_Response *r = (std_srvs__srv__Trigger_Response *)res;
    bool trigger = true;
    r->success = (xQueueSend(s_q_calibrate, &trigger, 0) == pdTRUE);
    const char *m = r->success ? "Calibration started" : "Calibration busy";
    strncpy(r->message.data, m, r->message.capacity - 1);
    r->message.size = strlen(r->message.data);
}

static const char *s_motor_param_names[] = {
    PARAM_MOTOR1_REVERSED,
    PARAM_MOTOR2_REVERSED,
    PARAM_MOTOR3_REVERSED,
};

static bool nvs_load_bool(const char *key, bool default_val) {
    nvs_handle_t h;
    uint8_t val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_u8(h, key, &val) == ESP_OK)
            default_val = (val != 0);
        nvs_close(h);
    }
    return default_val;
}

static void nvs_save_bool(const char *key, bool val) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, key, val ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static int32_t nvs_load_i32(const char *key, int32_t default_val) {
    nvs_handle_t h;
    int32_t val;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_i32(h, key, &val) == ESP_OK)
            default_val = val;
        nvs_close(h);
    }
    return default_val;
}

static void nvs_save_i32(const char *key, int32_t val) {
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, key, val);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool on_param_changed(const Parameter *old_p, const Parameter *new_p, void *ctx) {
    (void)ctx;
    if (!old_p && !new_p)
        return false;
    if (!old_p)
        return true;
    if (!new_p)
        return false;
    if (new_p->value.type == RCLC_PARAMETER_BOOL) {
        for (int i = 0; i < 3; i++) {
            if (strcmp(new_p->name.data, s_motor_param_names[i]) == 0) {
                robot_set_reversed(i, new_p->value.bool_value);
                nvs_save_bool(s_motor_param_names[i], new_p->value.bool_value);
                roslog("M%d reversed=%d", i + 1, new_p->value.bool_value);
                return true;
            }
        }
    }
    if (new_p->value.type == RCLC_PARAMETER_INT) {
        int64_t val = new_p->value.integer_value;
        bool is_min = strcmp(new_p->name.data, PARAM_WPN_PULSE_MIN) == 0;
        bool is_max = strcmp(new_p->name.data, PARAM_WPN_PULSE_MAX) == 0;
        if (is_min || is_max) {
            if (val < 500 || val > 3000)
                return false;
            nvs_save_i32(new_p->name.data, (int32_t)val);
            int64_t other;
            if (is_min) {
                rclc_parameter_get_int(&s_params, PARAM_WPN_PULSE_MAX, &other);
                robot_set_weapon_pulse_range((uint32_t)val, (uint32_t)other);
            } else {
                rclc_parameter_get_int(&s_params, PARAM_WPN_PULSE_MIN, &other);
                robot_set_weapon_pulse_range((uint32_t)other, (uint32_t)val);
            }
            roslog("Weapon pulse %s=%d", new_p->name.data, (int)val);
            return true;
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
    int64_t estop_release_time = 0;
    int64_t last_cmd_vel_time = 0;
    cal_state_t cal_state = CAL_IDLE;
    int64_t cal_timer = 0;

    while (true) {
        // cmd_vel: consume from queue, track timeout
        if (xQueueReceive(s_q_cmd_vel, &twist, 0) == pdTRUE)
            last_cmd_vel_time = esp_timer_get_time();

        bool cmd_vel_timed_out = (esp_timer_get_time() - last_cmd_vel_time) > (CMD_VEL_TIMEOUT_MS * 1000LL);

        xQueuePeek(s_q_weapon, &weapon, 0);
        xQueuePeek(s_q_flipped, &flipped, 0);

        // Check for calibration request
        bool cal_req;
        if (cal_state == CAL_IDLE &&
            xQueueReceive(s_q_calibrate, &cal_req, 0) == pdTRUE) {
            cal_state = CAL_POWER_OFF;
            cal_timer = esp_timer_get_time();
            robot_set_enabled(false);
            roslog("ESC cal: power off, sending max");
        }

        // ESC calibration state machine (overrides normal control)
        if (cal_state != CAL_IDLE) {
            int64_t elapsed = esp_timer_get_time() - cal_timer;

            switch (cal_state) {
            case CAL_POWER_OFF:
                robot_update(&zero_twist, 255, false);
                if (elapsed > CAL_POWER_OFF_MS * 1000LL) {
                    cal_state = CAL_POWER_ON_MAX;
                    cal_timer = esp_timer_get_time();
                    robot_set_enabled(true);
                    roslog("ESC cal: power on (max throttle)");
                }
                break;
            case CAL_POWER_ON_MAX:
                robot_update(&zero_twist, 255, false);
                if (elapsed > CAL_MAX_HOLD_MS * 1000LL) {
                    cal_state = CAL_SEND_MIN;
                    cal_timer = esp_timer_get_time();
                    roslog("ESC cal: sending min throttle");
                }
                break;
            case CAL_SEND_MIN:
                robot_update(&zero_twist, 0, false);
                if (elapsed > CAL_MIN_HOLD_MS * 1000LL) {
                    cal_state = CAL_IDLE;
                    robot_set_enabled(false);
                    estop_active = true;
                    led_status_set(LED_ESTOP);
                    roslog("ESC cal: done");
                }
                break;
            default:
                break;
            }
        } else {
            // Normal estop handling
            if (xQueuePeek(s_q_estop, &estop, 0) == pdTRUE) {
                if (estop.data && !estop_active) {
                    robot_set_enabled(false);
                    led_status_set(LED_ESTOP);
                    roslog("E-STOP activated");
                    estop_active = true;
                } else if (!estop.data && estop_active) {
                    xQueueReset(s_q_weapon);
                    weapon.data = 0;
                    robot_set_enabled(true);
                    estop_release_time = esp_timer_get_time();
                    led_status_set(LED_CONNECTED);
                    roslog("E-STOP released, weapon armed in %ds", WEAPON_ARM_DELAY_MS / 1000);
                    estop_active = false;
                }
            }

            if (!estop_active) {
                bool weapon_armed = (esp_timer_get_time() - estop_release_time) > (WEAPON_ARM_DELAY_MS * 1000LL);
                uint8_t wpn = weapon_armed ? weapon.data : 0;
                if (cmd_vel_timed_out)
                    robot_update(&zero_twist, wpn, flipped.data);
                else
                    robot_update(&twist, wpn, flipped.data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
    }
}

static bool microros_init(void) {
    RCCHECK(rclc_support_init(&s_support, 0, NULL, &s_alloc));

    s_node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&s_node, NODE_NAME, NODE_NAMESPACE, &s_support));

    const rclc_parameter_options_t param_opts = {
        .notify_changed_over_dds = true,
        .max_params = 5,
        .allow_undeclared_parameters = false,
        .low_mem_mode = false,
    };
    RCCHECK(rclc_parameter_server_init_with_option(&s_params, &s_node, &param_opts));

    s_exec = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&s_exec, &s_support.context, EXECUTOR_HANDLES, &s_alloc));
    RCCHECK(rclc_executor_add_parameter_server(&s_exec, &s_params, on_param_changed));

    for (int i = 0; i < 3; i++) {
        bool saved = nvs_load_bool(s_motor_param_names[i], false);
        RCCHECK(rclc_add_parameter(&s_params, s_motor_param_names[i], RCLC_PARAMETER_BOOL));
        RCCHECK(rclc_parameter_set_bool(&s_params, s_motor_param_names[i], saved));
        robot_set_reversed(i, saved);
    }

    int32_t wpn_min = nvs_load_i32(PARAM_WPN_PULSE_MIN, WEAPON_PULSE_MIN_US_DEFAULT);
    int32_t wpn_max = nvs_load_i32(PARAM_WPN_PULSE_MAX, WEAPON_PULSE_MAX_US_DEFAULT);
    RCCHECK(rclc_add_parameter(&s_params, PARAM_WPN_PULSE_MIN, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_params, PARAM_WPN_PULSE_MIN, wpn_min));
    RCCHECK(rclc_add_parameter(&s_params, PARAM_WPN_PULSE_MAX, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_params, PARAM_WPN_PULSE_MAX, wpn_max));
    robot_set_weapon_pulse_range((uint32_t)wpn_min, (uint32_t)wpn_max);

    s_sub_cmd_vel = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(&s_sub_cmd_vel, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_CMD_VEL));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_cmd_vel, &s_msg_cmd_vel, cb_cmd_vel, ON_NEW_DATA));

    s_sub_weapon = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_best_effort(&s_sub_weapon, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), TOPIC_WEAPON));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_weapon, &s_msg_weapon, cb_weapon, ON_NEW_DATA));

    s_sub_flipped = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(&s_sub_flipped, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_FLIPPED));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_flipped, &s_msg_flipped, cb_flipped, ON_NEW_DATA));

    s_sub_estop = rcl_get_zero_initialized_subscription();
    RCCHECK(rclc_subscription_init_default(&s_sub_estop, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_ESTOP));
    RCCHECK(rclc_executor_add_subscription(&s_exec, &s_sub_estop, &s_msg_estop, cb_estop, ON_NEW_DATA));

    s_pub_log = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(&s_pub_log, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_ROSLOG));
    s_msg_log.data.data = s_log_pub_buf;
    s_msg_log.data.size = 0;
    s_msg_log.data.capacity = sizeof(s_log_pub_buf);

    s_pub_battery = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(&s_pub_battery, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), TOPIC_BATTERY));

    s_srv_calibrate = rcl_get_zero_initialized_service();
    RCCHECK(rclc_service_init_default(&s_srv_calibrate, &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), SRV_CALIBRATE));
    s_res_calibrate.message.data = s_cal_msg_buf;
    s_res_calibrate.message.size = 0;
    s_res_calibrate.message.capacity = sizeof(s_cal_msg_buf);
    RCCHECK(rclc_executor_add_service(&s_exec, &s_srv_calibrate,
        &s_req_calibrate, &s_res_calibrate, cb_calibrate));

    ESP_LOGI(TAG, "ROS init OK");
    return true;
}

static void microros_task(void *arg) {
    (void)arg;

    led_status_set(LED_BLE_WAITING);
    while (!s_ble.connected)
        vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(1000));

    s_alloc = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(true, &s_ble, ble_transport_open, ble_transport_close, ble_transport_write, ble_transport_read);

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
        ESP_LOGE(TAG, "ROS init failed");
        led_status_set(LED_ERROR);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    for (int i = 0; i < 3 && !s_time_synced; i++) {
        if (time_sync())
            break;
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
        if (s_time_synced && (now - s_last_sync) > (TIME_RESYNC_PERIOD_MS * 1000LL))
            time_sync();

        if ((now - last_battery_pub) > (BATTERY_PUB_PERIOD_MS * 1000LL)) {
            s_msg_battery.data = robot_read_battery_voltage();
            rcl_ret_t rc = rcl_publish(&s_pub_battery, &s_msg_battery, NULL);
            if (rc != RCL_RET_OK)
                ESP_LOGW(TAG, "battery publish failed: %d", (int)rc);
            last_battery_pub = now;
        }

        rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
        roslog_drain();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
    s_q_calibrate = xQueueCreate(1, sizeof(bool));
    if (!s_q_cmd_vel || !s_q_weapon || !s_q_flipped || !s_q_estop || !s_q_log || !s_q_calibrate) {
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
