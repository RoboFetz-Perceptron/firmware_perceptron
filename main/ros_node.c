#include "include/ros_node.h"
#include "include/control.h"

#ifdef CONFIG_PERCEPTRON_ROSLOG
#include <stdarg.h>
#include <stdio.h>
#endif
#include <string.h>
#include <sys/time.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <nvs.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>
#ifdef CONFIG_PERCEPTRON_ROSLOG
#include <std_msgs/msg/string.h>
#endif
#include <std_srvs/srv/trigger.h>

#define TAG "PERCEPTRON"

#define NODE_NAME CONFIG_PERCEPTRON_NODE_NAME
#define NODE_NAMESPACE CONFIG_PERCEPTRON_NODE_NAMESPACE
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_WEAPON "weapon_duty"
#define TOPIC_FLIPPED "is_flipped"
#define TOPIC_ESTOP "estop"
#define TOPIC_BATTERY "battery_voltage"
#ifdef CONFIG_PERCEPTRON_ROSLOG
#define TOPIC_ROSLOG "roslog"
#endif
#define SRV_CALIBRATE "calibrate_esc"

#define PARAM_MOTOR1_REVERSED "motor1_reversed"
#define PARAM_MOTOR2_REVERSED "motor2_reversed"
#define PARAM_MOTOR3_REVERSED "motor3_reversed"
#define PARAM_WPN_PULSE_MIN "wpn_pulse_min"
#define PARAM_WPN_PULSE_MAX "wpn_pulse_max"
#define NVS_NAMESPACE "perceptron"

#define SPIN_TIMEOUT_MS 100
#ifdef CONFIG_PERCEPTRON_ROSLOG
#define LOG_MSG_MAX_LEN 192
#endif
#define TIME_SYNC_TIMEOUT_MS 1000

#define EXECUTOR_HANDLES (RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 5)

#define RCCHECK(fn)                                                                                                                                                                                                        \
    do {                                                                                                                                                                                                                   \
        rcl_ret_t rc = fn;                                                                                                                                                                                                 \
        if (rc != RCL_RET_OK) {                                                                                                                                                                                            \
            ESP_LOGE(TAG, "FAIL %s rc=%d [%s:%d]", #fn, (int)rc, __FILE__, __LINE__);                                                                                                                                      \
            return false;                                                                                                                                                                                                  \
        }                                                                                                                                                                                                                  \
    } while (0)

static bool s_connected;
static bool s_time_synced;
static int64_t s_last_sync;

#ifdef CONFIG_PERCEPTRON_ROSLOG
static QueueHandle_t s_q_log;
static vprintf_like_t s_orig_vprintf;
static bool s_draining;
#endif

static rcl_allocator_t s_alloc;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_exec;
static rclc_parameter_server_t s_params;

#ifdef CONFIG_PERCEPTRON_ROSLOG
static rcl_publisher_t s_pub_log;
static std_msgs__msg__String s_msg_log;
static char s_log_pub_buf[LOG_MSG_MAX_LEN];
#endif

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

static ros_queues_t *s_queues;

#ifdef CONFIG_PERCEPTRON_ROSLOG
// --- Log hook: forwards ESP_LOG messages with our TAG to ROS ---

static int ros_log_vprintf(const char *fmt, va_list args) {
    if (!s_connected || s_draining)
        return s_orig_vprintf(fmt, args);

    char buf[LOG_MSG_MAX_LEN];
    va_list args_copy;
    va_copy(args_copy, args);
    vsnprintf(buf, sizeof(buf), fmt, args_copy);
    va_end(args_copy);

    char *tag_pos = strstr(buf, TAG ": ");
    if (tag_pos) {
        char *msg = tag_pos + sizeof(TAG) + 1; // skip "TAG: "
        size_t len = strlen(msg);
        if (len > 0 && msg[len - 1] == '\n')
            msg[len - 1] = '\0';
        char q_buf[LOG_MSG_MAX_LEN];
        strncpy(q_buf, msg, sizeof(q_buf) - 1);
        q_buf[sizeof(q_buf) - 1] = '\0';
        xQueueSendToBack(s_q_log, q_buf, 0);
    }

    return s_orig_vprintf(fmt, args);
}

static void log_drain(void) {
    s_draining = true;
    char buf[LOG_MSG_MAX_LEN];
    while (xQueueReceive(s_q_log, buf, 0) == pdTRUE) {
        s_msg_log.data.size = strlen(buf);
        memcpy(s_log_pub_buf, buf, s_msg_log.data.size + 1);
        rcl_ret_t rc __attribute__((unused)) = rcl_publish(&s_pub_log, &s_msg_log, NULL);
    }
    s_draining = false;
}
#endif

// --- Time sync ---

bool ros_node_time_sync(void) {
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

bool ros_node_time_synced(void) { return s_time_synced; }

int64_t ros_node_last_sync_time(void) { return s_last_sync; }

// --- ROS callbacks ---

static void cb_cmd_vel(const void *msg) { xQueueOverwrite(s_queues->cmd_vel, msg); }
static void cb_weapon(const void *msg) { xQueueOverwrite(s_queues->weapon, msg); }
static void cb_flipped(const void *msg) { xQueueOverwrite(s_queues->flipped, msg); }
static void cb_estop(const void *msg) { xQueueOverwrite(s_queues->estop, msg); }

static void cb_calibrate(const void *req, void *res) {
    (void)req;
    std_srvs__srv__Trigger_Response *r = (std_srvs__srv__Trigger_Response *)res;
    bool trigger = true;
    r->success = (xQueueSend(s_queues->calibrate, &trigger, 0) == pdTRUE);
    const char *m = r->success ? "Calibration started" : "Calibration busy";
    strncpy(r->message.data, m, r->message.capacity - 1);
    r->message.size = strlen(r->message.data);
}

// --- NVS helpers ---

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

// --- Parameter callback ---

static const char *s_motor_param_names[] = {
    PARAM_MOTOR1_REVERSED,
    PARAM_MOTOR2_REVERSED,
    PARAM_MOTOR3_REVERSED,
};

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
                control_set_reversed(i, new_p->value.bool_value);
                nvs_save_bool(s_motor_param_names[i], new_p->value.bool_value);
                ESP_LOGI(TAG, "M%d reversed=%d", i + 1, new_p->value.bool_value);
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
                control_set_weapon_pulse_range((uint32_t)val, (uint32_t)other);
            } else {
                rclc_parameter_get_int(&s_params, PARAM_WPN_PULSE_MIN, &other);
                control_set_weapon_pulse_range((uint32_t)other, (uint32_t)val);
            }
            ESP_LOGI(TAG, "Weapon pulse %s=%d", new_p->name.data, (int)val);
            return true;
        }
    }
    return false;
}

// --- Public API ---

bool ros_node_init(ros_queues_t *queues) {
    s_queues = queues;

#ifdef CONFIG_PERCEPTRON_ROSLOG
    s_q_log = xQueueCreate(8, LOG_MSG_MAX_LEN);
    if (!s_q_log)
        return false;
    s_orig_vprintf = esp_log_set_vprintf(ros_log_vprintf);
#endif

    s_alloc = rcl_get_default_allocator();
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
        control_set_reversed(i, saved);
    }

    int32_t wpn_min = nvs_load_i32(PARAM_WPN_PULSE_MIN, WEAPON_PULSE_MIN_US_DEFAULT);
    int32_t wpn_max = nvs_load_i32(PARAM_WPN_PULSE_MAX, WEAPON_PULSE_MAX_US_DEFAULT);
    RCCHECK(rclc_add_parameter(&s_params, PARAM_WPN_PULSE_MIN, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_params, PARAM_WPN_PULSE_MIN, wpn_min));
    RCCHECK(rclc_add_parameter(&s_params, PARAM_WPN_PULSE_MAX, RCLC_PARAMETER_INT));
    RCCHECK(rclc_parameter_set_int(&s_params, PARAM_WPN_PULSE_MAX, wpn_max));
    control_set_weapon_pulse_range((uint32_t)wpn_min, (uint32_t)wpn_max);

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

#ifdef CONFIG_PERCEPTRON_ROSLOG
    s_pub_log = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_best_effort(&s_pub_log, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), TOPIC_ROSLOG));
    s_msg_log.data.data = s_log_pub_buf;
    s_msg_log.data.size = 0;
    s_msg_log.data.capacity = sizeof(s_log_pub_buf);
#endif

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

void ros_node_spin(void) {
    rclc_executor_spin_some(&s_exec, RCL_MS_TO_NS(SPIN_TIMEOUT_MS));
#ifdef CONFIG_PERCEPTRON_ROSLOG
    log_drain();
#endif
}

void ros_node_set_connected(bool connected) { s_connected = connected; }

void ros_node_log_drain(void) {
#ifdef CONFIG_PERCEPTRON_ROSLOG
    log_drain();
#endif
}

void ros_node_publish_battery(float voltage) {
    s_msg_battery.data = voltage;
    rcl_ret_t rc = rcl_publish(&s_pub_battery, &s_msg_battery, NULL);
    if (rc != RCL_RET_OK)
        ESP_LOGW(TAG, "battery publish failed: %d", (int)rc);
}
