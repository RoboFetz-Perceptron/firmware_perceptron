#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8.h>

#include "include/ble_transport.h"
#include "include/am32.h"
#include "include/control.h"
#include "include/ros_node.h"
#include "include/status_led.h"

#define TAG "PERCEPTRON"

#define CONTROLLER_PERIOD_MS CONFIG_PERCEPTRON_CONTROLLER_PERIOD_MS
#define CMD_VEL_TIMEOUT_MS CONFIG_PERCEPTRON_CMD_VEL_TIMEOUT_MS
#define WEAPON_ARM_DELAY_MS CONFIG_PERCEPTRON_WEAPON_ARM_DELAY_MS
#define BATTERY_PUB_PERIOD_MS CONFIG_PERCEPTRON_BATTERY_PUB_PERIOD_MS
#define TIME_RESYNC_PERIOD_MS CONFIG_PERCEPTRON_TIME_RESYNC_PERIOD_MS
typedef enum {
    CAL_IDLE = 0,
    CAL_AM32_START,
} cal_state_t;

static ble_transport_ctx_t s_ble;
static ros_queues_t s_queues;

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

    while (true) {
        if (xQueueReceive(s_queues.cmd_vel, &twist, 0) == pdTRUE)
            last_cmd_vel_time = esp_timer_get_time();

        // We stop motors if no cmd_vel received recently
        bool cmd_vel_timed_out = (esp_timer_get_time() - last_cmd_vel_time) > (CMD_VEL_TIMEOUT_MS * 1000LL);

        xQueuePeek(s_queues.weapon, &weapon, 0);
        xQueuePeek(s_queues.flipped, &flipped, 0);

        bool cal_req;
        if (cal_state == CAL_IDLE && xQueueReceive(s_queues.calibrate, &cal_req, 0) == pdTRUE) {
            cal_state = CAL_AM32_START;
            control_set_enabled(false);
            ESP_LOGI(TAG, "AM32 ESC config: starting");
        }

        if (cal_state != CAL_IDLE) {
            switch (cal_state) {
            case CAL_AM32_START: {
                vTaskDelay(pdMS_TO_TICKS(500));
                control_weapon_ledc_deinit();

                am32_settings_t s = am32_get_desired_settings();
                esp_err_t err = am32_configure(&s);

                control_weapon_ledc_init();
                control_set_enabled(false);
                estop_active = true;
                status_led_set(LED_STATUS_ESTOP);
                cal_state = CAL_IDLE;
                if (err == ESP_OK)
                    ESP_LOGI(TAG, "AM32 ESC config: OK");
                else
                    ESP_LOGE(TAG, "AM32 ESC config: FAILED (%s)", esp_err_to_name(err));
                break;
            }
            default:
                break;
            }
        } else {
            if (xQueuePeek(s_queues.estop, &estop, 0) == pdTRUE) {
                if (estop.data && !estop_active) {
                    control_set_enabled(false);
                    status_led_set(LED_STATUS_ESTOP);
                    ESP_LOGI(TAG, "E-STOP activated");
                    estop_active = true;
                } else if (!estop.data && estop_active) {
                    xQueueReset(s_queues.weapon);
                    weapon.data = 0;
                    control_set_enabled(true);
                    estop_release_time = esp_timer_get_time();
                    status_led_set(LED_STATUS_CONNECTED);
                    ESP_LOGI(TAG, "E-STOP released, weapon armed in %ds", WEAPON_ARM_DELAY_MS / 1000);
                    estop_active = false;
                }
            }

            if (!estop_active) {
                // Delay weapon arming after estop release for safety
                bool weapon_armed = (esp_timer_get_time() - estop_release_time) > (WEAPON_ARM_DELAY_MS * 1000LL);
                uint8_t wpn = weapon_armed ? weapon.data : 0;
                if (cmd_vel_timed_out)
                    control_update(&zero_twist, wpn, flipped.data);
                else
                    control_update(&twist, wpn, flipped.data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROLLER_PERIOD_MS));
    }
}

static void microros_task(void *arg) {
    (void)arg;

    status_led_set(LED_STATUS_BLE_WAITING);
    while (!s_ble.connected)
        vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(1000));

    rmw_uros_set_custom_transport(true, &s_ble, ble_transport_open, ble_transport_close, ble_transport_write, ble_transport_read);

    status_led_set(LED_STATUS_AGENT_WAITING);
    while (rmw_uros_ping_agent(5000, 1) != RMW_RET_OK) {
        if (!s_ble.connected) {
            status_led_set(LED_STATUS_BLE_WAITING);
            while (!s_ble.connected)
                vTaskDelay(pdMS_TO_TICKS(500));
            status_led_set(LED_STATUS_AGENT_WAITING);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    if (!ros_node_init(&s_queues)) {
        ESP_LOGE(TAG, "ROS init failed");
        status_led_set(LED_STATUS_ERROR);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    for (int i = 0; i < 3 && !ros_node_time_synced(); i++) {
        if (ros_node_time_sync())
            break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    status_led_set(LED_STATUS_ESTOP);
    ESP_LOGI(TAG, "Connected, waiting for estop release");

    int64_t last_battery_pub = 0;

    while (true) {
        if (!s_ble.connected) {
            ESP_LOGI(TAG, "BLE disconnected, restarting");
            control_set_enabled(false);
            status_led_set(LED_STATUS_ERROR);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }

        int64_t now = esp_timer_get_time();
        if (ros_node_time_synced() && (now - ros_node_last_sync_time()) > (TIME_RESYNC_PERIOD_MS * 1000LL))
            ros_node_time_sync();

        if ((now - last_battery_pub) > (BATTERY_PUB_PERIOD_MS * 1000LL)) {
            ros_node_publish_battery(control_read_battery_voltage());
            last_battery_pub = now;
        }

        ros_node_spin();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!status_led_init()) {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }
    for (int i = 0; i < 3; i++) {
        status_led_set(LED_STATUS_BOOT);
        vTaskDelay(pdMS_TO_TICKS(150));
        status_led_set(LED_STATUS_OFF);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    if (control_init() != ESP_OK) {
        status_led_set(LED_STATUS_ERROR);
        return;
    }

    s_queues.cmd_vel = xQueueCreate(1, sizeof(geometry_msgs__msg__Twist));
    s_queues.weapon = xQueueCreate(1, sizeof(std_msgs__msg__UInt8));
    s_queues.flipped = xQueueCreate(1, sizeof(std_msgs__msg__Bool));
    s_queues.estop = xQueueCreate(1, sizeof(std_msgs__msg__Bool));
    s_queues.calibrate = xQueueCreate(1, sizeof(bool));
    if (!s_queues.cmd_vel || !s_queues.weapon || !s_queues.flipped || !s_queues.estop || !s_queues.calibrate) {
        status_led_set(LED_STATUS_ERROR);
        return;
    }

    if (!microros_ble_init(&s_ble)) {
        status_led_set(LED_STATUS_ERROR);
        return;
    }

    xTaskCreate(microros_task, "microros", 16384, NULL, 5, NULL);
    xTaskCreate(controller_task, "controller", 4096, NULL, 4, NULL);
}
