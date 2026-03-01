#include "include/motor_control.h"

#include <math.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>

#define TAG "MOTOR"
#define NUM_MOTORS 3
#define SQRT3_2 0.866025403784f
#define PERIOD_TICKS (MOTOR_MCPWM_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ)

static mcpwm_timer_handle_t s_timers[NUM_MOTORS];
static mcpwm_oper_handle_t s_operators[NUM_MOTORS];
static mcpwm_cmpr_handle_t s_comparators[NUM_MOTORS];
static mcpwm_gen_handle_t s_gen_in1[NUM_MOTORS];
static mcpwm_gen_handle_t s_gen_in2[NUM_MOTORS];

static const int s_in1_pins[] = {MOTOR1_IN1_GPIO, MOTOR2_IN1_GPIO, MOTOR3_IN1_GPIO};
static const int s_in2_pins[] = {MOTOR1_IN2_GPIO, MOTOR2_IN2_GPIO, MOTOR3_IN2_GPIO};

static void set_motor(int idx, float speed) {
    float clamped = fmaxf(-1.0f, fminf(1.0f, speed));
    uint32_t duty = (uint32_t)(fabsf(clamped) * PERIOD_TICKS);

    mcpwm_comparator_set_compare_value(s_comparators[idx], duty);

    if (clamped > 0.001f) {
        mcpwm_generator_set_force_level(s_gen_in1[idx], -1, true);
        mcpwm_generator_set_force_level(s_gen_in2[idx], 0, true);
    } else if (clamped < -0.001f) {
        mcpwm_generator_set_force_level(s_gen_in1[idx], 0, true);
        mcpwm_generator_set_force_level(s_gen_in2[idx], -1, true);
    } else {
        mcpwm_generator_set_force_level(s_gen_in1[idx], 0, true);
        mcpwm_generator_set_force_level(s_gen_in2[idx], 0, true);
    }
}

esp_err_t motor_control_init(void) {
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    gpio_set_level(MOTOR_ENABLE_GPIO, 0);

    for (int i = 0; i < NUM_MOTORS; i++) {
        mcpwm_timer_config_t timer_cfg = {
            .group_id = MOTOR_MCPWM_GROUP_ID,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = MOTOR_MCPWM_RESOLUTION_HZ,
            .period_ticks = PERIOD_TICKS,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &s_timers[i]));

        mcpwm_operator_config_t oper_cfg = {.group_id = MOTOR_MCPWM_GROUP_ID};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &s_operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_operators[i], s_timers[i]));

        mcpwm_comparator_config_t cmpr_cfg = {.flags.update_cmp_on_tez = true};
        ESP_ERROR_CHECK(mcpwm_new_comparator(s_operators[i], &cmpr_cfg, &s_comparators[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_comparators[i], 0));

        mcpwm_generator_config_t gen1_cfg = {.gen_gpio_num = s_in1_pins[i]};
        ESP_ERROR_CHECK(mcpwm_new_generator(s_operators[i], &gen1_cfg, &s_gen_in1[i]));

        mcpwm_generator_config_t gen2_cfg = {.gen_gpio_num = s_in2_pins[i]};
        ESP_ERROR_CHECK(mcpwm_new_generator(s_operators[i], &gen2_cfg, &s_gen_in2[i]));

        // PWM pattern: high at period start, low at compare match
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_in1[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_in1[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparators[i], MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_in2[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_in2[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparators[i], MCPWM_GEN_ACTION_LOW)));

        // Start with both forced low
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_in1[i], 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_in2[i], 0, true));

        ESP_ERROR_CHECK(mcpwm_timer_enable(s_timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timers[i], MCPWM_TIMER_START_NO_STOP));
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = WEAPON_LEDC_TIMER,
        .duty_resolution = WEAPON_LEDC_RESOLUTION,
        .freq_hz = WEAPON_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = WEAPON_LEDC_CHANNEL,
        .timer_sel = WEAPON_LEDC_TIMER,
        .gpio_num = WEAPON_PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));

    ESP_LOGI(TAG, "Initialized: M1(%d,%d) M2(%d,%d) M3(%d,%d) wpn(%d) en(%d)",
             MOTOR1_IN1_GPIO, MOTOR1_IN2_GPIO,
             MOTOR2_IN1_GPIO, MOTOR2_IN2_GPIO,
             MOTOR3_IN1_GPIO, MOTOR3_IN2_GPIO,
             WEAPON_PWM_GPIO, MOTOR_ENABLE_GPIO);
    return ESP_OK;
}

void motor_control_set_enabled(bool enabled) {
    if (!enabled) {
        for (int i = 0; i < NUM_MOTORS; i++)
            set_motor(i, 0.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL);
    }
    gpio_set_level(MOTOR_ENABLE_GPIO, enabled ? 1 : 0);
    ESP_LOGI(TAG, "Power %s", enabled ? "enabled" : "disabled");
}

/*
 * Inverse kinematics: robot velocity (vx, vy, omega) -> wheel speeds (u0, u1, u2)
 *
 * u_i = -vx * sin(phi_i) + vy * cos(phi_i) + R * omega
 *
 * | u0 |   | -sqrt(3)/2    1/2   R | | vx    |
 * | u1 | = |  0           -1     R | | vy    |
 * | u2 |   |  sqrt(3)/2    1/2   R | | omega |
 *
 * Forward kinematics (unused, for future odometry):
 *
 * | vx    |   | -sqrt(3)/3     0       sqrt(3)/3 | | u0 |
 * | vy    | = |  1/3          -2/3      1/3      | | u1 |
 * | omega |   |  1/(3R)        1/(3R)   1/(3R)   | | u2 |
 */
void motor_control_update(const perceptron_msgs__msg__RcCarCommand *cmd) {
    float vx = cmd->cmd_vel.linear.x;
    float vy = cmd->cmd_vel.linear.y;
    float omega = cmd->cmd_vel.angular.z;

    float u[3];
    u[0] = -SQRT3_2 * vx + 0.5f * vy + ROBOT_RADIUS_M * omega;
    u[1] =                 -1.0f * vy + ROBOT_RADIUS_M * omega;
    u[2] =  SQRT3_2 * vx + 0.5f * vy + ROBOT_RADIUS_M * omega;

    // Normalize: if any wheel exceeds 1.0, scale all down proportionally
    float max_u = fmaxf(fmaxf(fabsf(u[0]), fabsf(u[1])), fabsf(u[2]));
    if (max_u > 1.0f) {
        float scale = 1.0f / max_u;
        u[0] *= scale;
        u[1] *= scale;
        u[2] *= scale;
    }

    for (int i = 0; i < NUM_MOTORS; i++)
        set_motor(i, u[i]);

    // Weapon motor: duty 0-255 mapped to LEDC resolution
    uint32_t wpn_duty = (cmd->weapon_duty * ((1 << WEAPON_LEDC_RESOLUTION) - 1)) / 255;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL, wpn_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL);

    ESP_LOGD(TAG, "vx=%.2f vy=%.2f w=%.2f -> u[%.2f,%.2f,%.2f] wpn=%u",
             vx, vy, omega, u[0], u[1], u[2], cmd->weapon_duty);
}
