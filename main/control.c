#include "include/control.h"

#include <math.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/mcpwm_prelude.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

#define TAG "PERCEPTRON"

#define SQRT3_2 0.866025403784f
#define PERIOD_TICKS (MOTOR_MCPWM_RESOLUTION_HZ / MOTOR_MCPWM_FREQ_HZ)

// MCPWM handles (per motor)
static mcpwm_timer_handle_t s_timers[NUM_MOTORS];
static mcpwm_oper_handle_t s_operators[NUM_MOTORS];
static mcpwm_cmpr_handle_t s_comparators[NUM_MOTORS];
static mcpwm_gen_handle_t s_gen_in1[NUM_MOTORS];
static mcpwm_gen_handle_t s_gen_in2[NUM_MOTORS];

// Motor config
static bool s_motor_reversed[NUM_MOTORS] = {false, false, false};
static float s_min_duty = 0.0f; // minimum duty cycle (0.0-1.0) to overcome static friction
static uint32_t s_weapon_pulse_min_us = WEAPON_PULSE_MIN_US_DEFAULT;
static uint32_t s_weapon_pulse_max_us = WEAPON_PULSE_MAX_US_DEFAULT;
static float s_max_motor_rps[NUM_MOTORS] = {
    CONFIG_PERCEPTRON_MAX_MOTOR_HZ,
    CONFIG_PERCEPTRON_MAX_MOTOR_HZ,
    CONFIG_PERCEPTRON_MAX_MOTOR_HZ,
};

// ADC (battery voltage monitoring)
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_handle;

// Pin tables
static const int s_in1_pins[] = {MOTOR1_IN1_GPIO, MOTOR2_IN1_GPIO, MOTOR3_IN1_GPIO};
static const int s_in2_pins[] = {MOTOR1_IN2_GPIO, MOTOR2_IN2_GPIO, MOTOR3_IN2_GPIO};

// Set motor duty: speed -1.0 (full reverse) to +1.0 (full forward), 0.0 = brake
// motor_reversed flips direction for physically reversed motors
static void set_motor(int idx, float speed) {
    float clamped = fmaxf(-1.0f, fminf(1.0f, speed));
    if (s_motor_reversed[idx])
        clamped = -clamped;
    uint32_t duty = (uint32_t)(fabsf(clamped) * PERIOD_TICKS);

    mcpwm_comparator_set_compare_value(s_comparators[idx], duty);

    if (clamped > 0.001f) {
        mcpwm_generator_set_force_level(s_gen_in1[idx], -1, true); // IN1 = PWM
        mcpwm_generator_set_force_level(s_gen_in2[idx], 0, true);  // IN2 = LOW
    } else if (clamped < -0.001f) {
        mcpwm_generator_set_force_level(s_gen_in1[idx], 0, true);  // IN1 = LOW
        mcpwm_generator_set_force_level(s_gen_in2[idx], -1, true); // IN2 = PWM
    } else {
        mcpwm_generator_set_force_level(s_gen_in1[idx], 0, true); // brake
        mcpwm_generator_set_force_level(s_gen_in2[idx], 0, true);
    }
}

void control_weapon_ledc_init(void) {
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
}

void control_weapon_ledc_deinit(void) {
    ledc_stop(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL, 0);
    gpio_reset_pin(WEAPON_PWM_GPIO);
}

esp_err_t control_init(void) {
    // Motor driver enable pin, active high, pulled low at boot
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en_cfg));
    gpio_set_level(MOTOR_ENABLE_GPIO, 0);

    // MCPWM: one timer + operator + comparator + two generators per motor
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

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_in1[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_in1[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparators[i], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_gen_in2[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_gen_in2[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparators[i], MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_in1[i], 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(s_gen_in2[i], 0, true));

        ESP_ERROR_CHECK(mcpwm_timer_enable(s_timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_timers[i], MCPWM_TIMER_START_NO_STOP));
    }

    control_weapon_ledc_init();

    // ADC for battery voltage
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_1, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali_handle));

    ESP_LOGI(TAG, "HW init OK (open-loop)");
    return ESP_OK;
}

void control_set_enabled(bool enabled) {
    if (!enabled) {
        for (int i = 0; i < NUM_MOTORS; i++)
            set_motor(i, 0.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL);
    }
    gpio_set_level(MOTOR_ENABLE_GPIO, enabled ? 1 : 0);
}

// 3-omni-wheel inverse kinematics
//
//   Motor 1 (rear, 180°):        u0 =              -vy + R*omega
//   Motor 2 (front-right, -60°):  u1 =  sqrt3/2*vx + vy/2 + R*omega
//   Motor 3 (front-left, 60°):    u2 = -sqrt3/2*vx + vy/2 + R*omega
void control_update(const geometry_msgs__msg__Twist *twist, uint8_t weapon_duty, bool is_flipped) {
    float vx = twist->linear.x;
    float vy = twist->linear.y;
    float omega = twist->angular.z;

    if (is_flipped) {
        vx = -vx;
    }

    // Inverse kinematics -> wheel linear velocity (m/s)
    float u_ms[NUM_MOTORS];
    u_ms[0] = -1.0f * vy + ROBOT_RADIUS_M * omega;
    u_ms[1] = SQRT3_2 * vx + 0.5f * vy + ROBOT_RADIUS_M * omega;
    u_ms[2] = -SQRT3_2 * vx + 0.5f * vy + ROBOT_RADIUS_M * omega;

    // Clamp to per-motor max speed while preserving direction ratio
    // Find highest utilization ratio across all motors
    float max_ratio = 0.0f;
    for (int i = 0; i < NUM_MOTORS; i++) {
        float limit = s_max_motor_rps[i] * WHEEL_RADIUS_M * 2.0f * (float)M_PI;
        float ratio = fabsf(u_ms[i]) / limit;
        if (ratio > max_ratio)
            max_ratio = ratio;
    }
    if (max_ratio > 1.0f) {
        float scale = 1.0f / max_ratio;
        for (int i = 0; i < NUM_MOTORS; i++)
            u_ms[i] *= scale;
    }

    // Open-loop: map wheel speed to duty cycle [-1, 1] per motor
    // Apply min_duty to overcome static friction
    for (int i = 0; i < NUM_MOTORS; i++) {
        float limit = s_max_motor_rps[i] * WHEEL_RADIUS_M * 2.0f * (float)M_PI;
        float duty = u_ms[i] / limit;
        if (duty > 0.001f && duty < s_min_duty)
            duty = s_min_duty;
        else if (duty < -0.001f && duty > -s_min_duty)
            duty = -s_min_duty;
        set_motor(i, duty);
    }

#if CONFIG_PERCEPTRON_WEAPON_BIDIRECTIONAL
    // Bidirectional ESC: 1500µs=stop, forward/reverse around center
    uint32_t center = WEAPON_US_TO_TICKS(1500);
    uint32_t wpn;
    if (is_flipped) {
        uint32_t end = WEAPON_US_TO_TICKS(s_weapon_pulse_min_us);
        wpn = center - (weapon_duty * (center - end)) / 255;
    } else {
        uint32_t end = WEAPON_US_TO_TICKS(s_weapon_pulse_max_us);
        wpn = center + (weapon_duty * (end - center)) / 255;
    }
#else
    // Unidirectional ESC: full range min→max for maximum speed
    uint32_t min_ticks = WEAPON_US_TO_TICKS(s_weapon_pulse_min_us);
    uint32_t max_ticks = WEAPON_US_TO_TICKS(s_weapon_pulse_max_us);
    uint32_t wpn = min_ticks + (weapon_duty * (max_ticks - min_ticks)) / 255;
#endif
    ledc_set_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL, wpn);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, WEAPON_LEDC_CHANNEL);
}

void control_set_reversed(int motor_idx, bool reversed) {
    if (motor_idx >= 0 && motor_idx < NUM_MOTORS)
        s_motor_reversed[motor_idx] = reversed;
}

void control_set_max_motor_hz(int motor_idx, uint32_t hz) {
    if (motor_idx >= 0 && motor_idx < NUM_MOTORS)
        s_max_motor_rps[motor_idx] = (float)hz;
}

void control_set_min_duty(float duty) {
    s_min_duty = fmaxf(0.0f, fminf(1.0f, duty / 100.0f));
}

float control_read_battery_voltage(void) {
    int raw, mv;
    adc_oneshot_read(s_adc_handle, ADC_CHANNEL_1, &raw);
    adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &mv);
    return (mv / 1000.0f) * BATTERY_DIVIDER_RATIO;
}
