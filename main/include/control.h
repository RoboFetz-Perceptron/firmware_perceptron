#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>
#include <geometry_msgs/msg/twist.h>

#define MOTOR1_IN1_GPIO 3
#define MOTOR1_IN2_GPIO 2
#define MOTOR2_IN1_GPIO 21
#define MOTOR2_IN2_GPIO 22
#define MOTOR3_IN1_GPIO 23
#define MOTOR3_IN2_GPIO 15

#define WEAPON_PWM_GPIO 0
#define MOTOR_ENABLE_GPIO 13
#define BATTERY_ADC_GPIO 1
#define BATTERY_DIVIDER_RATIO 4.0f // (30k + 10k) / 10k

#define MOTOR_MCPWM_GROUP_ID 0
#define MOTOR_MCPWM_RESOLUTION_HZ 10000000
#define MOTOR_MCPWM_FREQ_HZ 25000

#define WEAPON_LEDC_TIMER LEDC_TIMER_0
#define WEAPON_LEDC_CHANNEL LEDC_CHANNEL_0
#define WEAPON_LEDC_FREQ_HZ 50
#define WEAPON_LEDC_RESOLUTION LEDC_TIMER_14_BIT

// ESC servo pulse defaults in microseconds (bidirectional: 1000-2000µs)
#define WEAPON_PULSE_MIN_US_DEFAULT 1000
#define WEAPON_PULSE_MAX_US_DEFAULT 2000
// Convert microseconds to LEDC ticks (ceiling to never undershoot)
#define WEAPON_PERIOD_US (1000000 / WEAPON_LEDC_FREQ_HZ)
#define WEAPON_US_TO_TICKS(us) (((us) * (1 << WEAPON_LEDC_RESOLUTION) + WEAPON_PERIOD_US - 1) / WEAPON_PERIOD_US)

#define ROBOT_RADIUS_M 0.1f

esp_err_t control_init(void);
void control_set_enabled(bool enabled);
void control_set_reversed(int motor_idx, bool reversed);
void control_set_speed_pct(int motor_idx, int pct);
void control_set_weapon_pulse_range(uint32_t min_us, uint32_t max_us);
void control_update(const geometry_msgs__msg__Twist *twist, uint8_t weapon_duty, bool is_flipped);
float control_read_battery_voltage(void);
void control_weapon_ledc_deinit(void);
void control_weapon_ledc_init(void);

#endif
