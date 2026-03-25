#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include <esp_err.h>
#include <geometry_msgs/msg/twist.h>

#define NUM_MOTORS 3

// Motor H-bridge GPIO pins
#define MOTOR1_IN1_GPIO 23
#define MOTOR1_IN2_GPIO 15
#define MOTOR2_IN1_GPIO 3
#define MOTOR2_IN2_GPIO 2
#define MOTOR3_IN1_GPIO 21
#define MOTOR3_IN2_GPIO 22

// Encoder GPIO pins (quadrature A/B per motor)
#define MOTOR1_ENC_A_GPIO 4
#define MOTOR1_ENC_B_GPIO 5
#define MOTOR2_ENC_A_GPIO 19
#define MOTOR2_ENC_B_GPIO 20
#define MOTOR3_ENC_A_GPIO 18
#define MOTOR3_ENC_B_GPIO 11

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

// Physical constants from Kconfig (integer mm -> float m)
#define ROBOT_RADIUS_M (CONFIG_PERCEPTRON_ROBOT_RADIUS_MM / 1000.0f)
#define WHEEL_RADIUS_M (CONFIG_PERCEPTRON_WHEEL_RADIUS_MM / 1000.0f)
#define ENCODER_CPR CONFIG_PERCEPTRON_ENCODER_CPR

// PID output range matches nominal battery voltage
#define PID_NOMINAL_VOLTAGE 12.0f

esp_err_t control_init(void);
void control_set_enabled(bool enabled);
void control_update(const geometry_msgs__msg__Twist *twist, uint8_t weapon_duty, bool is_flipped);
float control_read_battery_voltage(void);
void control_weapon_ledc_deinit(void);
void control_weapon_ledc_init(void);
void control_set_max_motor_hz(int motor_idx, uint32_t hz);
void control_set_reversed(int motor_idx, bool reversed);
void control_set_pid_gains(float kp, float ki);
void control_pid_reset(void);
void control_update_battery_voltage(void);
void control_get_measured_rps(float out_rps[NUM_MOTORS]);

#endif
