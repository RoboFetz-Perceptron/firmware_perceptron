#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

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
#define BATTERY_DIVIDER_RATIO 4.0f  // (30k + 10k) / 10k

#define MOTOR_MCPWM_GROUP_ID 0
#define MOTOR_MCPWM_RESOLUTION_HZ 10000000
#define MOTOR_MCPWM_FREQ_HZ 25000

#define WEAPON_LEDC_TIMER LEDC_TIMER_0
#define WEAPON_LEDC_CHANNEL LEDC_CHANNEL_0
#define WEAPON_LEDC_FREQ_HZ 50
#define WEAPON_LEDC_RESOLUTION LEDC_TIMER_14_BIT

#define ROBOT_RADIUS_M 0.1f

esp_err_t robot_init(void);
void robot_set_enabled(bool enabled);
void robot_set_reversed(int motor_idx, bool reversed);
void robot_update(const geometry_msgs__msg__Twist *twist,
                  uint8_t weapon_duty, bool is_flipped);
float robot_read_battery_voltage(void);

#endif
