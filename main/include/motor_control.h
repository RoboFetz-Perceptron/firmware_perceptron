#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <esp_err.h>
#include <perceptron_msgs/msg/rc_car_command.h>

#define MOTOR1_IN1_GPIO 3
#define MOTOR1_IN2_GPIO 2
#define MOTOR2_IN1_GPIO 21
#define MOTOR2_IN2_GPIO 22
#define MOTOR3_IN1_GPIO 23
#define MOTOR3_IN2_GPIO 15

#define WEAPON_PWM_GPIO 0
#define MOTOR_ENABLE_GPIO 13

#define MOTOR_MCPWM_GROUP_ID 0
#define MOTOR_MCPWM_RESOLUTION_HZ 10000000
#define MOTOR_MCPWM_FREQ_HZ 25000

#define WEAPON_LEDC_TIMER LEDC_TIMER_0
#define WEAPON_LEDC_CHANNEL LEDC_CHANNEL_0
#define WEAPON_LEDC_FREQ_HZ 50
#define WEAPON_LEDC_RESOLUTION LEDC_TIMER_14_BIT

#define ROBOT_RADIUS_M 0.1f

esp_err_t motor_control_init(void);
void motor_control_set_enabled(bool enabled);
void motor_control_update(const perceptron_msgs__msg__RcCarCommand *cmd);

#endif
