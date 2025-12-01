#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <esp_err.h>

typedef struct {
    float delta_pos_l; // delta position of the left rear wheel [rad]
    float delta_pos_r; // delta position of the right rear wheel [rad]
    float vel_l;       // current velocity of the left rear wheel [rad/s]
    float vel_r;       // current velocity of the right rear wheel [rad/s]
} forward_kinematics_input_t;

typedef struct {
    float pos_x;    // chassis X position [m]
    float pos_y;    // chassis Y position [m]
    float qx;       // rotation quaternion x
    float qy;       // rotation quaternion y
    float qz;       // rotation quaternion z
    float qw;       // rotation quaternion w
    float vel_x;    // body-frame forward speed [m/s]
    float omega_z;  // body-frame yaw rate [rad/s]
} forward_kinematics_output_t;

typedef struct {
    float vel_x;    // desired chassis forward speed [m/s]
    float omega_z;  // desired chassis yaw rate [rad/s]
} inverse_kinematics_input_t;

typedef struct {
    float vel_l;  // left rear wheel omega [rad/s]
    float vel_r;  // right rear wheel omega [rad/s]
    float steer;  // front steer angle [rad]
} inverse_kinematics_output_t;

/**
 * @brief Forward kinematics calculation for an ackermann steering vehicle
 *
 * @param input Wheel positions and velocities
 * @param output Chassis pose and velocities (updated in place)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pointers are NULL,
 *                   ESP_ERR_INVALID_STATE if parameters are not available
 */
esp_err_t forward_kinematics(const forward_kinematics_input_t *input, forward_kinematics_output_t *output);

/**
 * @brief Inverse kinematics calculation for an ackermann steering vehicle
 *
 * The calculations for steering angle are based on differential drive kinematics
 * of the back wheels -> steering angle follows the back wheels
 *
 * @param input Desired chassis forward speed and yaw rate
 * @param output Wheel velocities and steering angle
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if pointers are NULL,
 *                   ESP_ERR_INVALID_STATE if parameters are not available
 */
esp_err_t inverse_kinematics(const inverse_kinematics_input_t *input, inverse_kinematics_output_t *output);

#endif // KINEMATICS_H