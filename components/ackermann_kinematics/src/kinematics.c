#include "kinematics.h"
#include "math_utils.h"

#include <math.h>

static float last_steer = 0.0f;

esp_err_t forward_kinematics(const ackermann_params_t *params,
                             const forward_kinematics_input_t *input,
                             forward_kinematics_output_t *output) {
    if (!params || !input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1) Get parameters (already in meters)
    float wheel_radius_m = params->wheel_radius_m;
    float wheel_base_m = params->wheel_base_m;  // L

    // 2) Compute linear rear-wheel displacements:
    //    delta_s_l = in_delta_pos_l * r
    //    delta_s_r = in_delta_pos_r * r
    float delta_pos_l_m = input->delta_pos_l * wheel_radius_m;
    float delta_pos_r_m = input->delta_pos_r * wheel_radius_m;
    // Midpoint rear travel:
    float delta_s = 0.5f * (delta_pos_l_m + delta_pos_r_m);

    // 3) Compute delta_θ from front-steer (bicycle-model):
    //    delta_θ = delta_s * (tan d) / L
    float delta_theta;
    float tan_delta = tanf(last_steer);
    if (fabsf(tan_delta) < 1e-6f) {
        // Treat as straight → no yaw change
        delta_theta = 0.0f;
    } else {
        delta_theta = delta_s * (tan_delta / wheel_base_m);
    }

    // 4) Unpack previous yaw from the quaternion:
    float roll, pitch, yaw;
    math_quaternion_to_euler(output->qx, output->qy, output->qz, output->qw, &roll, &pitch, &yaw);

    // 5) Midpoint heading for XY integration:
    float mid_yaw = yaw + 0.5f * delta_theta;
    output->pos_x += delta_s * cosf(mid_yaw);
    output->pos_y += delta_s * sinf(mid_yaw);

    // 6) Compute new yaw and pack back into quaternion:
    float new_yaw = math_normalize_angle(yaw + delta_theta);
    math_euler_to_quaternion(0.0f, 0.0f, new_yaw, &output->qx, &output->qy, &output->qz, &output->qw);

    // 7) Body-frame velocities:
    //    v_l = in_vel_l * r, v_r = in_vel_r * r
    float v_l = input->vel_l * wheel_radius_m;
    float v_r = input->vel_r * wheel_radius_m;
    //    v_CB = (v_l + v_r)/2:
    float v_CB = 0.5f * (v_l + v_r);
    //    omega_chassis = v_CB * tan(d) / L
    float omega_chassis;
    if (fabsf(tan_delta) < 1e-6f) {
        omega_chassis = 0.0f;
    } else {
        omega_chassis = v_CB * (tan_delta / wheel_base_m);
    }

    output->vel_x = v_CB;
    output->omega_z = omega_chassis;

    return ESP_OK;
}

esp_err_t inverse_kinematics(const ackermann_params_t *params,
                             const inverse_kinematics_input_t *input,
                             inverse_kinematics_output_t *output) {
    if (!params || !input || !output) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1) Get parameters (already in meters)
    float wheel_radius_m = params->wheel_radius_m;
    float track_width_m = params->track_width_m;  // b
    float wheel_base_m = params->wheel_base_m;    // L

    // 2) Compute d = atan(L * omega / v):
    float d;
    if (fabsf(input->vel_x) < 1e-6f) {
        // Allow steering even when the vehicle is stationary by mapping the
        // desired yaw rate directly to a steering angle. For small yaw rates
        // this approximates d ~ L * omega.
        d = atanf(wheel_base_m * input->omega_z);
        output->vel_l = 0.0f;
        output->vel_r = 0.0f;
    } else {
        d = atanf((wheel_base_m * input->omega_z) / input->vel_x);
    }
    output->steer = d;
    last_steer = d;

    // 3) Compute turning radius R = L / tan(d):
    float tan_delta = tanf(d);
    float R_CB;
    if (fabsf(tan_delta) < 1e-6f) {
        // Near straight:
        R_CB = 1e9f; // effectively infinite
    } else {
        R_CB = wheel_base_m / tan_delta;
    }

    // 4) Compute rear-wheel linear speeds:
    //    v_l = in_vel_x * (1 − b/(2 R_CB))
    //    v_r = in_vel_x * (1 + b/(2 R_CB))
    float v_l_lin, v_r_lin;
    if (fabsf(input->omega_z) < 1e-6f || fabsf(tan_delta) < 1e-6f) {
        // Straight line:
        v_l_lin = input->vel_x;
        v_r_lin = input->vel_x;
    } else {
        float ratio = (track_width_m / (2.0f * R_CB));
        v_l_lin = input->vel_x * (1.0f - ratio);
        v_r_lin = input->vel_x * (1.0f + ratio);
    }

    // 5) Convert to wheel angular speeds [rad/s]:
    output->vel_l = v_l_lin / wheel_radius_m;
    output->vel_r = v_r_lin / wheel_radius_m;

    return ESP_OK;
}
