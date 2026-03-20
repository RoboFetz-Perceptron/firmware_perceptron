#ifndef AM32_H
#define AM32_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool direction_reversed;
    bool bidirectional_mode;
    bool brake_on_stop;
    uint8_t motor_kv;      // EEPROM encoding: actual_kv = (motor_kv * 40) + 20
    uint8_t motor_poles;   // Number of magnetic poles (count magnets on bell)
    uint8_t startup_power; // Startup kick strength (50-150, default 100)
    bool auto_advance;     // Dynamic timing advance based on throttle
} am32_settings_t;

// Store desired AM32 settings (called from ROS param callbacks)
void am32_set_desired_settings(const am32_settings_t *settings);

// Retrieve current desired settings (called before calibration)
am32_settings_t am32_get_desired_settings(void);

// Run the full AM32 bootloader configuration sequence.
// Caller must:
//   1. Disable motors (control_set_enabled(false)) and wait for power-down
//   2. Deinit weapon LEDC (control_weapon_ledc_deinit())
// This function will:
//   1. Drive signal pin HIGH, power on ESC into bootloader
//   2. Read EEPROM, apply settings, write back if changed
//   3. Boot ESC into main firmware, power off ESC
// Caller must re-init weapon LEDC after return.
esp_err_t am32_configure(const am32_settings_t *settings);

#endif // AM32_H
