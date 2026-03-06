#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>

#define LED_GPIO 8

typedef enum {
    LED_STATUS_BOOT,
    LED_STATUS_BLE_WAITING,
    LED_STATUS_AGENT_WAITING,
    LED_STATUS_CONNECTED,
    LED_STATUS_ESTOP,
    LED_STATUS_ERROR,
    LED_STATUS_OFF,
} led_status_t;

bool status_led_init(void);
void status_led_set(led_status_t status);

#endif
