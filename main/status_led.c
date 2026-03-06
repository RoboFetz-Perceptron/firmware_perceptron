#include "include/status_led.h"

#include <esp_err.h>
#include <led_strip.h>

static led_strip_handle_t s_led;

bool status_led_init(void) {
    led_strip_config_t cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
    };
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_led) != ESP_OK)
        return false;
    led_strip_clear(s_led);
    return true;
}

void status_led_set(led_status_t status) {
    if (!s_led)
        return;
    switch (status) {
    case LED_STATUS_BOOT:
        led_strip_set_pixel(s_led, 0, 32, 32, 32);
        break;
    case LED_STATUS_BLE_WAITING:
        led_strip_set_pixel(s_led, 0, 0, 0, 40);
        break;
    case LED_STATUS_AGENT_WAITING:
        led_strip_set_pixel(s_led, 0, 0, 0, 15);
        break;
    case LED_STATUS_CONNECTED:
        led_strip_set_pixel(s_led, 0, 0, 40, 0);
        break;
    case LED_STATUS_ESTOP:
        led_strip_set_pixel(s_led, 0, 40, 20, 0);
        break;
    case LED_STATUS_ERROR:
        led_strip_set_pixel(s_led, 0, 40, 0, 0);
        break;
    case LED_STATUS_OFF:
        led_strip_clear(s_led);
        break;
    }
    led_strip_refresh(s_led);
}
