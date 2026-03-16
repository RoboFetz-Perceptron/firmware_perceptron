//! Due to time limitations, this am32 interface was vibe coded, so we cannot guarantee anything about it :(

#include "include/am32.h"

#include <string.h>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_ll.h>

#include "include/control.h"

#define TAG "AM32"

#define AM32_UART_NUM UART_NUM_1
#define AM32_BAUD_RATE 19200
#define AM32_RX_BUF_SIZE 256
#define AM32_TX_BUF_SIZE 256
#define AM32_TIMEOUT_MS 500
#define AM32_EEPROM_SIZE 48

// AM32 bootloader commands
#define CMD_RUN 0x00
#define CMD_PROG_FLASH 0x01
#define CMD_READ_FLASH_SIL 0x03
#define CMD_SET_ADDRESS 0xFF
#define CMD_SET_BUFFER 0xFE

// Magic address for EEPROM (proto >= 2 only)
#define ADDRESS_MAGIC_EEPROM 0x0020

// Device info byte indices
#define DEVINFO_FLASH_SIZE 4
#define DEVINFO_PROTO_VER 7

// Protocol responses
#define ACK_OK 0x30

// EEPROM byte offsets for settings we configure
#define EEPROM_OFF_DIR_REVERSED 17
#define EEPROM_OFF_BIDIRECTIONAL 18
#define EEPROM_OFF_COMP_PWM 20
#define EEPROM_OFF_STARTUP_POWER 25
#define EEPROM_OFF_BRAKE_ON_STOP 28

static am32_settings_t s_desired = {
    .direction_reversed = false,
#if CONFIG_PERCEPTRON_WEAPON_BIDIRECTIONAL
    .bidirectional_mode = true,
#else
    .bidirectional_mode = false,
#endif
    .brake_on_stop = true,
};

void am32_set_desired_settings(const am32_settings_t *settings) { s_desired = *settings; }

am32_settings_t am32_get_desired_settings(void) { return s_desired; }

// CRC-16 matching the AM32 bootloader (polynomial 0xA001)
static uint16_t crc16(const uint8_t *buf, uint16_t length) {
    uint16_t ret = 0;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t xb = buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (((xb & 0x01) ^ (ret & 0x0001)) != 0) {
                ret >>= 1;
                ret ^= 0xA001;
            } else {
                ret >>= 1;
            }
            xb >>= 1;
        }
    }
    return ret;
}

static void append_crc(uint8_t *buf, uint16_t payload_len) {
    uint16_t crc = crc16(buf, payload_len);
    buf[payload_len] = crc & 0xFF;
    buf[payload_len + 1] = (crc >> 8) & 0xFF;
}

static esp_err_t uart_init(void) {
    uart_config_t cfg = {
        .baud_rate = AM32_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(AM32_UART_NUM, AM32_RX_BUF_SIZE, AM32_TX_BUF_SIZE, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(AM32_UART_NUM, &cfg), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(AM32_UART_NUM, WEAPON_PWM_GPIO, WEAPON_PWM_GPIO, -1, -1), TAG, "uart pin");
    // Open-drain via LL keeps the UART GPIO matrix intact while allowing
    // the ESC to pull the shared line LOW for its responses.
    gpio_ll_od_enable(&GPIO, WEAPON_PWM_GPIO);
    gpio_pullup_en(WEAPON_PWM_GPIO);
    return ESP_OK;
}

static void uart_deinit(void) {
    uart_driver_delete(AM32_UART_NUM);
    gpio_reset_pin(WEAPON_PWM_GPIO);
}

// Send data and discard exactly `len` loopback bytes from RX
static esp_err_t am32_send(const uint8_t *data, size_t len) {
    int written = uart_write_bytes(AM32_UART_NUM, data, len);
    if (written != (int)len)
        return ESP_ERR_INVALID_SIZE;
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(AM32_UART_NUM, pdMS_TO_TICKS(AM32_TIMEOUT_MS)), TAG, "tx wait");
    uint8_t discard[len];
    uart_read_bytes(AM32_UART_NUM, discard, len, pdMS_TO_TICKS(AM32_TIMEOUT_MS));
    return ESP_OK;
}

static esp_err_t am32_recv(uint8_t *buf, size_t len) {
    int got = uart_read_bytes(AM32_UART_NUM, buf, len, pdMS_TO_TICKS(AM32_TIMEOUT_MS));
    if (got != (int)len) {
        ESP_LOGE(TAG, "recv: expected %d, got %d", (int)len, got);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t am32_wait_ack(void) {
    uint8_t ack;
    ESP_RETURN_ON_ERROR(am32_recv(&ack, 1), TAG, "ack recv");
    if (ack != ACK_OK) {
        ESP_LOGE(TAG, "bootloader: bad ack 0x%02x", ack);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

// Device info handshake: send magic bytes, read 9-byte response
static esp_err_t am32_get_device_info(uint8_t *info, size_t info_len) {
    uint8_t buf[17] = {0};
    buf[8] = 0x0D;
    buf[9] = 0x42;
    buf[16] = 0x7D;

    ESP_RETURN_ON_ERROR(am32_send(buf, sizeof(buf)), TAG, "devinfo send");
    return am32_recv(info, info_len);
}

static esp_err_t am32_set_address(uint16_t addr) {
    uint8_t cmd[6];
    cmd[0] = CMD_SET_ADDRESS;
    cmd[1] = 0x00;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;
    append_crc(cmd, 4);

    ESP_RETURN_ON_ERROR(am32_send(cmd, sizeof(cmd)), TAG, "setaddr send");
    return am32_wait_ack();
}

static esp_err_t am32_read_flash(uint8_t *out, size_t len) {
    uint8_t cmd[4];
    cmd[0] = CMD_READ_FLASH_SIL;
    cmd[1] = (uint8_t)len;
    append_crc(cmd, 2);

    ESP_RETURN_ON_ERROR(am32_send(cmd, sizeof(cmd)), TAG, "readflash send");

    uint8_t resp[len + 3];
    ESP_RETURN_ON_ERROR(am32_recv(resp, len + 3), TAG, "readflash recv");

    if (resp[len + 2] != ACK_OK) {
        ESP_LOGE(TAG, "readflash: bad ack 0x%02x", resp[len + 2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t resp_crc = resp[len] | (resp[len + 1] << 8);
    if (crc16(resp, len) != resp_crc) {
        ESP_LOGE(TAG, "readflash: CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    memcpy(out, resp, len);
    return ESP_OK;
}

static esp_err_t am32_write_flash(const uint8_t *data, size_t len) {
    // CMD_SET_BUFFER (no ACK)
    uint8_t set_buf[6];
    set_buf[0] = CMD_SET_BUFFER;
    set_buf[1] = 0x00;
    set_buf[2] = (len == 256) ? 0x01 : 0x00;
    set_buf[3] = (len == 256) ? 0x00 : (uint8_t)len;
    append_crc(set_buf, 4);
    ESP_RETURN_ON_ERROR(am32_send(set_buf, sizeof(set_buf)), TAG, "setbuf send");

    // Payload + CRC → ACK
    uint8_t payload[len + 2];
    memcpy(payload, data, len);
    append_crc(payload, len);
    ESP_RETURN_ON_ERROR(am32_send(payload, len + 2), TAG, "payload send");
    ESP_RETURN_ON_ERROR(am32_wait_ack(), TAG, "payload ack");

    // CMD_PROG_FLASH → ACK
    uint8_t prog[4];
    prog[0] = CMD_PROG_FLASH;
    prog[1] = 0x00;
    append_crc(prog, 2);
    ESP_RETURN_ON_ERROR(am32_send(prog, sizeof(prog)), TAG, "prog send");
    return am32_wait_ack();
}

static esp_err_t am32_send_run(void) {
    uint8_t cmd[4] = {CMD_RUN, 0x00, 0x00, 0x00};
    return am32_send(cmd, sizeof(cmd));
}

// Resolve EEPROM address based on protocol version and flash size
static esp_err_t am32_resolve_eeprom_addr(const uint8_t *devinfo, uint16_t *out_addr) {
    uint8_t proto = devinfo[DEVINFO_PROTO_VER];
    if (proto >= 2) {
        *out_addr = ADDRESS_MAGIC_EEPROM;
        return ESP_OK;
    }
    // Proto 1: calculate direct offset from flash size code
    uint8_t flash_code = devinfo[DEVINFO_FLASH_SIZE];
    uint32_t flash_kb;
    if (flash_code == 0x1f)
        flash_kb = 32;
    else if (flash_code == 0x35)
        flash_kb = 64;
    else if (flash_code == 0x2B)
        flash_kb = 128;
    else {
        ESP_LOGE(TAG, "unknown flash size code 0x%02x", flash_code);
        return ESP_ERR_NOT_SUPPORTED;
    }
    *out_addr = (uint16_t)(flash_kb * 1024 - 1024);
    return ESP_OK;
}

esp_err_t am32_configure(const am32_settings_t *settings) {
    esp_err_t ret;

    // UART TX idle = HIGH, which is what the bootloader needs on the signal pin
    ret = uart_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Power on ESC — bootloader sees HIGH signal pin and stays resident.
    // checkForSignal() takes ~50ms, then bootloader has a 20ms serial timeout.
    gpio_set_level(MOTOR_ENABLE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    uint8_t devinfo[9];
    ret = am32_get_device_info(devinfo, sizeof(devinfo));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to get device info");
        goto cleanup;
    }
    ESP_LOGI(TAG, "device: %02x%02x%02x flash=0x%02x proto=%d", devinfo[0], devinfo[1], devinfo[2], devinfo[4], devinfo[7]);

    uint16_t eeprom_addr;
    ret = am32_resolve_eeprom_addr(devinfo, &eeprom_addr);
    if (ret != ESP_OK)
        goto cleanup;

    ret = am32_set_address(eeprom_addr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set address failed");
        goto cleanup;
    }

    uint8_t eeprom[AM32_EEPROM_SIZE];
    ret = am32_read_flash(eeprom, AM32_EEPROM_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EEPROM read failed");
        goto cleanup;
    }
    ESP_LOGI(TAG, "EEPROM: ver=%d.%d name=%.12s dir=%d bidir=%d comp_pwm=%d brake=%d", eeprom[3], eeprom[4], (char *)&eeprom[5], eeprom[EEPROM_OFF_DIR_REVERSED], eeprom[EEPROM_OFF_BIDIRECTIONAL], eeprom[EEPROM_OFF_COMP_PWM], eeprom[EEPROM_OFF_BRAKE_ON_STOP]);

    uint8_t new_dir = settings->direction_reversed ? 1 : 0;
    uint8_t new_bidir = settings->bidirectional_mode ? 1 : 0;
    uint8_t new_comp_pwm = settings->bidirectional_mode ? 1 : 0; // comp_pwm required for bidirectional
    uint8_t new_brake = settings->brake_on_stop ? 1 : 0;

    bool needs_write = (eeprom[EEPROM_OFF_DIR_REVERSED] != new_dir) || (eeprom[EEPROM_OFF_BIDIRECTIONAL] != new_bidir) || (eeprom[EEPROM_OFF_COMP_PWM] != new_comp_pwm) || (eeprom[EEPROM_OFF_BRAKE_ON_STOP] != new_brake);

    if (needs_write) {
        eeprom[EEPROM_OFF_DIR_REVERSED] = new_dir;
        eeprom[EEPROM_OFF_BIDIRECTIONAL] = new_bidir;
        eeprom[EEPROM_OFF_COMP_PWM] = new_comp_pwm;
        eeprom[EEPROM_OFF_BRAKE_ON_STOP] = new_brake;

        ret = am32_set_address(eeprom_addr);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "set address (write) failed");
            goto cleanup;
        }

        ret = am32_write_flash(eeprom, AM32_EEPROM_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "EEPROM write failed");
            goto cleanup;
        }
        ESP_LOGI(TAG, "EEPROM updated: dir=%d bidir=%d comp_pwm=%d brake=%d", new_dir, new_bidir, new_comp_pwm, new_brake);
    } else {
        ESP_LOGI(TAG, "EEPROM already up to date");
    }

    am32_send_run();
    ret = ESP_OK;

cleanup:
    uart_deinit();
    gpio_set_level(MOTOR_ENABLE_GPIO, 0);
    return ret;
}
