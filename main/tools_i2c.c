#include "tools_handlers.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I2C_SCAN_PORT                I2C_NUM_1
#define I2C_SCAN_ADDR_FIRST          0x03
#define I2C_SCAN_ADDR_LAST           0x77
#define I2C_SCAN_DEFAULT_FREQ_HZ     100000
#define I2C_SCAN_MIN_FREQ_HZ         10000
#define I2C_SCAN_MAX_FREQ_HZ         1000000
#define I2C_SCAN_ADDR_TIMEOUT_MS     25

static bool gpio_pin_in_allowlist(int pin, const char *csv)
{
    const char *cursor;

    if (!csv || csv[0] == '\0') {
        return false;
    }

    cursor = csv;
    while (*cursor != '\0') {
        char *endptr = NULL;
        long value;

        while (*cursor == ' ' || *cursor == '\t' || *cursor == ',') {
            cursor++;
        }
        if (*cursor == '\0') {
            break;
        }

        value = strtol(cursor, &endptr, 10);
        if (endptr == cursor) {
            while (*cursor != '\0' && *cursor != ',') {
                cursor++;
            }
            continue;
        }

        if ((int)value == pin) {
            return true;
        }
        cursor = endptr;
    }

    return false;
}

static bool gpio_pin_is_allowed(int pin)
{
    if (GPIO_ALLOWED_PINS_CSV[0] != '\0') {
        return gpio_pin_in_allowlist(pin, GPIO_ALLOWED_PINS_CSV);
    }
    return pin >= GPIO_MIN_PIN && pin <= GPIO_MAX_PIN;
}

static bool validate_scan_pin(const char *field_name, int pin, char *result, size_t result_len)
{
    if (!gpio_pin_is_allowed(pin)) {
        if (GPIO_ALLOWED_PINS_CSV[0] != '\0') {
            snprintf(result, result_len, "Error: %s pin %d is not in allowed list", field_name, pin);
        } else {
            snprintf(result, result_len, "Error: %s pin must be %d-%d", field_name, GPIO_MIN_PIN, GPIO_MAX_PIN);
        }
        return false;
    }
    return true;
}

bool tools_i2c_scan_handler(const cJSON *input, char *result, size_t result_len)
{
    cJSON *sda_pin_json = cJSON_GetObjectItem(input, "sda_pin");
    cJSON *scl_pin_json = cJSON_GetObjectItem(input, "scl_pin");
    cJSON *freq_json = cJSON_GetObjectItem(input, "frequency_hz");

    if (!sda_pin_json || !cJSON_IsNumber(sda_pin_json)) {
        snprintf(result, result_len, "Error: 'sda_pin' required (number)");
        return false;
    }
    if (!scl_pin_json || !cJSON_IsNumber(scl_pin_json)) {
        snprintf(result, result_len, "Error: 'scl_pin' required (number)");
        return false;
    }

    int sda_pin = sda_pin_json->valueint;
    int scl_pin = scl_pin_json->valueint;
    int frequency_hz = I2C_SCAN_DEFAULT_FREQ_HZ;

    if (freq_json) {
        if (!cJSON_IsNumber(freq_json)) {
            snprintf(result, result_len, "Error: 'frequency_hz' must be a number");
            return false;
        }
        frequency_hz = freq_json->valueint;
    }

    if (sda_pin == scl_pin) {
        snprintf(result, result_len, "Error: SDA and SCL must be different pins");
        return false;
    }
    if (!validate_scan_pin("SDA", sda_pin, result, result_len)) {
        return false;
    }
    if (!validate_scan_pin("SCL", scl_pin, result, result_len)) {
        return false;
    }
    if (frequency_hz < I2C_SCAN_MIN_FREQ_HZ || frequency_hz > I2C_SCAN_MAX_FREQ_HZ) {
        snprintf(
            result,
            result_len,
            "Error: frequency_hz must be %d-%d",
            I2C_SCAN_MIN_FREQ_HZ,
            I2C_SCAN_MAX_FREQ_HZ
        );
        return false;
    }

    // Create a temporary I2C master bus on I2C_NUM_1 (I2C_NUM_0 is used by the BSP)
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_SCAN_PORT,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        snprintf(result, result_len, "Error: i2c_new_master_bus failed (%s)", esp_err_to_name(err));
        return false;
    }

    uint8_t found_addresses[I2C_SCAN_ADDR_LAST - I2C_SCAN_ADDR_FIRST + 1];
    int found_count = 0;

    for (int addr = I2C_SCAN_ADDR_FIRST; addr <= I2C_SCAN_ADDR_LAST; addr++) {
        err = i2c_master_probe(bus_handle, addr, I2C_SCAN_ADDR_TIMEOUT_MS);
        if (err == ESP_OK && found_count < (int)(sizeof(found_addresses))) {
            found_addresses[found_count++] = (uint8_t)addr;
        }
    }

    i2c_del_master_bus(bus_handle);

    if (found_count == 0) {
        snprintf(
            result,
            result_len,
            "No I2C devices found on SDA=%d SCL=%d @ %d Hz",
            sda_pin,
            scl_pin,
            frequency_hz
        );
        return true;
    }

    size_t offset = 0;
    int written = snprintf(
        result,
        result_len,
        "Found %d I2C device(s) on SDA=%d SCL=%d @ %d Hz: ",
        found_count,
        sda_pin,
        scl_pin,
        frequency_hz
    );
    if (written < 0) {
        snprintf(result, result_len, "Found %d I2C device(s)", found_count);
        return true;
    }
    offset = (size_t)written < result_len ? (size_t)written : result_len - 1;

    int listed = 0;
    for (int i = 0; i < found_count; i++) {
        if (offset >= result_len) {
            break;
        }

        written = snprintf(
            result + offset,
            result_len - offset,
            listed == 0 ? "0x%02X" : ", 0x%02X",
            found_addresses[i]
        );
        if (written < 0 || (size_t)written >= result_len - offset) {
            break;
        }
        offset += (size_t)written;
        listed++;
    }

    if (listed < found_count && offset < result_len) {
        snprintf(result + offset, result_len - offset, " ... (+%d more)", found_count - listed);
    }

    return true;
}
