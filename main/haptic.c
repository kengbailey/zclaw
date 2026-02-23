#include "haptic.h"
#include "i2c_bsp.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "haptic";

// DRV2605 registers
#define DRV2605_REG_STATUS    0x00
#define DRV2605_REG_MODE      0x01
#define DRV2605_REG_RTPIN     0x02
#define DRV2605_REG_LIBRARY   0x03
#define DRV2605_REG_WAVESEQ1  0x04
#define DRV2605_REG_WAVESEQ2  0x05
#define DRV2605_REG_GO        0x0C
#define DRV2605_REG_OVERDRIVE 0x0D
#define DRV2605_REG_SUSTAINP  0x0E
#define DRV2605_REG_SUSTAINN  0x0F
#define DRV2605_REG_BREAK     0x10
#define DRV2605_REG_AUDIOMAX  0x13
#define DRV2605_REG_FEEDBACK  0x1A
#define DRV2605_REG_CONTROL3  0x1D

#define DRV2605_MODE_INTTRIG  0x00

static bool s_initialized = false;

static esp_err_t drv_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t ret = i2c_write_buff(drv2605_dev_handle, reg, &val, 1);
    return (ret == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static esp_err_t drv_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t ret = i2c_read_buff(drv2605_dev_handle, reg, val, 1);
    return (ret == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t haptic_init(void)
{
    ESP_LOGI(TAG, "Initializing DRV2605 haptic driver");

    // Verify chip is present
    uint8_t status = 0;
    esp_err_t err = drv_read_reg(DRV2605_REG_STATUS, &status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read DRV2605 status register");
        return err;
    }
    ESP_LOGI(TAG, "DRV2605 status=0x%02x dev_id=%u", status, (status >> 5) & 0x07);

    // Exit standby — set internal trigger mode
    err = drv_write_reg(DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mode");
        return err;
    }

    drv_write_reg(DRV2605_REG_RTPIN, 0x00);
    drv_write_reg(DRV2605_REG_WAVESEQ1, 0x01);
    drv_write_reg(DRV2605_REG_WAVESEQ2, 0x00);
    drv_write_reg(DRV2605_REG_OVERDRIVE, 0x00);
    drv_write_reg(DRV2605_REG_SUSTAINP, 0x00);
    drv_write_reg(DRV2605_REG_SUSTAINN, 0x00);
    drv_write_reg(DRV2605_REG_BREAK, 0x00);
    drv_write_reg(DRV2605_REG_AUDIOMAX, 0x64);

    // Configure for ERM motor (open loop)
    uint8_t fb = 0;
    drv_read_reg(DRV2605_REG_FEEDBACK, &fb);
    drv_write_reg(DRV2605_REG_FEEDBACK, fb & 0x7F);  // Clear N_ERM_LRA bit

    uint8_t ctrl3 = 0;
    drv_read_reg(DRV2605_REG_CONTROL3, &ctrl3);
    drv_write_reg(DRV2605_REG_CONTROL3, ctrl3 | 0x20);  // ERM_OPEN_LOOP

    // Select library 1 (ERM)
    drv_write_reg(DRV2605_REG_LIBRARY, 1);

    s_initialized = true;
    ESP_LOGI(TAG, "DRV2605 initialized");
    return ESP_OK;
}

void haptic_play(haptic_effect_t effect)
{
    if (!s_initialized) return;

    uint8_t waveform;
    switch (effect) {
    case HAPTIC_CLICK:        waveform = 1;  break;  // Strong Click 100%
    case HAPTIC_SOFT_BUMP:    waveform = 7;  break;  // Soft Bump 100%
    case HAPTIC_DOUBLE_CLICK: waveform = 10; break;  // Double Click 100%
    case HAPTIC_TICK:         waveform = 6;  break;  // Sharp Click 30%
    case HAPTIC_ERROR:        waveform = 12; break;  // Triple Click 100%
    case HAPTIC_SUCCESS:      waveform = 4;  break;  // Sharp Click 100%
    default:                  waveform = 1;  break;
    }

    drv_write_reg(DRV2605_REG_WAVESEQ1, waveform);
    drv_write_reg(DRV2605_REG_WAVESEQ2, 0x00);
    drv_write_reg(DRV2605_REG_GO, 1);
}
