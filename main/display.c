#include "display.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "lvgl.h"
#include "esp_lcd_sh8601.h"
#include "i2c_bsp.h"
#include "lcd_touch_bsp.h"
#include "lcd_bl_pwm_bsp.h"

static const char *TAG = "display";

// -------------------------------------------------------------------------
// Hardware pin definitions
// -------------------------------------------------------------------------
#define LCD_HOST            SPI2_HOST
#define LCD_H_RES           360
#define LCD_V_RES           360
#define LCD_BUF_HEIGHT      36   // 1/10 of screen
#define LCD_BIT_PER_PIXEL   16

#define PIN_LCD_CS          GPIO_NUM_14
#define PIN_LCD_PCLK        GPIO_NUM_13
#define PIN_LCD_DATA0       GPIO_NUM_15
#define PIN_LCD_DATA1       GPIO_NUM_16
#define PIN_LCD_DATA2       GPIO_NUM_17
#define PIN_LCD_DATA3       GPIO_NUM_18
#define PIN_LCD_RST         GPIO_NUM_21

// -------------------------------------------------------------------------
// LVGL task config
// -------------------------------------------------------------------------
#define LVGL_TICK_MS        2
#define LVGL_TASK_STACK     (6 * 1024)
#define LVGL_TASK_PRIO      2
#define LVGL_MAX_DELAY_MS   500
#define LVGL_MIN_DELAY_MS   5

// -------------------------------------------------------------------------
// SH8601 init command table (from Waveshare demo)
// -------------------------------------------------------------------------
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xF0, (uint8_t[]){0x28}, 1, 0},
    {0xF2, (uint8_t[]){0x28}, 1, 0},
    {0x73, (uint8_t[]){0xF0}, 1, 0},
    {0x7C, (uint8_t[]){0xD1}, 1, 0},
    {0x83, (uint8_t[]){0xE0}, 1, 0},
    {0x84, (uint8_t[]){0x61}, 1, 0},
    {0xF2, (uint8_t[]){0x82}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x01}, 1, 0},
    {0xF1, (uint8_t[]){0x01}, 1, 0},
    {0xB0, (uint8_t[]){0x56}, 1, 0},
    {0xB1, (uint8_t[]){0x4D}, 1, 0},
    {0xB2, (uint8_t[]){0x24}, 1, 0},
    {0xB4, (uint8_t[]){0x87}, 1, 0},
    {0xB5, (uint8_t[]){0x44}, 1, 0},
    {0xB6, (uint8_t[]){0x8B}, 1, 0},
    {0xB7, (uint8_t[]){0x40}, 1, 0},
    {0xB8, (uint8_t[]){0x86}, 1, 0},
    {0xBA, (uint8_t[]){0x00}, 1, 0},
    {0xBB, (uint8_t[]){0x08}, 1, 0},
    {0xBC, (uint8_t[]){0x08}, 1, 0},
    {0xBD, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x80}, 1, 0},
    {0xC1, (uint8_t[]){0x10}, 1, 0},
    {0xC2, (uint8_t[]){0x37}, 1, 0},
    {0xC3, (uint8_t[]){0x80}, 1, 0},
    {0xC4, (uint8_t[]){0x10}, 1, 0},
    {0xC5, (uint8_t[]){0x37}, 1, 0},
    {0xC6, (uint8_t[]){0xA9}, 1, 0},
    {0xC7, (uint8_t[]){0x41}, 1, 0},
    {0xC8, (uint8_t[]){0x01}, 1, 0},
    {0xC9, (uint8_t[]){0xA9}, 1, 0},
    {0xCA, (uint8_t[]){0x41}, 1, 0},
    {0xCB, (uint8_t[]){0x01}, 1, 0},
    {0xD0, (uint8_t[]){0x91}, 1, 0},
    {0xD1, (uint8_t[]){0x68}, 1, 0},
    {0xD2, (uint8_t[]){0x68}, 1, 0},
    {0xF5, (uint8_t[]){0x00, 0xA5}, 2, 0},
    {0xDD, (uint8_t[]){0x4F}, 1, 0},
    {0xDE, (uint8_t[]){0x4F}, 1, 0},
    {0xF1, (uint8_t[]){0x10}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0xF0, (uint8_t[]){0x02}, 1, 0},
    {0xE0, (uint8_t[]){0xF0, 0x0A, 0x10, 0x09, 0x09, 0x36, 0x35, 0x33, 0x4A, 0x29, 0x15, 0x15, 0x2E, 0x34}, 14, 0},
    {0xE1, (uint8_t[]){0xF0, 0x0A, 0x0F, 0x08, 0x08, 0x05, 0x34, 0x33, 0x4A, 0x39, 0x15, 0x15, 0x2D, 0x33}, 14, 0},
    {0xF0, (uint8_t[]){0x10}, 1, 0},
    {0xF3, (uint8_t[]){0x10}, 1, 0},
    {0xE0, (uint8_t[]){0x07}, 1, 0},
    {0xE1, (uint8_t[]){0x00}, 1, 0},
    {0xE2, (uint8_t[]){0x00}, 1, 0},
    {0xE3, (uint8_t[]){0x00}, 1, 0},
    {0xE4, (uint8_t[]){0xE0}, 1, 0},
    {0xE5, (uint8_t[]){0x06}, 1, 0},
    {0xE6, (uint8_t[]){0x21}, 1, 0},
    {0xE7, (uint8_t[]){0x01}, 1, 0},
    {0xE8, (uint8_t[]){0x05}, 1, 0},
    {0xE9, (uint8_t[]){0x02}, 1, 0},
    {0xEA, (uint8_t[]){0xDA}, 1, 0},
    {0xEB, (uint8_t[]){0x00}, 1, 0},
    {0xEC, (uint8_t[]){0x00}, 1, 0},
    {0xED, (uint8_t[]){0x0F}, 1, 0},
    {0xEE, (uint8_t[]){0x00}, 1, 0},
    {0xEF, (uint8_t[]){0x00}, 1, 0},
    {0xF8, (uint8_t[]){0x00}, 1, 0},
    {0xF9, (uint8_t[]){0x00}, 1, 0},
    {0xFA, (uint8_t[]){0x00}, 1, 0},
    {0xFB, (uint8_t[]){0x00}, 1, 0},
    {0xFC, (uint8_t[]){0x00}, 1, 0},
    {0xFD, (uint8_t[]){0x00}, 1, 0},
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xFF, (uint8_t[]){0x00}, 1, 0},
    {0x60, (uint8_t[]){0x40}, 1, 0},
    {0x61, (uint8_t[]){0x04}, 1, 0},
    {0x62, (uint8_t[]){0x00}, 1, 0},
    {0x63, (uint8_t[]){0x42}, 1, 0},
    {0x64, (uint8_t[]){0xD9}, 1, 0},
    {0x65, (uint8_t[]){0x00}, 1, 0},
    {0x66, (uint8_t[]){0x00}, 1, 0},
    {0x67, (uint8_t[]){0x00}, 1, 0},
    {0x68, (uint8_t[]){0x00}, 1, 0},
    {0x69, (uint8_t[]){0x00}, 1, 0},
    {0x6A, (uint8_t[]){0x00}, 1, 0},
    {0x6B, (uint8_t[]){0x00}, 1, 0},
    {0x70, (uint8_t[]){0x40}, 1, 0},
    {0x71, (uint8_t[]){0x03}, 1, 0},
    {0x72, (uint8_t[]){0x00}, 1, 0},
    {0x73, (uint8_t[]){0x42}, 1, 0},
    {0x74, (uint8_t[]){0xD8}, 1, 0},
    {0x75, (uint8_t[]){0x00}, 1, 0},
    {0x76, (uint8_t[]){0x00}, 1, 0},
    {0x77, (uint8_t[]){0x00}, 1, 0},
    {0x78, (uint8_t[]){0x00}, 1, 0},
    {0x79, (uint8_t[]){0x00}, 1, 0},
    {0x7A, (uint8_t[]){0x00}, 1, 0},
    {0x7B, (uint8_t[]){0x00}, 1, 0},
    {0x80, (uint8_t[]){0x48}, 1, 0},
    {0x81, (uint8_t[]){0x00}, 1, 0},
    {0x82, (uint8_t[]){0x06}, 1, 0},
    {0x83, (uint8_t[]){0x02}, 1, 0},
    {0x84, (uint8_t[]){0xD6}, 1, 0},
    {0x85, (uint8_t[]){0x04}, 1, 0},
    {0x86, (uint8_t[]){0x00}, 1, 0},
    {0x87, (uint8_t[]){0x00}, 1, 0},
    {0x88, (uint8_t[]){0x48}, 1, 0},
    {0x89, (uint8_t[]){0x00}, 1, 0},
    {0x8A, (uint8_t[]){0x08}, 1, 0},
    {0x8B, (uint8_t[]){0x02}, 1, 0},
    {0x8C, (uint8_t[]){0xD8}, 1, 0},
    {0x8D, (uint8_t[]){0x04}, 1, 0},
    {0x8E, (uint8_t[]){0x00}, 1, 0},
    {0x8F, (uint8_t[]){0x00}, 1, 0},
    {0x90, (uint8_t[]){0x48}, 1, 0},
    {0x91, (uint8_t[]){0x00}, 1, 0},
    {0x92, (uint8_t[]){0x0A}, 1, 0},
    {0x93, (uint8_t[]){0x02}, 1, 0},
    {0x94, (uint8_t[]){0xDA}, 1, 0},
    {0x95, (uint8_t[]){0x04}, 1, 0},
    {0x96, (uint8_t[]){0x00}, 1, 0},
    {0x97, (uint8_t[]){0x00}, 1, 0},
    {0x98, (uint8_t[]){0x48}, 1, 0},
    {0x99, (uint8_t[]){0x00}, 1, 0},
    {0x9A, (uint8_t[]){0x0C}, 1, 0},
    {0x9B, (uint8_t[]){0x02}, 1, 0},
    {0x9C, (uint8_t[]){0xDC}, 1, 0},
    {0x9D, (uint8_t[]){0x04}, 1, 0},
    {0x9E, (uint8_t[]){0x00}, 1, 0},
    {0x9F, (uint8_t[]){0x00}, 1, 0},
    {0xA0, (uint8_t[]){0x48}, 1, 0},
    {0xA1, (uint8_t[]){0x00}, 1, 0},
    {0xA2, (uint8_t[]){0x05}, 1, 0},
    {0xA3, (uint8_t[]){0x02}, 1, 0},
    {0xA4, (uint8_t[]){0xD5}, 1, 0},
    {0xA5, (uint8_t[]){0x04}, 1, 0},
    {0xA6, (uint8_t[]){0x00}, 1, 0},
    {0xA7, (uint8_t[]){0x00}, 1, 0},
    {0xA8, (uint8_t[]){0x48}, 1, 0},
    {0xA9, (uint8_t[]){0x00}, 1, 0},
    {0xAA, (uint8_t[]){0x07}, 1, 0},
    {0xAB, (uint8_t[]){0x02}, 1, 0},
    {0xAC, (uint8_t[]){0xD7}, 1, 0},
    {0xAD, (uint8_t[]){0x04}, 1, 0},
    {0xAE, (uint8_t[]){0x00}, 1, 0},
    {0xAF, (uint8_t[]){0x00}, 1, 0},
    {0xB0, (uint8_t[]){0x48}, 1, 0},
    {0xB1, (uint8_t[]){0x00}, 1, 0},
    {0xB2, (uint8_t[]){0x09}, 1, 0},
    {0xB3, (uint8_t[]){0x02}, 1, 0},
    {0xB4, (uint8_t[]){0xD9}, 1, 0},
    {0xB5, (uint8_t[]){0x04}, 1, 0},
    {0xB6, (uint8_t[]){0x00}, 1, 0},
    {0xB7, (uint8_t[]){0x00}, 1, 0},
    {0xB8, (uint8_t[]){0x48}, 1, 0},
    {0xB9, (uint8_t[]){0x00}, 1, 0},
    {0xBA, (uint8_t[]){0x0B}, 1, 0},
    {0xBB, (uint8_t[]){0x02}, 1, 0},
    {0xBC, (uint8_t[]){0xDB}, 1, 0},
    {0xBD, (uint8_t[]){0x04}, 1, 0},
    {0xBE, (uint8_t[]){0x00}, 1, 0},
    {0xBF, (uint8_t[]){0x00}, 1, 0},
    {0xC0, (uint8_t[]){0x10}, 1, 0},
    {0xC1, (uint8_t[]){0x47}, 1, 0},
    {0xC2, (uint8_t[]){0x56}, 1, 0},
    {0xC3, (uint8_t[]){0x65}, 1, 0},
    {0xC4, (uint8_t[]){0x74}, 1, 0},
    {0xC5, (uint8_t[]){0x88}, 1, 0},
    {0xC6, (uint8_t[]){0x99}, 1, 0},
    {0xC7, (uint8_t[]){0x01}, 1, 0},
    {0xC8, (uint8_t[]){0xBB}, 1, 0},
    {0xC9, (uint8_t[]){0xAA}, 1, 0},
    {0xD0, (uint8_t[]){0x10}, 1, 0},
    {0xD1, (uint8_t[]){0x47}, 1, 0},
    {0xD2, (uint8_t[]){0x56}, 1, 0},
    {0xD3, (uint8_t[]){0x65}, 1, 0},
    {0xD4, (uint8_t[]){0x74}, 1, 0},
    {0xD5, (uint8_t[]){0x88}, 1, 0},
    {0xD6, (uint8_t[]){0x99}, 1, 0},
    {0xD7, (uint8_t[]){0x01}, 1, 0},
    {0xD8, (uint8_t[]){0xBB}, 1, 0},
    {0xD9, (uint8_t[]){0xAA}, 1, 0},
    {0xF3, (uint8_t[]){0x01}, 1, 0},
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    {0x21, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x29, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0x00}, 1, 0},
};

// -------------------------------------------------------------------------
// LVGL plumbing
// -------------------------------------------------------------------------
static SemaphoreHandle_t s_lvgl_mux = NULL;
static lv_disp_drv_t     s_disp_drv;

static bool lvgl_lock(int timeout_ms)
{
    const TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_lvgl_mux, ticks) == pdTRUE;
}

static void lvgl_unlock(void)
{
    xSemaphoreGive(s_lvgl_mux);
}

static bool flush_ready_cb(esp_lcd_panel_io_handle_t panel_io,
                           esp_lcd_panel_io_event_data_t *edata,
                           void *user_ctx)
{
    lv_disp_drv_t *drv = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(drv);
    return false;
}

static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1, color_map);
}

static void rounder_cb(lv_disp_drv_t *drv, lv_area_t *area)
{
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t x, y;
    if (tpGetCoordinates(&x, &y)) {
        data->point.x = x;
        data->point.y = y;
        if (data->point.x > LCD_H_RES) data->point.x = LCD_H_RES;
        if (data->point.y > LCD_V_RES) data->point.y = LCD_V_RES;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_MS);
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "LVGL task started");
    uint32_t delay_ms = LVGL_MAX_DELAY_MS;
    while (1) {
        if (lvgl_lock(-1)) {
            delay_ms = lv_timer_handler();
            lvgl_unlock();
        }
        if (delay_ms > LVGL_MAX_DELAY_MS) delay_ms = LVGL_MAX_DELAY_MS;
        else if (delay_ms < LVGL_MIN_DELAY_MS) delay_ms = LVGL_MIN_DELAY_MS;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// -------------------------------------------------------------------------
// UI widgets
// -------------------------------------------------------------------------
static lv_obj_t *s_status_label;
static lv_obj_t *s_wifi_label;
static lv_obj_t *s_conversation;  // scrollable label for messages

#define CONV_BUF_SIZE  2048
static char s_conv_buf[CONV_BUF_SIZE];
static size_t s_conv_len = 0;

static void conv_append(const char *prefix, const char *text)
{
    int written = snprintf(s_conv_buf + s_conv_len,
                           CONV_BUF_SIZE - s_conv_len,
                           "%s%s%s\n",
                           s_conv_len > 0 ? "\n" : "",
                           prefix, text);
    if (written > 0 && (s_conv_len + written) < CONV_BUF_SIZE) {
        s_conv_len += written;
    }
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x1A1A2E), 0);

    // Status indicator at top
    s_status_label = lv_label_create(scr);
    lv_label_set_text(s_status_label, "Booting...");
    lv_obj_set_style_text_font(s_status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00D4FF), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 15);

    // WiFi status
    s_wifi_label = lv_label_create(scr);
    lv_label_set_text(s_wifi_label, "WiFi: --");
    lv_obj_set_style_text_font(s_wifi_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_wifi_label, lv_color_hex(0x888888), 0);
    lv_obj_align(s_wifi_label, LV_ALIGN_TOP_MID, 0, 35);

    // Scrollable conversation area
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_set_size(cont, 320, 270);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -15);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x16213E), 0);
    lv_obj_set_style_border_color(cont, lv_color_hex(0x0F3460), 0);
    lv_obj_set_style_border_width(cont, 1, 0);
    lv_obj_set_style_radius(cont, 12, 0);
    lv_obj_set_style_pad_all(cont, 10, 0);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);

    s_conversation = lv_label_create(cont);
    lv_label_set_long_mode(s_conversation, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_conversation, 295);
    lv_label_set_text(s_conversation, "zclaw ready.\nSend a message via serial or Telegram.");
    lv_obj_set_style_text_font(s_conversation, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_conversation, lv_color_hex(0xE0E0E0), 0);
}

// -------------------------------------------------------------------------
// Public API
// -------------------------------------------------------------------------
esp_err_t display_init(void)
{
    ESP_LOGI(TAG, "Initializing display");

    // Backlight on
    lcd_bl_pwm_bsp_init(200);

    // SPI bus
    const spi_bus_config_t buscfg = {
        .data0_io_num = PIN_LCD_DATA0,
        .data1_io_num = PIN_LCD_DATA1,
        .sclk_io_num  = PIN_LCD_PCLK,
        .data2_io_num = PIN_LCD_DATA2,
        .data3_io_num = PIN_LCD_DATA3,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config =
        SH8601_PANEL_IO_QSPI_CONFIG(PIN_LCD_CS, flush_ready_cb, &s_disp_drv);

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = { .use_qspi_interface = 1 },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                              &io_config, &io_handle));

    // Panel driver
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config  = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // I2C + touch
    i2c_master_Init();
    lcd_touch_init();

    // LVGL init
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * LCD_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * LCD_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers");
        return ESP_ERR_NO_MEM;
    }

    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_BUF_HEIGHT);

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res    = LCD_H_RES;
    s_disp_drv.ver_res    = LCD_V_RES;
    s_disp_drv.flush_cb   = flush_cb;
    s_disp_drv.rounder_cb = rounder_cb;
    s_disp_drv.draw_buf   = &disp_buf;
    s_disp_drv.user_data  = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&s_disp_drv);

    // Tick timer
    const esp_timer_create_args_t tick_args = {
        .callback = tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000));

    // Touch input device
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.disp    = disp;
    indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&indev_drv);

    // Mutex + task
    s_lvgl_mux = xSemaphoreCreateMutex();
    assert(s_lvgl_mux);

    create_ui();

    xTaskCreate(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL, LVGL_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "Display initialized");
    return ESP_OK;
}

void display_set_state(display_state_t state)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    switch (state) {
    case DISPLAY_STATE_BOOTING:
        lv_label_set_text(s_status_label, "Booting...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFCC00), 0);
        break;
    case DISPLAY_STATE_CONNECTING:
        lv_label_set_text(s_status_label, "Connecting WiFi...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFCC00), 0);
        break;
    case DISPLAY_STATE_IDLE:
        lv_label_set_text(s_status_label, "Ready");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00E676), 0);
        break;
    case DISPLAY_STATE_THINKING:
        lv_label_set_text(s_status_label, "Thinking...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0x00D4FF), 0);
        break;
    case DISPLAY_STATE_TOOL_EXEC:
        // Tool name set separately via display_set_tool_status
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xBB86FC), 0);
        break;
    case DISPLAY_STATE_LISTENING:
        lv_label_set_text(s_status_label, "Listening...");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFA726), 0);
        break;
    case DISPLAY_STATE_ERROR:
        lv_label_set_text(s_status_label, "Error");
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFF5252), 0);
        break;
    }

    lvgl_unlock();
}

void display_add_user_message(const char *text)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    conv_append("> ", text);
    lv_label_set_text(s_conversation, s_conv_buf);
    // Scroll to bottom
    lv_obj_t *cont = lv_obj_get_parent(s_conversation);
    lv_obj_scroll_to_y(cont, LV_COORD_MAX, LV_ANIM_ON);

    lvgl_unlock();
}

void display_add_agent_message(const char *text)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    conv_append("", text);
    lv_label_set_text(s_conversation, s_conv_buf);
    lv_obj_t *cont = lv_obj_get_parent(s_conversation);
    lv_obj_scroll_to_y(cont, LV_COORD_MAX, LV_ANIM_ON);

    lvgl_unlock();
}

void display_set_tool_status(const char *tool_name)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    char buf[64];
    snprintf(buf, sizeof(buf), "Running: %s", tool_name);
    lv_label_set_text(s_status_label, buf);

    lvgl_unlock();
}

void display_set_wifi_status(const char *ip_addr)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    char buf[48];
    snprintf(buf, sizeof(buf), "WiFi: %s", ip_addr);
    lv_label_set_text(s_wifi_label, buf);

    lvgl_unlock();
}

void display_scroll_conversation(int pixels)
{
    if (!s_lvgl_mux) return;
    if (!lvgl_lock(100)) return;

    lv_obj_t *cont = lv_obj_get_parent(s_conversation);
    lv_coord_t cur_y = lv_obj_get_scroll_y(cont);
    lv_obj_scroll_to_y(cont, cur_y + pixels, LV_ANIM_ON);

    lvgl_unlock();
}
