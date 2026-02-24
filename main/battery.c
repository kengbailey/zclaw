#include "battery.h"
#include "display.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

#define BATTERY_ADC_UNIT     ADC_UNIT_1
#define BATTERY_ADC_CHANNEL  ADC_CHANNEL_0   // GPIO 1 on ESP32-S3
#define BATTERY_ADC_ATTEN    ADC_ATTEN_DB_12
#define BATTERY_ADC_BITWIDTH ADC_BITWIDTH_12

#define VOLTAGE_MAX          4.20f
#define VOLTAGE_MIN          3.00f
#define FILTER_SIZE          5
#define UPDATE_INTERVAL_US   (30 * 1000 * 1000)  // 30 seconds

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t         s_cali_handle;

static float s_voltage;
static int   s_percentage;
static float s_history[FILTER_SIZE];
static int   s_hist_idx;

// LiPo discharge curve: {voltage, percentage}
static const float s_lipo_curve[][2] = {
    {4.20f, 100.0f}, {4.15f, 95.0f}, {4.11f, 90.0f}, {4.08f, 85.0f},
    {4.02f, 80.0f},  {3.98f, 75.0f}, {3.95f, 70.0f}, {3.91f, 65.0f},
    {3.87f, 60.0f},  {3.85f, 55.0f}, {3.84f, 50.0f}, {3.82f, 45.0f},
    {3.80f, 40.0f},  {3.79f, 35.0f}, {3.77f, 30.0f}, {3.75f, 25.0f},
    {3.73f, 20.0f},  {3.71f, 15.0f}, {3.69f, 10.0f}, {3.61f,  5.0f},
    {3.27f,  0.0f},
};
#define CURVE_SIZE (sizeof(s_lipo_curve) / sizeof(s_lipo_curve[0]))

static int voltage_to_percentage(float v)
{
    if (v >= VOLTAGE_MAX) return 100;
    if (v <= VOLTAGE_MIN) return 0;

    for (int i = 0; i < (int)CURVE_SIZE - 1; i++) {
        if (v >= s_lipo_curve[i + 1][0]) {
            float v1 = s_lipo_curve[i][0], v2 = s_lipo_curve[i + 1][0];
            float p1 = s_lipo_curve[i][1], p2 = s_lipo_curve[i + 1][1];
            return (int)(p1 + (v - v1) * (p2 - p1) / (v2 - v1) + 0.5f);
        }
    }
    return 0;
}

static float read_voltage(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw) != ESP_OK)
        return 0.0f;

    float volts;
    if (s_cali_handle) {
        int mv = 0;
        adc_cali_raw_to_voltage(s_cali_handle, raw, &mv);
        volts = (mv * 2.0f) / 1000.0f;
    } else {
        volts = ((float)raw * 3.3f / 4096.0f) * 2.0f;
    }

    if (volts > VOLTAGE_MAX) volts = VOLTAGE_MAX;
    return volts;
}

static float filter_voltage(float new_v)
{
    s_history[s_hist_idx] = new_v;
    s_hist_idx = (s_hist_idx + 1) % FILTER_SIZE;

    float sum = 0.0f;
    int   cnt = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        if (s_history[i] > 0.0f) { sum += s_history[i]; cnt++; }
    }
    return cnt > 0 ? sum / cnt : new_v;
}

static void battery_update(void)
{
    s_voltage    = filter_voltage(read_voltage());
    s_percentage = voltage_to_percentage(s_voltage);
}

static void timer_cb(void *arg)
{
    (void)arg;
    battery_update();
    display_set_battery(s_percentage);
}

esp_err_t battery_init(void)
{
    ESP_LOGI(TAG, "Initializing battery monitor");

    // ADC calibration (non-fatal if unavailable)
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = BATTERY_ADC_UNIT,
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using manual conversion");
        s_cali_handle = NULL;
    }

    // ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = BATTERY_ADC_UNIT };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return err;
    }

    // ADC channel
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = BATTERY_ADC_ATTEN,
        .bitwidth = BATTERY_ADC_BITWIDTH,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &ch_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Pre-fill filter
    for (int i = 0; i < FILTER_SIZE; i++) {
        s_history[i] = read_voltage();
        if (i < FILTER_SIZE - 1) vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_hist_idx = 0;

    // Initial reading
    float sum = 0.0f;
    for (int i = 0; i < FILTER_SIZE; i++) sum += s_history[i];
    s_voltage    = sum / FILTER_SIZE;
    s_percentage = voltage_to_percentage(s_voltage);

    ESP_LOGI(TAG, "Battery: %.2fV (%d%%)", s_voltage, s_percentage);

    // Periodic timer
    const esp_timer_create_args_t timer_args = {
        .callback = timer_cb,
        .name     = "battery",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, UPDATE_INTERVAL_US));

    return ESP_OK;
}

int battery_get_percentage(void)
{
    return s_percentage;
}

float battery_get_voltage(void)
{
    return s_voltage;
}
