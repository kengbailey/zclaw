#include "input.h"
#include "display.h"
#include "haptic.h"
#include "user_encoder_bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "input";

#define SCROLL_PIXELS  30
#define INPUT_TASK_STACK  (3 * 1024)
#define INPUT_TASK_PRIO   2

static void input_task(void *arg)
{
    ESP_LOGI(TAG, "Input task started");

    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            knob_even_, BIT(0) | BIT(1), pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & BIT(0)) {
            // Knob left → scroll up
            haptic_play(HAPTIC_TICK);
            display_scroll_conversation(-SCROLL_PIXELS);
        }
        if (bits & BIT(1)) {
            // Knob right → scroll down
            haptic_play(HAPTIC_TICK);
            display_scroll_conversation(SCROLL_PIXELS);
        }
    }
}

esp_err_t input_init(void)
{
    ESP_LOGI(TAG, "Initializing input (encoder)");

    user_encoder_init();

    if (xTaskCreate(input_task, "input", INPUT_TASK_STACK, NULL,
                    INPUT_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create input task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Input initialized");
    return ESP_OK;
}
