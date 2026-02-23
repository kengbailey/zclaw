#ifndef VOICE_H
#define VOICE_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t voice_init(void);
esp_err_t voice_start(QueueHandle_t input_queue);

#endif // VOICE_H
