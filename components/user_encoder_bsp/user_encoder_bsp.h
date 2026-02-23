#ifndef USER_ENCODER_H
#define USER_ENCODER_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifndef SET_BIT
#define SET_BIT(x, bit)   ((x) |= (1 << (bit)))
#endif
#ifndef READ_BIT
#define READ_BIT(x, bit)  (((x) >> (bit)) & 1)
#endif
#ifndef BIT_EVEN_ALL
#define BIT_EVEN_ALL       (BIT(0) | BIT(1))
#endif

extern EventGroupHandle_t knob_even_;

#ifdef __cplusplus
extern "C" {
#endif

void user_encoder_init(void);

#ifdef __cplusplus
}
#endif

#endif


