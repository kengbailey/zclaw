#ifndef HAPTIC_H
#define HAPTIC_H

#include "esp_err.h"

typedef enum {
    HAPTIC_CLICK,        // Strong click (effect 1)
    HAPTIC_SOFT_BUMP,    // Soft bump (effect 7)
    HAPTIC_DOUBLE_CLICK, // Double click (effect 10)
    HAPTIC_TICK,         // Sharp click 30% (effect 6)
    HAPTIC_ERROR,        // Triple click (effect 12)
    HAPTIC_SUCCESS,      // Sharp click 100% (effect 4)
} haptic_effect_t;

esp_err_t haptic_init(void);
void haptic_play(haptic_effect_t effect);

#endif // HAPTIC_H
