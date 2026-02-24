#ifndef BATTERY_H
#define BATTERY_H

#include "esp_err.h"

esp_err_t battery_init(void);
int battery_get_percentage(void);
float battery_get_voltage(void);

#endif // BATTERY_H
