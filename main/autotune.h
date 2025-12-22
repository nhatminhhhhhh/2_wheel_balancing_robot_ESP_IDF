#ifndef AUTOTUNE_H
#define AUTOTUNE_H
#include "esp_err.h"
esp_err_t autotune_pid(float target_angle, int tuning_duration_seconds);
int fuzzy(float error, float delta_error);
#endif // AUTOTUNE_H