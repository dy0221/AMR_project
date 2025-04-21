#ifndef SHARED_VARIABLES_H_
#define SHARED_VARIABLES_H_

#include "freertos/FreeRTOS.h"
#include "esp_intr_alloc.h"

extern float motor1TargetSpeed, motor2TargetSpeed;        //목표 속도
extern portMUX_TYPE target_speed_mux;

#endif
