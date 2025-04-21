#include "shared_variables.h"

float motor1TargetSpeed = 0,      motor2TargetSpeed = 0;        //목표 속도
portMUX_TYPE target_speed_mux = portMUX_INITIALIZER_UNLOCKED;
