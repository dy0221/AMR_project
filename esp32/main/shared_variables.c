#include "shared_variables.h"

float motor1TargetSpeed = 0,      motor2TargetSpeed = 0;        //목표 속도
portMUX_TYPE shared_var_mux = portMUX_INITIALIZER_UNLOCKED;
float pose_x = 0.0, pose_y = 0.0, orientation_theta = 0.0;
QueueHandle_t log_queue = NULL;