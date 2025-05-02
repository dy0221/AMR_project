#ifndef SHARED_VARIABLES_H_
#define SHARED_VARIABLES_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"

extern float motor1TargetSpeed, motor2TargetSpeed;        //목표 속도
extern portMUX_TYPE shared_var_mux;
extern float pose_x, pose_y, orientation_theta;

// 로그 타입 정의
typedef enum {
    LOG_TYPE_PID,
    LOG_TYPE_ODOM,
    LOG_TYPE_CAN,
} log_type_t;

typedef struct {
    log_type_t type;
    TickType_t timestamp_ms;
    union {
        struct {
            float m1_target, m1_meas, m2_target, m2_meas;
        } pid;

        struct {
            float x, y, theta;
        } odom;

        struct {
            int msg_id;
        } can;
    };
} log_data_t;

// 전역 Queue 선언 (extern)
extern QueueHandle_t log_queue;

#endif
