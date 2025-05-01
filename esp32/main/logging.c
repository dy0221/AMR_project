#include "logging.h"

void logging_task(void *arg) {
    log_data_t msg;
    while (1) {
        if (xQueueReceive(log_queue, &msg, portMAX_DELAY)) {
            switch (msg.type) {
                case LOG_TYPE_PID:
                    ESP_LOGI("LOG_PID", "[%lu ms] M1: %.2f/%.2f  M2: %.2f/%.2f",
                             msg.timestamp_ms,
                             msg.pid.m1_target, msg.pid.m1_meas,
                             msg.pid.m2_target, msg.pid.m2_meas);
                    break;

                case LOG_TYPE_ODOM:
                    ESP_LOGI("LOG_ODOM", "[%lu ms] X: %.2f, Y: %.2f, Th: %.2f",
                             msg.timestamp_ms,
                             msg.odom.x, msg.odom.y, msg.odom.theta);
                    break;
            }
        }
    }
}
