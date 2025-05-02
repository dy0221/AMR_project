#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include<string.h>
#include "shared_variables.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"  // 꼭 include 해줘야 함

#define TAG "TWAI_REAL"
#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4
#define TWAI_SEND_TASK_PERIOD_MS 50


void twai_init();
void twai_rcv_task(void *arg);
void twai_send_task(void *arg);

#endif // CAN_PROTOCOL_H