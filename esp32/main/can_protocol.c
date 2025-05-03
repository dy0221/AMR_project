#include "can_protocol.h"


void twai_init(){
    // TWAI 구성 (Normal mode)
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = TX_GPIO_NUM,
        .rx_io = RX_GPIO_NUM,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 5,
        .rx_queue_len = 5,
        .alerts_enabled = TWAI_ALERT_RX_DATA,  
        .clkout_divider = 1,
        .intr_flags = 0    
    };
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 모든 ID 수신 허용

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI started");
}

void twai_rcv_task(void *arg){
    uint32_t alerts;
    while (1) {
        //rcv값을 최대 100ms까지 기다리지만, 이건 cpu점유를 안함
        if (twai_read_alerts(&alerts, pdMS_TO_TICKS(100)) == ESP_OK) {
            if (alerts & TWAI_ALERT_RX_DATA) {
                twai_message_t msg;
                if (twai_receive(&msg, 0) == ESP_OK) {
                    int left = (int16_t)((msg.data[1] << 8) | msg.data[0]);
                    int right = (int16_t)((msg.data[3] << 8) | msg.data[2]);
                    portENTER_CRITICAL(&shared_var_mux);
                    motor1TargetSpeed = right;
                    motor2TargetSpeed = left;
                    portEXIT_CRITICAL(&shared_var_mux);
                    // ESP_LOGI("CAN_RX", "L=%d R=%d", left, right);

                }
            }
        }
    }
}

void twai_send_task(void *arg){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1){
        //주기 20ms (50hz)
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(TWAI_SEND_TASK_PERIOD_MS) );
        uint64_t exec_time_us = esp_timer_get_time();
        static uint32_t pseudo_seed = 1;  // 초기값 (필요 시 바꿔도 됨)
        pseudo_seed = (pseudo_seed * 1103515245 + 12345) & 0x7FFFFFFF;
        uint8_t packet_id = pseudo_seed % 100;  // us 단위 , 0-99
        twai_message_t msg1 = {
            .identifier = 0x100,
            .data_length_code = 8,
            .extd = 0,
            .rtr = 0,
        };
        
        twai_message_t msg2 = {
            .identifier = 0x101,
            .data_length_code = 8,
            .extd = 0,
            .rtr = 0,
        };

        msg1.data[0] = packet_id;
        msg1.data[1] = 0;
        memcpy(&msg1.data[2], &pose_x, 4);
        memcpy(&msg1.data[6], &pose_y, 2);

        msg2.data[0] = packet_id;
        msg2.data[1] = 0;
        memcpy(&msg2.data[2], ((uint8_t *)&pose_y) + 2, 2);
        memcpy(&msg2.data[4], &orientation_theta, 4);

    
        if (twai_transmit(&msg1, pdMS_TO_TICKS(5)) == ESP_OK) {
            log_data_t can_msg1 = {
                .type = LOG_TYPE_CAN,
                .timestamp_ms = exec_time_us / 1000,
                .can = {
                    .msg_id = 1
                }
            };
            //xQueueSend(log_queue, &can_msg1, 0);
        }

        if (twai_transmit(&msg2, pdMS_TO_TICKS(5)) == ESP_OK) {
            log_data_t can_msg2 = {
                .type = LOG_TYPE_CAN,
                .timestamp_ms = exec_time_us / 1000,
                .can = {
                    .msg_id = 2
                }
            };
            //xQueueSend(log_queue, &can_msg2, 0);
        }
    }
}