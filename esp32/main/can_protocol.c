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
    
}