//*pin layout
//-------------------------------------------------------------------------------------------
//                    |   dir1    dir2    ENCA    ENCB    PWM(enable)    CAN_Rx   CAN_Tx
//-------------------------------------------------------------------------------------------
//  Right_motor(M1)   |   26       27     32      33        25            4        5
//  Left_motor (M2)   |   16       17     18      19        21
//-------------------------------------------------------------------------------------------
// gpio : https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html#_CPPv415gpio_int_type_t
// ESP_ERROR_CHECK : assert (https://www.youtube.com/watch?v=3ydH14AFhMQ&t=128s) 
// console : input   https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/console.html
//         : example https://www.youtube.com/watch?v=fzHefRPDy_w

// 표준 입력 출력 헤더 파일, 주로 printf와 같은 함수를 사용하기 위해서 선언
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_mac.h"

// FreeRTOS 헤더 파일, Task 관련 함수를 사용하기 위해서 선언
// 이 코드에서는 vTaskDelay 사용합니다.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared_variables.h"
#include "motor_control.h"
#include "can_protocol.h"
#include "logging.h"

int motor_speed = 0;

void app_main(void) {
    motor1_init();
    motor2_init();

    encoder_interrupt_init();
    twai_init();
    // PID 제어 태스크 생성
    log_queue = xQueueCreate(20, sizeof(log_data_t));
    xTaskCreatePinnedToCore(
        pid_control_task, "PID_Task", 4096, NULL, 5, NULL, 0
    );

    xTaskCreatePinnedToCore(
        twai_rcv_task, "Twai_rcv_Task", 4096, NULL, 3, NULL, 0
    );

    xTaskCreatePinnedToCore(
        motor_drive_test_task, "MotorTest", 2048, NULL, 1, NULL, 1 // Core 1에 고정, 우선순위 낮게
    );

    xTaskCreatePinnedToCore(
        twai_send_task, "Twai_send_Task", 2048, NULL, 2, NULL, 1 // Core 1에 고정
    );

    xTaskCreatePinnedToCore(logging_task, "Logging", 4096, NULL, 2, NULL, 1);
}