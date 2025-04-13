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

#include "motor_control.h"
#include "can_protocol.h"

int motor_speed = 0;


void app_main(void) {
    motor1_init();
    motor2_init();

    encoder_interrupt_init();
    pid_timer_init();

    int speed_step = 70; // 속도 변화 단계
    int max_speed = 100; // 최대 속도
    int min_speed = -100; // 최소 속도
    int direction = 1; // 속도 증가(1) 또는 감소(-1)

    TickType_t last_update_time = xTaskGetTickCount(); // 마지막 업데이트 시간

    //todo : 다음주 주말 (2025.04.20)에 할것
    //todo : 1. microros 공부하기
    //todo : 2. microros로 can 통신해서 esp32에서는 cmd_vel을 받고, raspberrypi에서는 odom을 보내기
     while (1) {
        // 3초마다 목표 속도 업데이트 0>70>140>70>0>-70>-140>-70>0
        if (xTaskGetTickCount() - last_update_time >= pdMS_TO_TICKS(3000)) {
            last_update_time = xTaskGetTickCount();

            motor1TargetSpeed += direction * speed_step;
            motor2TargetSpeed += direction * speed_step;

            // 속도가 최대값 또는 최소값에 도달하면 방향 반전
            if (motor1TargetSpeed >= max_speed || motor1TargetSpeed <= min_speed) {
                direction *= -1;
            }
        }
        
        if (pid_flag) {
            pid_flag = false;  // 플래그 초기화

            // 선속도 m/s >> RPM x (pi x 바퀴 지름 (m)) / 60 (s) (0.00455)
            // 반대로 선속도가 주어질시 RPM = V (m/s) x 60 / 바퀴 둘레(219.78) (m)
            motor1MeasuredSpeed = encoder1Delta1Count * 10 / 11; //RPM
            motor2MeasuredSpeed = encoder2Delta1Count * 10 / 11;

            // ERROR
            motor1SpeedError = motor1TargetSpeed - motor1MeasuredSpeed;
            motor1DeltaSpeedError = motor1SpeedError - motor1PrevSpeedError;
            motor1ErrorSum = motor1ErrorSum + motor1SpeedError;
            motor1PrevSpeedError = motor1SpeedError;

            motor2SpeedError = motor2TargetSpeed - motor2MeasuredSpeed;
            motor2DeltaSpeedError = motor2SpeedError - motor2PrevSpeedError;
            motor2ErrorSum = motor2ErrorSum + motor2SpeedError;
            motor2PrevSpeedError = motor2SpeedError;

            // PID
            motor1ControlP = motor1Kp * motor1SpeedError;
            motor1ControlI = motor1Ki * motor1ErrorSum;
            motor1ControlD = motor1Kd * motor1DeltaSpeedError;

            motor2ControlP = motor2Kp * motor2SpeedError;
            motor2ControlI = motor2Ki * motor2ErrorSum;
            motor2ControlD = motor2Kd * motor2DeltaSpeedError;

            motor1ControlOutput = (int)(motor1ControlP + motor1ControlI + motor1ControlD);
            motor2ControlOutput = (int)(motor2ControlP + motor2ControlI + motor2ControlD);

            if (motor1ControlOutput > 255)       motor1ControlOutput = 255;
            else if (motor1ControlOutput < -255) motor1ControlOutput = -255;
            if (motor2ControlOutput > 255)       motor2ControlOutput = 255;
            else if (motor2ControlOutput < -255) motor2ControlOutput = -255;

            motor1_drive(motor1ControlOutput);
            motor2_drive(motor2ControlOutput);

            motor1Angle = (int)encoder1Count / 11.0 * 3.0;
            motor2Angle = (int)encoder2Count / 11.0 * 3.0;

            printf("%f", motor1TargetSpeed);
            printf(",");
            printf("%f", motor1MeasuredSpeed);
            printf(",");
            printf("%f", motor2TargetSpeed);
            printf(",");
            printf("%f\n", motor2MeasuredSpeed);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms 대기
    }
}