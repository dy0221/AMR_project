//*pin layout
//-------------------------------------------------------------------------------------------
//                    |   dir1    dir2    ENCA    ENCB    PWM(enable)    CAN_Rx   CAN_Tx
//-------------------------------------------------------------------------------------------
//  Right_motor(M1)   |   26       27     32      33        25            4        5
//  Left_motor (M2)   |   16       17     18      19        21
//-------------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>  // abs 사용 시 필요
#include <stdbool.h> // bool자료형

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"

#include "esp_err.h"
// GPIO 관련 함수들을 사용하기 위해 선언
#include "driver/gpio.h"
#include "driver/ledc.h"
// pid timer
#include "driver/timer.h"


// interrupt
#include "esp_intr_types.h"

#include "motor_control.h"

//pid===========================
float motor1Angle = 0,            motor2Angle = 0;

//pid 게인 값
float motor1Kp = 1.4,             motor2Kp = 1.4;
float motor1Ki = 0.3,             motor2Ki = 0.3;
float motor1Kd = 0.5,             motor2Kd = 0.5;

float motor1MeasuredSpeed = 0,    motor2MeasuredSpeed = 0;      //엔코더를 통해 계산된 실제 모터 속도(m/s가 되어야함)
float motor1TargetSpeed = 0,      motor2TargetSpeed = 0;        //목표 속도
float motor1SpeedError = 0,       motor2SpeedError = 0;         //오차 속도 = 목표 속도 - 실제 속도
float motor1PrevSpeedError = 0,   motor2PrevSpeedError = 0;     //이전 주기에서 사용한 오차들을 저장
float motor1DeltaSpeedError = 0,  motor2DeltaSpeedError = 0;    //오차의 변화량
float motor1ErrorSum = 0,         motor2ErrorSum = 0;           //오차의 누적합

float motor1ControlP = 0,         motor2ControlP = 0;
float motor1ControlI = 0,         motor2ControlI = 0;
float motor1ControlD = 0,         motor2ControlD = 0;

int motor1ControlOutput = 0,      motor2ControlOutput = 0;   //모터의 pwm로 들어가야 하기 때문에 int여야함

long encoder1Count = 0,           encoder2Count = 0;         //엔코더 카운터
long encoder1CurrentCount = 0,    encoder1Prev1Count = 0, encoder1Delta1Count  = 0; //엔코더 현제값, 이전값,
long encoder2CurrentCount = 0,    encoder2Prev1Count = 0, encoder2Delta1Count  = 0;

volatile bool pid_flag = false;

//motor init =====================================================================================
void motor1_init(void)
{
    // 방향 제어 핀 설정 (출력)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << M1_DIR1_GPIO) | (1ULL << M1_DIR2_GPIO), //64bit로　표현
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);


    // 타이머를 먼저 설정한 다음 채널을 구성하는 것을 추천 (recomemnd)
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = M1_PWM_MODE,
        .duty_resolution  = M1_PWM_DUTY_RES,
        .timer_num        = M1_PWM_TIMER,
        .freq_hz          = M1_PWM_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = M1_PWM_MODE,
        .channel        = M1_PWM_CHANNEL,
        .timer_sel      = M1_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M1_PWM_GPIO,
        .duty           = 0, // Set duty to 0% (모터 정지)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));  

}

void motor2_init(void)
{
    // 방향 제어 핀 설정 (출력)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << M2_DIR1_GPIO) | (1ULL << M2_DIR2_GPIO), //64bit로　표현
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // 타이머를 먼저 설정한 다음 채널을 구성하는 것을 추천 (recomemnd)
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = M2_PWM_MODE,
        .duty_resolution  = M2_PWM_DUTY_RES,
        .timer_num        = M2_PWM_TIMER,
        .freq_hz          = M2_PWM_FREQUENCY,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = M2_PWM_MODE,
        .channel        = M2_PWM_CHANNEL,
        .timer_sel      = M2_PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = M2_PWM_GPIO,
        .duty           = 0, // Set duty to 0% (모터 정지)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));  
}

//int init ==================================================================================
void encoder_interrupt_init(void){

    gpio_config_t io_conf;
    // ENC1_CHA
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ENC1_CHA_GPIO);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ENC1_CHB
    io_conf.pin_bit_mask = (1ULL << ENC1_CHB_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ENC2_CHA
    io_conf.pin_bit_mask = (1ULL << ENC2_CHA_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // ENC2_CHB
    io_conf.pin_bit_mask = (1ULL << ENC2_CHB_GPIO);
    ESP_ERROR_CHECK(gpio_config(&io_conf));
   
    ESP_ERROR_CHECK(gpio_install_isr_service(0)); // 기본 플래그 사용

    // 각 엔코더 채널의 ISR 핸들러 등록
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC1_CHA_GPIO, m1chA_ISR, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC1_CHB_GPIO, m1chB_ISR, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC2_CHA_GPIO, m2chA_ISR, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC2_CHB_GPIO, m2chB_ISR, NULL));
}
//pid task ==================================================================================
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
//todo : pid제어가 50ms마다 작동하도록 정하고 수식이 짜여져 있어서 이를 20ms로 바꾸기
void pid_control_task(void *arg){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1){
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(PID_TASK_PERIOD_MS) );
        
        encoder1CurrentCount = encoder1Count;
        encoder2CurrentCount = encoder2Count;
        encoder1Delta1Count  = encoder1CurrentCount - encoder1Prev1Count;
        encoder2Delta1Count  = encoder2CurrentCount - encoder2Prev1Count;
        encoder1Prev1Count   = encoder1CurrentCount;
        encoder2Prev1Count   = encoder2CurrentCount;

        // 선속도 m/s >> RPM x (pi x 바퀴 지름 (m)) / 60 (s) (0.00455)
        // 반대로 선속도가 주어질시 RPM = V (m/s) x 60 / 바퀴 둘레(219.78) (m)
//{Delta_encoder(pulse) / 20(ms)} * {1(rotate)/1320(pulse)} * {60 * 1000 (ms)/ 1 (min)} = Delta_encoder * 25 (rotate) / 11 (min) << rpm
        motor1MeasuredSpeed = encoder1Delta1Count * 25 / 11; //RPM
        motor2MeasuredSpeed = encoder2Delta1Count * 25 / 11;

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

        printf("%f,%f,%f,%f\n", motor1TargetSpeed, motor1MeasuredSpeed, motor2TargetSpeed, motor2MeasuredSpeed);
    }
    
}

void motor_drive_test_task(void *arg){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    int speed_step = 70; // 속도 변화 단계
    int max_speed = 100; // 최대 속도
    int min_speed = -100; // 최소 속도
    int direction = 1; // 속도 증가(1) 또는 감소(-1)

    while (1){
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(3000) );

        motor1TargetSpeed += direction * speed_step;
        motor2TargetSpeed += direction * speed_step;
        // 속도가 최대값 또는 최소값에 도달하면 방향 반전
        if (motor1TargetSpeed >= max_speed || motor1TargetSpeed <= min_speed) {
            direction *= -1;
        }

    }
}

//motor drive ===============================================================================
void motor1_drive(int spd_val) 
{
    // 범위 제한 
    if (spd_val > 255) spd_val = 255;
    if (spd_val < -255) spd_val = -255;

    if (spd_val >= 0) {
        gpio_set_level(M1_DIR1_GPIO, 1);
        gpio_set_level(M1_DIR2_GPIO, 0);
        // PWM 듀티를 speed 값으로 설정
        ESP_ERROR_CHECK(ledc_set_duty(M1_PWM_MODE, M1_PWM_CHANNEL, spd_val));
    }
    else {
        gpio_set_level(M1_DIR1_GPIO, 0);
        gpio_set_level(M1_DIR2_GPIO, 1);
        // PWM 듀티를 speed 값으로 설정
        ESP_ERROR_CHECK(ledc_set_duty(M1_PWM_MODE, M1_PWM_CHANNEL, abs(spd_val)));
    }
    ESP_ERROR_CHECK(ledc_update_duty(M1_PWM_MODE, M1_PWM_CHANNEL));
}

void motor2_drive(int spd_val) 
{
    // 범위 제한 
    if (spd_val > 255) spd_val = 255;
    if (spd_val < -255) spd_val = -255;

    if (spd_val >= 0) {
        gpio_set_level(M2_DIR1_GPIO, 0);
        gpio_set_level(M2_DIR2_GPIO, 1);
        // PWM 듀티를 speed 값으로 설정
        ESP_ERROR_CHECK(ledc_set_duty(M2_PWM_MODE, M2_PWM_CHANNEL, spd_val));
    }
    else {
        gpio_set_level(M2_DIR1_GPIO, 1);
        gpio_set_level(M2_DIR2_GPIO, 0);
        // PWM 듀티를 speed 값으로 설정
        ESP_ERROR_CHECK(ledc_set_duty(M2_PWM_MODE, M2_PWM_CHANNEL, abs(spd_val)));
    }
    ESP_ERROR_CHECK(ledc_update_duty(M2_PWM_MODE, M2_PWM_CHANNEL));
}

// ENCODER Measure M1==========================
void m1chA_ISR(void* arg) {
    if (gpio_get_level(ENC1_CHA_GPIO) == 1) {
      if (gpio_get_level(ENC1_CHB_GPIO) == 0)  encoder1Count++;
      else                               encoder1Count--;
    }
    else {
      if (gpio_get_level(ENC1_CHB_GPIO) == 1) encoder1Count++;
      else                               encoder1Count--;
    }
}
  
void m1chB_ISR(void* arg) {
    if (gpio_get_level(ENC1_CHB_GPIO) == 1) {
      if (gpio_get_level(ENC1_CHA_GPIO) == 0)  encoder1Count--;
      else                               encoder1Count++;
    }
    else {
      if (gpio_get_level(ENC1_CHA_GPIO) == 1) encoder1Count--;
      else                               encoder1Count++;
    }
}
  
  // ENCODER Measure M2==========================
void m2chA_ISR(void* arg) {
    if (gpio_get_level(ENC2_CHA_GPIO) == 1) {
      if (gpio_get_level(ENC2_CHB_GPIO) == 1) encoder2Count++;
      else                              encoder2Count--;
    }
    else {
      if (gpio_get_level(ENC2_CHB_GPIO) == 0)  encoder2Count++;
      else                              encoder2Count--;
    }
}
  
void m2chB_ISR(void* arg) {
    if (gpio_get_level(ENC2_CHB_GPIO) == 1) {
      if (gpio_get_level(ENC2_CHA_GPIO) == 1) encoder2Count--;
      else                              encoder2Count++;
    }
    else {
      if (gpio_get_level(ENC2_CHA_GPIO) == 0)  encoder2Count--;
      else                              encoder2Count++;
    }
}