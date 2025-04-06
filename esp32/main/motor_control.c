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
float motor1Ki = 0.5,             motor2Ki = 0.3;
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
//pid timer =================================================================================
void pid_isr_callback(void *args)
{ 
    pid_flag = true;
    encoder1CurrentCount = encoder1Count;
    encoder2CurrentCount = encoder2Count;
    encoder1Delta1Count  = encoder1CurrentCount - encoder1Prev1Count;
    encoder2Delta1Count  = encoder2CurrentCount - encoder2Prev1Count;
    encoder1Prev1Count   = encoder1CurrentCount;
    encoder2Prev1Count   = encoder2CurrentCount;
}

void pid_timer_init()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    }; // default clock source is APB
    ESP_ERROR_CHECK(timer_init(PID_TIMER_GROUP, PID_TIMER_INDEX, &config));
    // load_val	타이머 카운터에 설정할 초기 값 (tick 단위) 
    ESP_ERROR_CHECK(timer_set_counter_value(PID_TIMER_GROUP, PID_TIMER_INDEX, 0));
    // timer 설정
    ESP_ERROR_CHECK(timer_set_alarm_value(PID_TIMER_GROUP, PID_TIMER_INDEX, 50000)); //50000us == 50ms
    // 타이머 인터럽트 설정
    ESP_ERROR_CHECK(timer_enable_intr(PID_TIMER_GROUP, PID_TIMER_INDEX));

    // 콜백함수 엮기
    ESP_ERROR_CHECK(timer_isr_callback_add(PID_TIMER_GROUP, PID_TIMER_INDEX, pid_isr_callback, NULL, 0));
    // 타이머 인터럽트 시작
    ESP_ERROR_CHECK(timer_start(PID_TIMER_GROUP, PID_TIMER_INDEX));
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