#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

//M1(right)===============================
#define ENC1_CHA_GPIO   32
#define ENC1_CHB_GPIO   33
#define M1_PWM_GPIO   25
#define M1_DIR1_GPIO   26
#define M1_DIR2_GPIO   27

#define M1_PWM_TIMER              LEDC_TIMER_1     //사용할　타이머　(0 ~ 15)
#define M1_PWM_MODE               LEDC_HIGH_SPEED_MODE
#define M1_PWM_CHANNEL            LEDC_CHANNEL_1   // 사용할 채널 설정 (0~8가능)
#define M1_PWM_DUTY_RES           LEDC_TIMER_8_BIT  // Set duty resolution to 8 bits / 8비트 해상도 (0~255)
#define M1_PWM_FREQUENCY          1000 // Frequency in Hertz. Set frequency at 1 kHz / PWM 주파수 (예: 1kHz)

//M2(left)================================
#define ENC2_CHA_GPIO             18
#define ENC2_CHB_GPIO             19
#define M2_PWM_GPIO               21 // Define the output GPIO
#define M2_DIR1_GPIO              16
#define M2_DIR2_GPIO              17

#define M2_PWM_TIMER              LEDC_TIMER_0     //사용할　타이머　(0 ~ 15)
#define M2_PWM_MODE               LEDC_HIGH_SPEED_MODE
#define M2_PWM_CHANNEL            LEDC_CHANNEL_0   // 사용할 채널 설정 (0~8가능)
#define M2_PWM_DUTY_RES           LEDC_TIMER_8_BIT  // Set duty resolution to 8 bits / 8비트 해상도 (0~255)
#define M2_PWM_FREQUENCY          1000 // Frequency in Hertz. Set frequency at 1 kHz / PWM 주파수 (예: 1kHz)

//pid task==================================
#define TAG "PID_TASK"
#define PID_TASK_PERIOD_MS 50

extern float motor1Angle, motor2Angle;

//pid 게인 값
extern float motor1Kp, motor2Kp;
extern float motor1Ki, motor2Ki;
extern float motor1Kd, motor2Kd;

extern float motor1MeasuredSpeed, motor2MeasuredSpeed;      //엔코더를 통해 계산된 실제 모터 속도(m/s가 되어야함)
extern float motor1TargetSpeed, motor2TargetSpeed;        //목표 속도
extern float motor1SpeedError, motor2SpeedError;         //오차 속도 = 목표 속도 - 실제 속도
extern float motor1PrevSpeedError, motor2PrevSpeedError;     //이전 주기에서 사용한 오차들을 저장
extern float motor1DeltaSpeedError, motor2DeltaSpeedError;    //오차의 변화량
extern float motor1ErrorSum, motor2ErrorSum;           //오차의 누적합

extern float motor1ControlP, motor2ControlP;
extern float motor1ControlI, motor2ControlI;
extern float motor1ControlD, motor2ControlD;

extern int motor1ControlOutput, motor2ControlOutput;   //모터의 pwm로 들어가야 하기 때문에 int여야함

extern long encoder1Count, encoder2Count;         //엔코더 카운터
extern long encoder1CurrentCount, encoder1Prev1Count, encoder1Delta1Count; //엔코더 현제값, 이전값,
extern long encoder2CurrentCount, encoder2Prev1Count, encoder2Delta1Count;

extern volatile bool pid_flag;
//Function================================
void motor1_init(void);
void motor2_init(void);
void motor1_drive(int spd_val);
void motor2_drive(int spd_val);
void m1chA_ISR(void* arg);
void m1chB_ISR(void* arg);
void m2chA_ISR(void* arg);
void m2chB_ISR(void* arg);
void encoder_interrupt_init(void);

void motor_drive_test_task(void *arg);
void pid_control_task(void *arg);
#endif