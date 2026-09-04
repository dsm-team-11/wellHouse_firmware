/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 메인 프로그램 본문
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* 헤더 파일 -----------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "lcd_i2c.h"

/* 전용 헤더 파일 ------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* 전용 자료형 정의 ----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* 전용 상수 정의 ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* 전용 매크로 ---------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* 전용 변수 -----------------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t waterValue = 0;
uint8_t Flood = 0;

// 물센서 3채널 진단용(SWV 로그 및 실시간 표현식 관찰용)
uint32_t adc_in5 = 0, adc_in6 = 0, adc_in7 = 0;

// LCD 깜빡임 방지를 위한 상태 정의
typedef enum {
    STATE_SAFE = 0,
    STATE_WARNING = 1,
    STATE_ALERT = 2,
    STATE_CAUTION = 3,
    STATE_CRITICAL = 4,
    STATE_DANGER = 5
} SystemState;

SystemState last_state = STATE_SAFE; // 이전 상태 저장
SystemState current_state = STATE_SAFE;

// 상태 디바운스 (센서 노이즈로 한두 번 값이 튀어도 출력이 오작동하지 않도록)
#define STATE_CONFIRM_COUNT 3            // 같은 상태를 연속 3회 받아야 실제 반영
#define EMERGENCY_WATER_CM  3U           // 부저와 비상 서보가 작동하는 기준 수위
SystemState pending_state = STATE_SAFE;  // 관찰 중인 후보 상태
uint8_t state_confirm = 0;               // 후보 상태가 연속으로 확인된 횟수
SystemState action_last_state = STATE_SAFE; // ALERT 재진입 감지용 이전 상태
uint8_t last_water_cm = 0xFF;
uint8_t current_water_cm = 0;
uint8_t display_water_cm = 0;  // 확정 상태와 함께 LCD에 반영할 수위

//spi 통신 버퍼
uint8_t spiTx[3];
uint8_t spiRx[3];

/* USER CODE END PV */

/* 전용 함수 원형 ------------------------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* 사용자 전용 코드 ----------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ── 디버그 로그: printf 출력을 SWV(ITM)로 보냄 ───────────────────
// STM32CubeIDE 디버그 세션의 [SWV ITM 데이터 콘솔] 0번 포트에서 확인
int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    return ch;
}

// ── ADC 단일 채널 읽기 헬퍼 ─────────────────────────────────────
// 스캔/DMA 미사용 구성이므로 변환 전에 1순위 채널을 매번 재설정한다
uint32_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    // 고임피던스 물센서: 샘플링 시간을 길게 잡아야 채널 간 크로스토크를 막는다
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

// 서보 번호
#define BREAKER_SERVO   1
#define GAS_SERVO       2
#define WINDOW_SERVO    3

// 가스·창문 서보 각도 (실험 후 수정)
#define GAS_INIT_ANGLE      0     // 가스 초기 위치
#define GAS_CLOSE_ANGLE     130   // 가스 잠금
#define GAS_RETURN_ANGLE    50    // 다음 ALERT 동작을 위한 복귀 위치
#define WINDOW_CLOSE_ANGLE  70    // 창문 닫힘
#define WINDOW_OPEN_ANGLE   0     // 창문 열림

// 차단기 서보: 초기 위치에서 첫 번째 위치로 이동한 뒤 최종 위치로 이동
#define BREAKER_INIT_ANGLE   90   // 초기(정지) 각도
#define BREAKER_FIRST_ANGLE  95  // 첫 번째 이동 위치
#define BREAKER_FINAL_ANGLE  0   // 최종 위치

// 침수 기준 값
//---------------------------------------------

// 각 서보의 현재 각도(논리값)를 추적한다 (천천히 이동시키기 위함). 부팅 시 중앙 90°.
// PA5, PA6, PA7 센서는 각각 독립적으로 ADC 0~4095를 읽고 0~4 cm를 측정한다.
// 0~100: 0 cm, 101~1000: 1 cm, 1001~2000: 2 cm,
// 2001~3754: 3 cm, 3755~4095: 4 cm
static uint8_t ADC_ToSensorCm(uint32_t adc)
{
    if (adc <= 500U)  return 0;
    if (adc <= 1500U) return 1;
    if (adc <= 2500U) return 2;
    if (adc < 3755U)  return 3;
    return 4;
}

uint8_t servoAngle[4] = {90, BREAKER_INIT_ANGLE, GAS_INIT_ANGLE, 90};

#define SERVO_STEP_TIME_MS          15U
#define BREAKER_STEP_TIME_MS         8U
#define GAS_START_DELAY_MS          500U
#define GAS_REASSERT_TIME_MS        500U

typedef enum {
    SERVO_SEQUENCE_IDLE = 0,
    SERVO_SEQUENCE_BREAKER_FIRST,
    SERVO_SEQUENCE_BREAKER_FINAL,
    SERVO_SEQUENCE_GAS_WAIT,
    SERVO_SEQUENCE_GAS_CLOSE,
    SERVO_SEQUENCE_GAS_RETURN,
    SERVO_SEQUENCE_WINDOW_CLOSE,
    SERVO_SEQUENCE_WINDOW_OPEN
} ServoSequenceState;

ServoSequenceState servo_sequence = SERVO_SEQUENCE_IDLE;
uint32_t sensor_update_tick = 0;
uint32_t gas_start_tick = 0;

typedef struct {
    uint8_t servo;
    uint8_t start_angle;
    uint8_t target_angle;
    uint32_t start_tick;
    uint32_t duration_ms;
} ServoMotion;

ServoMotion servo_motion = {0};

void Servo_SetAngle(uint8_t servo, uint8_t angle)
{
    uint16_t pulse;

    // 0~180°를 1000~2000마이크로초로 변환
    pulse = 1000 + (angle * 1000) / 180;

    switch(servo)
        {
            case BREAKER_SERVO:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
                break;

            case GAS_SERVO:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
                break;

            case WINDOW_SERVO:
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
                break;
        }
    }

    static uint32_t Servo_Channel(uint8_t servo)
    {
        if (servo == BREAKER_SERVO) return TIM_CHANNEL_1;
        if (servo == GAS_SERVO)     return TIM_CHANNEL_2;
        return TIM_CHANNEL_3;
    }

    /*
     * 현재 각도에서 목표 각도까지 이동을 예약한다. 이동 각도 1도당 15ms를
     * 사용하므로 기존 동작 속도는 유지한다.
     */
    static void Servo_BeginMove(uint8_t servo, uint8_t target, uint32_t now)
    {
        uint8_t start = servoAngle[servo];
        uint8_t distance = (start < target) ? (uint8_t)(target - start)
                                            : (uint8_t)(start - target);

        HAL_TIM_PWM_Start(&htim2, Servo_Channel(servo));
        Servo_SetAngle(servo, start);

        servo_motion.servo = servo;
        servo_motion.start_angle = start;
        servo_motion.target_angle = target;
        servo_motion.start_tick = now;
        uint32_t step_time_ms = (servo == BREAKER_SERVO)
                              ? BREAKER_STEP_TIME_MS
                              : SERVO_STEP_TIME_MS;
        servo_motion.duration_ms = (uint32_t)distance * step_time_ms;

        // 논리 각도가 이미 잠금 위치여도 실제 가스 서보에 PWM을 충분히 인가한다.
        if (servo == GAS_SERVO && servo_motion.duration_ms < GAS_REASSERT_TIME_MS)
        {
            servo_motion.duration_ms = GAS_REASSERT_TIME_MS;
        }
    }

    /*
     * 호출 횟수가 아니라 실제 경과시간으로 각도를 계산한다. 메인 루프가 잠시
     * 지연되어도 반환 동작이 중간에 멈추지 않는다.
     */
    static uint8_t Servo_UpdateMove(uint32_t now)
    {
        uint32_t elapsed = (uint32_t)(now - servo_motion.start_tick);
        uint8_t start = servo_motion.start_angle;
        uint8_t target = servo_motion.target_angle;
        uint8_t distance = (start < target) ? (uint8_t)(target - start)
                                            : (uint8_t)(start - target);
        uint32_t step_time_ms = (servo_motion.servo == BREAKER_SERVO)
                              ? BREAKER_STEP_TIME_MS
                              : SERVO_STEP_TIME_MS;

        if (servo_motion.duration_ms == 0U || elapsed >= servo_motion.duration_ms)
        {
            servoAngle[servo_motion.servo] = target;
            Servo_SetAngle(servo_motion.servo, target);
            return 1U;
        }

        uint8_t moved = (uint8_t)(elapsed / step_time_ms);
        if (moved > distance) moved = distance;

        uint8_t angle = (start < target) ? (uint8_t)(start + moved)
                                         : (uint8_t)(start - moved);
        servoAngle[servo_motion.servo] = angle;
        Servo_SetAngle(servo_motion.servo, angle);
        return 0U;
    }

    // 비상 서보 동작을 시작한다. 실제 이동은 메인 루프의 갱신 함수가 나누어 처리한다.
    static void Emergency_Action(void)
    {
        if (servo_sequence != SERVO_SEQUENCE_IDLE) return;

        // ALERT에 들어올 때마다 현재 위치에서 차단기 동작 순서를 다시 실행한다.
        servo_sequence = SERVO_SEQUENCE_BREAKER_FIRST;
        Servo_BeginMove(BREAKER_SERVO, BREAKER_FIRST_ANGLE, HAL_GetTick());
    }

    // 복구 시 창문 서보만 원래 위치로 이동한다. 가스 밸브 위치는 그대로 유지한다.
    static void Recovery_Action(void)
    {
        if (servo_sequence != SERVO_SEQUENCE_IDLE) return;

        servo_sequence = SERVO_SEQUENCE_WINDOW_OPEN;
        Servo_BeginMove(WINDOW_SERVO, WINDOW_OPEN_ANGLE, HAL_GetTick());
    }

    // 15ms마다 한 번 호출 효과를 내어 서보를 하나씩 순차적으로 움직인다.
    static void Servo_SequenceUpdate(void)
    {
        uint32_t now = HAL_GetTick();

        if (servo_sequence == SERVO_SEQUENCE_IDLE) return;

        switch (servo_sequence)
        {
            case SERVO_SEQUENCE_BREAKER_FIRST:
                if (Servo_UpdateMove(now))
                {
                    servo_sequence = SERVO_SEQUENCE_BREAKER_FINAL;
                    Servo_BeginMove(BREAKER_SERVO, BREAKER_FINAL_ANGLE, now);
                }
                break;

            case SERVO_SEQUENCE_BREAKER_FINAL:
                if (Servo_UpdateMove(now))
                {
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                    gas_start_tick = now;
                    servo_sequence = SERVO_SEQUENCE_GAS_WAIT;
                }
                break;

            case SERVO_SEQUENCE_GAS_WAIT:
                // 차단기 동작 직후의 전압 강하가 가라앉은 뒤 가스 서보를 구동한다.
                if ((uint32_t)(now - gas_start_tick) >= GAS_START_DELAY_MS)
                {
                    servo_sequence = SERVO_SEQUENCE_GAS_CLOSE;
                    Servo_BeginMove(GAS_SERVO, GAS_CLOSE_ANGLE, now);
                }
                break;

            case SERVO_SEQUENCE_GAS_CLOSE:
                if (Servo_UpdateMove(now))
                {
                    servo_sequence = SERVO_SEQUENCE_GAS_RETURN;
                    Servo_BeginMove(GAS_SERVO, GAS_RETURN_ANGLE, now);
                }
                break;

            case SERVO_SEQUENCE_GAS_RETURN:
                if (Servo_UpdateMove(now))
                {
                    // 50도 복귀 후 PWM을 끄고 다음 ALERT 동작을 준비한다.
                    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
                    servo_sequence = SERVO_SEQUENCE_WINDOW_CLOSE;
                    Servo_BeginMove(WINDOW_SERVO, WINDOW_CLOSE_ANGLE, now);
                }
                break;

            case SERVO_SEQUENCE_WINDOW_CLOSE:
                if (Servo_UpdateMove(now))
                {
                    servo_sequence = SERVO_SEQUENCE_IDLE;
                }
                break;

            case SERVO_SEQUENCE_WINDOW_OPEN:
                if (Servo_UpdateMove(now))
                {
                    servo_sequence = SERVO_SEQUENCE_IDLE;
                }
                break;

            default:
                break;
        }
    }

    // LED 극성: 원래의 높은 신호 활성 방식으로 복귀. 높음=켜짐, 낮음=꺼짐.
    //  (반대로 켜지면 1로 바꾸세요.)
    #define LED_ACTIVE_LOW  0

    void LED_ON(void)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, LED_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    void LED_OFF(void)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, LED_ACTIVE_LOW ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void Buzzer_ON(void)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

        // 부저가 처음 울리는 순간 차단기·가스 밸브·창문 서보도 함께 동작시킨다.
        if (Flood == 0 && servo_sequence == SERVO_SEQUENCE_IDLE)
        {
            Emergency_Action();
            Flood = 1;
        }
    }


    void Buzzer_OFF(void)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

        // 경보가 해제되면 진행 중인 비상 동작을 마친 뒤 창문 서보를 복구한다.
        if (Flood == 1 && servo_sequence == SERVO_SEQUENCE_IDLE)
        {
            Recovery_Action();
            Flood = 0;
        }
    }

/* USER CODE END 0 */

/**
  * @brief  프로그램 시작 함수
  * @retval 정수형 반환값
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* 마이크로컨트롤러 설정 ---------------------------------------------------*/

  /* 모든 주변장치를 초기화하고 플래시 인터페이스와 시스템 틱을 설정한다. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* 시스템 클럭 설정 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* 설정된 모든 주변장치 초기화 */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();

    lcd_clear();
    lcd_setCursor(0,0);
    lcd_print("Water: 0 cm");

    lcd_setCursor(0,1);
    lcd_print("SAFE");

    last_state = STATE_SAFE;

    // 부팅 시 서보 PWM을 켜지 않는다. 평상시 서보 출력을 꺼 전력 소모와 전압 강하를 방지한다.
    //  서보는 실제 이동이 필요할 때 해당 채널만 켠다.


  /* USER CODE END 2 */

  /* 무한 반복문 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // 서보를 조금씩 이동시키면서 메인 루프가 멈추지 않도록 한다.
	  Servo_SequenceUpdate();

	  // 센서, SPI, 출력 및 LCD 처리는 기존과 같이 200ms 주기로 실행한다.
	  uint32_t now = HAL_GetTick();
	  if ((uint32_t)(now - sensor_update_tick) < 200U) continue;
	  sensor_update_tick = now;

	  // ── 물센서 3채널(IN5/IN6/IN7) 개별 읽기 + 진단 로그 ────────j─────
	  //  IN7(PA7)만 정상이고 IN5/IN6이 오작동인지 확인하기 위한 코드
	  adc_in5 = ADC_ReadChannel(ADC_CHANNEL_5); // PA5
	  adc_in6 = ADC_ReadChannel(ADC_CHANNEL_6); // PA6
	  adc_in7 = ADC_ReadChannel(ADC_CHANNEL_7); // PA7

	  printf("IN5(PA5)=%4lu  IN6(PA6)=%4lu  IN7(PA7)=%4lu\r\n",
	         (unsigned long)adc_in5, (unsigned long)adc_in6, (unsigned long)adc_in7);

	  // 3개 센서 중 '가장 많이 잠긴(값이 큰)' 센서를 대표 수위로 사용
	  //  → 어느 센서든 물을 감지하면 시스템이 반응 (침수 보호에 안전한 방향)
	  waterValue = adc_in5;
	  if (adc_in6 > waterValue) waterValue = adc_in6;
	  if (adc_in7 > waterValue) waterValue = adc_in7;

	  // 센서 세 개가 각각 약 4 cm를 담당하므로 합산 측정 범위는 약 12 cm이다.
	  current_water_cm = ADC_ToSensorCm(adc_in5)
	                   + ADC_ToSensorCm(adc_in6)
	                   + ADC_ToSensorCm(adc_in7);
	  if (current_water_cm > 10U) current_water_cm = 10U;

	  printf("CM: PA5=%u PA6=%u PA7=%u TOTAL=%u\r\n",
	         ADC_ToSensorCm(adc_in5), ADC_ToSensorCm(adc_in6),
	         ADC_ToSensorCm(adc_in7), current_water_cm);

	  // ── 상태 판정: STM32가 물센서 값으로 '직접' 등급을 매긴다 ─────────────
	  //   → ESP32가 없거나 SPI가 끊겨도 LED·부저·서보가 확실히 동작한다.
	  SystemState rx_state;
	  if      (current_water_cm >= 10U) rx_state = STATE_DANGER;
	  else if (current_water_cm >= 7U)  rx_state = STATE_CRITICAL;
	  else if (current_water_cm >= 5U)  rx_state = STATE_CAUTION;
	  else if (current_water_cm >= 3U)  rx_state = STATE_ALERT;
	  else if (current_water_cm >= 1U)  rx_state = STATE_WARNING;
	  else                              rx_state = STATE_SAFE;

	  // 상태 안정화: 같은 등급이 설정 횟수만큼 연속 감지될 때만 확정한다(센서 잡음 방지).
	  if (rx_state == pending_state)
	  {
	      if (state_confirm < STATE_CONFIRM_COUNT) state_confirm++;
	  }
	  else
	  {
	      pending_state = rx_state;
	      state_confirm = 1;
	  }
	  if (state_confirm >= STATE_CONFIRM_COUNT)
	  {
	      current_state = pending_state;
	  }

	  // 다른 상태에서 ALERT로 다시 들어오면 비상 동작을 새로 시작할 수 있게 재무장한다.
	  // 진행 중인 서보 시퀀스가 있으면 Flood=0이 유지되어 종료 직후 자동으로 실행된다.
	  if (current_state != action_last_state)
	  {
	      if (current_state == STATE_ALERT) Flood = 0;
	      action_last_state = current_state;
	  }

	  /*
	   * LCD의 수위와 상태를 하나의 일관된 화면으로 유지한다.
	   * 상태 경계를 넘으면 새 상태가 확정될 때까지 기존 표시 수위를 유지하고,
	   * 확정되는 순간 아래 갱신 구간에서 두 줄을 함께 바꾼다.
	   * 같은 상태 범위 안에서 수위만 변하면 즉시 표시한다.
	   */
	  if (rx_state == current_state)
	  {
	      display_water_cm = current_water_cm;
	  }

	  // 전송 데이터 구성
	  spiTx[0] = 0xAA;
	  spiTx[1] = (waterValue >> 8) & 0xFF;
	  spiTx[2] = waterValue & 0xFF;

	  // ESP32에 센서 값을 전송한다. 수신 데이터는 현재 사용하지 않는다.
	  (void)HAL_SPI_TransmitReceive(&hspi2, spiTx, spiRx, 3, 50);


	  	      // LED는 물이 감지되면 켜고 안전 상태에서만 끈다.
	  	      if (current_state == STATE_SAFE) LED_OFF();
	  	      else                             LED_ON();

	  	      // 3cm 이상이면 부저를 켜고 차단기·가스 밸브·창문 비상 동작을 한 번 시작한다.
	  	      if (display_water_cm >= EMERGENCY_WATER_CM)
	  	      {
	  	          Buzzer_ON();
	  	      }
	  	      else
	  	      {
	  	          // 3cm 미만으로 내려가면 부저를 끄고 서보를 정상 위치로 복구한다.
	  	          Buzzer_OFF();
	  	      }

	  	      // 상태 또는 수위가 변했을 때 두 줄을 하나의 화면으로 갱신한다.
	  	      uint8_t water_cm = display_water_cm;
	  	      if (current_state != last_state || water_cm != last_water_cm) {
	  	          char water_text[17];
	  	          // 작성 중인 첫째/둘째 줄이 따로 보이지 않도록 완성 후 다시 켠다.
	  	          lcd_sendCommand(0x08); // 화면 끄기
	  	          lcd_clear();
	  	          snprintf(water_text, sizeof(water_text), "Water: %u cm", water_cm);
	  	          lcd_setCursor(0, 0); lcd_print(water_text);
	  	          switch (current_state) {
	  	              case STATE_SAFE:
	  	                  lcd_setCursor(0, 1); lcd_print("SAFE");
	  	                  break;
	  	              case STATE_WARNING:
	  	                  lcd_setCursor(0, 1); lcd_print("WARNING!");
	  	                  break;
	  	              case STATE_ALERT:
	  	                  lcd_setCursor(0, 1); lcd_print("ALERT!");
	  	                  break;
	  	              case STATE_CAUTION:
	  	                  lcd_setCursor(0, 1); lcd_print("CAUTION!");
	  	                  break;
	  	              case STATE_CRITICAL:
	  	                  lcd_setCursor(0, 1); lcd_print("CRITICAL!");
	  	                  break;
	  	              case STATE_DANGER:
	  	                  lcd_setCursor(0, 1); lcd_print("DANGER!");
	  	                  break;
	  	          }
	  	          lcd_sendCommand(0x0C); // 화면 켜기
	  	          last_state = current_state; // 현재 상태를 이전 상태로 저장
	  	          last_water_cm = water_cm;
	  	      }

	    }

  /* USER CODE END 3 */
}

/**
  * @brief 시스템 클럭 설정
  * @retval 반환값 없음
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 주 내부 레귤레이터의 출력 전압을 설정한다.
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** 지정된 매개변수와 RCC_OscInitTypeDef 구조체에 따라 RCC 발진기를 초기화한다.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 오버드라이브 모드를 활성화한다.
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** CPU, AHB 및 APB 버스 클럭을 초기화한다.
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  오류 발생 시 실행되는 함수
  * @retval 반환값 없음
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* HAL 오류 반환 상태를 보고하는 사용자 구현을 추가할 수 있다. */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  매개변수 검사 오류가 발생한 소스 파일명과 줄 번호를 보고한다.
  * @param  file: 소스 파일명을 가리키는 포인터
  * @param  line: 오류가 발생한 소스 줄 번호
  * @retval 반환값 없음
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 파일명과 줄 번호를 보고하는 사용자 구현을 추가할 수 있다.
     예: printf("잘못된 매개변수 값: 파일 %s, 줄 %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
