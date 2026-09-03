/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "lcd_i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t waterValue = 0;
uint8_t Flood = 0;

// 물센서 3채널 진단용 (SWV 로그 + Live Expressions 관찰용)
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
#define EMERGENCY_WATER_CM  4U           // 부저와 비상 서보가 작동하는 기준 수위
SystemState pending_state = STATE_SAFE;  // 관찰 중인 후보 상태
uint8_t state_confirm = 0;               // 후보 상태가 연속으로 확인된 횟수
uint8_t last_water_cm = 0xFF;
uint8_t current_water_cm = 0;
uint8_t display_water_cm = 0;  // 확정 상태와 함께 LCD에 반영할 수위

//spi 통신 버퍼
uint8_t spiTx[3];
uint8_t spiRx[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ── 디버그 로그: printf 출력을 SWV(ITM)로 보냄 ───────────────────
//  STM32CubeIDE 디버그 세션의 [SWV ITM Data Console] port 0 에서 확인
int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    return ch;
}

// ── ADC 단일 채널 읽기 헬퍼 ─────────────────────────────────────
//  Scan/DMA 미사용 구성이므로, 변환 전에 rank1 채널을 매번 재설정한다
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
#define GAS_CLOSE_ANGLE     90    // 가스 잠금
#define WINDOW_CLOSE_ANGLE  70    // 창문 닫힘
#define GAS_OPEN_ANGLE      0     // 가스 열림
#define WINDOW_OPEN_ANGLE   0     // 창문 열림

// 차단기(전기 차단) 서보: 초기 위치에서 반시계로 20°만 움직였다 돌아오는 '플릭' 동작
#define BREAKER_INIT_ANGLE   90   // 초기(정지) 각도
#define BREAKER_FLICK_ANGLE  70   // 반시계 20° 위치 (초기-20). 방향이 반대면 110으로 바꾸세요.
#define BREAKER_FLICK_HOLD   300  // 이동 후 유지 시간(ms)

// 침수 기준 값
//---------------------------------------------

// 각 서보의 현재 각도(논리값)를 추적한다 (천천히 이동시키기 위함). 부팅 시 중앙 90°.
// 센서 하나는 약 0~4 cm를 측정한다.
// ADC 값을 약 1000 단위로 나눈다. 센서 하나의 결과는 0~4cm이다.
static uint8_t ADC_ToSensorCm(uint32_t adc)
{
    if (adc <= 100U)  return 0;
    if (adc <= 1000U) return 1;
    if (adc <= 3000U) return 2;
    if (adc < 3755U)  return 3;
    return 4; // ADC 3755 이상
}

uint8_t servoAngle[4] = {90, 90, 90, 90};

typedef enum {
    SERVO_SEQUENCE_IDLE = 0,
    SERVO_SEQUENCE_BREAKER_TO_FLICK,
    SERVO_SEQUENCE_BREAKER_HOLD,
    SERVO_SEQUENCE_BREAKER_RETURN,
    SERVO_SEQUENCE_GAS_CLOSE,
    SERVO_SEQUENCE_WINDOW_CLOSE,
    SERVO_SEQUENCE_GAS_OPEN,
    SERVO_SEQUENCE_WINDOW_OPEN
} ServoSequenceState;

ServoSequenceState servo_sequence = SERVO_SEQUENCE_IDLE;
uint32_t servo_step_tick = 0;
uint32_t breaker_hold_tick = 0;
uint32_t sensor_update_tick = 0;

void Servo_SetAngle(uint8_t servo, uint8_t angle)
{
    uint16_t pulse;

    // 0~180° -> 1000~2000us
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

    // 지정한 서보의 PWM만 시작한다. 서보는 이후에도 목표 위치를 유지한다.
    static void Servo_Start(uint8_t servo)
    {
        uint32_t channel = (servo == BREAKER_SERVO) ? TIM_CHANNEL_1
                         : (servo == GAS_SERVO)     ? TIM_CHANNEL_2
                         :                           TIM_CHANNEL_3;
        HAL_TIM_PWM_Start(&htim2, channel);
        Servo_SetAngle(servo, servoAngle[servo]);
    }

    // 호출할 때마다 목표 방향으로 1°만 이동한다. 목표 도달 시 1을 반환한다.
    static uint8_t Servo_StepToward(uint8_t servo, uint8_t target)
    {
        uint8_t current = servoAngle[servo];

        if (current == target) return 1;

        current = (current < target) ? (uint8_t)(current + 1U)
                                     : (uint8_t)(current - 1U);
        servoAngle[servo] = current;
        Servo_SetAngle(servo, current);
        return (current == target);
    }

    // 비상 서보 동작을 시작한다. 실제 이동은 메인 루프의 갱신 함수가 나누어 처리한다.
    static void Emergency_Action(void)
    {
        if (servo_sequence != SERVO_SEQUENCE_IDLE) return;

        servo_sequence = SERVO_SEQUENCE_BREAKER_TO_FLICK;
        servo_step_tick = HAL_GetTick();
        Servo_Start(BREAKER_SERVO);
    }

    // 복구 서보 동작을 시작한다. 차단기는 비상 동작에서 이미 원위치로 복귀한다.
    static void Recovery_Action(void)
    {
        if (servo_sequence != SERVO_SEQUENCE_IDLE) return;

        servo_sequence = SERVO_SEQUENCE_GAS_OPEN;
        servo_step_tick = HAL_GetTick();
        Servo_Start(GAS_SERVO);
    }

    // 15ms마다 한 번 호출 효과를 내어 서보를 하나씩 순차적으로 움직인다.
    static void Servo_SequenceUpdate(void)
    {
        uint32_t now = HAL_GetTick();

        if (servo_sequence == SERVO_SEQUENCE_IDLE) return;

        if (servo_sequence == SERVO_SEQUENCE_BREAKER_HOLD)
        {
            if ((uint32_t)(now - breaker_hold_tick) >= BREAKER_FLICK_HOLD)
            {
                servo_sequence = SERVO_SEQUENCE_BREAKER_RETURN;
                servo_step_tick = now;
            }
            return;
        }

        if ((uint32_t)(now - servo_step_tick) < 15U) return;
        servo_step_tick = now;

        switch (servo_sequence)
        {
            case SERVO_SEQUENCE_BREAKER_TO_FLICK:
                if (Servo_StepToward(BREAKER_SERVO, BREAKER_FLICK_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_BREAKER_HOLD;
                    breaker_hold_tick = now;
                }
                break;

            case SERVO_SEQUENCE_BREAKER_RETURN:
                if (Servo_StepToward(BREAKER_SERVO, BREAKER_INIT_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_GAS_CLOSE;
                    Servo_Start(GAS_SERVO);
                }
                break;

            case SERVO_SEQUENCE_GAS_CLOSE:
                if (Servo_StepToward(GAS_SERVO, GAS_CLOSE_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_WINDOW_CLOSE;
                    Servo_Start(WINDOW_SERVO);
                }
                break;

            case SERVO_SEQUENCE_WINDOW_CLOSE:
                if (Servo_StepToward(WINDOW_SERVO, WINDOW_CLOSE_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_IDLE;
                }
                break;

            case SERVO_SEQUENCE_GAS_OPEN:
                if (Servo_StepToward(GAS_SERVO, GAS_OPEN_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_WINDOW_OPEN;
                    Servo_Start(WINDOW_SERVO);
                }
                break;

            case SERVO_SEQUENCE_WINDOW_OPEN:
                if (Servo_StepToward(WINDOW_SERVO, WINDOW_OPEN_ANGLE))
                {
                    servo_sequence = SERVO_SEQUENCE_IDLE;
                }
                break;

            default:
                break;
        }
    }

    // LED 극성: 원래(active-high) 동작으로 복귀. HIGH=ON, LOW=OFF.
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
    }


    void Buzzer_OFF(void)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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

    // 부팅 시 서보 PWM을 켜지 않는다 → 평상시 서보 전원 OFF (전력 최소화, brownout 방지).
    //  서보는 실제 이동이 필요할 때 해당 채널만 켠다.


  /* USER CODE END 2 */

  /* Infinite loop */
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

	  // ── 상태 판정: STM32가 물센서 값으로 '직접' 등급을 매긴다 ─────────────
	  //   → ESP32가 없거나 SPI가 끊겨도 LED·부저·서보가 확실히 동작한다.
	  SystemState rx_state;
	  if      (current_water_cm >= 10U) rx_state = STATE_DANGER;
	  else if (current_water_cm >= 7U)  rx_state = STATE_CRITICAL;
	  else if (current_water_cm >= 5U)  rx_state = STATE_CAUTION;
	  else if (current_water_cm >= 3U)  rx_state = STATE_ALERT;
	  else if (current_water_cm >= 1U)  rx_state = STATE_WARNING;
	  else                              rx_state = STATE_SAFE;

	  // 디바운스: 같은 등급이 연속 STATE_CONFIRM_COUNT회일 때만 확정 (센서 노이즈 방어)
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


	  	      // LED는 물이 감지되면 켜고, SAFE 상태에서만 끈다.
	  	      if (current_state == STATE_SAFE) LED_OFF();
	  	      else                             LED_ON();

	  	      // 4cm 이상이면 부저를 켜고 차단기·가스 밸브·창문 비상 동작을 한 번 시작한다.
	  	      if (display_water_cm >= EMERGENCY_WATER_CM)
	  	      {
	  	          Buzzer_ON();
	  	          if (Flood == 0 && servo_sequence == SERVO_SEQUENCE_IDLE)
	  	          {
	  	              Emergency_Action();
	  	              Flood = 1;
	  	          }
	  	      }
	  	      else
	  	      {
	  	          // 4cm 미만으로 내려가면 부저를 끄고 서보를 정상 위치로 복구한다.
	  	          Buzzer_OFF();
	  	          if (Flood == 1 && servo_sequence == SERVO_SEQUENCE_IDLE)
	  	          {
	  	              Recovery_Action();
	  	              Flood = 0;
	  	          }
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
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
