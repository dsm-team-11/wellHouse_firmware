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
    STATE_DANGER = 3
} SystemState;

SystemState last_state = STATE_SAFE; // 이전 상태 저장
SystemState current_state = STATE_SAFE;
// 세그먼트
uint8_t display[4] = {0, 0, 0, 0};

uint8_t currentDigit = 0;

const uint8_t segTable[10] =
{
    0x3F, //0
    0x06, //1
    0x5B, //2
    0x4F, //3
    0x66, //4
    0x6D, //5
    0x7D, //6
    0x07, //7
    0x7F, //8
    0x6F  //9
};

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

// 각도 설정(실험 후 수정)
#define BREAKER_OFF_ANGLE   35
#define GAS_CLOSE_ANGLE     90
#define WINDOW_CLOSE_ANGLE  70

// 침수 기준 값
#define SAFE_LEVEL      500   // 500 미만은 물이 없는 상태 (0cm)
#define WARNING_LEVEL   1200  // 1200 미만은 미세하게 찬 상태 (1~2cm)
#define ALERT_LEVEL     2500  // 2500 미만은 주의 상태 (3~5cm)
#define DANGER_LEVEL    3500  // 그 이상은 위험 상태 (10cm 이상)

//---------------------------------------------

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

    void Emergency_Action(void)
    {
        Servo_SetAngle(BREAKER_SERVO, BREAKER_OFF_ANGLE);

        Servo_SetAngle(GAS_SERVO, GAS_CLOSE_ANGLE);

        Servo_SetAngle(WINDOW_SERVO, WINDOW_CLOSE_ANGLE);
    }

    void LED_ON(void)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }

    void LED_OFF(void)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
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
  MX_TIM3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();

    lcd_clear();
    lcd_setCursor(0,0);
    lcd_print("Water Level");

    lcd_setCursor(0,1);
    lcd_print("SAFE");

    last_state = STATE_SAFE;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);


  //세그먼트
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // ── 물센서 3채널(IN5/IN6/IN7) 개별 읽기 + 진단 로그 ─────────────
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

	  // 전송 데이터 구성
	  spiTx[0] = 0xAA;
	  spiTx[1] = (waterValue >> 8) & 0xFF;
	  spiTx[2] = waterValue & 0xFF;

	  // 3. ESP32와 SPI 통신 진행 (타임아웃 50ms 설정으로 무한 대기 방지)
	        if (HAL_SPI_TransmitReceive(&hspi2, spiTx, spiRx, 3, 50) == HAL_OK)
	        {
	            // 4. 수신된 데이터 파싱
	            current_state = (SystemState)spiRx[0]; // 첫 바이트: ESP32가 판단한 수위 상태
	            uint8_t server_hour = spiRx[1];        // 두 번째 바이트: 시간(Hour)
	            uint8_t server_minute = spiRx[2];      // 세 번째 바이트: 분(Minute)

	            // 5. 7세그먼트 display 배열에 시간 값 업데이트 (시:분 구조)
	            display[0] = (server_hour / 10) % 10;
	            display[1] = server_hour % 10;
	            display[2] = (server_minute / 10) % 10;
	            display[3] = server_minute % 10;
	        }


	  	  //실행 될 코드
	  	      switch (current_state) {
	  	          case STATE_SAFE:
	  	              Buzzer_OFF();
	  	              LED_OFF();
	  	              Flood = 0;
	  	              break;

	  	          case STATE_WARNING:
	  	              Buzzer_OFF();
	  	              LED_ON();
	  	              // 필요시 조치 추가
	  	              break;

	  	          case STATE_ALERT:
	  	              Buzzer_ON();
	  	              LED_ON();
	  	              // 수위가 높아졌으니 긴급 조치 실행
	  	              if(Flood == 0) {
	  	                  Emergency_Action();
	  	                  Flood = 1;
	  	              }
	  	              break;

	  	          case STATE_DANGER:
	  	              Buzzer_ON();
	  	              LED_ON();
	  	              if(Flood == 0) {
	  	                  Emergency_Action();
	  	                  Flood = 1;
	  	              }
	  	              break;

	  	        default:
	  	           break;
	  	      }

	  	      // 상태가 '변했을 때만' LCD를 새로고침 (깜빡임 방지)
	  	      if (current_state != last_state) {
	  	          lcd_clear();
	  	          switch (current_state) {
	  	              case STATE_SAFE:
	  	                  lcd_setCursor(0, 0); lcd_print("Water Level");
	  	                  lcd_setCursor(0, 1); lcd_print("SAFE");
	  	                  break;
	  	              case STATE_WARNING:
	  	                  lcd_setCursor(0, 0); lcd_print("Water: 2 cm");
	  	                  lcd_setCursor(0, 1); lcd_print("WARNING!");
	  	                  break;
	  	              case STATE_ALERT:
	  	                  lcd_setCursor(0, 0); lcd_print("Water: 3 cm");
	  	                  lcd_setCursor(0, 1); lcd_print("ALERT!");
	  	                  break;
	  	              case STATE_DANGER:
	  	                  lcd_setCursor(0, 0); lcd_print("Water:10 cm");
	  	                  lcd_setCursor(0, 1); lcd_print("DANGER!");
	  	                  break;
	  	          }
	  	          last_state = current_state; // 현재 상태를 이전 상태로 저장
	  	      }

	  	      HAL_Delay(200);
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

void DisplayDigit(uint8_t digit, uint8_t num)
{
	// 0~9 범위를 벗어나는 데이터 방어코드 추가
	    if (num > 9) num = 0;
	    uint8_t seg = segTable[num];

    // 모든 자리 OFF
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);   // D1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // D2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   // D3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // D4

    // 세그먼트 출력
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, (seg & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET); // A
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (seg & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET); // B
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (seg & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET); // C
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (seg & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET); // D
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (seg & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET); // E
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (seg & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET); // F
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (seg & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET); // G

    // 시간 구분을 위해 두 번째 자리(D2) 뒤의 소수점(DP)을 켜주는 로직 추가
        if(digit == 1) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // DP ON
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // DP OFF
        }

    // 자리 선택 (LOW = ON)
    switch(digit)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        DisplayDigit(currentDigit, display[currentDigit]);

        currentDigit++;

        if(currentDigit >= 4)
            currentDigit = 0;
    }
}

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
