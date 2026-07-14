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
#include "tim.h"
#include "gpio.h"
#include "lcd_i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

// LCD 깜빡임 방지를 위한 상태 정의
typedef enum {
    STATE_SAFE,
    STATE_WARNING,
    STATE_ALERT,
    STATE_DANGER
} SystemState;

SystemState last_state = STATE_SAFE; // 이전 상태 저장

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  lcd_print("Water Level");
  lcd_print("SAFE");

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
  /* USER CODE BEGIN 2 */
  lcd_init();

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 물센서 ADC 읽기
	  HAL_ADC_Start(&hadc1);

	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	  waterValue = HAL_ADC_GetValue(&hadc1);

	  HAL_ADC_Stop(&hadc1);

	  SystemState current_state;


	  // 수위에 따른 상태 결정
	      if (waterValue < SAFE_LEVEL) {
	          current_state = STATE_SAFE;
	      } else if (waterValue < WARNING_LEVEL) {
	          current_state = STATE_WARNING;
	      } else if (waterValue < ALERT_LEVEL) { // ALERT_LEVEL(1800~2500) 조건 추가
	          current_state = STATE_ALERT;
	      } else {
	          current_state = STATE_DANGER;
	      }

	      // 상태에 따른 하드웨어 제어 (부저, LED, 서보)
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
