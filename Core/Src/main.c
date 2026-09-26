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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "OLED.h"
#include "track.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  OLED_Init();

  OLED_Clear();

	float Kp = 10.0f;      // 比例系数（主力纠偏）
	float Ki = 0.15f;      // 积分系数（修正物理不对称，微调角色）
	float Kd = 8.0f;       // 微分系数（抑制震荡）
	float sum_error = 0.0f;    // 积分累加
	float last_error = 0.0f;   // 上一次的偏差
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // 1. 读取传感器（带去抖滤波）
    uint8_t l1 = Track_L1();
    uint8_t l2 = Track_L2();
    uint8_t r2 = Track_R2();
    uint8_t r1 = Track_R1();

    // 2. 计算偏差error
    float error = 0.0f;
    if(l1 == 1) error += -3.0f;
    if(l2 == 1) error += -1.0f;
    if(r2 == 1) error += +1.0f;
    if(r1 == 1) error += +3.0f;

    // 3. �?测是否丢线（全白）并做渐进式搜索
    static uint16_t loss_count = 0;  // 丢线持续次数（每5ms+1�?
    uint8_t line_lost = 0;
    if(l1==0 && l2==0 && r2==0 && r1==0)
    {
      line_lost = 1;
      loss_count++;  // 丢得越久，计数越�?

      // 确定搜索方向：线�?后在哪边就往哪边�?
      float search_dir;
      if(last_error > 0)      search_dir = 1.0f;   // 线在右边，往右找
      else if(last_error < 0) search_dir = -1.0f;  // 线在左边，往左找
      else                    search_dir = 1.0f;   // 兜底：不知道就右�?

      // 丢线越久转越猛：起步3，每�?5ms�?0.5，上�?8
      float search_power = 3.0f + (float)loss_count * 0.5f;
      if(search_power > 8.0f) search_power = 8.0f;

      error = search_dir * search_power;
    }
    else
    {
      loss_count = 0;  // 找到线了，重置丢线计�?
    }

    // 4. PID计算（丢线时冻结积分，防止饱和）
    if(!line_lost)
    {
      sum_error += error;  // 正常巡线才累加积�?
    }
    // 积分限幅
    if(sum_error > 3) sum_error = 3;
    if(sum_error < -3) sum_error = -3;

    float diff_error = error - last_error;
    float output = Kp*error + Ki*sum_error + Kd*diff_error;

    // 输出限幅
    if(output > 400) output = 400;
    if(output < -400) output = -400;

    // 5. 弯道自�?�应降�?�：偏差越大 = 弯越�? = 速度越低
    float abs_error = (error > 0) ? error : -error;  // 取绝对�??
    int base_speed;
    if(abs_error > 3.0f) {
        base_speed = 250;   // 急弯/丢线搜索：慢速过
    } else if(abs_error > 1.5f) {
        base_speed = 350;   // 缓弯：中�?
    } else {
        base_speed = 450;   // 直线：全�?
    }

    // 6. 控制电机
    int left_speed = base_speed - output;
    int right_speed = base_speed + output;

    // 防止速度溢出
    if(left_speed < 0) left_speed = 0;
    if(right_speed < 0) right_speed = 0;
    if(left_speed > 999) left_speed = 999;
    if(right_speed > 999) right_speed = 999;

    motor_setspeed(left_speed, right_speed);

    // 7. 保存上一次的偏差
    last_error = error;

    // 8. OLED显示调试信息（每200ms刷新�?次）
    static uint32_t oled_timer = 0;
    if(HAL_GetTick() - oled_timer > 500)
    {
      oled_timer = HAL_GetTick();

      OLED_ShowNum(2,0,error*10,2);     // 偏差×10
      OLED_ShowNum(4,0,base_speed,4);   // 当前基础速度
    }

    // 9. 固定控制周期 5ms�?200Hz），保证PID积分/微分按时间计�?
    HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
