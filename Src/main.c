/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include <stdlib.h>
#include "stm32f3_discovery.h"
#include "glcd_arm.h"
#include "ugui.h"
#include "bmp280.h"
#include "profiling.h"

extern int8_t BMP280_Begin(void);
extern float BMP280_ReadTemperature(void);
extern float BMP280_ReadPressure(void);
extern uint32_t BMP280_ReadPressure_int64(void);
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
static UG_GUI lcd;
struct {
  int32_t v_actual_temp_s32;
  int32_t v_actual_press_u32;
  int32_t v_actual_temp_combined_s32;
  int32_t v_actual_press_combined_u32;
  uint32_t actual_press_u64;
  float v_actual_temp_float;
  float v_actual_press_float;
  double v_actual_temp_double;
  double v_actual_press_double;
  int32_t err_cnt;
} bmp_data;

char str[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
void Error_Handler(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * Period elapsed callback.
 * Прерывание Timer6 (1 kHz)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t delayT6_sec = 0;
  static uint16_t delayT7_sec = 0;
  if (htim->Instance == TIM6)
  {
    // Прерывание по переполнению TIM6
    // Моргаем светодиодом 1Гц
    if (++delayT6_sec >= 500)
    {
      delayT6_sec = 0;
      BSP_LED_Toggle(LED3);
    }
  }
  else if (htim->Instance == TIM7)
  {
    // Прерывание по переполнению TIM7
    // Моргаем светодиодом 1Гц
    if (++delayT7_sec >= 50)
    {
      delayT7_sec = 0;
      BSP_LED_Toggle(LED4);
    }
    //UG_Update();
  }
  else
  {
    Error_Handler();
  }
}


/* Callback function for the main menu */
void window_1_callback(UG_MESSAGE *msg)
{
  if (msg->type == MSG_TYPE_OBJECT)
  {
    if (msg->id == OBJ_TYPE_BUTTON)
    {
      switch (msg->sub_id)
      {
      case BTN_ID_0: /* Toggle green LED */
        if (msg->event == OBJ_EVENT_RELEASED)
          BSP_LED_Toggle(LED5);
        break;
      }
    }
    else if (msg->id == OBJ_TYPE_CHECKBOX)
    {

    }
  }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* На шине SPI1 два устройства, CS поднимаем до инициализации остальных GPIO
     PA4 -> CS BMP280
     PE3 -> CS L3GD20 */
  __GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  __GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2   */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  ST7735_Init();

  UG_Init(&lcd, GLSD_drawPixel, 128, 160);
  UG_DriverRegister(DRIVER_DRAW_LINE, &GLCD_DRIVER_DRAW_LINE);
  UG_DriverRegister(DRIVER_FILL_FRAME, &GLCD_DRIVER_FILL_FRAME);
  UG_DriverRegister(DRIVER_FILL_AREA, &GLCD_DRIVER_FILL_AREA);
  UG_FontSelect(&FONT_7X12);
  UG_FontSetHSpace(0);
  UG_FillScreen(C_BLACK);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bmp280_set_soft_rst();
  HAL_Delay(50);
  if (BMP280_Begin() == -1)
    bmp_data.err_cnt++;

  while (1)
  {
    bmp_data.v_actual_temp_float = BMP280_ReadTemperature();
    bmp_data.actual_press_u64 = BMP280_ReadPressure_int64();
    bmp_data.v_actual_press_float = BMP280_ReadPressure();
    lcd.console.y_pos = lcd.console.y_start;
    lcd.console.x_pos = lcd.console.x_start;

    UG_ConsoleSetForecolor(C_MEDIUM_SPRING_GREEN);
    UG_ConsolePutString("\n");

    sprintf(str, "T32: %u °C \n", bmp_data.v_actual_temp_s32);
    UG_ConsolePutString(str);
    sprintf(str, "P32: %u Pa \n", bmp_data.v_actual_press_u32);
    UG_ConsolePutString(str);

    sprintf(str, "P64: %.3f Pa\n", (float)bmp_data.actual_press_u64 / 256.0f);
    UG_ConsolePutString(str);

    UG_ConsoleSetForecolor(C_GREEN);
    sprintf(str, "Tf: %.3f °C\n", bmp_data.v_actual_temp_float);
    UG_ConsolePutString(str);
    sprintf(str, "Pf: %.3f Pa\n \n", bmp_data.v_actual_press_float);
    UG_ConsolePutString(str);

    UG_ConsoleSetForecolor(C_YELLOW);
    UG_FontSelect(&FONT_10X16);
    float altitude, seaLevelhPa, mmHg;

    mmHg = bmp_data.v_actual_press_float / 133.321995f;
    sprintf(str, "%.2f mmHg\n", mmHg);
    UG_ConsolePutString(str);

    seaLevelhPa = 101325.0f;
    altitude = 44330 * (1.0 - pow(bmp_data.v_actual_press_float / seaLevelhPa, 0.19029f));
    sprintf(str, "%.2f m \n", altitude);
    UG_ConsolePutString(str);

    UG_FontSelect(&FONT_7X12);
    UG_ConsoleSetForecolor(C_TOMATO);
    sprintf(str, " \nerror: %u\n", bmp_data.err_cnt);
    UG_ConsolePutString(str);

    while (BSP_PB_GetState(BUTTON_USER) == 1);

    profiling_start("HAL_DELAY");
    HAL_Delay(50);
    profiling_event("HAL_DELAY 50ms");
    profiling_stop();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; //SPI_BAUDRATEPRESCALER_8 = 9 MBIT/s
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 35999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM7 init function */
/**
 * @brief  Программирование прерывания TIMER7 100 Hz
 * @param  none
 */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 59999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 11;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
//__attribute__((section ("ccmram")))
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
