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
#include "cmsis_os.h"
#include "crc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i3g4250d_reg.h"
#include "i3g4250d_platform.h"
#include <stdio.h>  
#include <string.h>  
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

// Gyroscope context
static stmdev_ctx_t gyro_ctx;
// Test değişkenleri
static uint8_t whoami_id = 0;
static int16_t gyro_data[3] = {0};  // X, Y, Z (raw değerler)
static float gyro_dps[3] = {0};     // X, Y, Z (derece/saniye)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
  * @brief  Initialize gyroscope
  * @retval 0: OK, -1: Error
  */
static int32_t gyro_init(void)
{
  int32_t ret;
  
  // 1. WHO_AM_I kontrolü
  ret = i3g4250d_device_id_get(&gyro_ctx, &whoami_id);
  if (ret != 0) {
    return -1;  // SPI hatası
  }
  
  if (whoami_id != I3G4250D_ID) {
    return -2;  // Yanlış cihaz
  }
  
  // 2. Power-on, 100 Hz ODR, all axes enable
  ret = i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_100Hz);
  if (ret != 0) {
    return -3;
  }
  
  // 3. Full-scale: ±245 dps
  ret = i3g4250d_full_scale_set(&gyro_ctx, I3G4250D_245dps);
  if (ret != 0) {
    return -4;
  }
  
  // 4. Kısa delay (sensör boot için)
  gyro_ctx.mdelay(100);
  
  return 0;  // Başarılı
}

/**
  * @brief  Read gyroscope angular rate
  * @param  data  Pointer to 3x int16_t array [X, Y, Z]
  * @retval 0: OK, -1: Error
  */
static int32_t gyro_read(int16_t *data)
{
  return i3g4250d_angular_rate_raw_get(&gyro_ctx, data);
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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();  
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Platform layer'ı başlat
  i3g4250d_platform_init(&gyro_ctx, &hspi5);

  // Gyroscope başlat
  int32_t status = gyro_init();

  if (status == 0) {
    // ✅ Platform-agnostic LED control
    i3g4250d_platform_led_on(0);  // Green LED
    
    // ✅ Platform-agnostic print
    i3g4250d_platform_print("\r\n=== GYROSCOPE INITIALIZED ===\r\n");
    
    char msg[100];
    sprintf(msg, "WHO_AM_I: 0x%02X (OK)\r\n\r\n", whoami_id);
    i3g4250d_platform_print(msg);
    
    while(1) {
      gyro_read(gyro_data);
      
      gyro_dps[0] = i3g4250d_from_fs245dps_to_mdps(gyro_data[0]) / 1000.0f;
      gyro_dps[1] = i3g4250d_from_fs245dps_to_mdps(gyro_data[1]) / 1000.0f;
      gyro_dps[2] = i3g4250d_from_fs245dps_to_mdps(gyro_data[2]) / 1000.0f;
      
      int x_int = (int)(gyro_dps[0] * 100);
      int y_int = (int)(gyro_dps[1] * 100);
      int z_int = (int)(gyro_dps[2] * 100);

      sprintf(msg, "X:%4d.%02d  Y:%4d.%02d  Z:%4d.%02d DPS\r\n",
              x_int/100, x_int%100,
              y_int/100, y_int%100,
              z_int/100, z_int%100);
      i3g4250d_platform_print(msg);
      
      i3g4250d_platform_delay(200);
    }
  } else {  // ❌ Error
    i3g4250d_platform_led_on(1);  // Red LED
    while(1);
  }

  /* USER CODE END 2 */


  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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


// Printf SWV için redirect
int _write(int file, char *ptr, int len)
{
  for(int i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
