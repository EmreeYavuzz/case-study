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
// LED pins (STM32F429-DISCO)
#define LED_GREEN_PORT GPIOG
#define LED_GREEN_PIN  GPIO_PIN_13
#define LED_RED_PORT   GPIOG
#define LED_RED_PIN    GPIO_PIN_14
// Self-test limits for ±245 dps: 130 dps ± %20 tolerance
#define SELF_TEST_MIN_LSB 10400  // ~91 dps (130 - 30%)
#define SELF_TEST_MAX_LSB 18600  // ~163 dps (130 + 30%)

// UART handle (external)
extern UART_HandleTypeDef huart1;

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
static int16_t gyro_data[3] = {0};   // X, Y, Z (raw değerler)
static uint8_t temp_raw = 0;         // Ham sicaklik değeri
static float temp_celsius = 0.0f;    // Sicaklik (°C)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Turn on LED
 * @param  led_id: 0=Green, 1=Red
 */
void i3g4250d_platform_led_on(uint8_t led_id)
{
  if (led_id == PROPERTY_DISABLE) {
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
  } else if (led_id == PROPERTY_ENABLE) {
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
  }
}

/**
 * @brief  Turn off LED
 * @param  led_id: 0=Green, 1=Red
 */
void i3g4250d_platform_led_off(uint8_t led_id)
{
  if (led_id == PROPERTY_DISABLE) {
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
  } else if (led_id == PROPERTY_ENABLE) {
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
  }
}

/**
 * @brief  Print message via UART
 * @param  msg: Null-terminated string
 */
void i3g4250d_platform_print(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

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
    return -1;  // SPI hatasi
  }
  
  if (whoami_id != I3G4250D_ID) {
    return -2;  // Yanliş cihaz
  }
  
  // 2. Power-on mode (PD bit = 1) -
  
  ret = i3g4250d_power_mode_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) {
    return -3;
  }
  // 3. Enable all axes (X, Y, Z)
  ret = i3g4250d_axis_x_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return -4; }
  
  ret = i3g4250d_axis_y_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return -5; }
  
  ret = i3g4250d_axis_z_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return -6; }
  
  // 4. Output data rate: 100 Hz
  ret = i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_100Hz);
  if (ret != 0) {
    return -7;
  }
  
  // 5. Full-scale: ±245 dps
  ret = i3g4250d_full_scale_set(&gyro_ctx, I3G4250D_245dps);
  if (ret != 0) {
    return -8;
  }
  
  // 6. Test: Sicaklik oku (opsiyonel)
  ret = i3g4250d_temperature_raw_get(&gyro_ctx, &temp_raw);
  if (ret != 0) {
    return -9;  // Sicaklik okuma hatasi
  }


  // 7. Kisa delay (sensör boot için)
  gyro_ctx.mdelay(100);
  
  return 0;  // Basarili
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

/**
  * @brief  Gyroscope self-test procedure
  * @retval 0: OK (sensor healthy), -1: FAIL (sensor faulty)
  */
static int32_t gyro_self_test(void)
{
  int16_t normal_data[3] = {0};
  int16_t selftest_data[3] = {0};
  int16_t diff_x, diff_y, diff_z;
  i3g4250d_st_t mode;
  char msg[100];
  
  i3g4250d_platform_print("\r\n--- SELF-TEST START ---\r\n");
  
  // 1. Read normal mode (self-test OFF)
  i3g4250d_self_test_set(&gyro_ctx, I3G4250D_GY_ST_DISABLE);

  // 2. Gerçekten aktif oldu mu kontrol et (SPI doğrulama)
  i3g4250d_self_test_get(&gyro_ctx, &mode);
  if (mode != I3G4250D_GY_ST_DISABLE) {
    i3g4250d_platform_print("ERROR: Self-test activation failed\r\n");
    return -1;
  }

  gyro_ctx.mdelay(100);
  
  gyro_read(normal_data);
  sprintf(msg, "Normal mode: X=%d Y=%d Z=%d\r\n", 
          normal_data[0], normal_data[1], normal_data[2]);
  i3g4250d_platform_print(msg);
  
  // 2. Read self-test mode (positive actuation)
  i3g4250d_self_test_set(&gyro_ctx, I3G4250D_GY_ST_POSITIVE);

  // 2. Gerçekten aktif oldu mu kontrol et (SPI doğrulama)
  i3g4250d_self_test_get(&gyro_ctx, &mode);
  if (mode != I3G4250D_GY_ST_POSITIVE) {
    i3g4250d_platform_print("ERROR: Self-test activation failed\r\n");
    return -1;
  }

  gyro_ctx.mdelay(100);
  
  gyro_read(selftest_data);
  sprintf(msg, "Self-test mode: X=%d Y=%d Z=%d\r\n", 
          selftest_data[0], selftest_data[1], selftest_data[2]);
  i3g4250d_platform_print(msg);
  
  // 3. Disable self-test
  i3g4250d_self_test_set(&gyro_ctx, I3G4250D_GY_ST_DISABLE);
  gyro_ctx.mdelay(50);
  
  // 4. Calculate absolute differences
  diff_x = selftest_data[0] - normal_data[0];
  diff_y = selftest_data[1] - normal_data[1];
  diff_z = selftest_data[2] - normal_data[2];
  
  if (diff_x < 0) diff_x = -diff_x;
  if (diff_y < 0) diff_y = -diff_y;
  if (diff_z < 0) diff_z = -diff_z;
  
  sprintf(msg, "Delta: X=%d Y=%d Z=%d\r\n", diff_x, diff_y, diff_z);
  i3g4250d_platform_print(msg);
  
  // 5. Validate against datasheet limits
  if ((diff_x < SELF_TEST_MIN_LSB || diff_x > SELF_TEST_MAX_LSB) ||
      (diff_y < SELF_TEST_MIN_LSB || diff_y > SELF_TEST_MAX_LSB) ||
      (diff_z < SELF_TEST_MIN_LSB || diff_z > SELF_TEST_MAX_LSB)) {
    i3g4250d_platform_print("SELF-TEST FAILED - Sensor faulty\r\n\r\n");
    return -1;
  }
  
  i3g4250d_platform_print("SELF-TEST PASSED - Sensor OK\r\n\r\n");
  return 0;
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

  // Platform layer'i başlat
  i3g4250d_platform_init(&gyro_ctx, &hspi5);

  // Gyroscope başlat
  int32_t status = gyro_init();

  if (status == 0) {

    i3g4250d_platform_led_on(0);  // Green LED
    
    //  SELF-TEST ÇALIŞTIIR (başlangıçta bir kez)
    if (gyro_self_test() != 0) {
      // Self-test başarısız, red LED
      i3g4250d_platform_led_off(0);
      i3g4250d_platform_led_on(PROPERTY_ENABLE);
      while(1);  // Donma (sensör arızalı!)
    }

    i3g4250d_platform_print("\r\n=== GYROSCOPE INITIALIZED ===\r\n");
    
    char msg[100];
    sprintf(msg, "WHO_AM_I: 0x%02X (OK)\r\n\r\n", whoami_id);
    i3g4250d_platform_print(msg);
    
    while(1) {
      uint8_t data_ready = 0;
      
      // Veri hazir mi kontrol et
      i3g4250d_flag_data_ready_get(&gyro_ctx, &data_ready);
      
      if (data_ready) {
        // Yeni veri hazir, oku!
        gyro_read(gyro_data);
        
        // mdps'ye çevir
        int x_mdps = i3g4250d_from_fs245dps_to_mdps(gyro_data[0]);
        int y_mdps = i3g4250d_from_fs245dps_to_mdps(gyro_data[1]);
        int z_mdps = i3g4250d_from_fs245dps_to_mdps(gyro_data[2]);
        
        // Sicaklik oku
        i3g4250d_temperature_raw_get(&gyro_ctx, &temp_raw);
        temp_celsius = i3g4250d_from_lsb_to_celsius((int8_t)temp_raw);
        int temp_int = (int)temp_celsius;
        
        // UART'a gönder
        sprintf(msg, "X:%d Y:%d Z:%d MDPS TEMP:%d\r\n",
                x_mdps, y_mdps, z_mdps, temp_int);
        i3g4250d_platform_print(msg);

      }
      
      i3g4250d_platform_delay(100);  // Kisa delay (CPU'yu meşgul etme)
}
  } else {  //  Error
    i3g4250d_platform_led_on(PROPERTY_ENABLE);  // Red LED
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
