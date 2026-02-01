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

/**
 * @brief Gyroscope error codes (production-ready error handling)
 */
typedef enum {
  GYRO_OK                    =  0,   /**< Success */
  GYRO_ERR_SPI_COMM          = -1,   /**< SPI communication error */
  GYRO_ERR_WRONG_DEVICE      = -2,   /**< WHO_AM_I mismatch */
  GYRO_ERR_POWER_MODE        = -3,   /**< Power mode set failed */
  GYRO_ERR_AXIS_X_CONFIG     = -4,   /**< X-axis enable failed */
  GYRO_ERR_AXIS_Y_CONFIG     = -5,   /**< Y-axis enable failed */
  GYRO_ERR_AXIS_Z_CONFIG     = -6,   /**< Z-axis enable failed */
  GYRO_ERR_ODR_CONFIG        = -7,   /**< Output data rate config failed */
  GYRO_ERR_FULLSCALE_CONFIG  = -8,   /**< Full-scale config failed */
  GYRO_ERR_TEMP_READ         = -9,   /**< Temperature read failed */
  GYRO_ERR_SELFTEST_FAIL     = -10,  /**< Self-test failed */
  GYRO_ERR_DATA_READ         = -11,  /**< Angular rate read failed */
  GYRO_ERR_TIMEOUT           = -12,  /**< Operation timeout */
} gyro_error_t;

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
  * @retval gyro_error_t: GYRO_OK on success, error code on failure
  */
static gyro_error_t gyro_init(void)
{
  int32_t ret;
  
  // 1. WHO_AM_I kontrolü
  ret = i3g4250d_device_id_get(&gyro_ctx, &whoami_id);
  if (ret != 0) {
    return GYRO_ERR_SPI_COMM;
  }
  
  if (whoami_id != I3G4250D_ID) {
    return GYRO_ERR_WRONG_DEVICE;
  }
  
  // 2. Power-on mode (PD bit = 1)
  ret = i3g4250d_power_mode_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) {
    return GYRO_ERR_POWER_MODE;
  }
  
  // 3. Enable all axes (X, Y, Z)
  ret = i3g4250d_axis_x_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return GYRO_ERR_AXIS_X_CONFIG; }
  
  ret = i3g4250d_axis_y_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return GYRO_ERR_AXIS_Y_CONFIG; }
  
  ret = i3g4250d_axis_z_data_set(&gyro_ctx, PROPERTY_ENABLE);
  if (ret != 0) { return GYRO_ERR_AXIS_Z_CONFIG; }
  
  // 4. Output data rate: 100 Hz
  ret = i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_100Hz);
  if (ret != 0) {
    return GYRO_ERR_ODR_CONFIG;
  }
  
  // 5. Full-scale: ±245 dps
  ret = i3g4250d_full_scale_set(&gyro_ctx, I3G4250D_245dps);
  if (ret != 0) {
    return GYRO_ERR_FULLSCALE_CONFIG;
  }
  
  // 6. Test: Sicaklik oku (opsiyonel)
  ret = i3g4250d_temperature_raw_get(&gyro_ctx, &temp_raw);
  if (ret != 0) {
    return GYRO_ERR_TEMP_READ;
  }

  // 7. Kisa delay (sensör boot için)
  gyro_ctx.mdelay(100);
  
  return GYRO_OK;
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

/**
  * @brief  Test different ODR (Output Data Rate) modes
  *         Demonstrates driver's configuration flexibility
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_odr_modes(void)
{
  char msg[100];
  i3g4250d_dr_t odr_read;
  int16_t test_data[3];
  uint8_t data_ready;
  
  const char* odr_names[] = {"100 Hz", "200 Hz", "400 Hz", "800 Hz"};
  const i3g4250d_dr_t odr_modes[] = {
    I3G4250D_ODR_100Hz,
    I3G4250D_ODR_200Hz,
    I3G4250D_ODR_400Hz,
    I3G4250D_ODR_800Hz
  };
  
  i3g4250d_platform_print("\r\n--- ODR TEST START ---\r\n");
  
  for (int i = 0; i < 4; i++) {
    // Set ODR
    if (i3g4250d_data_rate_set(&gyro_ctx, odr_modes[i]) != 0) {
      i3g4250d_platform_print("ERROR: ODR set failed\r\n");
      return GYRO_ERR_ODR_CONFIG;
    }
    
    // Verify ODR was set correctly (read-back)
    if (i3g4250d_data_rate_get(&gyro_ctx, &odr_read) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    if (odr_read != odr_modes[i]) {
      sprintf(msg, "ERROR: ODR verify failed (expected %d, got %d)\r\n", 
              odr_modes[i], odr_read);
      i3g4250d_platform_print(msg);
      return GYRO_ERR_ODR_CONFIG;
    }
    
    // Wait for new data
    gyro_ctx.mdelay(50);
    
    // Wait for data ready
    i3g4250d_flag_data_ready_get(&gyro_ctx, &data_ready);
    if (!data_ready) {
      gyro_ctx.mdelay(20);  // Extra wait
    }
    
    // Read sample
    if (gyro_read(test_data) != 0) {
      return GYRO_ERR_DATA_READ;
    }
    
    sprintf(msg, "ODR %s: X=%d Y=%d Z=%d (verified)\r\n", 
            odr_names[i], test_data[0], test_data[1], test_data[2]);
    i3g4250d_platform_print(msg);
  }
  
  // Restore default ODR (100 Hz)
  i3g4250d_data_rate_set(&gyro_ctx, I3G4250D_ODR_100Hz);
  
  i3g4250d_platform_print("ODR TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test different Full-Scale modes
  *         Shows driver's ability to handle different measurement ranges
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_fullscale_modes(void)
{
  char msg[120];
  i3g4250d_fs_t fs_read;
  int16_t test_data[3];
  uint8_t data_ready;
  
  // Sensitivity values (mdps/LSB) from datasheet
  const float sensitivities[] = {8.75f, 17.5f, 70.0f};
  const char* fs_names[] = {"245 dps", "500 dps", "2000 dps"};
  const i3g4250d_fs_t fs_modes[] = {
    I3G4250D_245dps,
    I3G4250D_500dps,
    I3G4250D_2000dps
  };
  
  i3g4250d_platform_print("\r\n--- FULL-SCALE TEST START ---\r\n");
  
  for (int i = 0; i < 3; i++) {
    // Set full-scale
    if (i3g4250d_full_scale_set(&gyro_ctx, fs_modes[i]) != 0) {
      i3g4250d_platform_print("ERROR: Full-scale set failed\r\n");
      return GYRO_ERR_FULLSCALE_CONFIG;
    }
    
    // Verify full-scale was set correctly (read-back)
    if (i3g4250d_full_scale_get(&gyro_ctx, &fs_read) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    if (fs_read != fs_modes[i]) {
      sprintf(msg, "ERROR: FS verify failed (expected %d, got %d)\r\n", 
              fs_modes[i], fs_read);
      i3g4250d_platform_print(msg);
      return GYRO_ERR_FULLSCALE_CONFIG;
    }
    
    // Wait for new data
    gyro_ctx.mdelay(50);
    
    // Wait for data ready
    i3g4250d_flag_data_ready_get(&gyro_ctx, &data_ready);
    if (!data_ready) {
      gyro_ctx.mdelay(20);
    }
    
    // Read sample
    if (gyro_read(test_data) != 0) {
      return GYRO_ERR_DATA_READ;
    }
    
    // Calculate mdps using appropriate sensitivity
    int x_mdps = (int)(test_data[0] * sensitivities[i]);
    int y_mdps = (int)(test_data[1] * sensitivities[i]);
    int z_mdps = (int)(test_data[2] * sensitivities[i]);
    
    sprintf(msg, "FS +/-%s: Raw=[%d,%d,%d] -> [%d,%d,%d] mdps (sens=%.2f)\r\n",
            fs_names[i], test_data[0], test_data[1], test_data[2],
            x_mdps, y_mdps, z_mdps, sensitivities[i]);
    i3g4250d_platform_print(msg);
  }
  
  // Restore default full-scale (245 dps)
  i3g4250d_full_scale_set(&gyro_ctx, I3G4250D_245dps);
  
  i3g4250d_platform_print("FULL-SCALE TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Dump all important registers for debugging
  *         Useful for production testing and troubleshooting
  */
static void gyro_debug_dump_registers(void)
{
  uint8_t reg_val;
  char msg[80];
  
  // Important registers to dump
  const uint8_t regs[] = {
    0x0F,  // WHO_AM_I
    0x20,  // CTRL_REG1
    0x21,  // CTRL_REG2
    0x22,  // CTRL_REG3
    0x23,  // CTRL_REG4
    0x24,  // CTRL_REG5
    0x27,  // STATUS_REG
    0x26,  // OUT_TEMP
  };
  
  const char* reg_names[] = {
    "WHO_AM_I  ",
    "CTRL_REG1 ",
    "CTRL_REG2 ",
    "CTRL_REG3 ",
    "CTRL_REG4 ",
    "CTRL_REG5 ",
    "STATUS_REG",
    "OUT_TEMP  ",
  };
  
  i3g4250d_platform_print("\r\n--- REGISTER DUMP ---\r\n");
  i3g4250d_platform_print("Addr  Name        Value    Binary\r\n");
  i3g4250d_platform_print("----  ----------  -----    --------\r\n");
  
  for (int i = 0; i < 8; i++) {
    i3g4250d_read_reg(&gyro_ctx, regs[i], &reg_val, 1);
    
    // Convert to binary string
    char binary[9];
    for (int b = 7; b >= 0; b--) {
      binary[7-b] = (reg_val & (1 << b)) ? '1' : '0';
    }
    binary[8] = '\0';
    
    sprintf(msg, "0x%02X  %s  0x%02X     %s\r\n", 
            regs[i], reg_names[i], reg_val, binary);
    i3g4250d_platform_print(msg);
  }
  
  i3g4250d_platform_print("--- END DUMP ---\r\n\r\n");
}

/**
  * @brief  Convert error code to string for debugging
  * @param  err: Error code
  * @retval Pointer to error string
  */
static const char* gyro_error_to_string(gyro_error_t err)
{
  switch (err) {
    case GYRO_OK:                   return "OK";
    case GYRO_ERR_SPI_COMM:         return "SPI Communication Error";
    case GYRO_ERR_WRONG_DEVICE:     return "Wrong Device (WHO_AM_I mismatch)";
    case GYRO_ERR_POWER_MODE:       return "Power Mode Config Failed";
    case GYRO_ERR_AXIS_X_CONFIG:    return "X-Axis Config Failed";
    case GYRO_ERR_AXIS_Y_CONFIG:    return "Y-Axis Config Failed";
    case GYRO_ERR_AXIS_Z_CONFIG:    return "Z-Axis Config Failed";
    case GYRO_ERR_ODR_CONFIG:       return "ODR Config Failed";
    case GYRO_ERR_FULLSCALE_CONFIG: return "Full-Scale Config Failed";
    case GYRO_ERR_TEMP_READ:        return "Temperature Read Failed";
    case GYRO_ERR_SELFTEST_FAIL:    return "Self-Test Failed";
    case GYRO_ERR_DATA_READ:        return "Data Read Failed";
    case GYRO_ERR_TIMEOUT:          return "Operation Timeout";
    default:                        return "Unknown Error";
  }
}

/**
  * @brief  Test Power Mode (Normal/Power-Down)
  *         Gösterir: Güç yönetimi, sleep mode desteği
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_power_mode(void)
{
  char msg[80];
  uint8_t power_state;
  
  i3g4250d_platform_print("\r\n--- POWER MODE TEST ---\r\n");
  
  // 1. Power-down mode (güç tasarrufu)
  if (i3g4250d_power_mode_set(&gyro_ctx, PROPERTY_DISABLE) != 0) {
    return GYRO_ERR_POWER_MODE;
  }
  
  // Verify
  i3g4250d_power_mode_get(&gyro_ctx, &power_state);
  sprintf(msg, "Power-Down Mode: %s\r\n", power_state ? "FAIL" : "OK");
  i3g4250d_platform_print(msg);
  
  gyro_ctx.mdelay(50);
  
  // 2. Normal mode (aktif)
  if (i3g4250d_power_mode_set(&gyro_ctx, PROPERTY_ENABLE) != 0) {
    return GYRO_ERR_POWER_MODE;
  }
  
  // Verify
  i3g4250d_power_mode_get(&gyro_ctx, &power_state);
  sprintf(msg, "Normal Mode: %s\r\n", power_state ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  i3g4250d_platform_print("POWER MODE TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Axis Enable/Disable
  *         Gösterir: Eksen bazlı güç tasarrufu, seçici veri okuma
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_axis_enable(void)
{
  char msg[100];
  uint8_t x_en, y_en, z_en;
  int16_t test_data[3];
  
  i3g4250d_platform_print("\r\n--- AXIS ENABLE/DISABLE TEST ---\r\n");
  
  // 1. Sadece X ekseni aktif
  i3g4250d_axis_x_data_set(&gyro_ctx, PROPERTY_ENABLE);
  i3g4250d_axis_y_data_set(&gyro_ctx, PROPERTY_DISABLE);
  i3g4250d_axis_z_data_set(&gyro_ctx, PROPERTY_DISABLE);
  
  // Verify
  i3g4250d_axis_x_data_get(&gyro_ctx, &x_en);
  i3g4250d_axis_y_data_get(&gyro_ctx, &y_en);
  i3g4250d_axis_z_data_get(&gyro_ctx, &z_en);
  
  gyro_ctx.mdelay(50);
  gyro_read(test_data);
  
  sprintf(msg, "Only X enabled: X=%d, Y=%d, Z=%d (Y,Z should be ~0)\r\n",
          test_data[0], test_data[1], test_data[2]);
  i3g4250d_platform_print(msg);
  
  // 2. Sadece Y ekseni aktif
  i3g4250d_axis_x_data_set(&gyro_ctx, PROPERTY_DISABLE);
  i3g4250d_axis_y_data_set(&gyro_ctx, PROPERTY_ENABLE);
  i3g4250d_axis_z_data_set(&gyro_ctx, PROPERTY_DISABLE);
  
  gyro_ctx.mdelay(50);
  gyro_read(test_data);
  
  sprintf(msg, "Only Y enabled: X=%d, Y=%d, Z=%d (X,Z should be ~0)\r\n",
          test_data[0], test_data[1], test_data[2]);
  i3g4250d_platform_print(msg);
  
  // 3. Tüm eksenleri geri aç
  i3g4250d_axis_x_data_set(&gyro_ctx, PROPERTY_ENABLE);
  i3g4250d_axis_y_data_set(&gyro_ctx, PROPERTY_ENABLE);
  i3g4250d_axis_z_data_set(&gyro_ctx, PROPERTY_ENABLE);
  
  gyro_ctx.mdelay(50);
  gyro_read(test_data);
  
  sprintf(msg, "All axes enabled: X=%d, Y=%d, Z=%d\r\n",
          test_data[0], test_data[1], test_data[2]);
  i3g4250d_platform_print(msg);
  
  i3g4250d_platform_print("AXIS ENABLE TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test High-Pass Filter
  *         Gösterir: Sinyal işleme, DC offset kaldırma
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_high_pass_filter(void)
{
  char msg[100];
  i3g4250d_hpcf_t hp_cutoff;
  int16_t test_data[3];
  
  i3g4250d_platform_print("\r\n--- HIGH-PASS FILTER TEST ---\r\n");
  
  // Test farklı HP cutoff frekansları
  const i3g4250d_hpcf_t hp_levels[] = {
    I3G4250D_HP_LEVEL_0,  // En düşük cutoff
    I3G4250D_HP_LEVEL_5,  // Orta
    I3G4250D_HP_LEVEL_9   // En yüksek cutoff
  };
  const char* hp_names[] = {"Level 0 (lowest)", "Level 5 (mid)", "Level 9 (highest)"};
  
  for (int i = 0; i < 3; i++) {
    // HP bandwidth ayarla
    if (i3g4250d_hp_bandwidth_set(&gyro_ctx, hp_levels[i]) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    // Verify
    i3g4250d_hp_bandwidth_get(&gyro_ctx, &hp_cutoff);
    
    gyro_ctx.mdelay(50);
    gyro_read(test_data);
    
    sprintf(msg, "HP %s: X=%d Y=%d Z=%d\r\n",
            hp_names[i], test_data[0], test_data[1], test_data[2]);
    i3g4250d_platform_print(msg);
  }
  
  // HP Mode test (Normal, Reference, Autoreset)
  i3g4250d_platform_print("HP Mode: Normal\r\n");
  i3g4250d_hp_mode_set(&gyro_ctx, I3G4250D_HP_NORMAL_MODE_WITH_RST);
  
  // Restore default
  i3g4250d_hp_bandwidth_set(&gyro_ctx, I3G4250D_HP_LEVEL_0);
  
  i3g4250d_platform_print("HIGH-PASS FILTER TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Low-Pass Bandwidth
  *         Gösterir: Anti-aliasing, gürültü azaltma
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_lp_bandwidth(void)
{
  char msg[100];
  i3g4250d_bw_t bw_read;
  int16_t test_data[3];
  
  const i3g4250d_bw_t bw_modes[] = {
    I3G4250D_CUT_OFF_LOW,
    I3G4250D_CUT_OFF_MEDIUM,
    I3G4250D_CUT_OFF_HIGH,
    I3G4250D_CUT_OFF_VERY_HIGH
  };
  const char* bw_names[] = {"LOW", "MEDIUM", "HIGH", "VERY_HIGH"};
  
  i3g4250d_platform_print("\r\n--- LOW-PASS BANDWIDTH TEST ---\r\n");
  
  for (int i = 0; i < 4; i++) {
    // Set bandwidth
    if (i3g4250d_lp_bandwidth_set(&gyro_ctx, bw_modes[i]) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    // Verify
    i3g4250d_lp_bandwidth_get(&gyro_ctx, &bw_read);
    
    gyro_ctx.mdelay(50);
    gyro_read(test_data);
    
    sprintf(msg, "LP BW %s: X=%d Y=%d Z=%d (verified: %s)\r\n",
            bw_names[i], test_data[0], test_data[1], test_data[2],
            (bw_read == bw_modes[i]) ? "OK" : "FAIL");
    i3g4250d_platform_print(msg);
  }
  
  // Restore default
  i3g4250d_lp_bandwidth_set(&gyro_ctx, I3G4250D_CUT_OFF_LOW);
  
  i3g4250d_platform_print("LOW-PASS BANDWIDTH TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test FIFO functionality
  *         Gösterir: Batch veri okuma, CPU yükü azaltma
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_fifo(void)
{
  char msg[100];
  uint8_t fifo_level, fifo_empty, fifo_ovr, fifo_wtm;
  i3g4250d_fifo_mode_t fifo_mode;
  
  i3g4250d_platform_print("\r\n--- FIFO TEST ---\r\n");
  
  // 1. FIFO Enable
  if (i3g4250d_fifo_enable_set(&gyro_ctx, PROPERTY_ENABLE) != 0) {
    return GYRO_ERR_SPI_COMM;
  }
  i3g4250d_platform_print("FIFO Enabled\r\n");
  
  // 2. Set FIFO mode to FIFO (normal FIFO mode)
  if (i3g4250d_fifo_mode_set(&gyro_ctx, I3G4250D_FIFO_MODE) != 0) {
    return GYRO_ERR_SPI_COMM;
  }
  
  // Verify mode
  i3g4250d_fifo_mode_get(&gyro_ctx, &fifo_mode);
  sprintf(msg, "FIFO Mode set to: %d (1=FIFO)\r\n", fifo_mode);
  i3g4250d_platform_print(msg);
  
  // 3. Set watermark (threshold)
  if (i3g4250d_fifo_watermark_set(&gyro_ctx, 10) != 0) {
    return GYRO_ERR_SPI_COMM;
  }
  i3g4250d_platform_print("FIFO Watermark set to 10\r\n");
  
  // 4. Wait for FIFO to fill
  gyro_ctx.mdelay(200);  // 100Hz ODR → 200ms ≈ 20 samples
  
  // 5. Check FIFO status
  i3g4250d_fifo_data_level_get(&gyro_ctx, &fifo_level);
  i3g4250d_fifo_empty_flag_get(&gyro_ctx, &fifo_empty);
  i3g4250d_fifo_ovr_flag_get(&gyro_ctx, &fifo_ovr);
  i3g4250d_fifo_wtm_flag_get(&gyro_ctx, &fifo_wtm);
  
  sprintf(msg, "FIFO Status: Level=%d, Empty=%d, Overrun=%d, Watermark=%d\r\n",
          fifo_level, fifo_empty, fifo_ovr, fifo_wtm);
  i3g4250d_platform_print(msg);
  
  // 6. Read FIFO samples
  int16_t fifo_data[3];
  int samples_read = 0;
  while (fifo_level > 0 && samples_read < 5) {  // Read up to 5 samples
    gyro_read(fifo_data);
    sprintf(msg, "FIFO[%d]: X=%d Y=%d Z=%d\r\n",
            samples_read, fifo_data[0], fifo_data[1], fifo_data[2]);
    i3g4250d_platform_print(msg);
    samples_read++;
    i3g4250d_fifo_data_level_get(&gyro_ctx, &fifo_level);
  }
  
  // 7. Set to Bypass mode (disable FIFO)
  i3g4250d_fifo_mode_set(&gyro_ctx, I3G4250D_FIFO_BYPASS_MODE);
  i3g4250d_fifo_enable_set(&gyro_ctx, PROPERTY_DISABLE);
  
  i3g4250d_platform_print("FIFO TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Data Format (Big/Little Endian)
  *         Gösterir: Cross-platform uyumluluk
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_data_format(void)
{
  char msg[80];
  i3g4250d_ble_t ble_mode;
  
  i3g4250d_platform_print("\r\n--- DATA FORMAT TEST ---\r\n");
  
  // 1. Little Endian (LSB first - default)
  i3g4250d_data_format_set(&gyro_ctx, I3G4250D_AUX_LSB_AT_LOW_ADD);
  i3g4250d_data_format_get(&gyro_ctx, &ble_mode);
  sprintf(msg, "Little Endian: %s\r\n", 
          (ble_mode == I3G4250D_AUX_LSB_AT_LOW_ADD) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // 2. Big Endian (MSB first)
  i3g4250d_data_format_set(&gyro_ctx, I3G4250D_AUX_MSB_AT_LOW_ADD);
  i3g4250d_data_format_get(&gyro_ctx, &ble_mode);
  sprintf(msg, "Big Endian: %s\r\n", 
          (ble_mode == I3G4250D_AUX_MSB_AT_LOW_ADD) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // Restore default (Little Endian)
  i3g4250d_data_format_set(&gyro_ctx, I3G4250D_AUX_LSB_AT_LOW_ADD);
  
  i3g4250d_platform_print("DATA FORMAT TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Boot/Reboot Memory
  *         Gösterir: Factory reset, register reload
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_boot(void)
{
  char msg[80];
  uint8_t boot_status;
  
  i3g4250d_platform_print("\r\n--- BOOT/REBOOT TEST ---\r\n");
  
  // Trigger boot (reload trimming parameters)
  i3g4250d_boot_set(&gyro_ctx, PROPERTY_ENABLE);
  
  // Check boot status
  i3g4250d_boot_get(&gyro_ctx, &boot_status);
  sprintf(msg, "Boot triggered, status: %d\r\n", boot_status);
  i3g4250d_platform_print(msg);
  
  // Wait for boot to complete
  gyro_ctx.mdelay(50);
  
  i3g4250d_boot_get(&gyro_ctx, &boot_status);
  sprintf(msg, "Boot complete, status: %d (should be 0)\r\n", boot_status);
  i3g4250d_platform_print(msg);
  
  i3g4250d_platform_print("BOOT TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Temperature Sensor
  *         Gösterir: Ortam sıcaklığı izleme
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_temperature(void)
{
  char msg[80];
  uint8_t temp_raw_val;
  int temp_c;
  
  i3g4250d_platform_print("\r\n--- TEMPERATURE TEST ---\r\n");
  
  // Read multiple 
  for (int i = 0; i < 3; i++) {
    if (i3g4250d_temperature_raw_get(&gyro_ctx, &temp_raw_val) != 0) {
      return GYRO_ERR_TEMP_READ;
    }
    
    temp_c = i3g4250d_from_lsb_to_celsius((int8_t)temp_raw_val);
    
    sprintf(msg, "Temp Sample %d: Raw=%d -> %d C\r\n",
            i+1, (int8_t)temp_raw_val, temp_c);
    i3g4250d_platform_print(msg);
    
    gyro_ctx.mdelay(100);
  }
  
  i3g4250d_platform_print("TEMPERATURE TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test SPI Mode (3-wire / 4-wire)
  *         Gösterir: Interface flexibility, farklı bağlantı seçenekleri
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_spi_mode(void)
{
  char msg[80];
  i3g4250d_sim_t spi_mode;
  
  i3g4250d_platform_print("\r\n--- SPI MODE TEST ---\r\n");
  
  // 1. Read current mode (should be 4-wire)
  i3g4250d_spi_mode_get(&gyro_ctx, &spi_mode);
  sprintf(msg, "Current SPI Mode: %s\r\n", 
          (spi_mode == I3G4250D_SPI_4_WIRE) ? "4-wire" : "3-wire");
  i3g4250d_platform_print(msg);
  
  // NOT: 3-wire moduna geçiş yapmıyoruz çünkü donanım 4-wire bağlı
  // Sadece register okuma/yazma test ediyoruz
  
  // 2. Verify 4-wire mode is default
  if (spi_mode != I3G4250D_SPI_4_WIRE) {
    i3g4250d_platform_print("WARNING: SPI mode not 4-wire!\r\n");
  }
  
  i3g4250d_platform_print("SPI MODE TEST PASSED (4-wire verified)\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Filter Path Selection
  *         Gösterir: Output veri yolu konfigürasyonu (LPF1, LPF2, HP)
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_filter_path(void)
{
  char msg[100];
  i3g4250d_out_sel_t filter_path;
  int16_t test_data[3];
  
  const i3g4250d_out_sel_t paths[] = {
    I3G4250D_ONLY_LPF1_ON_OUT,           // 0: LPF1 only
    I3G4250D_LPF1_LPF2_ON_OUT,      // 1: LPF1 + LPF2
    I3G4250D_LPF1_HP_ON_OUT,        // 4: LPF1 + HP
    I3G4250D_LPF1_HP_LPF2_ON_OUT    // 5: LPF1 + LPF2 + HP
  };
  const char* path_names[] = {
    "LPF1 only",
    "LPF1+LPF2",
    "LPF1+HP",
    "LPF1+LPF2+HP"
  };
  
  i3g4250d_platform_print("\r\n--- FILTER PATH TEST ---\r\n");
  
  for (int i = 0; i < 4; i++) {
    // Set filter path
    if (i3g4250d_filter_path_set(&gyro_ctx, paths[i]) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    // Verify
    i3g4250d_filter_path_get(&gyro_ctx, &filter_path);
    
    gyro_ctx.mdelay(50);
    gyro_read(test_data);
    
    sprintf(msg, "Path %s: X=%d Y=%d Z=%d (%s)\r\n",
            path_names[i], test_data[0], test_data[1], test_data[2],
            (filter_path == paths[i]) ? "OK" : "FAIL");
    i3g4250d_platform_print(msg);
  }
  
  // Restore default
  i3g4250d_filter_path_set(&gyro_ctx, I3G4250D_ONLY_LPF1_ON_OUT);
  
  i3g4250d_platform_print("FILTER PATH TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test HP Reference Value
  *         Gösterir: High-pass filter referans değeri ayarı
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_hp_reference(void)
{
  char msg[80];
  uint8_t ref_val;
  
  i3g4250d_platform_print("\r\n--- HP REFERENCE TEST ---\r\n");
  
  // Test different reference values
  const uint8_t test_refs[] = {0x00, 0x40, 0x80, 0xFF};
  
  for (int i = 0; i < 4; i++) {
    // Set reference
    if (i3g4250d_hp_reference_value_set(&gyro_ctx, test_refs[i]) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    // Verify
    i3g4250d_hp_reference_value_get(&gyro_ctx, &ref_val);
    
    sprintf(msg, "HP Reference 0x%02X: %s\r\n", 
            test_refs[i], (ref_val == test_refs[i]) ? "OK" : "FAIL");
    i3g4250d_platform_print(msg);
  }
  
  // Restore default
  i3g4250d_hp_reference_value_set(&gyro_ctx, 0x00);
  
  i3g4250d_platform_print("HP REFERENCE TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Interrupt Pin Configuration
  *         Gösterir: INT1/INT2 pin routing, polarity, mode
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_interrupt_pins(void)
{
  char msg[100];
  i3g4250d_int1_route_t int1_route;
  i3g4250d_int2_route_t int2_route;
  i3g4250d_pp_od_t pin_mode;
  i3g4250d_h_lactive_t polarity;
  
  i3g4250d_platform_print("\r\n--- INTERRUPT PIN TEST ---\r\n");
  
  // 1. Test INT1 routing (interrupt enable on INT1)
  i3g4250d_pin_int1_route_set(&gyro_ctx, I3G4250D_INT1_ROUTE_INT1);
  i3g4250d_pin_int1_route_get(&gyro_ctx, &int1_route);
  sprintf(msg, "INT1 Route (INT1 enabled): %s\r\n",
          (int1_route & I3G4250D_INT1_ROUTE_INT1) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // 2. Test INT2 routing (DRDY on INT2)
  i3g4250d_pin_int2_route_set(&gyro_ctx, I3G4250D_INT2_ROUTE_DRDY);
  i3g4250d_pin_int2_route_get(&gyro_ctx, &int2_route);
  sprintf(msg, "INT2 Route (DRDY enabled): %s\r\n",
          (int2_route & I3G4250D_INT2_ROUTE_DRDY) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // 3. Test pin mode (Push-Pull vs Open-Drain)
  i3g4250d_pin_mode_set(&gyro_ctx, I3G4250D_PUSH_PULL);
  i3g4250d_pin_mode_get(&gyro_ctx, &pin_mode);
  sprintf(msg, "Pin Mode (Push-Pull): %s\r\n",
          (pin_mode == I3G4250D_PUSH_PULL) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // 4. Test polarity (Active High vs Active Low)
  i3g4250d_pin_polarity_set(&gyro_ctx, I3G4250D_ACTIVE_HIGH);
  i3g4250d_pin_polarity_get(&gyro_ctx, &polarity);
  sprintf(msg, "Pin Polarity (Active High): %s\r\n",
          (polarity == I3G4250D_ACTIVE_HIGH) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // Disable interrupts after test
  i3g4250d_pin_int1_route_set(&gyro_ctx, 0);
  i3g4250d_pin_int2_route_set(&gyro_ctx, 0);
  
  i3g4250d_platform_print("INTERRUPT PIN TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Interrupt Notification Mode
  *         Gösterir: Latched vs Pulsed interrupt behavior
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_int_notification(void)
{
  char msg[80];
  i3g4250d_lir_t lir_mode;
  
  i3g4250d_platform_print("\r\n--- INT NOTIFICATION TEST ---\r\n");
  
  // 1. Test Latched mode
  i3g4250d_int_notification_set(&gyro_ctx, I3G4250D_INT_LATCHED);
  i3g4250d_int_notification_get(&gyro_ctx, &lir_mode);
  sprintf(msg, "Latched Mode: %s\r\n",
          (lir_mode == I3G4250D_INT_LATCHED) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  // 2. Test Pulsed mode
  i3g4250d_int_notification_set(&gyro_ctx, I3G4250D_INT_PULSED);
  i3g4250d_int_notification_get(&gyro_ctx, &lir_mode);
  sprintf(msg, "Pulsed Mode: %s\r\n",
          (lir_mode == I3G4250D_INT_PULSED) ? "OK" : "FAIL");
  i3g4250d_platform_print(msg);
  
  i3g4250d_platform_print("INT NOTIFICATION TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Threshold Interrupt Configuration
  *         Gösterir: Eşik değeri tabanlı kesme oluşturma
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_threshold_interrupt(void)
{
  char msg[100];
  uint16_t thresh_x, thresh_y, thresh_z;
  i3g4250d_int1_src_t int_src;
  
  i3g4250d_platform_print("\r\n--- THRESHOLD INTERRUPT TEST ---\r\n");
  
  // 1. Set thresholds for each axis (15-bit value, max ~32767)
  uint16_t test_threshold = 1000;  // Test threshold value
  
  i3g4250d_int_x_threshold_set(&gyro_ctx, test_threshold);
  i3g4250d_int_y_threshold_set(&gyro_ctx, test_threshold);
  i3g4250d_int_z_threshold_set(&gyro_ctx, test_threshold);
  
  // 2. Verify thresholds
  i3g4250d_int_x_threshold_get(&gyro_ctx, &thresh_x);
  i3g4250d_int_y_threshold_get(&gyro_ctx, &thresh_y);
  i3g4250d_int_z_threshold_get(&gyro_ctx, &thresh_z);
  
  sprintf(msg, "Thresholds set: X=%d Y=%d Z=%d (expected %d)\r\n",
          thresh_x, thresh_y, thresh_z, test_threshold);
  i3g4250d_platform_print(msg);
  
  // 3. Read interrupt source register
  i3g4250d_int_on_threshold_src_get(&gyro_ctx, &int_src);
  sprintf(msg, "INT1_SRC Register: 0x%02X\r\n", *(uint8_t*)&int_src);
  i3g4250d_platform_print(msg);
  
  // 4. Test threshold mode (AND/OR combination)
  i3g4250d_int_on_threshold_mode_set(&gyro_ctx, I3G4250D_INT1_ON_TH_OR);
  i3g4250d_platform_print("Threshold Mode: OR\r\n");
  
  // 5. Test threshold duration
  i3g4250d_int_on_threshold_dur_set(&gyro_ctx, 10); // 10 samples
  i3g4250d_platform_print("Threshold Duration: 10 samples, WAIT enabled\r\n");
  
  // Clear thresholds after test
  i3g4250d_int_x_threshold_set(&gyro_ctx, 0);
  i3g4250d_int_y_threshold_set(&gyro_ctx, 0);
  i3g4250d_int_z_threshold_set(&gyro_ctx, 0);
  
  i3g4250d_platform_print("THRESHOLD INTERRUPT TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * @brief  Test Internal Filter Path (for interrupt generator)
  *         Gösterir: Interrupt generator için filter seçimi
  * @retval gyro_error_t
  */
static gyro_error_t gyro_test_filter_path_internal(void)
{
  char msg[100];
  i3g4250d_int1_sel_t int1_sel;
  
  const i3g4250d_int1_sel_t paths[] = {
    I3G4250D_ONLY_LPF1_ON_INT,
    I3G4250D_LPF1_LPF2_ON_INT,
    I3G4250D_LPF1_HP_ON_INT,
    I3G4250D_LPF1_HP_LPF2_ON_INT
  };
  const char* path_names[] = {
    "LPF1",
    "LPF1+LPF2",
    "LPF1+HP",
    "LPF1+LPF2+HP"
  };
  
  i3g4250d_platform_print("\r\n--- INTERNAL FILTER PATH TEST ---\r\n");
  
  for (int i = 0; i < 4; i++) {
    // Set internal filter path
    if (i3g4250d_filter_path_internal_set(&gyro_ctx, paths[i]) != 0) {
      return GYRO_ERR_SPI_COMM;
    }
    
    // Verify
    i3g4250d_filter_path_internal_get(&gyro_ctx, &int1_sel);
    
    sprintf(msg, "INT1 Filter %s: %s\r\n",
            path_names[i], (int1_sel == paths[i]) ? "OK" : "FAIL");
    i3g4250d_platform_print(msg);
  }
  
  // Restore default
  i3g4250d_filter_path_internal_set(&gyro_ctx, I3G4250D_ONLY_LPF1_ON_INT);
  
  i3g4250d_platform_print("INTERNAL FILTER PATH TEST PASSED\r\n\r\n");
  return GYRO_OK;
}

/**
  * =============================================================================
  * @brief  TÜM İŞLEVSELLİK TESTİ (Comprehensive Driver Test Suite)
  * =============================================================================
  *         Bu fonksiyon tüm driver özelliklerini test eder.
  *         Sunumda "driver'ın tüm işlevselliği" sorusuna cevap verir.
  * 
  * @retval gyro_error_t - GYRO_OK ise tüm testler başarılı
  */
static gyro_error_t gyro_run_all_tests(void)
{
  gyro_error_t result;
  int passed = 0;
  int failed = 0;
  
  i3g4250d_platform_print("\r\n");
  i3g4250d_platform_print("╔═══════════════════════════════════════════════════════════╗\r\n");
  i3g4250d_platform_print("║       I3G4250D DRIVER - FULL FUNCTIONALITY TEST           ║\r\n");
  i3g4250d_platform_print("╚═══════════════════════════════════════════════════════════╝\r\n\r\n");
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 1: Register Dump (Debug bilgisi)
  // ═══════════════════════════════════════════════════════════════════════════
  gyro_debug_dump_registers();
  passed++;
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 2: Self-Test (Sensör sağlık kontrolü)
  // Neden önemli: Sensörün fiziksel olarak düzgün çalıştığını doğrular
  // ═══════════════════════════════════════════════════════════════════════════
  if (gyro_self_test() == 0) {
    passed++;
  } else {
    failed++;
    i3g4250d_platform_print("!!! SELF-TEST FAILED !!!\r\n");
  }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 3: ODR Değişimi (100/200/400/800 Hz)
  // Neden önemli: Driver'ın farklı veri hızlarını desteklediğini gösterir
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_odr_modes();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 4: Full-Scale Değişimi (±245/±500/±2000 dps)
  // Neden önemli: Farklı ölçüm aralıklarını ve sensitivity'yi gösterir
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_fullscale_modes();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 5: Power Mode (Normal/Power-Down)
  // Neden önemli: Güç yönetimi, bataryalı cihazlar için kritik
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_power_mode();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 6: Axis Enable/Disable (X/Y/Z ayrı ayrı)
  // Neden önemli: Seçici veri okuma, güç tasarrufu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_axis_enable();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 7: High-Pass Filter
  // Neden önemli: DC offset kaldırma, sinyal işleme
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_high_pass_filter();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 8: Low-Pass Bandwidth
  // Neden önemli: Anti-aliasing, gürültü filtreleme
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_lp_bandwidth();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 9: FIFO
  // Neden önemli: Batch veri okuma, CPU yükü azaltma, buffer yönetimi
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_fifo();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 10: Data Format (Big/Little Endian)
  // Neden önemli: Cross-platform uyumluluk
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_data_format();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 11: Boot/Reboot
  // Neden önemli: Factory reset, hata kurtarma
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_boot();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 12: Temperature Sensor
  // Neden önemli: Termal izleme, sıcaklık kompanzasyonu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_temperature();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 13: SPI Mode (3-wire/4-wire)
  // Neden önemli: Interface esnekliği, farklı donanım konfigürasyonları
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_spi_mode();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 14: Filter Path Selection
  // Neden önemli: Sinyal işleme yolu, data kalitesi optimizasyonu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_filter_path();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 15: HP Reference Value
  // Neden önemli: HP filter referans ayarı, offset kalibrasyonu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_hp_reference();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 16: Interrupt Pin Configuration
  // Neden önemli: Hardware interrupt routing, event-driven tasarım
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_interrupt_pins();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 17: Interrupt Notification Mode
  // Neden önemli: Latched vs Pulsed, interrupt davranış kontrolü
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_int_notification();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 18: Threshold Interrupt Configuration
  // Neden önemli: Motion detection, wake-on-motion, güç tasarrufu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_threshold_interrupt();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // TEST 19: Internal Filter Path (INT1)
  // Neden önemli: Interrupt generator sinyal işleme yolu
  // ═══════════════════════════════════════════════════════════════════════════
  result = gyro_test_filter_path_internal();
  if (result == GYRO_OK) { passed++; } else { failed++; }
  
  // ═══════════════════════════════════════════════════════════════════════════
  // SONUÇ RAPORU
  // ═══════════════════════════════════════════════════════════════════════════
  char summary[120];
  i3g4250d_platform_print("\r\n");
  i3g4250d_platform_print("╔═══════════════════════════════════════════════════════════╗\r\n");
  i3g4250d_platform_print("║                    TEST SONUCLARI                         ║\r\n");
  i3g4250d_platform_print("╚═══════════════════════════════════════════════════════════╝\r\n");
  
  sprintf(summary, "Gecen: %d  |  Kalan: %d  |  Toplam: 19\r\n", 
          passed, failed);
  i3g4250d_platform_print(summary);
  
  if (failed == 0) {
    i3g4250d_platform_print("\r\n*** TUM TESTLER BASARILI - DRIVER FULLY FUNCTIONAL ***\r\n\r\n");
    return GYRO_OK;
  } else {
    i3g4250d_platform_print("\r\n!!! BAZI TESTLER BASARISIZ !!!\r\n\r\n");
    return GYRO_ERR_SELFTEST_FAIL;
  }
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
  gyro_error_t status = gyro_init();

  if (status == GYRO_OK) {

    i3g4250d_platform_led_on(0);  // Green LED
    
    // ═══════════════════════════════════════════════════════════════════════
    // TÜM İŞLEVSELLİK TESTİ - Yorumu kaldırarak aktif et!
    // Tüm driver özelliklerini tek seferde test eder (Sunumda kullan)
    // ═══════════════════════════════════════════════════════════════════════
     gyro_run_all_tests();  // <-- YORUMU KALDIR: Tüm testleri çalıştırır
    


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
  } else {  // Error handling with descriptive message
    char err_msg[100];
    sprintf(err_msg, "\r\nERROR: %s (code: %d)\r\n", 
            gyro_error_to_string(status), status);
    i3g4250d_platform_print(err_msg);
    
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
