#include "i3g4250d_platform.h"
#include "spi.h"               
#include "gpio.h"
#include "usart.h"
#include <string.h>

// SPI CS pins
#define GYRO_CS_PORT GPIOC
#define GYRO_CS_PIN  GPIO_PIN_1

// LED pins (STM32F429-DISCO)
#define LED_GREEN_PORT GPIOG
#define LED_GREEN_PIN  GPIO_PIN_13
#define LED_RED_PORT   GPIOG
#define LED_RED_PIN    GPIO_PIN_14

// UART handle (external)
extern UART_HandleTypeDef huart1;

// Platform initialization
int32_t i3g4250d_platform_init(stmdev_ctx_t *ctx, void *spi_handle)
{
  ctx->write_reg = i3g4250d_platform_write;
  ctx->read_reg  = i3g4250d_platform_read;
  ctx->mdelay    = i3g4250d_platform_delay;
  ctx->handle    = spi_handle;
  return 0;
}
/**
  * @brief  Platform-specific SPI read function
  * @param  handle  SPI handle pointer
  * @param  reg     Register address
  * @param  data    Data buffer
  * @param  len     Number of bytes
  * @retval 0: OK, -1: Error
  */
int32_t i3g4250d_platform_read(void *handle, uint8_t reg, 
                             uint8_t *data, uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;
  uint8_t addr = reg | 0x80;  // Set bit 7 (READ = 1)
  
  if (len > 1) {
    addr |= 0x40;  // Multi-byte: set bit 6
  }
  
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  
  if (HAL_SPI_Transmit(hspi, &addr, 1, 100) != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }
  
  if (HAL_SPI_Receive(hspi, data, len, 100) != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }
  
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  return 0;
}


/**
  * @brief  Platform-specific SPI write function
  * @param  handle  SPI handle pointer
  * @param  reg     Register address
  * @param  data    Data buffer
  * @param  len     Number of bytes
  * @retval 0: OK, -1: Error
  */
int32_t i3g4250d_platform_write(void *handle, uint8_t reg,
                                const uint8_t *data, uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;
  uint8_t addr = reg & 0x7F;

  if (len > 1) addr |= 0x40;

  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(hspi, &addr, 1, 100) != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }

  if (HAL_SPI_Transmit(hspi, (uint8_t*)data, len, 100) != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }

  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  return 0;
}

/**
 * @brief  Platform delay function
 * @param  ms  Milliseconds to delay
 */
void i3g4250d_platform_delay(uint32_t ms){
  HAL_Delay(ms);
}



/**
 * @brief  Turn on LED
 * @param  led_id: 0=Green, 1=Red
 */
void i3g4250d_platform_led_on(uint8_t led_id)
{
  if (led_id == 0) {
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
  } else if (led_id == 1) {
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
  }
}

/**
 * @brief  Turn off LED
 * @param  led_id: 0=Green, 1=Red
 */
void i3g4250d_platform_led_off(uint8_t led_id)
{
  if (led_id == 0) {
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
  } else if (led_id == 1) {
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
