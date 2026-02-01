#include "i3g4250d_platform.h"
#include "spi.h"               
#include "gpio.h"

// SPI CS pins
#define GYRO_CS_PORT GPIOC
#define GYRO_CS_PIN  GPIO_PIN_1

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

