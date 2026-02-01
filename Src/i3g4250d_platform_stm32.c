/**
  ******************************************************************************
  * @file    i3g4250d_platform_stm32.c
  * @author  Emre Yavuz
  * @brief   STM32F429I-DISC1 platform implementation for I3G4250D driver
  ******************************************************************************
  *
  * IMPLEMENTATION NOTES:
  * =====================
  * - Uses full-duplex SPI with HAL_SPI_TransmitReceive for proper timing
  * - CS (Chip Select) managed via GPIO PC1
  * - Timeout: 100ms for all SPI operations
  * - Debug output via UART1 (controlled by I3G4250D_DEBUG_ENABLE)
  *
  ******************************************************************************
  */

#include "i3g4250d_platform.h"
#include "spi.h"               
#include "gpio.h"
#include <string.h>

/* ============================================================================
 * HARDWARE CONFIGURATION
 * ============================================================================ */

/** @brief SPI Chip Select GPIO Port */
#define GYRO_CS_PORT    GPIOC

/** @brief SPI Chip Select GPIO Pin */
#define GYRO_CS_PIN     GPIO_PIN_1

/** @brief SPI timeout in milliseconds */
#define SPI_TIMEOUT_MS  100U

/* ============================================================================
 * STATIC VARIABLES
 * ============================================================================ */

/** @brief Driver state (singleton for simple applications) */
static i3g4250d_state_t g_driver_state = {0};

/** @brief DRDY callback storage */
static i3g4250d_drdy_callback_t g_drdy_callback = NULL;
static void *g_drdy_user_data = NULL;



/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief  Platform delay function
 * @param  ms  Milliseconds to delay
 */
void i3g4250d_platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/* ============================================================================
 * PLATFORM INITIALIZATION
 * ============================================================================ */

/**
  * @brief  Initialize platform layer and driver context
  * @param  ctx        Driver context pointer
  * @param  spi_handle SPI peripheral handle (e.g., &hspi5)
  * @retval 0: OK, -1: Error
  * 
  * @note   CRITICAL: SPI must be configured as follows BEFORE calling this:
  *         - Mode: SPI_MODE_3 (CPOL=1, CPHA=1)
  *         - Data Size: 8-bit
  *         - First Bit: MSB First
  *         - Baud Rate: ≤10 MHz
  */
int32_t i3g4250d_platform_init(stmdev_ctx_t *ctx, void *spi_handle)
{
  if (ctx == NULL || spi_handle == NULL) {
    return -1;
  }
  
  /* Setup function pointers (bus abstraction) */
  ctx->write_reg = i3g4250d_platform_write;
  ctx->read_reg  = i3g4250d_platform_read;
  ctx->mdelay    = i3g4250d_platform_delay;
  ctx->handle    = spi_handle;
  
  /* Initialize driver state */
  g_driver_state.initialized = 0;  /* Will be set after WHO_AM_I check */
  g_driver_state.power_mode  = 0;
  g_driver_state.odr         = 0;
  g_driver_state.fullscale   = 0;
  
  /* Ensure CS is high (deselected) at startup */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  
  I3G4250D_DEBUG_PRINT("[PLATFORM] Initialized\r\n");
  
  return 0;
}



/* ============================================================================
 * STATE MANAGEMENT FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Get current driver state
  * @retval Pointer to driver state structure
  */
i3g4250d_state_t* i3g4250d_platform_get_state(void)
{
  return &g_driver_state;
}

/**
  * @brief  Mark driver as initialized
  * @param  odr        Current ODR setting
  * @param  fullscale  Current full-scale setting
  */
void i3g4250d_platform_set_initialized(uint8_t odr, uint8_t fullscale)
{
  g_driver_state.initialized = 1;
  g_driver_state.power_mode  = 1;
  g_driver_state.odr         = odr;
  g_driver_state.fullscale   = fullscale;
}

/**
  * @brief  Check if driver is initialized
  * @retval 1: initialized, 0: not initialized
  */
uint8_t i3g4250d_platform_is_initialized(void)
{
  return g_driver_state.initialized;
}

/* ============================================================================
 * DRDY CALLBACK FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Register DRDY callback function
  * @param  pctx      Extended platform context (can be NULL for global)
  * @param  callback  Callback function (NULL to disable)
  * @param  user_data User data passed to callback
  */
void i3g4250d_platform_set_drdy_callback(i3g4250d_platform_ctx_t *pctx,
                                          i3g4250d_drdy_callback_t callback,
                                          void *user_data)
{
  if (pctx != NULL) {
    pctx->drdy_cb   = callback;
    pctx->drdy_user = user_data;
  } else {
    /* Use global callbacks */
    g_drdy_callback  = callback;
    g_drdy_user_data = user_data;
  }
}

/**
  * @brief  DRDY interrupt handler - call from EXTI ISR
  * @param  pctx  Extended platform context (can be NULL for global)
  * @note   Example usage in stm32f4xx_it.c:
  *         void EXTI2_IRQHandler(void) {
  *           if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2)) {
  *             __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  *             i3g4250d_platform_drdy_irq_handler(NULL);
  *           }
  *         }
  */
void i3g4250d_platform_drdy_irq_handler(i3g4250d_platform_ctx_t *pctx)
{
  i3g4250d_drdy_callback_t cb;
  void *user;
  
  if (pctx != NULL) {
    cb   = pctx->drdy_cb;
    user = pctx->drdy_user;
  } else {
    cb   = g_drdy_callback;
    user = g_drdy_user_data;
  }
  
  if (cb != NULL) {
    cb(user);
  }
}

/* ============================================================================
 * SPI READ/WRITE FUNCTIONS (Full-Duplex Implementation)
 * ============================================================================ */

/**
  * @brief  Platform-specific SPI read function (FULL-DUPLEX)
  * @param  handle  SPI handle pointer
  * @param  reg     Register address
  * @param  data    Data buffer
  * @param  len     Number of bytes
  * @retval 0: OK, -1: Error
  *
  * @note   I3G4250D SPI Protocol:
  *         - Bit 7: RW (1=Read, 0=Write)
  *         - Bit 6: MS (1=Auto-increment for multi-byte)
  *         - Bit 5-0: Register address
  *
  *         Full-duplex is used because SPI is synchronous:
  *         - Master sends dummy bytes while receiving data
  *         - This ensures proper clock generation
  */
int32_t i3g4250d_platform_read(void *handle, uint8_t reg, 
                               uint8_t *data, uint16_t len)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;
  HAL_StatusTypeDef status;
  
  /* Build address byte */
  uint8_t addr = reg | 0x80;  /* Set bit 7: READ = 1 */
  
  if (len > 1) {
    addr |= 0x40;  /* Set bit 6: Auto-increment for multi-byte */
  }
  
  /* CS LOW - Start transaction */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  
  /* Send address byte */
  status = HAL_SPI_Transmit(hspi, &addr, 1, SPI_TIMEOUT_MS);
  if (status != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }
  
  /* Full-duplex read: send dummy bytes (0xFF) while receiving data */
  /* This is the correct way to read in SPI - clock is generated by TX */
  uint8_t tx_dummy[16];  /* Max 16 bytes per transaction */
  memset(tx_dummy, 0xFF, sizeof(tx_dummy));
  
  if (len <= sizeof(tx_dummy)) {
    status = HAL_SPI_TransmitReceive(hspi, tx_dummy, data, len, SPI_TIMEOUT_MS);
  } else {
    /* Fallback for larger reads (shouldn't happen normally) */
    status = HAL_SPI_Receive(hspi, data, len, SPI_TIMEOUT_MS);
  }
  
  /* CS HIGH - End transaction */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  
  return (status == HAL_OK) ? 0 : -1;
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
  HAL_StatusTypeDef status;
  
  /* Build address byte */
  uint8_t addr = reg & 0x7F;  /* Clear bit 7: WRITE = 0 */
  
  if (len > 1) {
    addr |= 0x40;  /* Set bit 6: Auto-increment for multi-byte */
  }
  
  /* CS LOW - Start transaction */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
  
  /* Send address byte */
  status = HAL_SPI_Transmit(hspi, &addr, 1, SPI_TIMEOUT_MS);
  if (status != HAL_OK) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
    return -1;
  }
  
  /* Send data */
  status = HAL_SPI_Transmit(hspi, (uint8_t*)data, len, SPI_TIMEOUT_MS);
  
  /* CS HIGH - End transaction */
  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
  
  return (status == HAL_OK) ? 0 : -1;
}


