#ifndef I3G4250D_PLATFORM_H
#define I3G4250D_PLATFORM_H

#include "i3g4250d_reg.h"

/**
  ******************************************************************************
  * @file    i3g4250d_platform.h
  * @author  Emre Yavuz
  * @brief   Platform abstraction layer for I3G4250D gyroscope driver
  ******************************************************************************
  *
  * PLATFORM CONTRACT (STM32F429I-DISC1):
  * =====================================
  * 
  * SPI CONFIGURATION REQUIREMENTS:
  * - Mode: SPI_MODE_3 (CPOL=1, CPHA=1) - MANDATORY for I3G4250D
  * - Data Size: 8-bit
  * - First Bit: MSB First
  * - NSS: Software controlled (GPIO)
  * - Max Clock: 10 MHz
  * 
  * PIN MAPPING (Discovery Board):
  * - SPI5_SCK:  PF7
  * - SPI5_MISO: PF8
  * - SPI5_MOSI: PF9
  * - CS (NSS):  PC1 (Software controlled)
  * 
  * THREAD SAFETY:
  * ==============
  * This driver is NOT thread-safe by default.
  * If using with RTOS, caller must ensure:
  * - Mutex protection around read/write sequences
  * - No concurrent access to same stmdev_ctx_t instance
  * 
  * For thread-safe operation, implement mutex in platform layer:
  *   platform_read() { mutex_lock(); spi_read(); mutex_unlock(); }
  *
  ******************************************************************************
  */

/* Debug output control - define to enable debug prints */
/* #define I3G4250D_DEBUG_ENABLE */

#ifdef I3G4250D_DEBUG_ENABLE
  #define I3G4250D_DEBUG_PRINT(msg)  i3g4250d_platform_print(msg)
#else
  #define I3G4250D_DEBUG_PRINT(msg)  ((void)0)
#endif

/**
  * @brief  Driver state structure
  */
typedef struct {
  uint8_t initialized;      /**< 0: not initialized, 1: initialized */
  uint8_t power_mode;       /**< 0: power-down, 1: normal */
  uint8_t odr;              /**< Current ODR setting */
  uint8_t fullscale;        /**< Current full-scale setting */
} i3g4250d_state_t;

/**
  * @brief  DRDY callback function type
  *         Called when data ready interrupt occurs (if configured)
  * @param  user_data  User-provided context pointer
  */
typedef void (*i3g4250d_drdy_callback_t)(void *user_data);

/**
  * @brief  Extended platform context with state tracking
  */
typedef struct {
  stmdev_ctx_t           ctx;           /**< Base driver context */
  i3g4250d_state_t       state;         /**< Driver state */
  i3g4250d_drdy_callback_t drdy_cb;     /**< DRDY callback (optional) */
  void                   *drdy_user;    /**< DRDY callback user data */
} i3g4250d_platform_ctx_t;

/* ============================================================================
 * CORE PLATFORM FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Initialize platform layer
  * @param  ctx        Driver context pointer
  * @param  spi_handle SPI peripheral handle (e.g., &hspi5)
  * @retval 0: OK, -1: Error
  * @note   SPI must be pre-configured with Mode 3 (CPOL=1, CPHA=1)
  */
int32_t i3g4250d_platform_init(stmdev_ctx_t *ctx, void *spi_handle);

/**
  * @brief  Platform SPI write function (bus abstraction)
  * @param  handle  SPI handle pointer
  * @param  reg     Register address (bit7 cleared for write)
  * @param  data    Data buffer to write
  * @param  len     Number of bytes
  * @retval 0: OK, -1: Error
  */
int32_t i3g4250d_platform_write(void *handle, uint8_t reg,
                                const uint8_t *data, uint16_t len);

/**
  * @brief  Platform SPI read function (bus abstraction)
  * @param  handle  SPI handle pointer
  * @param  reg     Register address (bit7 set for read, bit6 for auto-increment)
  * @param  data    Data buffer to read into
  * @param  len     Number of bytes
  * @retval 0: OK, -1: Error
  * @note   Uses full-duplex SPI with dummy bytes for proper timing
  */
int32_t i3g4250d_platform_read(void *handle, uint8_t reg,
                               uint8_t *data, uint16_t len);


/* ============================================================================
 * DRDY CALLBACK FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Register DRDY callback
  * @param  pctx      Extended platform context
  * @param  callback  Callback function (NULL to disable)
  * @param  user_data User data passed to callback
  */
void i3g4250d_platform_set_drdy_callback(i3g4250d_platform_ctx_t *pctx,
                                          i3g4250d_drdy_callback_t callback,
                                          void *user_data);

/**
  * @brief  Call from EXTI ISR when DRDY pin triggers
  * @param  pctx  Extended platform context
  * @note   Should be called from GPIO EXTI interrupt handler
  */
void i3g4250d_platform_drdy_irq_handler(i3g4250d_platform_ctx_t *pctx);

#endif /* I3G4250D_PLATFORM_H */
