/**
  ******************************************************************************
  * @file    i3g4250d_mock.c
  * @author  Emre Yavuz
  * @brief   Mock/Simulated platform for unit testing I3G4250D driver
  ******************************************************************************
  *
  * PURPOSE:
  * ========
  * This file provides a simulated register map that allows testing the
  * I3G4250D driver WITHOUT actual hardware. This enables:
  * - Unit testing on PC (no STM32 needed)
  * - CI/CD integration testing
  * - Protocol verification
  * - Edge case testing
  *
  * USAGE:
  * ======
  * 1. Compile this file instead of i3g4250d_platform_stm32.c
  * 2. Link with i3g4250d_reg.c
  * 3. Run tests on PC
  *
  * BUILD (GCC on PC):
  *   gcc -DI3G4250D_MOCK_TEST i3g4250d_mock.c i3g4250d_reg.c -o test_gyro
  *
  ******************************************************************************
  */

#ifdef I3G4250D_MOCK_TEST  /* Only compile when testing */

#include "i3g4250d_reg.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ============================================================================
 * SIMULATED REGISTER MAP
 * ============================================================================ */

/**
  * @brief  Simulated I3G4250D register map (power-on defaults from datasheet)
  */
static uint8_t mock_registers[256] = {
  /* 0x00-0x0E: Reserved */
  [0x00 ... 0x0E] = 0x00,
  
  /* 0x0F: WHO_AM_I (Read-only, fixed value) */
  [0x0F] = 0xD3,  /* I3G4250D_ID */
  
  /* 0x10-0x1F: Reserved */
  [0x10 ... 0x1F] = 0x00,
  
  /* 0x20: CTRL_REG1 (Default: 0x07 - all axes enabled, PD=0) */
  [0x20] = 0x07,
  
  /* 0x21: CTRL_REG2 (Default: 0x00) */
  [0x21] = 0x00,
  
  /* 0x22: CTRL_REG3 (Default: 0x00) */
  [0x22] = 0x00,
  
  /* 0x23: CTRL_REG4 (Default: 0x00) */
  [0x23] = 0x00,
  
  /* 0x24: CTRL_REG5 (Default: 0x00) */
  [0x24] = 0x00,
  
  /* 0x25: REFERENCE (Default: 0x00) */
  [0x25] = 0x00,
  
  /* 0x26: OUT_TEMP (Temperature: ~25°C offset) */
  [0x26] = 0x00,
  
  /* 0x27: STATUS_REG (Default: 0x00, will be simulated) */
  [0x27] = 0x00,
  
  /* 0x28-0x2D: Output data registers (X, Y, Z) */
  [0x28] = 0x00,  /* OUT_X_L */
  [0x29] = 0x00,  /* OUT_X_H */
  [0x2A] = 0x00,  /* OUT_Y_L */
  [0x2B] = 0x00,  /* OUT_Y_H */
  [0x2C] = 0x00,  /* OUT_Z_L */
  [0x2D] = 0x00,  /* OUT_Z_H */
  
  /* 0x2E: FIFO_CTRL_REG */
  [0x2E] = 0x00,
  
  /* 0x2F: FIFO_SRC_REG */
  [0x2F] = 0x20,  /* EMPTY flag set */
  
  /* 0x30-0x37: INT1 configuration registers */
  [0x30] = 0x00,  /* INT1_CFG */
  [0x31] = 0x00,  /* INT1_SRC */
  [0x32] = 0x00,  /* INT1_THS_XH */
  [0x33] = 0x00,  /* INT1_THS_XL */
  [0x34] = 0x00,  /* INT1_THS_YH */
  [0x35] = 0x00,  /* INT1_THS_YL */
  [0x36] = 0x00,  /* INT1_THS_ZH */
  [0x37] = 0x00,  /* INT1_THS_ZL */
  [0x38] = 0x00,  /* INT1_DURATION */
};

/* Statistics for test verification */
static struct {
  uint32_t read_count;
  uint32_t write_count;
  uint8_t  last_reg_read;
  uint8_t  last_reg_written;
  uint8_t  data_ready_flag;  /* Simulated DRDY */
} mock_stats = {0};

/* ============================================================================
 * MOCK PLATFORM FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Mock SPI read function
  */
int32_t i3g4250d_platform_read(void *handle, uint8_t reg, 
                               uint8_t *data, uint16_t len)
{
  (void)handle;  /* Unused in mock */
  
  mock_stats.read_count++;
  mock_stats.last_reg_read = reg;
  
  /* Simulate auto-increment read */
  for (uint16_t i = 0; i < len; i++) {
    data[i] = mock_registers[(reg + i) & 0xFF];
  }
  
  /* Special behavior: STATUS_REG shows data ready after power-on */
  if (reg == 0x27 && (mock_registers[0x20] & 0x08)) {  /* If PD=1 */
    mock_registers[0x27] |= 0x0F;  /* Set ZYXDA, XDA, YDA, ZDA */
  }
  
  return 0;  /* Success */
}

/**
  * @brief  Mock SPI write function
  */
int32_t i3g4250d_platform_write(void *handle, uint8_t reg,
                                const uint8_t *data, uint16_t len)
{
  (void)handle;  /* Unused in mock */
  
  mock_stats.write_count++;
  mock_stats.last_reg_written = reg;
  
  /* WHO_AM_I is read-only */
  if (reg == 0x0F) {
    return 0;  /* Silently ignore */
  }
  
  /* Simulate auto-increment write */
  for (uint16_t i = 0; i < len; i++) {
    mock_registers[(reg + i) & 0xFF] = data[i];
  }
  
  return 0;  /* Success */
}

/**
  * @brief  Mock delay function
  */
void i3g4250d_platform_delay(uint32_t ms)
{
  (void)ms;  /* No actual delay in mock */
}

/**
  * @brief  Initialize mock platform
  */
int32_t i3g4250d_platform_init(stmdev_ctx_t *ctx, void *spi_handle)
{
  ctx->write_reg = i3g4250d_platform_write;
  ctx->read_reg  = i3g4250d_platform_read;
  ctx->mdelay    = i3g4250d_platform_delay;
  ctx->handle    = spi_handle;
  
  /* Reset statistics */
  memset(&mock_stats, 0, sizeof(mock_stats));
  
  return 0;
}

/* ============================================================================
 * TEST HELPER FUNCTIONS
 * ============================================================================ */

/**
  * @brief  Set simulated gyro output values
  * @param  x, y, z  Raw 16-bit values
  */
void mock_set_gyro_data(int16_t x, int16_t y, int16_t z)
{
  mock_registers[0x28] = (uint8_t)(x & 0xFF);
  mock_registers[0x29] = (uint8_t)(x >> 8);
  mock_registers[0x2A] = (uint8_t)(y & 0xFF);
  mock_registers[0x2B] = (uint8_t)(y >> 8);
  mock_registers[0x2C] = (uint8_t)(z & 0xFF);
  mock_registers[0x2D] = (uint8_t)(z >> 8);
}

/**
  * @brief  Set simulated temperature
  * @param  temp  Raw temperature value (offset from 25°C)
  */
void mock_set_temperature(int8_t temp)
{
  mock_registers[0x26] = (uint8_t)temp;
}

/**
  * @brief  Get read operation count
  */
uint32_t mock_get_read_count(void)
{
  return mock_stats.read_count;
}

/**
  * @brief  Get write operation count
  */
uint32_t mock_get_write_count(void)
{
  return mock_stats.write_count;
}

/**
  * @brief  Reset mock to power-on state
  */
void mock_reset(void)
{
  /* Reset registers to power-on defaults */
  memset(mock_registers, 0, sizeof(mock_registers));
  mock_registers[0x0F] = 0xD3;  /* WHO_AM_I */
  mock_registers[0x20] = 0x07;  /* CTRL_REG1 default */
  mock_registers[0x2F] = 0x20;  /* FIFO empty */
  
  /* Reset statistics */
  memset(&mock_stats, 0, sizeof(mock_stats));
}

/* ============================================================================
 * UNIT TESTS (Simple Test Framework)
 * ============================================================================ */

#include <assert.h>

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) void test_##name(void)
#define RUN_TEST(name) do { \
    printf("Testing: " #name "... "); \
    mock_reset(); \
    test_##name(); \
    printf("PASSED\n"); \
    tests_passed++; \
  } while(0)

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
      printf("FAILED: %s != %s (line %d)\n", #a, #b, __LINE__); \
      tests_failed++; \
      return; \
    } \
  } while(0)

/* Test: WHO_AM_I read */
TEST(who_am_i)
{
  stmdev_ctx_t ctx;
  uint8_t id;
  
  i3g4250d_platform_init(&ctx, NULL);
  i3g4250d_device_id_get(&ctx, &id);
  
  ASSERT_EQ(id, 0xD3);
}

/* Test: Power mode set/get */
TEST(power_mode)
{
  stmdev_ctx_t ctx;
  uint8_t mode;
  
  i3g4250d_platform_init(&ctx, NULL);
  
  /* Set normal mode */
  i3g4250d_power_mode_set(&ctx, 1);
  i3g4250d_power_mode_get(&ctx, &mode);
  ASSERT_EQ(mode, 1);
  
  /* Set power-down */
  i3g4250d_power_mode_set(&ctx, 0);
  i3g4250d_power_mode_get(&ctx, &mode);
  ASSERT_EQ(mode, 0);
}

/* Test: Full-scale configuration */
TEST(full_scale)
{
  stmdev_ctx_t ctx;
  i3g4250d_fs_t fs;
  
  i3g4250d_platform_init(&ctx, NULL);
  
  i3g4250d_full_scale_set(&ctx, I3G4250D_245dps);
  i3g4250d_full_scale_get(&ctx, &fs);
  ASSERT_EQ(fs, I3G4250D_245dps);
  
  i3g4250d_full_scale_set(&ctx, I3G4250D_500dps);
  i3g4250d_full_scale_get(&ctx, &fs);
  ASSERT_EQ(fs, I3G4250D_500dps);
  
  i3g4250d_full_scale_set(&ctx, I3G4250D_2000dps);
  i3g4250d_full_scale_get(&ctx, &fs);
  ASSERT_EQ(fs, I3G4250D_2000dps);
}

/* Test: ODR configuration */
TEST(odr)
{
  stmdev_ctx_t ctx;
  i3g4250d_dr_t odr;
  
  i3g4250d_platform_init(&ctx, NULL);
  
  i3g4250d_data_rate_set(&ctx, I3G4250D_ODR_100Hz);
  i3g4250d_data_rate_get(&ctx, &odr);
  ASSERT_EQ(odr, I3G4250D_ODR_100Hz);
  
  i3g4250d_data_rate_set(&ctx, I3G4250D_ODR_400Hz);
  i3g4250d_data_rate_get(&ctx, &odr);
  ASSERT_EQ(odr, I3G4250D_ODR_400Hz);
}

/* Test: Multi-byte read (gyro data) */
TEST(multi_byte_read)
{
  stmdev_ctx_t ctx;
  int16_t data[3];
  
  i3g4250d_platform_init(&ctx, NULL);
  
  /* Set known values */
  mock_set_gyro_data(1000, -2000, 3000);
  
  /* Enable power for data ready */
  i3g4250d_power_mode_set(&ctx, 1);
  
  /* Read data */
  i3g4250d_angular_rate_raw_get(&ctx, data);
  
  ASSERT_EQ(data[0], 1000);
  ASSERT_EQ(data[1], -2000);
  ASSERT_EQ(data[2], 3000);
}

/* Test: Temperature read */
TEST(temperature)
{
  stmdev_ctx_t ctx;
  uint8_t temp;
  
  i3g4250d_platform_init(&ctx, NULL);
  
  mock_set_temperature(-5);  /* 20°C (25 - 5) */
  i3g4250d_temperature_raw_get(&ctx, &temp);
  
  ASSERT_EQ((int8_t)temp, -5);
}

/* Main test runner */
int main(void)
{
  printf("\n=== I3G4250D MOCK UNIT TESTS ===\n\n");
  
  RUN_TEST(who_am_i);
  RUN_TEST(power_mode);
  RUN_TEST(full_scale);
  RUN_TEST(odr);
  RUN_TEST(multi_byte_read);
  RUN_TEST(temperature);
  
  printf("\n=== RESULTS: %d passed, %d failed ===\n\n", 
         tests_passed, tests_failed);
  
  return tests_failed > 0 ? 1 : 0;
}

#endif /* I3G4250D_MOCK_TEST */
