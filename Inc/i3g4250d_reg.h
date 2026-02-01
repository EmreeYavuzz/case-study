/**
  ******************************************************************************
  * @file    i3g4250d_reg.h
  * @author  Emre Yavuz
  * @brief   This file contains all the functions prototypes for the
  *          i3g4250d_reg.c driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I3G4250D_REGS_H
#define I3G4250D_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup I3G4250D
  * @{
  *
  */

/** @defgroup  Endianness definitions
  * @{
  *
  */

#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if DRV_BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER         DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER         DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN    __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER    __BYTE_ORDER__

#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES
typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

typedef struct
{
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Component optional fields **/
  stmdev_mdelay_ptr   mdelay;
  /** Customizable optional pointer **/
  void *handle;

  /** private data **/
  void *priv_data;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES   

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct
{
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/**
  * @defgroup i3g4250d_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  if SA0=0 -> 0xD1 if SA0=1 -> 0xD3 **/
#define I3G4250D_I2C_ADD_L               0xD1U
#define I3G4250D_I2C_ADD_H               0xD3U

/** Device Identification (Who am I) **/
#define I3G4250D_ID                      0xD3U

#define I3G4250D_WHO_AM_I                0x0FU

/**
  * @}
  *
  */

/**
 * @brief Generate a bit mask with bit @p n set.
 *
 * This macro is used to define single-bit fields in registers
 * in a readable and portable way.
 *
 * Example:
 *   BIT(3) -> 0b00001000
 */
#define BIT(n)                          (1U << (n))

/**
 * @brief Prepare a value for insertion into a multi-bit register field.
 *
 * This macro shifts the given value to the field position
 * and applies the corresponding mask.
 *
 * This implementation is fully portable and does not rely
 * on compiler-specific built-ins.
 *
 * @param mask Bit mask defining the target field.
 * @param pos  Bit position of the field LSB.
 * @param val  Value to be written into the field.
 */
#define FIELD_PREP(mask, pos, val)  ((((uint8_t)(val)) << (pos)) & (mask))

/**
 * @brief Extract a value from a multi-bit register field.
 *
 * This macro masks and right-shifts a register value
 * to obtain the raw field value.
 *
 * @param mask Bit mask defining the target field.
 * @param pos  Bit position of the field LSB.
 * @param reg  Register value.
 * @return     Extracted field value.
 */
#define FIELD_GET(mask, pos, reg) ((((uint8_t)(reg) & (mask)) >> (pos)))


/**
 * @brief CTRL_REG1 (0x20)
 * Main control register.
 *
 * This register configures the main operating mode of the gyroscope,
 * including output data rate, bandwidth, power state, and axis enabling.
 *
 * Bit mapping (datasheet):
 *  bit7: DR1   - Output data rate selection
 *  bit6: DR0   - Output data rate selection
 *  bit5: BW1   - Bandwidth selection
 *  bit4: BW0   - Bandwidth selection
 *  bit3: PD    - Power-down control (0: power-down, 1: normal mode)
 *  bit2: ZEN   - Z-axis enable
 *  bit1: YEN   - Y-axis enable
 *  bit0: XEN   - X-axis enable
 */
#define I3G4250D_CTRL_REG1               0x20U

/* Output Data Rate (DR[1:0]) */
#define I3G4250D_CTRL1_DR_POS            6U
#define I3G4250D_CTRL1_DR_MASK           (0x3U << I3G4250D_CTRL1_DR_POS)

/* Bandwidth (BW[1:0]) */
#define I3G4250D_CTRL1_BW_POS            4U
#define I3G4250D_CTRL1_BW_MASK           (0x3U << I3G4250D_CTRL1_BW_POS)

/* Power and axis enable bits */
#define I3G4250D_CTRL1_PD_BIT            BIT(3)
#define I3G4250D_CTRL1_ZEN_BIT           BIT(2)
#define I3G4250D_CTRL1_YEN_BIT           BIT(1)
#define I3G4250D_CTRL1_XEN_BIT           BIT(0)



/**
 * @brief CTRL_REG2 (0x21)
 * High-pass filter configuration register.
 *
 * This register controls the operating mode and cutoff frequency
 * of the internal high-pass filter applied to the angular rate data.
 *
 * Bit mapping (datasheet):
 *  bit7: NOT USED
 *  bit6: NOT USED
 *  bit5: HPM1  - High-pass filter mode selection
 *  bit4: HPM0  - High-pass filter mode selection
 *  bit3: HPCF3 - High-pass filter cutoff frequency
 *  bit2: HPCF2 - High-pass filter cutoff frequency
 *  bit1: HPCF1 - High-pass filter cutoff frequency
 *  bit0: HPCF0 - High-pass filter cutoff frequency
 */
#define I3G4250D_CTRL_REG2               0x21U

/* High-pass filter mode (HPM[1:0]) */
#define I3G4250D_CTRL2_HPM_POS           4U
#define I3G4250D_CTRL2_HPM_MASK          (0x3U << I3G4250D_CTRL2_HPM_POS)

/* High-pass filter cutoff frequency (HPCF[3:0]) */
#define I3G4250D_CTRL2_HPCF_POS          0U
#define I3G4250D_CTRL2_HPCF_MASK         (0xFU << I3G4250D_CTRL2_HPCF_POS)

                                         

/**
 * @brief CTRL_REG3 (0x22)
 * Interrupt pin configuration register.
 *
 * This register controls which events are routed to INT1 and INT2 pins
 * and defines the electrical behavior of interrupt pins.
 *
 * Bit mapping (datasheet):
 *  bit7: I1_INT1   - Enable INT1 interrupt on INT1 pin
 *  bit6: I1_BOOT   - Enable boot status on INT1 pin
 *  bit5: H_LACTIVE - Interrupt active level (0: high, 1: low)
 *  bit4: PP_OD     - Push-pull / open-drain selection
 *  bit3: I2_DRDY   - Data-ready interrupt on INT2
 *  bit2: I2_WTM    - FIFO watermark interrupt on INT2
 *  bit1: I2_ORUN   - FIFO overrun interrupt on INT2
 *  bit0: I2_EMPTY  - FIFO empty interrupt on INT2
 */
#define I3G4250D_CTRL_REG3               0x22U

#define I3G4250D_CTRL3_I1_INT1_BIT       BIT(7)
#define I3G4250D_CTRL3_I1_BOOT_BIT       BIT(6)
#define I3G4250D_CTRL3_H_LACTIVE_BIT     BIT(5)
#define I3G4250D_CTRL3_PP_OD_BIT         BIT(4)
#define I3G4250D_CTRL3_I2_DRDY_BIT       BIT(3)
#define I3G4250D_CTRL3_I2_WTM_BIT        BIT(2)
#define I3G4250D_CTRL3_I2_ORUN_BIT       BIT(1)
#define I3G4250D_CTRL3_I2_EMPTY_BIT      BIT(0)


/**
 * @brief CTRL_REG4 (0x23)
 * Interface and full-scale configuration register.
 *
 * This register configures the measurement range, data endianness,
 * self-test mode, and SPI interface mode.
 *
 * Bit mapping (datasheet):
 *  bit7: NOT USED
 *  bit6: BLE   - Big/Little Endian data selection
 *  bit5: FS1   - Full-scale selection
 *  bit4: FS0   - Full-scale selection
 *  bit3: NOT USED
 *  bit2: ST1   - Self-test enable
 *  bit1: ST0   - Self-test enable
 *  bit0: SIM   - SPI interface mode selection
 */
#define I3G4250D_CTRL_REG4               0x23U

#define I3G4250D_CTRL4_BLE_BIT           BIT(6)

/* Full-scale selection (FS[1:0]) */
#define I3G4250D_CTRL4_FS_POS            4U
#define I3G4250D_CTRL4_FS_MASK           (0x3U << I3G4250D_CTRL4_FS_POS)

/* Self-test enable (ST[1:0]) */
#define I3G4250D_CTRL4_ST_POS            1U
#define I3G4250D_CTRL4_ST_MASK           (0x3U << I3G4250D_CTRL4_ST_POS)

#define I3G4250D_CTRL4_SIM_BIT           BIT(0)




/**
 * @brief CTRL_REG5 (0x24)
 * FIFO, reboot memory and HP filter enable register.
 *
 * Bit mapping (datasheet):
 *  bit7: BOOT      - Reboot memory content
 *  bit6: FIFO_EN  - FIFO enable
 *  bit5: NOT USED
 *  bit4: HPEN     - High-pass filter enable
 *  bit3: INT1_SEL1
 *  bit2: INT1_SEL0
 *  bit1: OUT_SEL1
 *  bit0: OUT_SEL0
 */
#define I3G4250D_CTRL_REG5               0x24U

#define I3G4250D_CTRL5_BOOT_BIT          BIT(7)
#define I3G4250D_CTRL5_FIFO_EN_BIT       BIT(6)
#define I3G4250D_CTRL5_HPEN_BIT          BIT(4)

/* INT1 selection configuration (INT1_SEL[1:0]) */
#define I3G4250D_CTRL5_INT1_SEL_POS      2U
#define I3G4250D_CTRL5_INT1_SEL_MASK     (0x3U << I3G4250D_CTRL5_INT1_SEL_POS)

/* Out selection configuration (OUT_SEL[1:0]) */
#define I3G4250D_CTRL5_OUT_SEL_POS       0U
#define I3G4250D_CTRL5_OUT_SEL_MASK      (0x3U << I3G4250D_CTRL5_OUT_SEL_POS)



/**
 * @brief REFERENCE (0x25)
 * Reference value for HP filter.
 */
#define I3G4250D_REFERENCE               0x25U


/**
 * @brief OUT_TEMP (0x26)
 * Temperature output register.
 *
 * This register contains the raw temperature data measured
 * by the internal temperature sensor.
 *
 * The temperature value is provided for monitoring purposes
 * and is not intended for precise temperature measurement.
 */
#define I3G4250D_OUT_TEMP                0x26U


/**
 * @brief STATUS_REG (0x27)
 * Data availability and overrun status register.
 *
 * Bit mapping:
 *  bit7: ZYXOR  - X/Y/Z overrun
 *  bit6: ZOR    - Z overrun
 *  bit5: YOR    - Y overrun
 *  bit4: XOR    - X overrun
 *  bit3: ZYXDA  - X/Y/Z new data available
 *  bit2: ZDA    - Z new data available
 *  bit1: YDA    - Y new data available
 *  bit0: XDA    - X new data available
 */
#define I3G4250D_STATUS_REG              0x27U

#define I3G4250D_STATUS_XDA_BIT          BIT(0)
#define I3G4250D_STATUS_YDA_BIT          BIT(1)
#define I3G4250D_STATUS_ZDA_BIT          BIT(2)
#define I3G4250D_STATUS_ZYXDA_BIT        BIT(3)

#define I3G4250D_STATUS_XOR_BIT          BIT(4)
#define I3G4250D_STATUS_YOR_BIT          BIT(5)
#define I3G4250D_STATUS_ZOR_BIT          BIT(6)
#define I3G4250D_STATUS_ZYXOR_BIT        BIT(7)




#define I3G4250D_OUT_X_L                 0x28U
#define I3G4250D_OUT_X_H                 0x29U
#define I3G4250D_OUT_Y_L                 0x2AU
#define I3G4250D_OUT_Y_H                 0x2BU
#define I3G4250D_OUT_Z_L                 0x2CU
#define I3G4250D_OUT_Z_H                 0x2DU

/**
 * @brief FIFO_CTRL_REG (0x2E)
 * FIFO control register.
 *
 * Bit mapping:
 *  bit7-5: FM[2:0]  - FIFO mode selection
 *  bit4-0: WTM[4:0] - FIFO watermark level (0-31)
 */
#define I3G4250D_FIFO_CTRL_REG           0x2EU

/* FIFO watermark level (WTM[4:0]) */
#define I3G4250D_FIFO_CTRL_WTM_POS       0U
#define I3G4250D_FIFO_CTRL_WTM_MASK      (0x1FU << I3G4250D_FIFO_CTRL_WTM_POS)

/* FIFO mode selection (FM[2:0]) */
#define I3G4250D_FIFO_CTRL_FM_POS        5U
#define I3G4250D_FIFO_CTRL_FM_MASK       (0x7U << I3G4250D_FIFO_CTRL_FM_POS)

/**
 * @brief FIFO_SRC_REG (0x2F)
 * FIFO source register.
 *
 * Bit mapping:
 *  bit7: WTM   - Watermark status (1: FIFO >= watermark level)
 *  bit6: OVRN  - Overrun status (1: FIFO is full)
 *  bit5: EMPTY - FIFO empty bit (1: FIFO is empty)
 *  bit4-0: FSS[4:0] - FIFO stored data level (0-32)
 */
#define I3G4250D_FIFO_SRC_REG            0x2FU

/* FIFO stored data level (FSS[4:0]) */
#define I3G4250D_FIFO_SRC_FSS_POS        0U
#define I3G4250D_FIFO_SRC_FSS_MASK       (0x1FU << I3G4250D_FIFO_SRC_FSS_POS)

#define I3G4250D_FIFO_SRC_EMPTY_BIT      BIT(5)
#define I3G4250D_FIFO_SRC_OVRN_BIT       BIT(6)
#define I3G4250D_FIFO_SRC_WTM_BIT        BIT(7)

/**
 * @brief INT1_CFG (0x30)
 * Interrupt 1 configuration register.
 *
 * Bit mapping:
 *  bit7: AND/OR - AND/OR combination of interrupt events
 *  bit6: LIR    - Latch interrupt request
 *  bit5: ZHIE   - Enable interrupt on Z high event
 *  bit4: ZLIE   - Enable interrupt on Z low event
 *  bit3: YHIE   - Enable interrupt on Y high event
 *  bit2: YLIE   - Enable interrupt on Y low event
 *  bit1: XHIE   - Enable interrupt on X high event
 *  bit0: XLIE   - Enable interrupt on X low event
 */
#define I3G4250D_INT1_CFG                0x30U

#define I3G4250D_INT1_CFG_XLIE_BIT       BIT(0)
#define I3G4250D_INT1_CFG_XHIE_BIT       BIT(1)
#define I3G4250D_INT1_CFG_YLIE_BIT       BIT(2)
#define I3G4250D_INT1_CFG_YHIE_BIT       BIT(3)
#define I3G4250D_INT1_CFG_ZLIE_BIT       BIT(4)
#define I3G4250D_INT1_CFG_ZHIE_BIT       BIT(5)
#define I3G4250D_INT1_CFG_LIR_BIT        BIT(6)
#define I3G4250D_INT1_CFG_AND_OR_BIT     BIT(7)

/* INT1_CFG as uint8_t typedef for direct register access */
typedef uint8_t i3g4250d_int1_cfg_t;

/**
 * @brief INT1_SRC (0x31)
 * Interrupt 1 source register.
 *
 * Bit mapping:
 *  bit7: NOT USED
 *  bit6: IA - Interrupt active (one or more interrupts generated)
 *  bit5: ZH - Z high event occurred
 *  bit4: ZL - Z low event occurred
 *  bit3: YH - Y high event occurred
 *  bit2: YL - Y low event occurred
 *  bit1: XH - X high event occurred
 *  bit0: XL - X low event occurred
 */
#define I3G4250D_INT1_SRC                0x31U

#define I3G4250D_INT1_SRC_XL_BIT         BIT(0)
#define I3G4250D_INT1_SRC_XH_BIT         BIT(1)
#define I3G4250D_INT1_SRC_YL_BIT         BIT(2)
#define I3G4250D_INT1_SRC_YH_BIT         BIT(3)
#define I3G4250D_INT1_SRC_ZL_BIT         BIT(4)
#define I3G4250D_INT1_SRC_ZH_BIT         BIT(5)
#define I3G4250D_INT1_SRC_IA_BIT         BIT(6)

/* INT1_SRC as uint8_t typedef for direct register access */
typedef uint8_t i3g4250d_int1_src_t;

/**
 * @brief INT1_TSH_XH (0x32)
 * Interrupt 1 threshold X-axis high byte.
 *
 * Bit mapping:
 *  bit7: NOT USED
 *  bit6-0: THSX[14:8] - X-axis threshold high byte
 */
#define I3G4250D_INT1_TSH_XH             0x32U

/* X-axis threshold high byte (THSX[14:8]) */
#define I3G4250D_INT1_TSH_XH_POS         0U
#define I3G4250D_INT1_TSH_XH_MASK        (0x7FU << I3G4250D_INT1_TSH_XH_POS)

/**
 * @brief INT1_TSH_XL (0x33)
 * Interrupt 1 threshold X-axis low byte.
 *
 * Bit mapping:
 *  bit7-0: THSX[7:0] - X-axis threshold low byte (full 8-bit value)
 */
#define I3G4250D_INT1_TSH_XL             0x33U

/**
 * @brief INT1_TSH_YH (0x34)
 * Interrupt 1 threshold Y-axis high byte.
 *
 * Bit mapping:
 *  bit7: NOT USED
 *  bit6-0: THSY[14:8] - Y-axis threshold high byte
 */
#define I3G4250D_INT1_TSH_YH             0x34U

/* Y-axis threshold high byte (THSY[14:8]) */
#define I3G4250D_INT1_TSH_YH_POS         0U
#define I3G4250D_INT1_TSH_YH_MASK        (0x7FU << I3G4250D_INT1_TSH_YH_POS)

/**
 * @brief INT1_TSH_YL (0x35)
 * Interrupt 1 threshold Y-axis low byte.
 *
 * Bit mapping:
 *  bit7-0: THSY[7:0] - Y-axis threshold low byte (full 8-bit value)
 */
#define I3G4250D_INT1_TSH_YL             0x35U

/**
 * @brief INT1_TSH_ZH (0x36)
 * Interrupt 1 threshold Z-axis high byte.
 *
 * Bit mapping:
 *  bit7: NOT USED
 *  bit6-0: THSZ[14:8] - Z-axis threshold high byte
 */
#define I3G4250D_INT1_TSH_ZH             0x36U

/* Z-axis threshold high byte (THSZ[14:8]) */
#define I3G4250D_INT1_TSH_ZH_POS         0U
#define I3G4250D_INT1_TSH_ZH_MASK        (0x7FU << I3G4250D_INT1_TSH_ZH_POS)

/**
 * @brief INT1_TSH_ZL (0x37)
 * Interrupt 1 threshold Z-axis low byte.
 *
 * Bit mapping:
 *  bit7-0: THSZ[7:0] - Z-axis threshold low byte (full 8-bit value)
 */
#define I3G4250D_INT1_TSH_ZL             0x37U

/**
 * @brief INT1_DURATION (0x38)
 * Interrupt 1 duration register.
 *
 * Bit mapping:
 *  bit7: WAIT - Wait enable (0: disabled, 1: enabled)
 *  bit6-0: D[6:0] - Duration value (0-127)
 */
#define I3G4250D_INT1_DURATION           0x38U

/* Duration value (D[6:0]) */
#define I3G4250D_INT1_DURATION_D_POS     0U
#define I3G4250D_INT1_DURATION_D_MASK    (0x7FU << I3G4250D_INT1_DURATION_D_POS)

#define I3G4250D_INT1_DURATION_WAIT_BIT  BIT(7)



/**
  * @note  Bitfield structures have been removed in favor of mask-based
  *        register access for better portability and MISRA-C compliance.
  *        Use BIT() macros and FIELD_PREP/FIELD_GET for register manipulation.
  */

/*
 * These are the basic platform dependent I/O routines to read
 * and write device registers connected on a standard bus.
 * The driver keeps offering a default implementation based on function
 * pointers to read/write routines for backward compatibility.
 */

int32_t i3g4250d_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len);
int32_t i3g4250d_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

float_t i3g4250d_from_fs245dps_to_mdps(int16_t lsb);
float_t i3g4250d_from_lsb_to_celsius(int16_t lsb);

int32_t i3g4250d_axis_x_data_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_axis_x_data_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_axis_y_data_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_axis_y_data_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_axis_z_data_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_axis_z_data_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief Power mode control
 * Controls the PD bit (bit3) in CTRL_REG1
 */
int32_t i3g4250d_power_mode_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_power_mode_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief Output data rate selection (DR[1:0] field)
 * Field values for CTRL_REG1 bits [7:6]
 * 
 * Note: Power mode (PD bit) must be set separately via i3g4250d_power_mode_set()
 *       Axis enable must be set separately via i3g4250d_axis_x/y/z_data_set()
 */
typedef enum
{
  I3G4250D_ODR_100Hz   = 0,   /* DR[1:0] = 00 */
  I3G4250D_ODR_200Hz   = 1,   /* DR[1:0] = 01 */
  I3G4250D_ODR_400Hz   = 2,   /* DR[1:0] = 10 */
  I3G4250D_ODR_800Hz   = 3,   /* DR[1:0] = 11 */
} i3g4250d_dr_t;
int32_t i3g4250d_data_rate_set(const stmdev_ctx_t *ctx, i3g4250d_dr_t val);
int32_t i3g4250d_data_rate_get(const stmdev_ctx_t *ctx, i3g4250d_dr_t *val);

typedef enum
{
  I3G4250D_245dps     = 0x00,
  I3G4250D_500dps     = 0x01,
  I3G4250D_2000dps    = 0x02,
} i3g4250d_fs_t;
int32_t i3g4250d_full_scale_set(const stmdev_ctx_t *ctx, i3g4250d_fs_t val);
int32_t i3g4250d_full_scale_get(const stmdev_ctx_t *ctx,
                                i3g4250d_fs_t *val);

int32_t i3g4250d_status_reg_get(const stmdev_ctx_t *ctx,
                                uint8_t *val);

int32_t i3g4250d_flag_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_temperature_raw_get(const stmdev_ctx_t *ctx,
                                     uint8_t *buff);

int32_t i3g4250d_angular_rate_raw_get(const stmdev_ctx_t *ctx,
                                      int16_t *val);

int32_t i3g4250d_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum
{
  I3G4250D_GY_ST_DISABLE    = 0,
  I3G4250D_GY_ST_POSITIVE   = 1,
  I3G4250D_GY_ST_NEGATIVE   = 3,
} i3g4250d_st_t;
int32_t i3g4250d_self_test_set(const stmdev_ctx_t *ctx, i3g4250d_st_t val);
int32_t i3g4250d_self_test_get(const stmdev_ctx_t *ctx, i3g4250d_st_t *val);

typedef enum
{
  I3G4250D_AUX_LSB_AT_LOW_ADD   = 0,
  I3G4250D_AUX_MSB_AT_LOW_ADD   = 1,
} i3g4250d_ble_t;
int32_t i3g4250d_data_format_set(const stmdev_ctx_t *ctx,
                                 i3g4250d_ble_t val);
int32_t i3g4250d_data_format_get(const stmdev_ctx_t *ctx,
                                 i3g4250d_ble_t *val);

int32_t i3g4250d_boot_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_boot_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  I3G4250D_CUT_OFF_LOW        = 0,
  I3G4250D_CUT_OFF_MEDIUM     = 1,
  I3G4250D_CUT_OFF_HIGH       = 2,
  I3G4250D_CUT_OFF_VERY_HIGH  = 3,
} i3g4250d_bw_t;
int32_t i3g4250d_lp_bandwidth_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_bw_t val);
int32_t i3g4250d_lp_bandwidth_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_bw_t *val);

typedef enum
{
  I3G4250D_HP_LEVEL_0   = 0,
  I3G4250D_HP_LEVEL_1   = 1,
  I3G4250D_HP_LEVEL_2   = 2,
  I3G4250D_HP_LEVEL_3   = 3,
  I3G4250D_HP_LEVEL_4   = 4,
  I3G4250D_HP_LEVEL_5   = 5,
  I3G4250D_HP_LEVEL_6   = 6,
  I3G4250D_HP_LEVEL_7   = 7,
  I3G4250D_HP_LEVEL_8   = 8,
  I3G4250D_HP_LEVEL_9   = 9,
} i3g4250d_hpcf_t;
int32_t i3g4250d_hp_bandwidth_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_hpcf_t val);
int32_t i3g4250d_hp_bandwidth_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_hpcf_t *val);

typedef enum
{
  I3G4250D_HP_NORMAL_MODE_WITH_RST  = 0,
  I3G4250D_HP_REFERENCE_SIGNAL      = 1,
  I3G4250D_HP_NORMAL_MODE           = 2,
  I3G4250D_HP_AUTO_RESET_ON_INT     = 3,
} i3g4250d_hpm_t;
int32_t i3g4250d_hp_mode_set(const stmdev_ctx_t *ctx, i3g4250d_hpm_t val);
int32_t i3g4250d_hp_mode_get(const stmdev_ctx_t *ctx, i3g4250d_hpm_t *val);

typedef enum
{
  I3G4250D_ONLY_LPF1_ON_OUT     = 0,
  I3G4250D_LPF1_HP_ON_OUT       = 1,
  I3G4250D_LPF1_LPF2_ON_OUT     = 2,
  I3G4250D_LPF1_HP_LPF2_ON_OUT  = 6,
} i3g4250d_out_sel_t;
int32_t i3g4250d_filter_path_set(const stmdev_ctx_t *ctx,
                                 i3g4250d_out_sel_t val);
int32_t i3g4250d_filter_path_get(const stmdev_ctx_t *ctx,
                                 i3g4250d_out_sel_t *val);

typedef enum
{
  I3G4250D_ONLY_LPF1_ON_INT     = 0,
  I3G4250D_LPF1_HP_ON_INT       = 1,
  I3G4250D_LPF1_LPF2_ON_INT     = 2,
  I3G4250D_LPF1_HP_LPF2_ON_INT  = 6,
} i3g4250d_int1_sel_t;
int32_t i3g4250d_filter_path_internal_set(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_sel_t val);
int32_t i3g4250d_filter_path_internal_get(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_sel_t *val);

int32_t i3g4250d_hp_reference_value_set(const stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t i3g4250d_hp_reference_value_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum
{
  I3G4250D_SPI_4_WIRE  = 0,
  I3G4250D_SPI_3_WIRE  = 1,
} i3g4250d_sim_t;
int32_t i3g4250d_spi_mode_set(const stmdev_ctx_t *ctx, i3g4250d_sim_t val);
int32_t i3g4250d_spi_mode_get(const stmdev_ctx_t *ctx, i3g4250d_sim_t *val);

/**
 * @brief INT1 pin routing configuration (mask-based)
 * Use bitwise OR to combine multiple routing options
 */
typedef uint8_t i3g4250d_int1_route_t;

#define I3G4250D_INT1_ROUTE_INT1  BIT(7)  /* INT1 interrupt on INT1 pin */
#define I3G4250D_INT1_ROUTE_BOOT  BIT(6)  /* Boot status on INT1 pin */

int32_t i3g4250d_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                    i3g4250d_int1_route_t val);
int32_t i3g4250d_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                    i3g4250d_int1_route_t *val);

/**
 * @brief INT2 pin routing configuration (mask-based)
 * Use bitwise OR to combine multiple routing options
 */
typedef uint8_t i3g4250d_int2_route_t;

#define I3G4250D_INT2_ROUTE_EMPTY BIT(0)  /* FIFO empty interrupt on INT2 */
#define I3G4250D_INT2_ROUTE_ORUN  BIT(1)  /* FIFO overrun interrupt on INT2 */
#define I3G4250D_INT2_ROUTE_WTM   BIT(2)  /* FIFO watermark interrupt on INT2 */
#define I3G4250D_INT2_ROUTE_DRDY  BIT(3)  /* Data ready interrupt on INT2 */

int32_t i3g4250d_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                    i3g4250d_int2_route_t val);
int32_t i3g4250d_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                    i3g4250d_int2_route_t *val);

typedef enum
{
  I3G4250D_PUSH_PULL   = 0,
  I3G4250D_OPEN_DRAIN  = 1,
} i3g4250d_pp_od_t;
int32_t i3g4250d_pin_mode_set(const stmdev_ctx_t *ctx,
                              i3g4250d_pp_od_t val);
int32_t i3g4250d_pin_mode_get(const stmdev_ctx_t *ctx,
                              i3g4250d_pp_od_t *val);

typedef enum
{
  I3G4250D_ACTIVE_HIGH  = 0,
  I3G4250D_ACTIVE_LOW   = 1,
} i3g4250d_h_lactive_t;
int32_t i3g4250d_pin_polarity_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_h_lactive_t val);
int32_t i3g4250d_pin_polarity_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_h_lactive_t *val);

typedef enum
{
  I3G4250D_INT_PULSED   = 0,
  I3G4250D_INT_LATCHED  = 1,
} i3g4250d_lir_t;
int32_t i3g4250d_int_notification_set(const stmdev_ctx_t *ctx,
                                      i3g4250d_lir_t val);
int32_t i3g4250d_int_notification_get(const stmdev_ctx_t *ctx,
                                      i3g4250d_lir_t *val);

int32_t i3g4250d_int_on_threshold_conf_set(const stmdev_ctx_t *ctx,
                                           i3g4250d_int1_cfg_t val);
int32_t i3g4250d_int_on_threshold_conf_get(const stmdev_ctx_t *ctx,
                                           i3g4250d_int1_cfg_t *val);

typedef enum
{
  I3G4250D_INT1_ON_TH_AND  = 1,
  I3G4250D_INT1_ON_TH_OR   = 0,
} i3g4250d_and_or_t;
int32_t i3g4250d_int_on_threshold_mode_set(const stmdev_ctx_t *ctx,
                                           i3g4250d_and_or_t val);
int32_t i3g4250d_int_on_threshold_mode_get(const stmdev_ctx_t *ctx,
                                           i3g4250d_and_or_t *val);

int32_t i3g4250d_int_on_threshold_src_get(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_src_t *val);

int32_t i3g4250d_int_x_threshold_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t i3g4250d_int_x_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val);

int32_t i3g4250d_int_y_threshold_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t i3g4250d_int_y_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val);

int32_t i3g4250d_int_z_threshold_set(const stmdev_ctx_t *ctx, uint16_t val);
int32_t i3g4250d_int_z_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val);

/**
 * @brief  Interrupt duration and WAIT configuration
 * @note   Original function - auto-enables WAIT when duration > 0
 */
int32_t i3g4250d_int_on_threshold_dur_set(const stmdev_ctx_t *ctx,
                                          uint8_t val);
int32_t i3g4250d_int_on_threshold_dur_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val);

/**
 * @brief  Interrupt duration with explicit WAIT control
 * @param  ctx         Read/write interface definitions
 * @param  duration    Duration value (D[6:0], 0-127)
 * @param  wait_enable WAIT bit: PROPERTY_DISABLE (0) or PROPERTY_ENABLE (1)
 * @retval 0: OK, -1: Error
 * 
 * @note   WAIT bit meaning (from datasheet Figure 19-20):
 *         - WAIT=0: Interrupt falls immediately when signal crosses threshold
 *         - WAIT=1: Interrupt falls only after duration samples counted
 */
int32_t i3g4250d_int_on_threshold_dur_set_ex(const stmdev_ctx_t *ctx,
                                              uint8_t duration,
                                              uint8_t wait_enable);

/**
 * @brief  Get interrupt duration and WAIT status
 * @param  ctx         Read/write interface definitions
 * @param  duration    Duration value (D[6:0]) - can be NULL
 * @param  wait_enable WAIT bit status - can be NULL
 * @retval 0: OK, -1: Error
 */
int32_t i3g4250d_int_on_threshold_dur_get_ex(const stmdev_ctx_t *ctx,
                                              uint8_t *duration,
                                              uint8_t *wait_enable);

/**
 * @brief  Set WAIT bit only (preserve duration value)
 * @param  ctx  Read/write interface definitions
 * @param  val  PROPERTY_DISABLE (0) or PROPERTY_ENABLE (1)
 * @retval 0: OK, -1: Error
 */
int32_t i3g4250d_int_wait_enable_set(const stmdev_ctx_t *ctx, uint8_t val);

/**
 * @brief  Get WAIT bit status
 * @param  ctx  Read/write interface definitions
 * @param  val  WAIT bit value (0 or 1)
 * @retval 0: OK, -1: Error
 */
int32_t i3g4250d_int_wait_enable_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_fifo_enable_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_fifo_enable_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val);
int32_t i3g4250d_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val);

typedef enum
{
  I3G4250D_FIFO_BYPASS_MODE     = 0x00,
  I3G4250D_FIFO_MODE            = 0x01,
  I3G4250D_FIFO_STREAM_MODE     = 0x02,
} i3g4250d_fifo_mode_t;
int32_t i3g4250d_fifo_mode_set(const stmdev_ctx_t *ctx,
                               i3g4250d_fifo_mode_t val);
int32_t i3g4250d_fifo_mode_get(const stmdev_ctx_t *ctx,
                               i3g4250d_fifo_mode_t *val);

int32_t i3g4250d_fifo_data_level_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_fifo_empty_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

int32_t i3g4250d_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* I3G4250D_REGS_H */