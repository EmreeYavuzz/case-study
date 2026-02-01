/**
  ******************************************************************************
  * @file    i3g4250d_reg.c
  * @author  Emre Yavuz
  * @brief   I3G4250D driver file
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

#include "i3g4250d_reg.h"

/**
  * @defgroup    I3G4250D
  * @brief       This file provides a set of functions needed to drive the
  *              i3g4250d enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    I3G4250D_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len)
{
  int32_t ret;

  if (ctx == NULL) return -1;

  ret = ctx->read_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                           uint8_t *data,
                           uint16_t len)
{
  int32_t ret;

  if (ctx == NULL) return -1;

  ret = ctx->write_reg(ctx->handle, reg, data, len);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    I3G4250D_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t i3g4250d_from_fs245dps_to_mdps(int16_t lsb)
{
  return ((float_t)lsb * 8.75f);
}

float_t i3g4250d_from_lsb_to_celsius(int16_t lsb)
{
  return ((float_t)lsb + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_data_generation
  * @brief      This section groups all the functions concerning
  *             data generation
  * @{
  *
  */

/**
  * @brief  Power mode control (PD bit in CTRL_REG1).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: power-down mode, 1: normal mode
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_power_mode_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg1 |= I3G4250D_CTRL1_PD_BIT;  // pd bitini 1 yap (diğerler bitler ellemeden) (normal mod)
  }
  else
  {
    ctrl_reg1 &= ~I3G4250D_CTRL1_PD_BIT; // pd bitini 0 yap (diğerler bitler ellemeden) (güç tasarrufu)
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  Power mode control (PD bit in CTRL_REG1).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: power-down mode, 1: normal mode.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_power_mode_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg1 & I3G4250D_CTRL1_PD_BIT) ? 1U : 0U;     // pd bitini oku

  return ret;
}

/**
  * @brief  Output data rate selection (DR[1:0] in CTRL_REG1).[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of dr in reg CTRL_REG1
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_data_rate_set(const stmdev_ctx_t *ctx, i3g4250d_dr_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  ctrl_reg1 &= ~I3G4250D_CTRL1_DR_MASK;     // DR[1:0] bitlerini sıfırla digerlerini eskileri koru
  ctrl_reg1 |= FIELD_PREP(I3G4250D_CTRL1_DR_MASK, I3G4250D_CTRL1_DR_POS, (uint8_t)val); // yeni degeri yaz sonra sıfırlanmışla orla

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  Output data rate selection (DR[1:0] in CTRL_REG1).[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dr in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_data_rate_get(const stmdev_ctx_t *ctx, i3g4250d_dr_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_dr_t)FIELD_GET(I3G4250D_CTRL1_DR_MASK, I3G4250D_CTRL1_DR_POS, ctrl_reg1);

  return ret;
}

/**
  * @brief  Gyroscope full-scale selection.[set]
  *
  * @param  ctx    read / write interface definitions(ptr)
  * @param  val    change the values of fs in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_full_scale_set(const stmdev_ctx_t *ctx, i3g4250d_fs_t val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  ctrl_reg4 &= ~I3G4250D_CTRL4_FS_MASK;
  ctrl_reg4 |= FIELD_PREP(I3G4250D_CTRL4_FS_MASK, I3G4250D_CTRL4_FS_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);

  return ret;
}

/**
  * @brief  Gyroscope full-scale selection.[get]
  *
  * @param  ctx    read / write interface definitions(ptr)
  * @param  val    Get the values of fs in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_full_scale_get(const stmdev_ctx_t *ctx, i3g4250d_fs_t *val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_fs_t)FIELD_GET(I3G4250D_CTRL4_FS_MASK, I3G4250D_CTRL4_FS_POS, ctrl_reg4);

  return ret;
}

/**
  * @brief  The STATUS_REG register is read by the primary interface.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    registers STATUS_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_status_reg_get(const stmdev_ctx_t *ctx,
                                uint8_t *val)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_STATUS_REG, val, 1);

  return ret;
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "zyxda" in reg STATUS_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_flag_data_ready_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t status_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_STATUS_REG, &status_reg, 1);
  if (ret != 0) { return ret; }

  *val = (status_reg & I3G4250D_STATUS_ZYXDA_BIT) ? 1U : 0U;

  return ret;
}
/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_Dataoutput
  * @brief      This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Temperature data.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_temperature_raw_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_OUT_TEMP, buff, 1);

  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit word in
  *         two's complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_angular_rate_raw_get(const stmdev_ctx_t *ctx, int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  ret =  i3g4250d_read_reg(ctx, I3G4250D_OUT_X_L, buff, 6);
  if (ret != 0) { return ret; }

  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_common
  * @brief      This section groups common useful functions.
  * @{
  *
  */

/**
  * @brief  Device Who amI.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_device_id_get(const stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_WHO_AM_I, buff, 1);

  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable. [set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    change the values of st in reg CTRL_REG4.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_self_test_set(const stmdev_ctx_t *ctx, i3g4250d_st_t val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  ctrl_reg4 &= ~I3G4250D_CTRL4_ST_MASK;
  ctrl_reg4 |= FIELD_PREP(I3G4250D_CTRL4_ST_MASK, I3G4250D_CTRL4_ST_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);

  return ret;
}

/**
  * @brief  Angular rate sensor self-test enable. [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_self_test_get(const stmdev_ctx_t *ctx, i3g4250d_st_t *val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_st_t)FIELD_GET(I3G4250D_CTRL4_ST_MASK, I3G4250D_CTRL4_ST_POS, ctrl_reg4);

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "ble" in reg CTRL_REG4.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_data_format_set(const stmdev_ctx_t *ctx,
                                 i3g4250d_ble_t val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_AUX_MSB_AT_LOW_ADD)
  {
    ctrl_reg4 |= I3G4250D_CTRL4_BLE_BIT;
  }
  else
  {
    ctrl_reg4 &= ~I3G4250D_CTRL4_BLE_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of "ble" in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_data_format_get(const stmdev_ctx_t *ctx,
                                 i3g4250d_ble_t *val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg4 & I3G4250D_CTRL4_BLE_BIT) ? I3G4250D_AUX_MSB_AT_LOW_ADD 
                                              : I3G4250D_AUX_LSB_AT_LOW_ADD;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL_REG5.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_boot_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg5 |= I3G4250D_CTRL5_BOOT_BIT;
  }
  else
  {
    ctrl_reg5 &= ~I3G4250D_CTRL5_BOOT_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of boot in reg CTRL_REG5.(ptr)
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_boot_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg5 & I3G4250D_CTRL5_BOOT_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_filters
  * @brief      This section group all the functions concerning the
  *             filters configuration.
  * @{
  *
  */

/**
  * @brief  Lowpass filter bandwidth selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "bw" in reg CTRL_REG1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_lp_bandwidth_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_bw_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  ctrl_reg1 &= ~I3G4250D_CTRL1_BW_MASK;
  ctrl_reg1 |= FIELD_PREP(I3G4250D_CTRL1_BW_MASK, I3G4250D_CTRL1_BW_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  Lowpass filter bandwidth selection.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of "bw" in reg CTRL_REG1.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_lp_bandwidth_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_bw_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_bw_t)FIELD_GET(I3G4250D_CTRL1_BW_MASK, I3G4250D_CTRL1_BW_POS, ctrl_reg1);

  return ret;
}

/**
  * @brief  High-pass filter bandwidth selection.[set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "hpcf" in reg CTRL_REG2.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_bandwidth_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_hpcf_t val)
{
  uint8_t ctrl_reg2;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);
  if (ret != 0) { return ret; }

  ctrl_reg2 &= ~I3G4250D_CTRL2_HPCF_MASK;
  ctrl_reg2 |= FIELD_PREP(I3G4250D_CTRL2_HPCF_MASK, I3G4250D_CTRL2_HPCF_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);

  return ret;
}

/**
  * @brief  High-pass filter bandwidth selection.[get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of hpcf in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_bandwidth_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_hpcf_t *val)
{
  uint8_t ctrl_reg2;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_hpcf_t)FIELD_GET(I3G4250D_CTRL2_HPCF_MASK, I3G4250D_CTRL2_HPCF_POS, ctrl_reg2);

  return ret;
}

/**
  * @brief  High-pass filter mode selection. [set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "hpm" in reg CTRL_REG2.
  * @retval         Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_mode_set(const stmdev_ctx_t *ctx, i3g4250d_hpm_t val)
{
  uint8_t ctrl_reg2;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);
  if (ret != 0) { return ret; }

  ctrl_reg2 &= ~I3G4250D_CTRL2_HPM_MASK;
  ctrl_reg2 |= FIELD_PREP(I3G4250D_CTRL2_HPM_MASK, I3G4250D_CTRL2_HPM_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);

  return ret;
}

/**
  * @brief  High-pass filter mode selection. [get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of hpm in reg CTRL_REG2.(ptr)
  * @retval         Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_mode_get(const stmdev_ctx_t *ctx, i3g4250d_hpm_t *val)
{
  uint8_t ctrl_reg2;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG2, &ctrl_reg2, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_hpm_t)FIELD_GET(I3G4250D_CTRL2_HPM_MASK, I3G4250D_CTRL2_HPM_POS, ctrl_reg2);

  return ret;
}

/**
  * @brief  Out/FIFO selection path. [set]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Change the values of "out_sel" in reg CTRL_REG5.
  * @retval         Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_filter_path_set(const stmdev_ctx_t *ctx,
                                 i3g4250d_out_sel_t val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  /* OUT_SEL uses bits [1:0], HPEN uses bit 4 encoded in enum bit 2 */
  ctrl_reg5 &= ~(I3G4250D_CTRL5_OUT_SEL_MASK | I3G4250D_CTRL5_HPEN_BIT);
  ctrl_reg5 |= FIELD_PREP(I3G4250D_CTRL5_OUT_SEL_MASK, I3G4250D_CTRL5_OUT_SEL_POS, 
                          (uint8_t)val & 0x03U);
  if ((uint8_t)val & 0x04U)
  {
    ctrl_reg5 |= I3G4250D_CTRL5_HPEN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);

  return ret;
}

/**
  * @brief  Out/FIFO selection path. [get]
  *
  * @param  ctx     Read / write interface definitions.(ptr)
  * @param  val     Get the values of out_sel in reg CTRL_REG5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_filter_path_get(const stmdev_ctx_t *ctx,
                                 i3g4250d_out_sel_t *val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  /* Reconstruct enum: HPEN goes to bit 2, OUT_SEL stays in bits [1:0] */
  uint8_t out_sel = FIELD_GET(I3G4250D_CTRL5_OUT_SEL_MASK, I3G4250D_CTRL5_OUT_SEL_POS, ctrl_reg5);
  uint8_t hpen = (ctrl_reg5 & I3G4250D_CTRL5_HPEN_BIT) ? 0x04U : 0x00U;
  *val = (i3g4250d_out_sel_t)(hpen | out_sel);

  return ret;
}

/**
  * @brief  Interrupt generator selection path.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of int1_sel in reg CTRL_REG5
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_filter_path_internal_set(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_sel_t val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  /* INT1_SEL uses bits [3:2], HPEN uses bit 4 encoded in enum bit 2 */
  ctrl_reg5 &= ~(I3G4250D_CTRL5_INT1_SEL_MASK | I3G4250D_CTRL5_HPEN_BIT);
  ctrl_reg5 |= FIELD_PREP(I3G4250D_CTRL5_INT1_SEL_MASK, I3G4250D_CTRL5_INT1_SEL_POS, 
                          (uint8_t)val & 0x03U);
  if ((uint8_t)val & 0x04U)
  {
    ctrl_reg5 |= I3G4250D_CTRL5_HPEN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);

  return ret;
}

/**
  * @brief  Interrupt generator selection path.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of int1_sel in reg CTRL_REG5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_filter_path_internal_get(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_sel_t *val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  /* Reconstruct enum: HPEN goes to bit 2, INT1_SEL stays in bits [1:0] */
  uint8_t int1_sel = FIELD_GET(I3G4250D_CTRL5_INT1_SEL_MASK, I3G4250D_CTRL5_INT1_SEL_POS, ctrl_reg5);
  uint8_t hpen = (ctrl_reg5 & I3G4250D_CTRL5_HPEN_BIT) ? 0x04U : 0x00U;
  *val = (i3g4250d_int1_sel_t)(hpen | int1_sel);

  return ret;
}

/**
  * @brief  Reference value for high-pass filter.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of ref in reg REFERENCE
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_reference_value_set(const stmdev_ctx_t *ctx,
                                        uint8_t val)
{
  int32_t ret;

  ret = i3g4250d_write_reg(ctx, I3G4250D_REFERENCE, &val, 1);

  return ret;
}

/**
  * @brief  Reference value for high-pass filter.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ref in reg REFERENCE.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_hp_reference_value_get(const stmdev_ctx_t *ctx,
                                        uint8_t *val)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_REFERENCE, val, 1);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_serial_interface
  * @brief   This section groups all the functions concerning main serial
  *          interface management.
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sim in reg CTRL_REG4
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_spi_mode_set(const stmdev_ctx_t *ctx, i3g4250d_sim_t val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_SPI_3_WIRE)
  {
    ctrl_reg4 |= I3G4250D_CTRL4_SIM_BIT;
  }
  else
  {
    ctrl_reg4 &= ~I3G4250D_CTRL4_SIM_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);

  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL_REG4.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_spi_mode_get(const stmdev_ctx_t *ctx, i3g4250d_sim_t *val)
{
  uint8_t ctrl_reg4;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG4, &ctrl_reg4, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg4 & I3G4250D_CTRL4_SIM_BIT) ? I3G4250D_SPI_3_WIRE 
                                              : I3G4250D_SPI_4_WIRE;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_interrupt_pins
  * @brief      This section groups all the functions that manage interrupt pins
  * @{
  *
  */


/**
  * @brief  Select the signal that need to route on int1 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configure CTRL_REG3 int1 pad
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_int1_route_set(const stmdev_ctx_t *ctx,
                                    i3g4250d_int1_route_t val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  /* Clear INT1 route bits and set new value */
  ctrl_reg3 &= ~(I3G4250D_INT1_ROUTE_INT1 | I3G4250D_INT1_ROUTE_BOOT);
  ctrl_reg3 |= (val & (I3G4250D_INT1_ROUTE_INT1 | I3G4250D_INT1_ROUTE_BOOT));

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);

  return ret;
}

/**
  * @brief  Select the signal that need to route on int1 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read CTRL_REG3 int1 pad.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t i3g4250d_pin_int1_route_get(const stmdev_ctx_t *ctx,
                                    i3g4250d_int1_route_t *val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  *val = ctrl_reg3 & (I3G4250D_INT1_ROUTE_INT1 | I3G4250D_INT1_ROUTE_BOOT);

  return ret;
}
/**
  * @brief  Select the signal that need to route on int2 pad.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Configure CTRL_REG3 int2 pad
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_int2_route_set(const stmdev_ctx_t *ctx,
                                    i3g4250d_int2_route_t val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  /* Clear INT2 route bits and set new value */
  ctrl_reg3 &= ~(I3G4250D_INT2_ROUTE_EMPTY | I3G4250D_INT2_ROUTE_ORUN |
                 I3G4250D_INT2_ROUTE_WTM | I3G4250D_INT2_ROUTE_DRDY);
  ctrl_reg3 |= (val & (I3G4250D_INT2_ROUTE_EMPTY | I3G4250D_INT2_ROUTE_ORUN |
                       I3G4250D_INT2_ROUTE_WTM | I3G4250D_INT2_ROUTE_DRDY));

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);

  return ret;
}

/**
  * @brief  Select the signal that need to route on int2 pad.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Read CTRL_REG3 int2 pad.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_int2_route_get(const stmdev_ctx_t *ctx,
                                    i3g4250d_int2_route_t *val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  *val = ctrl_reg3 & (I3G4250D_INT2_ROUTE_EMPTY | I3G4250D_INT2_ROUTE_ORUN |
                      I3G4250D_INT2_ROUTE_WTM | I3G4250D_INT2_ROUTE_DRDY);

  return ret;
}
/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of pp_od in reg CTRL_REG3
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t i3g4250d_pin_mode_set(const stmdev_ctx_t *ctx, i3g4250d_pp_od_t val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_OPEN_DRAIN)
  {
    ctrl_reg3 |= I3G4250D_CTRL3_PP_OD_BIT;
  }
  else
  {
    ctrl_reg3 &= ~I3G4250D_CTRL3_PP_OD_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pp_od in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_mode_get(const stmdev_ctx_t *ctx,
                              i3g4250d_pp_od_t *val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg3 & I3G4250D_CTRL3_PP_OD_BIT) ? I3G4250D_OPEN_DRAIN 
                                                : I3G4250D_PUSH_PULL;

  return ret;
}

/**
  * @brief  Pin active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of h_lactive in reg CTRL_REG3.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_polarity_set(const stmdev_ctx_t *ctx,
                                  i3g4250d_h_lactive_t val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_ACTIVE_LOW)
  {
    ctrl_reg3 |= I3G4250D_CTRL3_H_LACTIVE_BIT;
  }
  else
  {
    ctrl_reg3 &= ~I3G4250D_CTRL3_H_LACTIVE_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);

  return ret;
}

/**
  * @brief  Pin active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of h_lactive in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_pin_polarity_get(const stmdev_ctx_t *ctx,
                                  i3g4250d_h_lactive_t *val)
{
  uint8_t ctrl_reg3;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG3, &ctrl_reg3, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg3 & I3G4250D_CTRL3_H_LACTIVE_BIT) ? I3G4250D_ACTIVE_LOW 
                                                    : I3G4250D_ACTIVE_HIGH;

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of lir in reg INT1_CFG.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_notification_set(const stmdev_ctx_t *ctx,
                                      i3g4250d_lir_t val)
{
  uint8_t int1_cfg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_INT_LATCHED)
  {
    int1_cfg |= I3G4250D_INT1_CFG_LIR_BIT;
  }
  else
  {
    int1_cfg &= ~I3G4250D_INT1_CFG_LIR_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);

  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lir in reg INT1_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_notification_get(const stmdev_ctx_t *ctx,
                                      i3g4250d_lir_t *val)
{
  uint8_t int1_cfg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);
  if (ret != 0) { return ret; }

  *val = (int1_cfg & I3G4250D_INT1_CFG_LIR_BIT) ? I3G4250D_INT_LATCHED 
                                                : I3G4250D_INT_PULSED;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_ interrupt_on_threshold
  * @brief      This section groups all the functions that manage the event
  *             generation on threshold.
  * @{
  *
  */

/**
  * @brief  Configure the interrupt threshold sign.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Struct of registers INT1_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_conf_set(const stmdev_ctx_t *ctx,
                                           i3g4250d_int1_cfg_t val)
{
  int32_t ret;

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_CFG, &val, 1);

  return ret;
}

/**
  * @brief  Configure the interrupt threshold sign.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Struct of registers from INT1_CFG to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_conf_get(const stmdev_ctx_t *ctx,
                                           i3g4250d_int1_cfg_t *val)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_CFG, val, 1);

  return ret;
}
/**
  * @brief  AND/OR combination of interrupt events.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of and_or in reg INT1_CFG
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_mode_set(const stmdev_ctx_t *ctx,
                                           i3g4250d_and_or_t val)
{
  uint8_t int1_cfg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);
  if (ret != 0) { return ret; }

  if (val == I3G4250D_INT1_ON_TH_AND)
  {
    int1_cfg |= I3G4250D_INT1_CFG_AND_OR_BIT;
  }
  else
  {
    int1_cfg &= ~I3G4250D_INT1_CFG_AND_OR_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);

  return ret;
}

/**
  * @brief  AND/OR combination of interrupt events.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of and_or in reg INT1_CFG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_mode_get(const stmdev_ctx_t *ctx,
                                           i3g4250d_and_or_t *val)
{
  uint8_t int1_cfg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_CFG, &int1_cfg, 1);
  if (ret != 0) { return ret; }

  *val = (int1_cfg & I3G4250D_INT1_CFG_AND_OR_BIT) ? I3G4250D_INT1_ON_TH_AND 
                                                   : I3G4250D_INT1_ON_TH_OR;

  return ret;
}

/**
  * @brief   int_on_threshold_src: [get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Union of registers from INT1_SRC to.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_src_get(const stmdev_ctx_t *ctx,
                                          i3g4250d_int1_src_t *val)
{
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_SRC, val, 1);

  return ret;
}

/**
  * @brief  Interrupt threshold on X.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsx in reg INT1_TSH_XH
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_x_threshold_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t int1_tsh_xh;
  uint8_t int1_tsh_xl;
  int32_t ret;

  /* 15-bit threshold value: bit14-8 -> XH, bit7-0 -> XL */
  int1_tsh_xh = (uint8_t)((val >> 8) & 0x7FU);
  int1_tsh_xl = (uint8_t)(val & 0xFFU);

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_XH, &int1_tsh_xh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_XL, &int1_tsh_xl, 1);

  return ret;
}

/**
  * @brief  Interrupt threshold on X.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsx in reg INT1_TSH_XH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_x_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t int1_tsh_xh;
  uint8_t int1_tsh_xl;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_XH, &int1_tsh_xh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_XL, &int1_tsh_xl, 1);
  if (ret != 0) { return ret; }

  /* 15-bit threshold value: bit14-8 <- XH, bit7-0 <- XL */
  *val = (uint16_t)(((uint16_t)(int1_tsh_xh & 0x7FU) << 8) | int1_tsh_xl);

  return ret;
}

/**
  * @brief  Interrupt threshold on Y.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsy in reg INT1_TSH_YH
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_y_threshold_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t int1_tsh_yh;
  uint8_t int1_tsh_yl;
  int32_t ret;

  /* 15-bit threshold value: bit14-8 -> YH, bit7-0 -> YL */
  int1_tsh_yh = (uint8_t)((val >> 8) & 0x7FU);
  int1_tsh_yl = (uint8_t)(val & 0xFFU);

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_YH, &int1_tsh_yh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_YL, &int1_tsh_yl, 1);

  return ret;
}

/**
  * @brief  Interrupt threshold on Y.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsy in reg INT1_TSH_YH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_y_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t int1_tsh_yh;
  uint8_t int1_tsh_yl;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_YH, &int1_tsh_yh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_YL, &int1_tsh_yl, 1);
  if (ret != 0) { return ret; }

  /* 15-bit threshold value: bit14-8 <- YH, bit7-0 <- YL */
  *val = (uint16_t)(((uint16_t)(int1_tsh_yh & 0x7FU) << 8) | int1_tsh_yl);

  return ret;
}

/**
  * @brief  Interrupt threshold on Z.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of thsz in reg INT1_TSH_ZH.
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_z_threshold_set(const stmdev_ctx_t *ctx, uint16_t val)
{
  uint8_t int1_tsh_zh;
  uint8_t int1_tsh_zl;
  int32_t ret;

  /* 15-bit threshold value: bit14-8 -> ZH, bit7-0 -> ZL */
  int1_tsh_zh = (uint8_t)((val >> 8) & 0x7FU);
  int1_tsh_zl = (uint8_t)(val & 0xFFU);

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_ZH, &int1_tsh_zh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_TSH_ZL, &int1_tsh_zl, 1);

  return ret;
}

/**
  * @brief  Interrupt threshold on Z.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of thsz in reg INT1_TSH_ZH.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_z_threshold_get(const stmdev_ctx_t *ctx, uint16_t *val)
{
  uint8_t int1_tsh_zh;
  uint8_t int1_tsh_zl;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_ZH, &int1_tsh_zh, 1);
  if (ret != 0) { return ret; }

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_TSH_ZL, &int1_tsh_zl, 1);
  if (ret != 0) { return ret; }

  /* 15-bit threshold value: bit14-8 <- ZH, bit7-0 <- ZL */
  *val = (uint16_t)(((uint16_t)(int1_tsh_zh & 0x7FU) << 8) | int1_tsh_zl);

  return ret;
}

/**
  * @brief  Durationvalue.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of d in reg INT1_DURATION
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_dur_set(const stmdev_ctx_t *ctx,
                                          uint8_t val)
{
  uint8_t int1_duration;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_DURATION, &int1_duration, 1);
  if (ret != 0) { return ret; }

  /* Clear D field (bits 6:0) and set new value */
  int1_duration &= ~I3G4250D_INT1_DURATION_D_MASK;
  int1_duration |= FIELD_PREP(I3G4250D_INT1_DURATION_D_MASK, I3G4250D_INT1_DURATION_D_POS, val);

  /* Enable WAIT bit if duration is non-zero */
  if (val != 0U)
  {
    int1_duration |= I3G4250D_INT1_DURATION_WAIT_BIT;
  }
  else
  {
    int1_duration &= ~I3G4250D_INT1_DURATION_WAIT_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_INT1_DURATION, &int1_duration, 1);

  return ret;
}

/**
  * @brief  Durationvalue.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of d in reg INT1_DURATION.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_int_on_threshold_dur_get(const stmdev_ctx_t *ctx,
                                          uint8_t *val)
{
  uint8_t int1_duration;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_INT1_DURATION, &int1_duration, 1);
  if (ret != 0) { return ret; }

  *val = FIELD_GET(I3G4250D_INT1_DURATION_D_MASK, I3G4250D_INT1_DURATION_D_POS, int1_duration);

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_fifo
  * @brief   This section group all the functions concerning the fifo usage
  * @{
  *
  */

/**
  * @brief  FIFOenable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fifo_en in reg CTRL_REG5
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_enable_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg5 |= I3G4250D_CTRL5_FIFO_EN_BIT;
  }
  else
  {
    ctrl_reg5 &= ~I3G4250D_CTRL5_FIFO_EN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);

  return ret;
}

/**
  * @brief  FIFOenable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fifo_en in reg CTRL_REG5.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_enable_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg5;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG5, &ctrl_reg5, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg5 & I3G4250D_CTRL5_FIFO_EN_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of wtm in reg FIFO_CTRL_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_watermark_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t fifo_ctrl_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);
  if (ret != 0) { return ret; }

  /* Clear WTM field and set new value */
  fifo_ctrl_reg &= ~I3G4250D_FIFO_CTRL_WTM_MASK;
  fifo_ctrl_reg |= FIELD_PREP(I3G4250D_FIFO_CTRL_WTM_MASK, I3G4250D_FIFO_CTRL_WTM_POS, val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wtm in reg FIFO_CTRL_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_watermark_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t fifo_ctrl_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);
  if (ret != 0) { return ret; }

  *val = FIELD_GET(I3G4250D_FIFO_CTRL_WTM_MASK, I3G4250D_FIFO_CTRL_WTM_POS, fifo_ctrl_reg);

  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of fm in reg FIFO_CTRL_REG
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_mode_set(const stmdev_ctx_t *ctx,
                               i3g4250d_fifo_mode_t val)
{
  uint8_t fifo_ctrl_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);
  if (ret != 0) { return ret; }

  /* Clear FM field and set new value */
  fifo_ctrl_reg &= ~I3G4250D_FIFO_CTRL_FM_MASK;
  fifo_ctrl_reg |= FIELD_PREP(I3G4250D_FIFO_CTRL_FM_MASK, I3G4250D_FIFO_CTRL_FM_POS, (uint8_t)val);

  ret = i3g4250d_write_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fm in reg FIFO_CTRL_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_mode_get(const stmdev_ctx_t *ctx,
                               i3g4250d_fifo_mode_t *val)
{
  uint8_t fifo_ctrl_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_CTRL_REG, &fifo_ctrl_reg, 1);
  if (ret != 0) { return ret; }

  *val = (i3g4250d_fifo_mode_t)FIELD_GET(I3G4250D_FIFO_CTRL_FM_MASK, I3G4250D_FIFO_CTRL_FM_POS, fifo_ctrl_reg);

  return ret;
}

/**
  * @brief  FIFO stored data level[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fss in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_data_level_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t fifo_src_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_SRC_REG, &fifo_src_reg, 1);
  if (ret != 0) { return ret; }

  *val = FIELD_GET(I3G4250D_FIFO_SRC_FSS_MASK, I3G4250D_FIFO_SRC_FSS_POS, fifo_src_reg);

  return ret;
}

/**
  * @brief  FIFOemptybit.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of empty in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_empty_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t fifo_src_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_SRC_REG, &fifo_src_reg, 1);
  if (ret != 0) { return ret; }

  *val = (fifo_src_reg & I3G4250D_FIFO_SRC_EMPTY_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @brief  Overrun bit status.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ovrn in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_fifo_ovr_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t fifo_src_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_SRC_REG, &fifo_src_reg, 1);
  if (ret != 0) { return ret; }

  *val = (fifo_src_reg & I3G4250D_FIFO_SRC_OVRN_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @brief  Watermark status:[get]
  *                0: FIFO filling is lower than WTM level;
  *                1: FIFO filling is equal or higher than WTM level)
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of wtm in reg FIFO_SRC_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t i3g4250d_fifo_wtm_flag_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t fifo_src_reg;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_FIFO_SRC_REG, &fifo_src_reg, 1);
  if (ret != 0) { return ret; }

  *val = (fifo_src_reg & I3G4250D_FIFO_SRC_WTM_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup   I3G4250D_axis_control
  * @brief      This section groups all the functions for axis enable control.
  * @{
  *
  */

/**
  * @brief  X-axis output enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_x_data_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg1 |= I3G4250D_CTRL1_XEN_BIT;
  }
  else
  {
    ctrl_reg1 &= ~I3G4250D_CTRL1_XEN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  Y-axis output enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_y_data_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg1 |= I3G4250D_CTRL1_YEN_BIT;
  }
  else
  {
    ctrl_reg1 &= ~I3G4250D_CTRL1_YEN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  Z-axis output enable.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_z_data_set(const stmdev_ctx_t *ctx, uint8_t val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  if (val != 0U)
  {
    ctrl_reg1 |= I3G4250D_CTRL1_ZEN_BIT;
  }
  else
  {
    ctrl_reg1 &= ~I3G4250D_CTRL1_ZEN_BIT;
  }

  ret = i3g4250d_write_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);

  return ret;
}

/**
  * @brief  X-axis output enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_x_data_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg1 & I3G4250D_CTRL1_XEN_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @brief  Y-axis output enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_y_data_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg1 & I3G4250D_CTRL1_YEN_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @brief  Z-axis output enable.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    0: disabled, 1: enabled.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t i3g4250d_axis_z_data_get(const stmdev_ctx_t *ctx, uint8_t *val)
{
  uint8_t ctrl_reg1;
  int32_t ret;

  ret = i3g4250d_read_reg(ctx, I3G4250D_CTRL_REG1, &ctrl_reg1, 1);
  if (ret != 0) { return ret; }

  *val = (ctrl_reg1 & I3G4250D_CTRL1_ZEN_BIT) ? 1U : 0U;

  return ret;
}

/**
  * @}
  *
  */