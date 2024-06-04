/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "AD5683.h"

#include <stdint.h>

#include "Communication.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * Write commands to DAC
 *
 * @param device - device structure
 * @param cmd - command value
 * @param data - data to be written
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_write_cmd(struct ad5683_device *device, uint8_t cmd, uint16_t data) {
   uint8_t buf[3];

   buf[2] = ((data << 4) & 0xF0);
   buf[1] = (data >> 4);
   buf[0] = (data >> 12) | (cmd << 4);

   return SPI_Write(device->slave_select_id, buf, 3);
}

/**
 * Readback the contents of the input register.
 *
 * @param device - device structure
 * @param data - read data to be stored
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_readback_register(struct ad5683_device *device, uint32_t *data) {
   int32_t ret;
   uint8_t buf[3] = {0, 0, 0};

   AD5683_write_cmd(device, AD5683_CMD_RB_IN_REG, 0x0000);
   ret = SPI_Read(device->slave_select_id, buf, 3);
   *data = (((buf[0] << 16) | (buf[1] << 8) | (buf[2])) & 0x0FFFF0) >> 4;

   return ret;
}

/**
 * Write value to DAC
 *
 * @param device - device structure
 * @param data - data to be written
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_write_dac_value(struct ad5683_device *ad5683_device, uint16_t data) {
   return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_DAC_IN_REG, data);
}

/**
 * Perform DAC Soft Reset
 *
 * @param device - device structure
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_soft_reset(struct ad5683_device *ad5683_device) {
   return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_CTRL_REG,
                           AD5683_SW_RESET(AD5683_RESET_ENABLE));
}

/**
 * Write command to DAC
 *
 * @param device - device structure
 * @param slave_select_id - chip select value corresponding to DAC
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t AD5683_setup(struct ad5683_device *ad5683_device, int slave_select_id) {
   ad5683_device->slave_select_id = slave_select_id;
   ad5683_device->dac_reg_value = 0;

   if (AD5683_soft_reset(ad5683_device) == AD5683_FAILURE)
      return AD5683_FAILURE;
   else
      return AD5683_write_cmd(ad5683_device, AD5683_CMD_WR_CTRL_REG,
                              AD5683_DCEN(AD5683_DC_DISABLE) | AD5683_CFG_GAIN(AD5683_AMP_GAIN_1) | AD5683_REF_EN(AD5683_INT_REF_ON) | AD5683_OP_MOD(AD5683_PWR_NORMAL) | AD5683_SW_RESET(AD5683_RESET_DISABLE));
}