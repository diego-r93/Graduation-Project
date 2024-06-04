#ifndef _AD5683_H_
#define _AD5683_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD5683_FAILURE -1
#define AD5683_CMD_NOP 0x0            // No operation.
#define AD5683_CMD_WR_IN_REG 0x1      // Write Input Register.
#define AD5683_CMD_WR_DAC_REG 0x2     // Update DAC Register.
#define AD5683_CMD_WR_DAC_IN_REG 0x3  // Write DAC and Input Register.
#define AD5683_CMD_WR_CTRL_REG 0x4    // Write Control Register.
#define AD5683_CMD_RB_IN_REG 0x5      // Readback input register.

/*************************** Write Control Register Bits **********************/

#define AD5683_DCEN(x) (((((x) & 0x1) << 0) << 10) & 0xFC00)
#define AD5683_CFG_GAIN(x) (((((x) & 0x1) << 1) << 10) & 0xFC00)
#define AD5683_REF_EN(x) (((((x) & 0x1) << 2) << 10) & 0xFC00)
#define AD5683_OP_MOD(x) (((((x) & 0x3) << 3) << 10) & 0xFC00)
#define AD5683_SW_RESET(x) (((((x) & 0x1) << 5) << 10) & 0xFC00)

/******************************************************************************/
/**************************** Variable Declarations ***************************/
/******************************************************************************/
struct ad5683_device {
   int slave_select_id;
   uint32_t dac_reg_value;
};

enum ad5683_state {
   AD5683_DC_DISABLE,
   AD5683_DC_ENABLE
};

enum ad5683_voltage_ref {
   AD5683_INT_REF_ON,
   AD5683_INT_REF_OFF
};

enum ad5683_amp_gain {
   AD5683_AMP_GAIN_1,
   AD5683_AMP_GAIN_2
};

enum ad5683_power_mode {
   AD5683_PWR_NORMAL,
   AD5683_PD_1K,
   AD5683_PD_100K,
   AD5683_PD_3STATE
};

enum ad5683_reset {
   AD5683_RESET_DISABLE,
   AD5683_RESET_ENABLE
};

/******************************************************************************/
/**************************** Function Declarations ***************************/
/******************************************************************************/

int32_t AD5683_write_cmd(struct ad5683_device *device, uint8_t cmd, uint16_t data);

int32_t AD5683_readback_register(struct ad5683_device *device, uint32_t *data);

int32_t AD5683_write_dac_value(struct ad5683_device *device, uint16_t data);

int32_t AD5683_soft_reset(struct ad5683_device *device);

int32_t AD5683_setup(struct ad5683_device *device, int slave_select_id);

#endif /* _AD5683_H_ */