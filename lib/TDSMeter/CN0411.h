#ifndef _CN0411_H_
#define _CN0411_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "AD5683.h"
#include "AD7124.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

namespace CN0411 {
const uint32_t CN0411_SUCCESS = 0;
const int32_t CN0411_FAILURE = -1;
const uint32_t CS_AD7124 = 0;
const uint32_t CS_AD5683 = 1;
const uint32_t INSTR_AMP_GAIN = 10;
const uint32_t ADC_TIMEOUT = 10000;
const uint32_t ADC_CH_DISABLE = 0;
const uint32_t ADC_CH_ENABLE = 1;
const uint32_t ADC_SETUP0 = 0;
const uint32_t ADC_SETUP1 = 1;
const uint32_t ADC_CH0 = 0;
const uint32_t ADC_CH1 = 1;
const uint32_t ADC_CH2 = 2;
const uint32_t ADC_CH3 = 3;
const uint32_t ADC_CH4 = 4;
const uint32_t ADC_CH5 = 5;
const uint32_t ADC_CONTINUOUS_CONV = 1;
const uint32_t ADC_SINGLE_CONV = 2;
const uint32_t ADC_CH_RDY_MSK = 0x8F;
const uint32_t PWM_SYSCALIB_AIN7 = 1;
const uint32_t PWM_SYSCALIB_AIN8 = 2;
const uint32_t PWM_CONVERSION = 3;
const uint32_t PWM_FREQ_94 = 94;
const uint32_t PWM_FREQ_2400 = 2400;
const uint32_t PWM_2400_STEP = 6;
#define PWM2_2400_HIGH (1 * PWM_2400_STEP)
#define PWM2_2400_LOW (2 * PWM_2400_STEP)
#define PWM0_2400_HIGH (3 * PWM_2400_STEP)
#define PWM1_2400_HIGH (4 * PWM_2400_STEP)
#define PWM1_2400_LOW (5 * PWM_2400_STEP)
#define PWM0_2400_LOW (6 * PWM_2400_STEP)
const uint32_t PWM_100_STEP = 161;
#define PWM2_100_HIGH (1 * PWM_100_STEP)
#define PWM2_100_LOW (2 * PWM_100_STEP)
#define PWM0_100_HIGH (3 * PWM_100_STEP)
#define PWM1_100_HIGH (4 * PWM_100_STEP)
#define PWM1_100_LOW (5 * PWM_100_STEP)
#define PWM0_100_LOW (6 * PWM_100_STEP)
const uint32_t PWM_CLR = 0;
const uint32_t PWM_SET = 1;

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
static inline uint32_t ADC_SET_CH(uint32_t x) { return x << 15; }

const uint32_t DAC_FS_VAL = 0xFFFF;
const uint32_t CH_GAIN_RES_20 = 0;
const uint32_t CH_GAIN_RES_200 = 1;
const uint32_t CH_GAIN_RES_2K = 2;
const uint32_t CH_GAIN_RES_20K = 3;
const uint32_t CH_GAIN_RES_200K = 4;
const uint32_t CH_GAIN_RES_2M = 5;
const uint32_t CH_GAIN_RES_20M = 6;
const uint32_t GAIN_RES_20 = 20;
const uint32_t GAIN_RES_200 = 200;
const uint32_t GAIN_RES_2K = 2000;
const uint32_t GAIN_RES_20K = 20000;
const uint32_t GAIN_RES_200K = 200000;
const uint32_t GAIN_RES_2M = 2000000;
const uint32_t GAIN_RES_20M = 20000000;
const float DAC_OUT_DEFAULT_VAL = 0.4;
const uint32_t DAC_IN_DEFAULT_VAL = 0;
const uint32_t TEMP_DEFAULT_VAL = 0;
const uint32_t VPP_DEFAULT_VAL = 0;
const uint32_t VINP_DEFAULT_VAL = 0;
const uint32_t VINN_DEFAULT_VAL = 0;
const uint32_t VR20S_DEFAULT_VAL = 0;
const uint32_t VR200S_DEFAULT_VAL = 0;
const uint32_t COND_DEFAULT_VAL = 0;
const uint32_t COMP_COND_DEFAULT_VAL = 0;
const uint32_t TDS_DEFAULT_VAL = 0;
const uint32_t EXC_DEFAULT_VAL = 0;
const uint32_t RES_GAIN_DEFAULT_CH = 6;
static inline float VREFIN() { return (0.250 * 4.02); }
const uint32_t OFFSET_RES_INIT = 0;
const uint32_t RDRES_DEFAULT_VAL = 0;
const uint32_t PREC_REF_RES = 1500;
const uint32_t RTD_REF_RES = 4020;
const uint32_t RTD_RES_100 = 100;
const uint32_t RTD_RES_1K = 1000;
const uint32_t TCAL = 25;
const float CELL_CONST_LOW = 0.1;
const uint32_t CELL_CONST_NORMAL = 1;
const uint32_t CELL_CONST_HIGH = 10;
const float TDS_KCL = 0.5;
const float TDS_NACL = 0.47;
const float TEMP_COEFF_KCL = 1.88;
const float TEMP_COEFF_NACL = 2.14;
const float VREF = 2.5;

// #define A (3.9083*pow(10,-3))
// #define B (-5.775*pow(10,-7))

}  // namespace CN0411

/******************************************************************************/
/************************** Variable Declaration ******************************/
/******************************************************************************/

struct solution {
   float temp_coeff;
   float tds_factor;
};

struct init_solution {
   float init_temp_coeff;
   float init_tds_factor;
};

struct cn0411_device {
   uint8_t ch_gain;
   uint8_t conv_type;
   uint16_t rtd_res;
   uint32_t r_gain[7];
   float offset_res;
   float v_dac;
   float read_dac;
   float read_v_r20s;
   float read_v_r200s;
   float rdres;
   float v_exc;
   float cell_const;
   float temp;
   float vpp;
   float vin_p;
   float vin_n;
   float cond;
   float comp_cond;
   float tds;
   struct solution solution;
   struct ad7124_device ad7124_dev;
   struct ad5683_device ad5683_dev;
};

struct cn0411_init_params {
   uint8_t init_ch_gain;
   uint8_t init_conv_type;
   uint16_t init_rtd_res;
   uint32_t init_r_gain[7];
   float init_offset_res;
   float init_v_dac;
   float init_read_dac;
   float init_read_v_r20s;
   float init_read_v_r200s;
   float init_rdres;
   float init_v_exc;
   float init_cell_const;
   float init_temp;
   float init_vpp;
   float init_vin_p;
   float init_vin_n;
   float init_cond;
   float init_comp_cond;
   float init_tds;
   struct init_solution init_solution;
};

typedef void (*cmd_func)(uint8_t *, struct cn0411_device *);

/******************************************************************************/
/************************** Function Declaration ******************************/
/******************************************************************************/

int32_t CN0411_DAC_set_value(struct cn0411_device *cn0411_dev,
                             float output_val);
int32_t CN0411_ADC_operation_mode(struct cn0411_device *cn0411_dev,
                                  enum op_mode mode);
int32_t CN0411_ADC_setup(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_set_ch(struct cn0411_device *cn0411_dev, uint8_t channel,
                          uint8_t ch_en);
int32_t CN0411_ADC_set_io1(struct cn0411_device *cn0411_dev, uint8_t ch_gain);
int32_t CN0411_ADC_set_io2(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_conv_init(struct cn0411_device *cn0411_dev,
                             uint8_t conv_mod);
int32_t CN0411_ADC_read_ch(struct cn0411_device *cn0411_dev, uint8_t ch);
int32_t CN0411_read_temp(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_vpp(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_vdac(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_R20S(struct cn0411_device *cn0411_dev);
int32_t CN0411_read_R200S(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_rdres(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_cond(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_off_res(struct cn0411_device *cn0411_dev);
int32_t CN0411_compensate_cond(struct cn0411_device *cn0411_dev);
int32_t CN0411_compute_tds(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_int_calibrate(struct cn0411_device *cn0411_dev);
int32_t CN0411_ADC_sys_calibrate(struct cn0411_device *cn0411_dev);
int32_t CN0411_premeasurement(struct cn0411_device *cn0411_dev);
int32_t CN0411_set_gain_res(struct cn0411_device *cn0411_dev, int8_t ch_gain);
void CN0411_pwm_freq(uint16_t freq);
void CN0411_pwm_gen(void);
int32_t CN0411_init(struct cn0411_device *cn0411_dev, struct cn0411_init_params cn0411_init_params);

void CN0411_cmd_read_dac(struct cn0411_device *cn0411_dev);

#endif /* _CN0411_H_ */