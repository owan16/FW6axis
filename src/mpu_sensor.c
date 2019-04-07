/*
 * Copyright (c) 2014 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Chun Ting Ding <jiunting.d@gmail.comu>
 *
 * 22/07/2014
 */
#include "SEGGER_RTT.h"

#include <stdbool.h>
#include <stdint.h>
#include "string.h"
#include "mpu_sensor.h"
#include "mpu_sensor_err.h"
//#include "sd_twi_master.h"
#include "nrf_delay.h"
//#include "debug.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "device.h"

#include "ble_mts.h"
#include "mpu_sensor_dmp.h"

extern ble_mts_t	m_mts;

/* The following functions must be defined for this platform:*/

/*
#if defined(MPU9250)
	#define  AK89xx_SECONDARY
	#define  AK8963_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#endif
*/
#define ICM_20689 1
#define  delay_ms  										nrf_delay_ms
#define  min(a,b) 										((a<b)?a:b)

#define  LOAD_CHUNK  									(16)
extern uint8_t				m_buffer[];
extern	uint8_t	 			motion_sampling_rate;
extern	uint8_t				accel_fsr_val;	
extern	uint8_t				gyro_fsr_val;	
extern	uint16_t			int_threshold_val;	


/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_1_25HZ,
    INV_LPA_5HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ
};

#ifdef AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)
//#define AKM_REG_WHOAMI 			0x49

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#endif

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

/* Hardware registers needed by driver. */
struct mpu_reg_s {
    uint8_t who_am_i;
    uint8_t rate_div;
    uint8_t lpf;
    uint8_t prod_id;
    uint8_t user_ctrl;
    uint8_t fifo_en;
    uint8_t gyro_cfg;
    uint8_t accel_cfg;
    uint8_t accel_cfg2;
    uint8_t lp_accel_odr;
    #if ICM_20689
	uint8_t motion_thr_x;
	uint8_t motion_thr_y;
	uint8_t motion_thr_z;
    #else	
    uint8_t motion_thr;
    #endif
    uint8_t motion_dur;
    uint8_t fifo_count_h;
    uint8_t fifo_r_w;
    uint8_t raw_gyro;
    uint8_t raw_accel;
    uint8_t temp;
    uint8_t int_enable;
    uint8_t dmp_int_status;
    uint8_t int_status;
    uint8_t accel_intel;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    uint8_t int_pin_cfg;
    uint8_t mem_r_w;
    uint8_t accel_offs;
    uint8_t i2c_mst;
    uint8_t bank_sel;
    uint8_t mem_start_addr;
    uint8_t prgm_start_h;
#if defined AK89xx_SECONDARY
    uint8_t s0_addr;
    uint8_t s0_reg;
    uint8_t s0_ctrl;
    uint8_t s1_addr;
    uint8_t s1_reg;
    uint8_t s1_ctrl;
    uint8_t s4_ctrl;
    uint8_t s0_do;
    uint8_t s1_do;
    uint8_t i2c_delay_ctrl;
    uint8_t raw_compass;
    /* The I2C_MST_VDDIO bit is in this register. */
    uint8_t yg_offs_tc;
#endif
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
    uint16_t gyro_fsr;
    uint8_t accel_fsr;
    uint16_t lpf;
    uint16_t sample_rate;
    uint8_t sensors_on;
    uint8_t fifo_sensors;
    uint8_t dmp_on;
};


/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    uint8_t gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    uint8_t accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    uint8_t sensors;
    /* Matches config register. */
    uint8_t lpf;
    uint8_t clk_src;
    /* Sample rate, NOT rate divider. */
    uint16_t sample_rate;
    /* Matches fifo_en register. */
    uint8_t fifo_enable;
    /* Matches int enable register. */
    uint8_t int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    uint8_t bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    uint8_t accel_half;
    /* 1 if device in low-power accel-only mode. */
    uint8_t lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    uint8_t int_motion_only;
				
    /* 1 for active low interrupts. */
    uint8_t active_low_int;
    /* 1 for latched interrupts. */
    uint8_t latched_int;
    /* 1 if DMP is enabled. */
    uint8_t dmp_on;
    /* Ensures that DMP will only be loaded once. */
    uint8_t  dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
		
    uint16_t dmp_sample_rate;
		
		struct motion_int_cache_s cache;
#ifdef AK89xx_SECONDARY
    /* Compass sample rate. */
    uint16_t compass_sample_rate;
    uint8_t compass_addr;
    short mag_sens_adj[3];
#endif
};

/* Information specific to a particular device. */
struct hw_s {
    uint8_t addr;
    uint16_t max_fifo;
    uint8_t num_reg;
    uint16_t temp_sens;
    short temp_offset;
    uint16_t bank_size;
#if defined AK89xx_SECONDARY
    uint16_t compass_fsr;
#endif
};


/* Motion driver state variables. */
struct mpu_state_s {
    const struct mpu_reg_s *reg;
    const struct hw_s *hw;
		struct chip_cfg_s chip_cfg;
};


/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};


/* Init driver state variables. */
const struct mpu_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .accel_cfg2     = 0x1D,
		.lp_accel_odr		= 0x1E,
	#if ICM_20689
	.motion_thr_x     = 0x1F,
	.motion_thr_y     = 0x20,
	.motion_thr_z     = 0x21,
	#else
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    #endif
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70
		// AK89xx_SECONDARY
#ifdef AK89xx_SECONDARY
    ,.raw_compass   = 0x49,
    .yg_offs_tc     = 0x01,
    .s0_addr        = 0x25,
    .s0_reg         = 0x26,
    .s0_ctrl        = 0x27,
    .s1_addr        = 0x28,
    .s1_reg         = 0x29,
    .s1_ctrl        = 0x2A,
    .s4_ctrl        = 0x34,
    .s0_do          = 0x63,
    .s1_do          = 0x64,
    .i2c_delay_ctrl = 0x67
#endif
};

const struct hw_s hw = {
//#if defined(EVK_AK2) || defined(ATP1)  || defined(ATP1_NOM) || defined(ATP1_MOU) || defined(BTAG_2) || defined(BTAG_3)
//    .addr           = 0xD0,
    .addr           = 0x68,
//#else
//		.addr           = 0xD2,
//#endif
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
#if defined AK89xx_SECONDARY
    ,.compass_fsr    = AK89xx_FSR
#endif
};


static struct mpu_state_s mpu_st = {
    .reg = &reg,
    .hw = &hw
};
		static uint8_t reg_data[10];

static uint8_t twi_tx_buf[64];
ret_code_t twi_ret_code;

#ifdef AK89xx_SECONDARY
static uint8_t setup_compass(void);
#define MAX_COMPASS_SAMPLE_RATE (100)
#endif
uint8_t twi_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data)
{
	twi_tx_buf[0]=reg_addr;
	twi_ret_code=nrf_drv_twi_tx(&m_app_twi,dev_addr,twi_tx_buf,1,false);
//	if (twi_ret_code!=NRF_SUCCESS)
//	return twi_ret_code;
	return (nrf_drv_twi_rx(&m_app_twi,dev_addr,data,1));

}
uint8_t twi_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint8_t len)
{
	twi_tx_buf[0]=reg_addr;
	twi_ret_code=nrf_drv_twi_tx(&m_app_twi,dev_addr,twi_tx_buf,1,false);
//	if (twi_ret_code!=NRF_SUCCESS)
//	return twi_ret_code;
	return (nrf_drv_twi_rx(&m_app_twi,dev_addr,data,len));
}
uint8_t twi_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t tmp)
{
	twi_tx_buf[0]=reg_addr;
	twi_tx_buf[1]=tmp;
	return (nrf_drv_twi_tx(&m_app_twi,dev_addr,twi_tx_buf,2,false));

}
uint8_t twi_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t * tmp)
{
	uint8_t i;
	twi_tx_buf[0]=reg_addr;
	for (i=0;i<len;i++)
	twi_tx_buf[i+1]=*(tmp+i);
	return (nrf_drv_twi_tx(&m_app_twi,dev_addr,twi_tx_buf,len+1,false));
}
static uint8_t twi_write_stream(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t * tmp)
{
	return (nrf_drv_twi_tx(&m_app_twi,dev_addr,tmp,len,false));
}
/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
uint8_t set_int_enable(uint8_t enable)
{
		uint8_t tmp;
	uint8_t ret_code;

    if (mpu_st.chip_cfg.dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, tmp);
				if (ret_code!=0)
            return 3;
        mpu_st.chip_cfg.int_enable = tmp;
    } else {
        if (!mpu_st.chip_cfg.sensors)
            return 3;
        if (enable && mpu_st.chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, tmp);
				if (ret_code!=0)
            return 1;
        mpu_st.chip_cfg.int_enable = tmp;
    }
    return 0;
}


uint8_t mpu_sensor_fast_accel_init()
{
	uint8_t ret_code;
//		uint8_t reg_data[6];
		uint8_t data = 0x80;
		//uint8_t ret_code;
/*	
		// Reset device first.
		twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1 , data);  //6B,w,80
		delay_ms(100);
	
		// Wake up chip.
    data = 0x00;
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, data);  //6B,w,00
		if (ret_code!=0)
        return MPU_ERROR_WAKE_UP_WRITE;
		
		delay_ms(5);
*/
		mpu_st.chip_cfg.sensors=INV_XYZ_ACCEL;
		data = 0x07;		//disable Gyro X/Y/Z
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_2, data);
		if (ret_code!=0)
        return 1;
		data = 0x09;		//184HZ
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->accel_cfg2, data);
		if (ret_code!=0)
        return 2;
		data = 0x40;		//Enable interrupt for wake on motion only
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, data);
		if (ret_code!=0)
        return 3;
		data = 0xc0;		//enables the Wake-on-Motion,Compare the current sample with the previous sample.
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->accel_intel, data);
		if (ret_code!=0)
        return 4;
		data = 0x06;		//15.6HZ
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->lp_accel_odr, data);
		if (ret_code!=0)
        return 5;
    data = 0x20;	//CYCLE=1
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, data);
		if (ret_code!=0)
        return MPU_ERROR_WAKE_UP_WRITE;
    //ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, &data);
		
		return 0;
}


/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 200DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 200Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active high, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_init()
{
//		static uint8_t reg_data[10];
		uint8_t ret_code;
		
		/* Reset device first. */
		reg_data[0] = 0x80;
		ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1 , reg_data[0]);  //6B,w,80
		//SEGGER_RTT_printf(0, "Reset= %02x \r\n ", ret_code);

		delay_ms(100);

    /* Wake up chip. */
/*
    reg_data[0] = 0x00;
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, reg_data[0]);  //6B,w,00
		SEGGER_RTT_printf(0, "Wake up chip= %02x \r\n ", ret_code);
		if (ret_code!=0)
        return MPU_ERROR_WAKE_UP_WRITE;
*/
		/* Check product ID. */  
    ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->who_am_i, reg_data);
		SEGGER_RTT_printf(0, "PID= %02x \r\n ", reg_data[0]);

		#if ICM_20689
			if (reg_data[0]!=0x98)//ICM-20689
		#else
			if (reg_data[0]!=0x70)//MPU-6500
		#endif
			return 1;
		/* Set to invalid values to ensure no I2C writes are skipped. */
    mpu_st.chip_cfg.sensors = 0xFF;
    mpu_st.chip_cfg.gyro_fsr = 0xFF;
    mpu_st.chip_cfg.accel_fsr = 0xFF;
    mpu_st.chip_cfg.lpf = 0xFF;
    mpu_st.chip_cfg.sample_rate = 0xFFFF;
    mpu_st.chip_cfg.fifo_enable = 0xFF;
    mpu_st.chip_cfg.bypass_mode = 0xFF;
#ifdef AK89xx_SECONDARY
    mpu_st.chip_cfg.compass_sample_rate = 0xFFFF;
#endif
		
		/* mpu_sensor_set_sensors always preserves this setting. */
    mpu_st.chip_cfg.clk_src = INV_CLK_PLL;
    /* Handled in next call to mpu_set_bypass. */
    mpu_st.chip_cfg.active_low_int = 0;
    mpu_st.chip_cfg.latched_int = 0;
    mpu_st.chip_cfg.int_motion_only = 0;
    mpu_st.chip_cfg.lp_accel_mode = 0;
    memset(&mpu_st.chip_cfg.cache, 0, sizeof(mpu_st.chip_cfg.cache));
    mpu_st.chip_cfg.dmp_on = 0;
    mpu_st.chip_cfg.dmp_loaded = 0;
    mpu_st.chip_cfg.dmp_sample_rate = 0;
		reg_data[0]=int_threshold_val>>2;

		#if ICM_20689
			ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->motion_thr_x, reg_data[0]);
			ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->motion_thr_y, reg_data[0]);
			ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->motion_thr_z, reg_data[0]);

		#else
			ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->motion_thr, reg_data[0]);
		#endif
		
		mpu_sensor_set_gyro_fsr(gyro_fsr_val);
    mpu_sensor_set_accel_fsr(accel_fsr_val);
		set_sampling_rate(motion_sampling_rate);
    //if (mpu_sensor_set_sample_rate(20)) //19,w,13    1a,w,04
    //    return MPU_ERROR_SET_SAMP_RATE_WRITE;
    if (mpu_sensor_configure_fifo(0))  //
       return MPU_ERROR_CONF_FIFO_WRITE;
		
#ifdef AK89xx_SECONDARY
   
		ret_code = setup_compass();
		if (ret_code) {
				return ret_code;
		}
		
		ret_code = mpu_sensor_set_compass_sample_rate(10);
    if (ret_code){
			return ret_code;
		}
       
#else
    /* Already disabled by setup_compass. */
    if (mpu_sensor_set_bypass(0))
        return 0x0b;
#endif

		ret_code=mpu_sensor_set_sensors();
		if (ret_code){
			return ret_code;
		}
		
		//set up DMP

		ret_code=mpu_sensor_dmp_load_firmware();
		if (ret_code!=0)
			return 10;
		ret_code=mpu_sensor_set_dmp_state(0);			//disable DMP

		mpu_sensor_set_sleep_mode(1);
		
    return 0;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see RA_PWR_MGMT_1
 * @see PWR1_SLEEP_BIT = 6
 */
uint8_t mpu_sensor_set_sleep_mode(uint8_t enabled)
{
		//twi_write_bit(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, 6, enabled );
		twi_read_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, reg_data);
		if (enabled) {
			reg_data[0]|=0x40;
		} else {
			reg_data[0]&=0x9f;
		}
    twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, reg_data[0]);
		nrf_delay_ms(10);
		return MPU_SUCCESS;
}
uint8_t start_sensor()
{
	uint8_t ret_code;
		mpu_sensor_set_sleep_mode(0);
		ret_code=mpu_sensor_set_sensors();
		SEGGER_RTT_printf(0, "start_sensor=%x\r\n",ret_code);
    if (ret_code){
			return ret_code;
		}
		
		mpu_sensor_set_gyro_fsr(gyro_fsr_val);
    mpu_sensor_set_accel_fsr(accel_fsr_val);
    set_sampling_rate(motion_sampling_rate); 
//    if (mpu_sensor_configure_fifo(0))
//       return MPU_ERROR_CONF_FIFO_WRITE;
				
		nrf_delay_ms(50);
		return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 *
 *	@detial			0=>250,   1=>500,    2=>1000,     3=>2000
 */
uint8_t mpu_sensor_set_gyro_fsr(uint8_t fsr)
{
    uint8_t data;
	uint8_t ret_code;

    if (!(mpu_st.chip_cfg.sensors))
        return 1;

    switch (fsr) {
    case 0:
        data = INV_FSR_250DPS << 3;
        break;
    case 1:
        data = INV_FSR_500DPS << 3;
        break;
    case 2:
        data = INV_FSR_1000DPS << 3;
        break;
    case 3:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        data = INV_FSR_250DPS << 3;
        //data = INV_FSR_2000DPS << 3;
    }
		
		// check the value at gyro fsr register
    if (mpu_st.chip_cfg.gyro_fsr == (data >> 3))
        return 0;
		
			ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->gyro_cfg, data);   //default, 1B,w,18
			if (ret_code!=0)
        return 1;
		
    mpu_st.chip_cfg.gyro_fsr = data >> 3;
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 *
 *  @detial			0=>2g,   1=>4g,    2=>8g,     3=>16g
 */
uint8_t mpu_sensor_set_accel_fsr(uint8_t fsr)
{
    uint8_t data;
	uint8_t ret_code;

    if (!(mpu_st.chip_cfg.sensors))
        return 1;

    switch (fsr) {
    case 0:
        data = INV_FSR_2G << 3;
        break;
    case 1:
        data = INV_FSR_4G << 3;
        break;
    case 2:
        data = INV_FSR_8G << 3;
        break;
    case 3:
        data = INV_FSR_16G << 3;
        break;
    default:
        data = INV_FSR_2G << 3;
    }

    if (mpu_st.chip_cfg.accel_fsr == (data >> 3))
        return 0;
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->accel_cfg, data);
		if (ret_code!=0)
        return 1;
    mpu_st.chip_cfg.accel_fsr = data >> 3;
    return 0;
}


/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 98, 42, 20, 10
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_lpf(uint16_t lpf)
{
    uint8_t data;
	uint8_t ret_code;
	

    if (!(mpu_st.chip_cfg.sensors))
        return 1;

    if (lpf >= 98)
        data = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data = INV_FILTER_20HZ;
    else  
        data = INV_FILTER_10HZ;

    if (mpu_st.chip_cfg.lpf == data)
        return 0;
		
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->lpf, data);
		if (ret_code!=0)		
        return 1;
		
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->accel_cfg2, data);    
		mpu_st.chip_cfg.lpf = data;
    return 0;
}


/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 1Hz and 50Hz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_sample_rate(uint16_t rate)
{
    uint8_t data;
	uint8_t ret_code;

    if (!(mpu_st.chip_cfg.sensors))
        return 1;

    //if (mpu_st.chip_cfg.dmp_on)
    //    return 1;
    
        if (mpu_st.chip_cfg.lp_accel_mode) {
            if (rate && (rate <= 40)) {
                /* Just stay in low-power accel mode. */
                mpu_sensor_set_lp_accel_mode(rate);
                return 0;
            }
            /* Requested rate exceeds the allowed frequencies in LP accel mode,
             * switch back to full-power mode.
             */
            mpu_sensor_set_lp_accel_mode(0);
        }
				/*
        if (rate < 4)
            rate = 4;
        else if (rate > 1000)
            rate = 1000;
				*/
        data = 1000 / rate - 1;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->rate_div, data);
				if (ret_code!=0)
           return 2;

        mpu_st.chip_cfg.sample_rate = 1000 / (1 + data);


        /* Automatically set LPF to 1/2 sampling rate. */
        mpu_sensor_set_lpf(mpu_st.chip_cfg.sample_rate >> 1);
        return 0;
    
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_configure_fifo(uint8_t sensors)
{
    uint8_t prev;
    int result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    if (mpu_st.chip_cfg.dmp_on)
        return 0;
    else {
        if (!(mpu_st.chip_cfg.sensors))
            return 1;
        prev = mpu_st.chip_cfg.fifo_enable;
        mpu_st.chip_cfg.fifo_enable = sensors & mpu_st.chip_cfg.sensors;
        if (mpu_st.chip_cfg.fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = 1;
        else
            result = 0;
        if (sensors || mpu_st.chip_cfg.lp_accel_mode)
            set_int_enable(1);
        else
            set_int_enable(0);
        if (sensors) {
            if (mpu_sensor_reset_fifo()) {
                mpu_st.chip_cfg.fifo_enable = prev;
                return 1;
            }
        }
    }
    return result;
}

uint8_t mpu_sensor_get_fifo_count(uint16_t *count)
{
		uint8_t data[2];
		uint16_t fifo_count;
	uint8_t ret_code;
		
		ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_count_h, data, 2);
 		if (ret_code!=0)
       return 33;
		fifo_count = data[0];
		fifo_count = fifo_count << 8;
		fifo_count |= data[1];
    //fifo_count = (data[0] << 8) | data[1];
		*count = fifo_count;
		return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_fifo_config(uint8_t *sensors)
{
    sensors[0] = mpu_st.chip_cfg.fifo_enable;
    return 0;
}

uint8_t mpu_sensor_set_fifo_clear(void)
{
		uint8_t data = 0;
	uint8_t ret_code;
		 
	  // disable interrupt
	
		// disable the sensor output to FIFO
	
	
		ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
		if (ret_code!=0)
			return 9;
			
	  data = 4;
		ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
		if (ret_code!=0)
			return 9;
		
		data = 64;
		ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
		if (ret_code!=0)
			return 9;
		
		twi_write_byte(mpu_st.hw->addr, 0x23, 8);
		
		return 0;
}


/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
uint8_t mpu_sensor_reset_fifo(void)
{
    uint8_t data;
	uint8_t ret_code;

    if (!(mpu_st.chip_cfg.sensors))
        return 1;

    data = 0;
		
		// disable interrupt
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, data);
		if (ret_code!=0)
        return 2;
		
		// disable the sensor output to FIFO
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, data);
		if (ret_code!=0)
        return 3;
		
		// disable fifo reading
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
		if (ret_code!=0)
        return 4;

    if (mpu_st.chip_cfg.dmp_on) {
        data = BIT_FIFO_RST | BIT_DMP_RST;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
				if (ret_code!=0)
           return 5;
				
        delay_ms(50);
				
        data = BIT_DMP_EN | BIT_FIFO_EN;
        //if (mpu_st.chip_cfg.sensors & INV_XYZ_COMPASS)
        //    data |= BIT_AUX_IF_EN;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
				if (ret_code!=0)
           return 6;
        if (mpu_st.chip_cfg.int_enable)
            data = BIT_DMP_INT_EN;
            //data = BIT_DATA_RDY_EN;
        else
            data = 0;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, data);
				if (ret_code!=0)
            return 7;
        data = 0;
        //data = 0x79;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, data);
				if (ret_code!=0)
            return 8;
    } else {
        // reset FIFO and possibly reset I2C
			  data = BIT_FIFO_RST;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
				if (ret_code!=0)
           return 9;
				
				// mdelay(POWER_UP_TIME);
				delay_ms(50);
				
				if (mpu_st.chip_cfg.fifo_enable) {
					  data = BIT_DATA_RDY_EN;
					  twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, data);
				}
				
				// enable FIFO reading and I2C master interface
				data = BIT_FIFO_EN;
				ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
				if (ret_code!=0)
            return 11;
				
				data = 0x08;
				ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, data);
				if (ret_code!=0)
            return 12;
				
				// enable sensor output to FIFO
				
				
        if (mpu_st.chip_cfg.bypass_mode || !(mpu_st.chip_cfg.sensors & INV_XYZ_COMPASS))
            data = BIT_FIFO_EN;
        else
            data = BIT_FIFO_EN | BIT_AUX_IF_EN;
				
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, data);
				if (ret_code!=0)
            return 10;
			
        delay_ms(50);
				
        if (mpu_st.chip_cfg.int_enable)
            data = BIT_DATA_RDY_EN;
        else
            data = 0;
				
				//data = BIT_DATA_RDY_EN;
        
				ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, data);
				if (ret_code!=0)
            return 11;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, mpu_st.chip_cfg.fifo_enable);
				if (ret_code!=0)
            return 12;
				
				
				
    }
    return 0;
}


/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following parameters:
 *  \n accel_fsr_val
 *  \n gyro_fsr_val
 *  \n no_mag
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_sensors()
{
    uint8_t data;
		uint8_t ret_code;
	uint8_t sensors;
#ifdef AK89xx_SECONDARY
    uint8_t user_ctrl;
#endif
		sensors=0;
			sensors|=INV_XYZ_ACCEL;
			sensors|=INV_XYZ_GYRO;
    if (sensors & INV_XYZ_GYRO)
        data = INV_CLK_PLL;
    else if (sensors)
        data = 0;
    else
        data = BIT_SLEEP;
		
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, data);
		if (ret_code!=0)
		{
        mpu_st.chip_cfg.sensors = 0;
        return 1;
    }
		
    mpu_st.chip_cfg.clk_src = data & ~BIT_SLEEP;

    data = 0;
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA;
		
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_2, data);
		if (ret_code!=0)
		{
        mpu_st.chip_cfg.sensors = 0;
        return 1;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL))
        /* Latched interrupts only used in LP accel mode. */
        mpu_sensor_set_int_latched(0);

#ifdef AK89xx_SECONDARY
#ifdef AK89xx_BYPASS
    if (sensors & INV_XYZ_COMPASS)
        mpu_set_bypass(1);
    else
        mpu_set_bypass(0);
#else
    ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, &user_ctrl);
		if (ret_code!=0)
		{
			   return 1;
		}
     
    /* Handle AKM power management. */
    if (sensors & INV_XYZ_COMPASS) {
        data = AKM_SINGLE_MEASUREMENT;
        user_ctrl |= BIT_AUX_IF_EN;
    } else {
        data = AKM_POWER_DOWN;
        user_ctrl &= ~BIT_AUX_IF_EN;
    }
    if (mpu_st.chip_cfg.dmp_on)
        user_ctrl |= BIT_DMP_EN;
    else
        user_ctrl &= ~BIT_DMP_EN;
		
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s1_do, data);
		if (ret_code!=0)
		{
				return 1;
		}
        
    /* Enable/disable I2C master mode. */
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, user_ctrl);
		if (ret_code!=0)
		{
			return 1;
		}
        
#endif
#endif

    mpu_st.chip_cfg.sensors = sensors;
    mpu_st.chip_cfg.lp_accel_mode = 0;
		
    //delay_ms(50);
		
    return MPU_SUCCESS;
}

/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_compass_sample_rate(uint16_t rate)
{
#ifdef AK89xx_SECONDARY
    uint8_t div;
		uint8_t ret_code;
	
    if (!rate || rate > mpu_st.chip_cfg.sample_rate || rate > MAX_COMPASS_SAMPLE_RATE)
        return (MPU_ERROR_SET_COMPASS_SAMP_RATE + 0);

    div = mpu_st.chip_cfg.sample_rate / rate - 1;
		if (div>1)
			div=div>>1;
		//if (div>0x31) div=0x31;

    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s4_ctrl, div);
				//SEGGER_RTT_printf(0, "compass_sample_rate=%04x %02x \r\n ",mpu_st.chip_cfg.sample_rate, div);
		if (ret_code!=0)
        return (MPU_ERROR_SET_COMPASS_SAMP_RATE + 1);
		
    mpu_st.chip_cfg.compass_sample_rate = mpu_st.chip_cfg.sample_rate / (div + 1);
		
    return MPU_SUCCESS;
#else
    return MPU_ERROR_NOT_DEFINE;
#endif
}


/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_bypass(uint8_t bypass_on)
{
    uint8_t tmp;
		uint8_t ret_code;

    if (mpu_st.chip_cfg.bypass_mode == bypass_on)
        return 0;
		
		// bypass mode is working, disable it.
    if (bypass_on) {
        ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, &tmp);
				if (ret_code!=0)
						return 1;
        tmp &= ~BIT_AUX_IF_EN;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, tmp);
				if (ret_code!=0)
            return 1;				
        delay_ms(3);
				
        tmp = BIT_BYPASS_EN;
        if (mpu_st.chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (mpu_st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
				ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_pin_cfg, tmp);
				if (ret_code!=0)
            return 1;

    } else {
        /* Enable I2C master mode if compass is being used. */
        ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, &tmp);
					if (ret_code!=0)
           return 1;
        if (mpu_st.chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, tmp);
				if (ret_code!=0)
            return 1;
				
        delay_ms(3);
				
        if (mpu_st.chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (mpu_st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_pin_cfg, tmp);
				if (ret_code!=0)
            return 1;
    }
    mpu_st.chip_cfg.bypass_mode = bypass_on;
    return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_int_latched(uint8_t enable)
{
    uint8_t tmp;
	uint8_t ret_code;
    if (mpu_st.chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (mpu_st.chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (mpu_st.chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_pin_cfg, tmp);
		if (ret_code!=0)
        return 1;
    mpu_st.chip_cfg.latched_int = enable;
    return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6500: 10Hz, 20Hz, 40Hz, 92Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_lp_accel_mode(uint8_t rate)
{
    uint8_t tmp[2];
	uint8_t ret_code;

    if (rate > 40)
        return 1;

    if (!rate) {
        mpu_sensor_set_int_latched(0);
        tmp[0] = 0;
        tmp[1] = BIT_STBY_XYZG;
        ret_code=twi_write_bytes(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, 2, tmp);
				if (ret_code!=0)
            return 3;
				
        mpu_st.chip_cfg.lp_accel_mode = 0;
        return 0;
    }
    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    mpu_sensor_set_int_latched(1);

    tmp[0] = BIT_LPA_CYCLE;
    if (rate == 1) {
        tmp[1] = INV_LPA_1_25HZ;
        mpu_sensor_set_lpf(5);
    } else if (rate <= 5) {
        tmp[1] = INV_LPA_5HZ;
        mpu_sensor_set_lpf(5);
    } else if (rate <= 20) {
        tmp[1] = INV_LPA_20HZ;
        mpu_sensor_set_lpf(10);
    } else {
        tmp[1] = INV_LPA_40HZ;
        mpu_sensor_set_lpf(20);
    }
    tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
    ret_code=twi_write_bytes(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, 2, tmp);
		if (ret_code!=0)
        return 1;

    mpu_st.chip_cfg.sensors = INV_XYZ_ACCEL;
    mpu_st.chip_cfg.clk_src = 0;
    mpu_st.chip_cfg.lp_accel_mode = 1;
   // mpu_sensor_configure_fifo(0);

    return 0;
}


/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_dmp_state(uint8_t enable)
{
    uint8_t tmp,ret_code;
//    if (mpu_st.chip_cfg.dmp_on == enable)
//        return 0;

    if (enable) {
        if (!mpu_st.chip_cfg.dmp_loaded)
            return 1;
        /* Disable data ready interrupt. */
        set_int_enable(0);
        /* Disable bypass mode. */
        mpu_sensor_set_bypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
				ret_code=mpu_sensor_dmp_set_fifo_rate(motion_sampling_rate);	
				//ret_code=mpu_sensor_dmp_set_fifo_rate(0);	
        //mpu_sensor_set_sample_rate(mpu_st.chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        tmp = 0;
        twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, tmp);
        mpu_st.chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        set_int_enable(1);
        ret_code=mpu_sensor_reset_fifo();
				nrf_delay_ms(50);
    } else {
        /* Disable DMP interrupt. */
        set_int_enable(0);
        /* Restore FIFO settings. */
        tmp = mpu_st.chip_cfg.fifo_enable;
        twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, tmp);
        mpu_st.chip_cfg.dmp_on = 0;
        mpu_sensor_reset_fifo();
    }
    return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_dmp_state(uint8_t *enabled)
{
    enabled[0] = mpu_st.chip_cfg.dmp_on;
    return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_gyro_fsr(uint16_t *fsr)
{
    switch (mpu_st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        fsr[0] = 250;
        break;
    case INV_FSR_500DPS:
        fsr[0] = 500;
        break;
    case INV_FSR_1000DPS:
        fsr[0] = 1000;
        break;
    case INV_FSR_2000DPS:
        fsr[0] = 2000;
        break;
    default:
        //fsr[0] = 0;
        fsr[0] = 250;
        break;
    }
    return 0;
}


/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_accel_fsr(uint8_t *fsr)
{

    switch (mpu_st.chip_cfg.accel_fsr) {
    case INV_FSR_2G:
        fsr[0] = 2;
        break;
    case INV_FSR_4G:
        fsr[0] = 4;
        break;
    case INV_FSR_8G:
        fsr[0] = 8;
        break;
    case INV_FSR_16G:
        fsr[0] = 16;
        break;
    default:
        //return 1;
        fsr[0] = 2;
				break;
    }
    if (mpu_st.chip_cfg.accel_half)
        fsr[0] <<= 1;
    return 0;
}


/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
uint8_t mpu_sensor_get_lpf(uint16_t *lpf)
{
    switch (mpu_st.chip_cfg.lpf) {
    case INV_FILTER_98HZ:
        lpf[0] = 98;
        break;
    case INV_FILTER_42HZ:
        lpf[0] = 42;
        break;
    case INV_FILTER_20HZ:
        lpf[0] = 20;
        break;
    case INV_FILTER_10HZ:
        lpf[0] = 10;
        break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_sample_rate(uint16_t *rate)
{
    if (mpu_st.chip_cfg.dmp_on)
        return 1;
    else
        rate[0] = mpu_st.chip_cfg.sample_rate;
    return 0;
}


/**
 *  @brief      Read raw data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_raw_data(uint8_t *data)
{
	uint8_t ret_code;
    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->raw_accel, &data[0], 6);
		if (ret_code!=0)
        return 1;
		ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->raw_gyro, &data[6], 6);
		if (ret_code!=0)
        return 1;
		
		ret_code=mpu_sensor_get_compass_reg(&data[12]);
		if (ret_code!=0)
        return 1;
		
    return 0;
}

/**
 *  @brief      Read raw data directly from the FIFO registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_fifo_reg()
{
	uint8_t ret_code;
    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_r_w, m_buffer, 18);
		if (ret_code!=0)
        return 3;
		return 0;
}
/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_accel_reg(uint8_t *data)
{
   // uint8_t tmp[6];
	uint8_t ret_code;

	
//Owan close it.

    if (!(mpu_st.chip_cfg.sensors & INV_XYZ_ACCEL)){
		SEGGER_RTT_printf(0, "mpu_st.chip_cfg.sensors\r\n");
        return 9;
    	}
    	

    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->raw_accel, data, 6);
		if (ret_code!=0){
        return 3;
			}
/*		
    data[0] = tmp[0];
    data[1] = tmp[1];
    data[2] = tmp[2];
		data[3] = tmp[3];
		data[4] = tmp[4];
		data[5] = tmp[5];
*/
#if 0
SEGGER_RTT_printf(0, "mpu_sensor_get_accel_reg start\r\n");
uint16_t i;
			for (i=0;i<6;i++)
				SEGGER_RTT_printf(0, "%02x  ",data[i]);
			SEGGER_RTT_printf(0, "\r\n");
SEGGER_RTT_printf(0, "mpu_sensor_get_accel_reg End\r\n");
#endif
    return 0;
}



/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_gyro_reg(uint8_t *data)
{
	uint8_t ret_code;
    if (!(mpu_st.chip_cfg.sensors & INV_XYZ_GYRO))
        return 1;

    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->raw_gyro, data, 6);
		if (ret_code!=0)
        return 3;
    
    return 0;
}


/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_get_compass_reg(uint8_t *data)
{
#ifdef AK89xx_SECONDARY
		short sData[3];
    uint8_t tmp[9];
	uint8_t ret_code;

    if (!(mpu_st.chip_cfg.sensors & INV_XYZ_COMPASS))
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 0);

#ifdef AK89xx_BYPASS
    ret_code=twi_read_bytes(st.chip_cfg.compass_addr, AKM_REG_ST1, tmp, 8);
		if (ret_code!=0)
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 1);
    tmp[8] = AKM_SINGLE_MEASUREMENT;
    ret_code=twi_read_byte(mpu_st.chip_cfg.compass_addr, AKM_REG_CNTL, tmp+8,1);
		if (ret_code!=0)
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 1);
#else
    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->raw_compass, tmp, 8);
		//		SEGGER_RTT_printf(0, "%02x  ", tmp[0]);
		if (ret_code!=0)
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 1);
#endif

#if defined AK8975_SECONDARY
    /* AK8975 doesn't have the overrun error bit. */
    if (!(tmp[0] & AKM_DATA_READY))
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 2);
    if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 3);
#elif defined AK8963_SECONDARY
    /* AK8963 doesn't have the data read error bit. */
    if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 4);
    if (tmp[7] & AKM_OVERFLOW)
        return (MPU_ERROR_GET_COMPASS_REG_DATA + 5);
#endif
    sData[0] = (tmp[2] << 8) | tmp[1];
    sData[1] = (tmp[4] << 8) | tmp[3];
    sData[2] = (tmp[6] << 8) | tmp[5];

    sData[0] = ((long)sData[0] * mpu_st.chip_cfg.mag_sens_adj[0]) >> 8;
    sData[1] = ((long)sData[1] * mpu_st.chip_cfg.mag_sens_adj[1]) >> 8;
    sData[2] = ((long)sData[2] * mpu_st.chip_cfg.mag_sens_adj[2]) >> 8;
		
		data[0] = (uint8_t)(sData[0] >> 8);
		data[1] = (uint8_t)(sData[0] & 0x00ff);
		data[2] = (uint8_t)(sData[1] >> 8);
		data[3] = (uint8_t)(sData[1] & 0x00ff);
		data[4] = (uint8_t)(sData[2] >> 8);
		data[5] = (uint8_t)(sData[2] & 0x00ff);

    return 0;
#else
    return MPU_ERROR_NOT_DEFINE;
#endif
}


#ifdef AK89xx_SECONDARY
/* This initialization is similar to the one in ak8975.c. */
static uint8_t setup_compass(void)
{
#ifdef AK89xx_SECONDARY
    uint8_t data[4], akm_addr, ret_code;

    ret_code = mpu_sensor_set_bypass(1);
		if (ret_code!=0)
			return MPU_ERROR_SET_BYPASS_MODE;

		//akm_addr = (0x0C)<<1;
		akm_addr = (0x0C);
		//akm_addr = (0x68);
		ret_code = twi_read_byte(akm_addr, AKM_REG_WHOAMI, data);
    if (data[0] != AKM_WHOAMI)
				return MPU_ERROR_SETUP_COMPASS;

    /* We are configuring MPU9250 to populate data of its friend AK8975 into EXT_SENS_DATA_00. 
     * As MPU9250 talks to his friend using 7-bit address, we are shifting back 1-bit right.
     */
    //mpu_st.chip_cfg.compass_addr = akm_addr>>1;
    mpu_st.chip_cfg.compass_addr = akm_addr;
		
    data[0] = AKM_POWER_DOWN;
    ret_code = twi_write_byte( akm_addr, AKM_REG_CNTL, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 1);
		
    delay_ms(1);

    data[0] = AKM_FUSE_ROM_ACCESS;
    ret_code = twi_write_byte(akm_addr, AKM_REG_CNTL, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 2);
		
    delay_ms(1);

    /* Get sensitivity adjustment data from fuse ROM. */
    ret_code = twi_read_bytes(akm_addr, AKM_REG_ASAX, data, 3);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 3);
		
    mpu_st.chip_cfg.mag_sens_adj[0] = (long)data[0] + 128;
    mpu_st.chip_cfg.mag_sens_adj[1] = (long)data[1] + 128;
    mpu_st.chip_cfg.mag_sens_adj[2] = (long)data[2] + 128;

    data[0] = AKM_POWER_DOWN;
    ret_code = twi_write_byte( akm_addr, AKM_REG_CNTL, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 4);
		
    delay_ms(1);

		// disable bypass mode
    ret_code = mpu_sensor_set_bypass(0);
		if (ret_code!=0)
			return MPU_ERROR_SET_BYPASS_MODE;

		
    /* Set up master mode, master clock, and ES bit. */
    data[0] = 0x40;
    ret_code = twi_write_byte(mpu_st.hw->addr , mpu_st.reg->i2c_mst, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 5);

    /* Slave 0 reads from AKM data registers. */
    data[0] = BIT_I2C_READ | mpu_st.chip_cfg.compass_addr;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s0_addr, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 6);

    /* Compass reads start at this register. */
    data[0] = AKM_REG_ST1;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s0_reg, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 7);

    /* Enable slave 0, 8-byte reads. */
    data[0] = BIT_SLAVE_EN | 8;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s0_ctrl, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 8);

    /* Slave 1 changes AKM measurement mode. */
    data[0] = mpu_st.chip_cfg.compass_addr;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s1_addr, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 9);

    /* AKM measurement mode register. */
    data[0] = AKM_REG_CNTL;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s1_reg, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 10);

    /* Enable slave 1, 1-byte writes. */
    data[0] = BIT_SLAVE_EN | 1;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s1_ctrl, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 11);

    /* Set slave 1 data. */
    data[0] = AKM_SINGLE_MEASUREMENT;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->s1_do, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 12);

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data[0] = 0x03;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->i2c_delay_ctrl, data[0]);
		if (ret_code!=0)
			return (MPU_ERROR_SETUP_COMPASS + 13);

#ifdef MPU9250
    /* For the MPU9250, the auxiliary I2C bus needs to be set to VDD. */
/*
    data[0] = BIT_I2C_MST_VDDIO;
    ret_code = twi_write_byte(mpu_st.hw->addr, mpu_st.reg->yg_offs_tc, data[0]);
		if (ret_code!=0)
        return (MPU_ERROR_SETUP_COMPASS + 14);
*/
#endif

    return 0;
#else
    return MPU_ERROR_NOT_DEFINE;  
#endif
}
#endif


/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_read_reg(uint8_t reg, uint8_t *data)
{
  /*  if (reg == mpu_st.reg->fifo_r_w || reg == mpu_st.reg->mem_r_w)
        return 5;
    if (reg >= mpu_st.hw->num_reg)
        return 6;
  */  
		return ~twi_read_byte(mpu_st.hw->addr, reg, data);
}


/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
uint8_t mpu_sensor_read_fifo_stream(uint16_t length, uint8_t *data,
    uint8_t *more)
{
    uint8_t ret_code,tmp[2];
    uint16_t fifo_count;
    //if (!mpu_st.chip_cfg.dmp_on)
    //    return 1;
    if (!mpu_st.chip_cfg.sensors)
        return 2;

    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_count_h, tmp, 2);
		if (ret_code!=0)
       return 3;
		
    fifo_count = (tmp[0] << 8) | tmp[1];
//SEGGER_RTT_printf(0, "%02x \r", fifo_count);
		if (fifo_count < length) {
        more[0] = 0;
        return 4;
    }
    if (fifo_count > (mpu_st.hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        ret_code=twi_read_byte(mpu_st.hw->addr, mpu_st.reg->int_status, tmp);
				if (ret_code!=0)
            return 5;
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu_sensor_reset_fifo();
            return 6;
        }
    }

    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_r_w, data, length);
		if (ret_code!=0)
       return 7;
		
    more[0] = fifo_count / length - 1;
    return 0;
}

/**
 *  @brief  		Set bank address
 *  This function prevents I2C writes the bank address. 
 *	@param[in]  memory_addr    Memory location of bank
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_memory_bank(uint8_t memory_bank)
{
	uint8_t ret_code;
	  ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->bank_sel, memory_bank);
		if (ret_code!=0)
				return 1;
		return 0;
}

/**
 *  @brief  		Set memory start address
 *  This function prevents I2C writes the start memory address. 
 *	@param[in]  memory_addr    Memory location of start address
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_set_memory_address(uint8_t memory_addr)
{
	uint8_t ret_code;
	  ret_code=twi_write_byte(mpu_st.hw->addr, mpu_st.reg->mem_start_addr, memory_addr);
		if (ret_code!=0)
				return 1;
		return 0;
}


/**
 *  @brief      Write to the DMP memory. 
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake. 
 *  It is important that the data must be sended just only one stop signal in TWI protocol.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_write_mem(uint16_t mem_addr, uint8_t length, uint8_t *data)
{
    uint8_t tmp[LOAD_CHUNK + 1]; 			// register + chunk
		uint8_t j;
	uint8_t ret_code;
	
    if (!data)
        return (MPU_ERROR_WRITE_MEMORY_BASE +0);
//    if (!mpu_st.chip_cfg.sensors)
//        return (MPU_ERROR_WRITE_MEMORY_BASE +1);

		mpu_sensor_set_memory_bank((mem_addr >> 8));

		mpu_sensor_set_memory_address( (mem_addr & 0xFF));
		
    /* Check bank boundaries. */
    if ( (mem_addr & 0xFF) + length > mpu_st.hw->bank_size)
        return (MPU_ERROR_WRITE_MEMORY_BASE +2);

		tmp[0] = mpu_st.reg->mem_r_w;
		for (j = 1; j <= length; j++)
        	tmp[j] = data[j - 1];
		
		// invoke stream call.(** important)
		ret_code=twi_write_stream(mpu_st.hw->addr, mpu_st.reg->mem_r_w, length + 1, tmp);
		if (ret_code!=0)
			return (MPU_ERROR_WRITE_MEMORY_BASE +4);

    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_read_mem(uint16_t mem_addr, uint8_t length, uint8_t *data)
{
	uint8_t ret_code;
    if (!data)
        return 1;
//    if (!mpu_st.chip_cfg.sensors)
//        return 2;

		mpu_sensor_set_memory_bank((mem_addr >> 8));
		
		mpu_sensor_set_memory_address( (mem_addr & 0xFF));

    /* Check bank boundaries. */
    if ((mem_addr & 0xFF) + length > mpu_st.hw->bank_size)
        return 61;
		
    ret_code=twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->mem_r_w, data, length);
		if (ret_code!=0)
        return 63;
    return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
uint8_t mpu_sensor_load_firmware(uint16_t length, const uint8_t *firmware,
    uint16_t start_addr, uint16_t sample_rate)
{
    uint16_t ii;
    uint8_t this_write;
		uint8_t ret_code;
		
    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
    uint8_t cur[LOAD_CHUNK], tmp[2];

//    if (mpu_st.chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
//        return (MPU_ERROR_DMP_LOAD_FIRMWARE_BASE + 0);

    if (!firmware)
        return (MPU_ERROR_DMP_LOAD_FIRMWARE_BASE + 1);
		
    for (ii = 0, this_write = 0 ; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
			
				ret_code = mpu_sensor_write_mem(ii, this_write, (uint8_t*)&firmware[ii]);
        if ( ret_code )
            return ret_code;
				ret_code = mpu_sensor_read_mem(ii, this_write, cur);
        if (ret_code)
            return ret_code;
				ret_code = memcmp(firmware+ii, cur, this_write);
        if (ret_code)
            return ret_code;
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    ret_code=twi_write_bytes(mpu_st.hw->addr, mpu_st.reg->prgm_start_h, 2, tmp);
		if (ret_code!=0)
        return (MPU_ERROR_DMP_LOAD_FIRMWARE_BASE + 5);;

    mpu_st.chip_cfg.dmp_loaded = 1;
    mpu_st.chip_cfg.dmp_sample_rate = sample_rate;
		
		
    return 0;
}
//
#define MAX_PACKET_LENGTH (12)
#define HWST_MAX_PACKET_LENGTH (512)
#define REG_6500_XG_ST_DATA     0x0
#define REG_6500_XA_ST_DATA     0xD
#define fabs(x)     (((x)>0)?(x):-(x))
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
    float max_g_offset;
static const unsigned short mpu_6500_st_tb[256] = {
	2620,2646,2672,2699,2726,2753,2781,2808, //7
	2837,2865,2894,2923,2952,2981,3011,3041, //15
	3072,3102,3133,3165,3196,3228,3261,3293, //23
	3326,3359,3393,3427,3461,3496,3531,3566, //31
	3602,3638,3674,3711,3748,3786,3823,3862, //39
	3900,3939,3979,4019,4059,4099,4140,4182, //47
	4224,4266,4308,4352,4395,4439,4483,4528, //55
	4574,4619,4665,4712,4759,4807,4855,4903, //63
	4953,5002,5052,5103,5154,5205,5257,5310, //71
	5363,5417,5471,5525,5581,5636,5693,5750, //79
	5807,5865,5924,5983,6043,6104,6165,6226, //87
	6289,6351,6415,6479,6544,6609,6675,6742, //95
	6810,6878,6946,7016,7086,7157,7229,7301, //103
	7374,7448,7522,7597,7673,7750,7828,7906, //111
	7985,8065,8145,8227,8309,8392,8476,8561, //119
	8647,8733,8820,8909,8998,9088,9178,9270,
	9363,9457,9551,9647,9743,9841,9939,10038,
	10139,10240,10343,10446,10550,10656,10763,10870,
	10979,11089,11200,11312,11425,11539,11654,11771,
	11889,12008,12128,12249,12371,12495,12620,12746,
	12874,13002,13132,13264,13396,13530,13666,13802,
	13940,14080,14221,14363,14506,14652,14798,14946,
	15096,15247,15399,15553,15709,15866,16024,16184,
	16346,16510,16675,16842,17010,17180,17352,17526,
	17701,17878,18057,18237,18420,18604,18790,18978,
	19167,19359,19553,19748,19946,20145,20347,20550,
	20756,20963,21173,21385,21598,21814,22033,22253,
	22475,22700,22927,23156,23388,23622,23858,24097,
	24338,24581,24827,25075,25326,25579,25835,26093,
	26354,26618,26884,27153,27424,27699,27976,28255,
	28538,28823,29112,29403,29697,29994,30294,30597,
	30903,31212,31524,31839,32157,32479,32804,33132
};

/*
static int compass_self_test(void)
{
		uint8_t ret_code,akm_addr;
    uint8_t tmp[6];
    unsigned char tries = 10;
    int result = 0x07;
    short data;
		akm_addr = (0x0C);

    ret_code = mpu_sensor_set_bypass(1);
		if (ret_code!=0)
			return MPU_ERROR_SET_BYPASS_MODE;
		twi_read_byte(akm_addr, AKM_REG_WHOAMI, &tmp[0]);
			SEGGER_RTT_printf(0, "ID=%x \r\n",tmp[0]);

    tmp[0] = AKM_POWER_DOWN;
    if (twi_write_byte(akm_addr, AKM_REG_CNTL, tmp[0]))
    //tmp[0] = AKM_POWER_DOWN;
    //if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
        return 0x07;
    tmp[0] = AKM_BIT_SELF_TEST;
    if (twi_write_byte(akm_addr, AKM_REG_ASTC, tmp[0]))
    //if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp))
        goto AKM_restore;
    tmp[0] = AKM_MODE_SELF_TEST;
    if (twi_write_byte(akm_addr, AKM_REG_CNTL, tmp[0]))
    //if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp))
        goto AKM_restore;

    do {
        delay_ms(10);
				if (twi_read_byte(akm_addr, AKM_REG_ST1, tmp))
        //if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ST1, 1, tmp))
            goto AKM_restore;
        if (tmp[0] & AKM_DATA_READY)
            break;
    } while (tries--);
    if (!(tmp[0] & AKM_DATA_READY))
        goto AKM_restore;

 				if (twi_read_bytes(akm_addr, AKM_REG_HXL, tmp,6))
				//if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_HXL, 6, tmp))
        goto AKM_restore;

    result = 0;
    data = (short)(tmp[1] << 8) | tmp[0];
			SEGGER_RTT_printf(0, "%x ",data);
    if ((data > 200) || (data < -200))  
        result |= 0x01;
    data = (short)(tmp[3] << 8) | tmp[2];
			SEGGER_RTT_printf(0, "%x ",data);
    if ((data > 200) || (data < -200))  
        result |= 0x02;
    data = (short)(tmp[5] << 8) | tmp[4];
			SEGGER_RTT_printf(0, "%x ",data);
    if ((data > -800) || (data < -3200))  
        result |= 0x04;
		AKM_restore:		
    tmp[0] = 0 | SUPPORTS_AK89xx_HIGH_SENS;
    if (twi_write_byte(akm_addr, AKM_REG_ASTC, tmp[0]))
    //i2c_write(st.chip_cfg.compass_addr, AKM_REG_ASTC, 1, tmp);
    tmp[0] = SUPPORTS_AK89xx_HIGH_SENS;
    if (twi_write_byte(akm_addr, AKM_REG_CNTL, tmp[0]))
    //i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp);
    mpu_sensor_set_bypass(0);
		SEGGER_RTT_printf(0, " CMP result=%x\r\n",result);
		return result;
}

*/
static int accel_6500_self_test(long *bias_regular, long *bias_st)
{
    int i, result = 0, otp_value_zero = 0;
    float accel_st_al_min, accel_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], accel_offset_max;
    unsigned char regs[3];
		
		if (twi_read_bytes(mpu_st.hw->addr, REG_6500_XA_ST_DATA, regs,3)) {
    //if (i2c_read(st.hw->addr, REG_6500_XA_ST_DATA, 3, regs)) {
    	return 0x07;
    }
	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= accel_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	if(otp_value_zero == 0) {
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i] - 1.f;

			if (fabs(st_shift_ratio[i]) > max_accel_var) {
				result |= 1 << i;	//Error condition
			}
		}
	}
	else {
		/* Self Test Pass/Fail Criteria B */
		accel_st_al_min = min_g * 65536.f;
		accel_st_al_max = max_g * 65536.f;

		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(st_shift_cust[i] < accel_st_al_min || st_shift_cust[i] > accel_st_al_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}

	if(result == 0) {
	/* Self Test Pass/Fail Criteria C */
		accel_offset_max = max_g_offset * 65536.f;
		for (i = 0; i < 3; i++) {
			if(fabs(bias_regular[i]) > accel_offset_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}
		SEGGER_RTT_printf(0, "ACC result=%x\r\n",result);

    return result;
}

static int gyro_6500_self_test(long *bias_regular, long *bias_st)
{
    int i, result = 0, otp_value_zero = 0;
    float gyro_st_al_max;
    float st_shift_cust[3], st_shift_ratio[3], ct_shift_prod[3], gyro_offset_max;
    unsigned char regs[3];

		if (twi_read_bytes(mpu_st.hw->addr, REG_6500_XG_ST_DATA, regs,3)) {
    //if (i2c_read(st.hw->addr, REG_6500_XG_ST_DATA, 3, regs)) {
        return 0x07;
    }


	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			ct_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
			ct_shift_prod[i] *= 65536.f;
			ct_shift_prod[i] /= gyro_sens;
		}
		else {
			ct_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}

	if(otp_value_zero == 0) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];


			st_shift_ratio[i] = st_shift_cust[i] / ct_shift_prod[i];


			if (fabs(st_shift_ratio[i]) < max_gyro_var) {
				result |= 1 << i;	//Error condition
			}
		}
	}
	else {
		/* Self Test Pass/Fail Criteria B */
		gyro_st_al_max = max_dps * 65536.f;


		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = bias_st[i] - bias_regular[i];

			if(st_shift_cust[i] < gyro_st_al_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}

	if(result == 0) {
	/* Self Test Pass/Fail Criteria C */
		gyro_offset_max = min_dps * 65536.f;
		for (i = 0; i < 3; i++) {
			if(fabs(bias_regular[i]) > gyro_offset_max) {
				result |= 1 << i;	//Error condition
			}
		}
	}
		SEGGER_RTT_printf(0, "GYRO result=%x\r\n",result);
    return result;
}

static int get_st_6500_biases(long *gyro, long *accel, unsigned char hw_test)
{
    uint8_t data[HWST_MAX_PACKET_LENGTH];
    unsigned char packet_count, ii;
    unsigned short fifo_count;
    int s = 0, read_size = 0, ind;
		uint8_t tmp;

    data[0] = 0x01;
    data[1] = 0;
    if (twi_write_bytes(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, 2,data))
    //if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
		tmp=0;
    //data[0] = 0;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->int_enable, tmp))
    //if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
        return -1;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, tmp))
    //if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->pwr_mgmt_1, tmp))
    //if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->i2c_mst, tmp))
   //if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, tmp))
    //if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    tmp = BIT_FIFO_RST | BIT_DMP_RST;
    //data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, tmp))
    //if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    tmp  = reg_lpf;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->lpf, tmp))
    //if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
        return -1;
    tmp  = reg_rate_div;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->rate_div, tmp))
    //if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        tmp  = reg_gyro_fsr | 0xE0;
    else
        tmp  = reg_gyro_fsr;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->gyro_cfg, tmp))
    //if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
        return -1;

    if (hw_test)
        tmp  = reg_accel_fsr | 0xE0;
    else
        tmp  = reg_accel_fsr;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->accel_cfg, tmp))
    //if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
        return -1;

    delay_ms(200);  //wait 200ms for sensors to stabilize

    /* Enable FIFO */
    tmp  = BIT_FIFO_EN;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->user_ctrl, tmp))
    //if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    tmp  = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, tmp))
    //if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    //initialize the bias return values
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;


    //start reading samples
    while (s < packet_thresh) {
    	delay_ms(10); //wait 10ms to fill FIFO
		if (twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_count_h, data,2)) 
		//if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
			return -1;
		fifo_count = (data[0] << 8) | data[1];
		packet_count = fifo_count / MAX_PACKET_LENGTH;
		if ((packet_thresh - s) < packet_count)
		            packet_count = packet_thresh - s;
		read_size = packet_count * MAX_PACKET_LENGTH;

		//burst read from FIFO
		if (twi_read_bytes(mpu_st.hw->addr, mpu_st.reg->fifo_r_w, data,read_size)) 
		//if (i2c_read(st.hw->addr, st.reg->fifo_r_w, read_size, data))
						return -1;
		ind = 0;
		for (ii = 0; ii < packet_count; ii++) {
			short accel_cur[3], gyro_cur[3];
			accel_cur[0] = ((short)data[ind + 0] << 8) | data[ind + 1];
			accel_cur[1] = ((short)data[ind + 2] << 8) | data[ind + 3];
			accel_cur[2] = ((short)data[ind + 4] << 8) | data[ind + 5];
			accel[0] += (long)accel_cur[0];
			accel[1] += (long)accel_cur[1];
			accel[2] += (long)accel_cur[2];
			gyro_cur[0] = (((short)data[ind + 6] << 8) | data[ind + 7]);
			gyro_cur[1] = (((short)data[ind + 8] << 8) | data[ind + 9]);
			gyro_cur[2] = (((short)data[ind + 10] << 8) | data[ind + 11]);
			gyro[0] += (long)gyro_cur[0];
			gyro[1] += (long)gyro_cur[1];
			gyro[2] += (long)gyro_cur[2];
			ind += MAX_PACKET_LENGTH;
		}
		s += packet_count;
    }


    //stop FIFO
    tmp  = 0;
    if (twi_write_byte(mpu_st.hw->addr, mpu_st.reg->fifo_en, tmp ))
    //if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    gyro[0] = (long)(((long long)gyro[0]<<16) / gyro_sens / s);
    gyro[1] = (long)(((long long)gyro[1]<<16) / gyro_sens / s);
    gyro[2] = (long)(((long long)gyro[2]<<16) / gyro_sens / s);
    accel[0] = (long)(((long long)accel[0]<<16) / accel_sens / s);
    accel[1] = (long)(((long long)accel[1]<<16) / accel_sens / s);
    accel[2] = (long)(((long long)accel[2]<<16) / accel_sens / s);
    /* remove gravity from bias calculation */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;



    return 0;
}
/**
 *  @brief      Trigger gyro/accel/compass self-test for MPU6500/MPU9250
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @param[in]  debug       Debug flag used to print out more detailed logs. Must first set up logging in Motion Driver.
 *  @return     Result mask (see above).
 */
int mpu_run_20689_self_test(long *gyro, long *accel)
{
    const unsigned char tries = 2;
    long gyro_st[3], accel_st[3];
    unsigned char accel_result, gyro_result;
#ifdef AK89xx_SECONDARY
    unsigned char compass_result;
#endif
    int ii;

    int result;
//    unsigned char accel_fsr, fifo_sensors, sensors_on;
//    unsigned short gyro_fsr, sample_rate, lpf;
//    unsigned char dmp_was_on;

		gyro_sens			= 32768/250;
    accel_sens     = 32768/2;  //FSR = +-2G = 16384 LSB/G
    reg_rate_div   = 0;    // 1kHz. 
    reg_lpf        = 2;    // 92Hz low pass filter
    reg_gyro_fsr   = 0;    // 250dps. 
    reg_accel_fsr  = 0x0;  // Accel FSR setting = 2g. 
    packet_thresh  = 200;    // 200 samples 
    min_dps        = 20.f;  //20 dps for Gyro Criteria C
    max_dps        = 60.f; //Must exceed 60 dps threshold for Gyro Criteria B
    max_gyro_var   = .5f; //Must exceed +50% variation for Gyro Criteria A
    min_g          = .225f; //Accel must exceed Min 225 mg for Criteria B
    max_g          = .675f; //Accel cannot exceed Max 675 mg for Criteria B
    max_accel_var  = .5f;  //Accel must be within 50% variation for Criteria A
    max_g_offset   = .5f;   //500 mg for Accel Criteria C

		SEGGER_RTT_WriteString(0, "mpu_run_20689_self_test\r\n");

    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro, accel, 0))
            break;
    if (ii == tries) {
        /* If we reach this point, we most likely encountered an I2C error.
         * We'll just report an error for all three sensors.
         */

        result = 0;
        goto restore;
    }


    for (ii = 0; ii < tries; ii++)
        if (!get_st_6500_biases(gyro_st, accel_st, 1))
            break;
    if (ii == tries) {


        /* Again, probably an I2C error. */
        result = 0;
        goto restore;
    }

    accel_result = accel_6500_self_test(accel, accel_st);
 
    gyro_result = gyro_6500_self_test(gyro, gyro_st);

    result = 0;
    if (gyro_result!=0)
        result |= 0x01;
    if (accel_result!=0)
        result |= 0x02;

/*
#ifdef AK89xx_SECONDARY
    compass_result = compass_self_test();
    if (compass_result!=0) {
        result |= 0x04;
		}
#else
    result |= 0x04;
#endif
*/
restore:

	return result;
}


