/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "SEGGER_RTT.h"

#include <stdio.h>
//#include "nrf_drv_spi.h"
#include "boards.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "app_error.h"
#include "app_timer.h"
//#include "app_twi.h"
#include "device.h"
#include "compiler_abstraction.h"
#define NRF_LOG_MODULE_NAME "DEV"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "ble.h"
#include "ble_mts.h"
#include "mpu_sensor.h"


#define MAX_PENDING_TRANSACTIONS    5

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     5
#define SYS_DATA_PAGE_NO						0
//#define TEST_FLASH	1

int twi_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint8_t len);
//uint8_t mpu_sensor_init(mpu_sensor_init_t * sensor_init);
extern	uint16_t	m_conn_handle;                  /**< Handle of the current connection. */
extern	ble_mts_t	m_mts;
extern sys_flag_t sys_flag;
extern uint8_t 		ble_tx_power_level;

extern 	beacon_t   						beacon;														/**< beacon struct  */
extern 	uint16_t							beacon_major;
extern 	uint16_t							beacon_minor;
extern 	uint16_t							beacon_interval;
extern 	uint16_t							beacon_period;
extern	uint8_t	 			motion_sampling_rate;
extern	uint8_t				accel_fsr_val;	
extern	uint8_t				gyro_fsr_val;	
extern	uint16_t			int_threshold_val;	
extern	uint8_t 			dmp_lp_quat;
extern	uint8_t 			dmp_gyro_cal;
extern	uint8_t 			dmp_get_gyro;
extern	uint8_t 			dmp_get_accel;
extern 	uint8_t				dmp_mode;
extern 	uint8_t				count_data_on;
extern 	uint16_t			cnt_threshold_val;

extern uint16_t			cur_count;
extern uint16_t			total_count;

extern 	uint8_t 			tx_buf[];
extern uint8_t		device_name[];
extern uint8_t		device_name_len;
extern uint32_t 	start_page_no;
extern uint32_t * p_start_addr;


//spi buffer
uint8_t		m_tx_buf[64];				//< TX buffer. 
//uint8_t		m_rx_buf[0x84];    				//< RX buffer. 
uint8_t		m_rx_buf[234];    				//< RX buffer. 
// Buffer for data read from sensors.
uint8_t m_buffer[64];
uint32_t f_buffer[16];
uint16_t			motion_sampling_time_cnt=100;		//motion sampling time count for each rate (unit is 1ms)

void set_default_dev_name()
{
			device_name[0]='N';
			device_name[1]='A';
			device_name[2]='X';
			device_name[3]='S';
			device_name[4]='E';
			device_name[5]='N';
			device_name[6]='0';
			device_name[7]='0';
			device_name[8]='0';
			device_name[9]='0';
			device_name[10]='0';
			device_name[11]='1';
			device_name_len=12;
}
void set_default_system_val()
{
//		SEGGER_RTT_WriteString(0, "set_default_system_val\r\n");
			beacon_major=0;
			beacon_minor=0;
			beacon_interval=500;		//unit 0.625ms
			beacon_period=30;				//unit sec
			ble_tx_power_level=1;
			motion_sampling_rate=1;
			accel_fsr_val=0;
			gyro_fsr_val=0;
			int_threshold_val=274;
			dmp_lp_quat=1;
			dmp_gyro_cal=1;
			dmp_get_gyro=1;
			dmp_get_accel=1;
			count_data_on=0;
			cnt_threshold_val=3000;
			set_default_dev_name();
	
			save_system_data();
			
}
void read_device_name()
{
	uint8_t i,tch;
	
//	bool data_fail,data1_fail;
	uint8_t ret_code;
	uint8_t t_name_len;
	uint8_t t_name[13];
		sys_flag.device_name_chged=false;
			t_name_len=m_rx_buf[4];
			//SEGGER_RTT_printf(0, "%d\r\n",t_name_len);
			if ((t_name_len==0xff)|| (t_name_len==0)){
				sys_flag.sys_data_updated=true;
				sys_flag.device_name_chged=true;
				set_default_dev_name();
				return;
			}
			if (t_name_len>13) t_name_len=13;
			for (i=0;i<t_name_len;i++) {
				t_name[i]=m_rx_buf[i+5];
			//SEGGER_RTT_printf(0, "%02x",device_name[i]);
				if (t_name[i]!=device_name[i])
					sys_flag.device_name_chged=true;
			}
			//SEGGER_RTT_printf(0, "\r\n");
			if (t_name_len!=device_name_len) 
				sys_flag.device_name_chged=true;
			device_name_len=t_name_len;	
			for (i=0;i<t_name_len;i++) {
				device_name[i]=t_name[i];
			}
}
void read_system_data()
{
	uint8_t i,j,tch;
	bool data_fail;
	uint8_t ret_code;
	uint8_t temp_buf[48];
		data_fail=false;
		for (i=0;i<12;i++) {
				f_buffer[i]=(uint32_t)* (p_start_addr+i);
		}
		j=0;
		for (i=0;i<12;i++) {
				temp_buf[j++]=(uint8_t)(f_buffer[i]>>24);
				temp_buf[j++]=(uint8_t)(f_buffer[i]>>16);
				temp_buf[j++]=(uint8_t)(f_buffer[i]>>8);
				temp_buf[j++]=(uint8_t)(f_buffer[i]&0xff);
		}
		/*
		for (i=9;i<48;i++) {
			SEGGER_RTT_printf(0, "%02x ",temp_buf[i]);
		}
		SEGGER_RTT_WriteString(0, "\r\n");
		*/	
		if ((temp_buf[46]==0xff)&(temp_buf[47]==0xff)) {
		//Flash data is blank
			data_fail=true;
		SEGGER_RTT_WriteString(0, "data x_fail\r\n");
		} else {
			if ((temp_buf[46]!=0x55)||(temp_buf[47]!=0xAA)) {
				//chech the data mark, data is not correct
				data_fail=true;
		SEGGER_RTT_WriteString(0, "data xx_fail\r\n");
								bsp_board_led_off(BSP_BOARD_LED_0);
								bsp_board_led_on(BSP_BOARD_LED_1);
								nrf_delay_ms(1000);
								bsp_board_led_off(BSP_BOARD_LED_1);							
				//return;
			}
		}
//		data_fail=true;
		if (data_fail)
		//flash data is not correct, initialize it
		{
		SEGGER_RTT_WriteString(0, "data_fail\r\n");
			set_default_system_val();			
		} else {
			beacon_major=(uint16_t)(temp_buf[0]<<8)+temp_buf[1];
			beacon_minor=(uint16_t)(temp_buf[2]<<8)+temp_buf[3];
			beacon_interval=(uint16_t)(temp_buf[4]<<8)+temp_buf[5];
			beacon_period=(uint16_t)(temp_buf[6]<<8)+temp_buf[7];
			ble_tx_power_level=temp_buf[8];
			accel_fsr_val=temp_buf[9];
			gyro_fsr_val=temp_buf[10];
			int_threshold_val=(uint16_t)(temp_buf[11]<<8)+temp_buf[12];
			dmp_lp_quat=temp_buf[13];
			dmp_gyro_cal=temp_buf[14];
			dmp_get_gyro=temp_buf[15];
			dmp_get_accel=temp_buf[16];
			motion_sampling_rate=temp_buf[17];
			dmp_mode=temp_buf[18];
			count_data_on=temp_buf[19];
			cnt_threshold_val=(uint16_t)(temp_buf[20]<<8)+temp_buf[21];
			device_name_len=temp_buf[22];
			if (device_name_len>12) device_name_len=12;
			//SEGGER_RTT_printf(0, "device_name_len=%02x\r\n",device_name_len);
			for (i=0;i<device_name_len;i++) {
				device_name[i]=temp_buf[i+23];
			//SEGGER_RTT_printf(0, "%02x ",device_name[i]);
			}
		//SEGGER_RTT_WriteString(0, "\r\n");
		}
}
void save_system_data()
{
	uint8_t i,j,tch,ret_code;

		m_buffer[0]=beacon_major>>8;
		m_buffer[1]=beacon_major&0xff;
		m_buffer[2]=beacon_minor>>8;
		m_buffer[3]=beacon_minor&0xff;
		m_buffer[4]=beacon_interval>>8;
		m_buffer[5]=beacon_interval&0xff;
		m_buffer[6]=beacon_period>>8;
		m_buffer[7]=beacon_period&0xff;
		m_buffer[8]=ble_tx_power_level;
		m_buffer[9]=accel_fsr_val;
		m_buffer[10]=gyro_fsr_val;
		m_buffer[11]=int_threshold_val>>8;
		m_buffer[12]=int_threshold_val&0xff;
		m_buffer[13]=dmp_lp_quat;
		m_buffer[14]=dmp_gyro_cal;
		m_buffer[15]=dmp_get_gyro;
		m_buffer[16]=dmp_get_accel;
		m_buffer[17]=motion_sampling_rate;
		m_buffer[18]=dmp_mode;
		m_buffer[19]=count_data_on;
		m_buffer[20]=cnt_threshold_val>>8;
		m_buffer[21]=cnt_threshold_val&0xff;
		m_buffer[22]=device_name_len;
		//SEGGER_RTT_printf(0, "device_name_len=%02x\r\n",device_name_len);
		for (uint8_t i=0;i<device_name_len;i++) {
			tch=device_name[i];
			m_buffer[i+23]=tch;
			//SEGGER_RTT_printf(0, "%02x ",tch);
		}
		//SEGGER_RTT_printf(0, "\r\n");
		m_buffer[46]=0x55;;
		m_buffer[47]=0xAA;;
		j=0;
		for (i=0;i<12;i++) {
				f_buffer[i]=(m_buffer[j])<<24;
				j++;
				f_buffer[i]+=(m_buffer[j])<<16;
				j++;
				f_buffer[i]+=(m_buffer[j])<<8;
				j++;
				f_buffer[i]+=m_buffer[j];
				j++;
		}
		
			sd_flash_page_erase(start_page_no);
			nrf_delay_ms(100);
			sd_flash_write(p_start_addr, (uint32_t *)f_buffer,12);
			nrf_delay_ms(1);
}


void save_count_data()
{
//	uint8_t ret_code;
		m_buffer[0]=cur_count>>8;
		m_buffer[1]=cur_count&0xff;
		m_buffer[2]=total_count>>8;
		m_buffer[3]=total_count&0xff;
		
			sd_flash_page_erase(start_page_no+1);
			nrf_delay_ms(100);
			sd_flash_write(p_start_addr, (uint32_t *)m_buffer,4);
			nrf_delay_ms(1);

}


void sensor_init(void)
{
uint8_t ret_code;
	ret_code=mpu_sensor_init();
			SEGGER_RTT_printf(0, "mpu sensor init=%x\r\n", ret_code);
}

// TWI (with transaction manager) initialization.
void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
//       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_app_twi, &config,NULL,NULL);
		//SEGGER_RTT_printf(0, "nrf_drv_twi_init=%x\r\n", err_code);
    nrf_drv_twi_enable(&m_app_twi);
}

/*
void spi_config(void)
{
    nrf_drv_spi_config_t spix_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spix_config.ss_pin   = SPI_SS_PIN;
    spix_config.miso_pin = SPI_MISO_PIN;
    spix_config.mosi_pin = SPI_MOSI_PIN;
    spix_config.sck_pin  = SPI_SCK_PIN;
		spix_config.frequency=NRF_DRV_SPI_FREQ_8M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spix_config, NULL,NULL));
}
*/

/** @} */
