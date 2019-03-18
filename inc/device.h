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


#include "app_twi.h"
#include "nrf_drv_spi.h"
//#include "nrf_drv_rtc.h"
#include "app_timer.h"
//#include "nrf_drv_clock.h"

#define SPI_INSTANCE  1 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
#define SPI_INSTANCE1  2 /**< SPI instance index. */
//static const nrf_drv_spi_t spi1 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE1);  /**< SPI instance. */

//static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    5
//APP_TWI_DEF(m_app_twi, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
static const nrf_drv_twi_t m_app_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);  /**< TWI instance. */
typedef struct 
{
	bool	buttons_disabled;
	bool	init_f;
	bool	test_end;
	bool	test_device;
	bool	timer_1s_f;
	bool	timer_1ms_f;
	bool	mpu_int_f;
	bool	bsp_event_f;
	bool	idle;
	bool	advertising;
	bool	counting;
	bool	connected;
	bool	charging;
	bool	chrg_full;
	bool	battery_low;
	bool	sys_data_updated;
	bool	cnt_updated;
//	bool	dmp_recording;
	bool	chk_bat;
	bool	mem_full;
	bool	device_error;
	bool	device_name_chged;
	bool	show_batval;
	bool	blinking_led;
	bool	start_blink_led;
	bool	stop_blink_led;
	bool	pc_connected;
	bool	pc_send_data;
}sys_flag_t;
typedef __packed struct
{
		uint8_t 	device_type;		// Manufacturer specific information. Specifies the device type in this implementation.
		uint8_t 	adv_data_length;	// Manufacturer specific information. Specifies the length of the 
																// manufacturer specific data in this implementation.
    uint8_t 	uuid[2];					// 16 bit UUID value.
    uint16_t 	major;						// Major arbitrary value that can be used to distinguish between beacons. 
    uint16_t 	minor;						// Minor arbitrary value that can be used to distinguish between beacons.
    uint8_t 	measured_rssi;		// Manufacturer specific information. The beacon's measured TX power in 
																// this implementation. 
} beacon_t;

//void	spi_config(void);
void	twi_config(void);
void	sensor_init(void);
void	read_system_data(void);
void save_system_data(void);
void save_count_data(void);


#ifdef __cplusplus
extern "C" {
#endif

