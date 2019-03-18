/** @file
 *
 * @defgroup  main.c
 * 
 * @brief project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "math.h"

#include "SEGGER_RTT.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "nrf_gpio.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "bsp_config.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_wdt.h"
#include "nrf_delay.h"

#include "app_gpiote.h"
#include "device.h"
#include "ble_tps.h"
#include "ble_bas.h"
//#include "ble_dis.h"
#include "ble_mts.h"
#include "ble_advs.h"
#include "ble_cds.h"
#include "mpu_sensor.h"
#include "mpu_sensor_dmp.h"
#include "app_uart.h"


#define APP_GPIOTE_MAX_USERS            	2    										/**< Size of gpiote users. */
//#define APP_GPIOTE_MAX_USERS            		1    										/**< Size of gpiote users. */
#define LL_HEADER_LEN                   4                                               /**< Link layer header length. */
#define CONN_CFG_TAG        						1                                               /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define NRF_BLE_MAX_MTU_SIZE            BLE_GATT_ATT_MTU_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */
#define APP_COMPANY_IDENTIFIER        	0x0059                           			 			 /**< Company identifier for Nordic */
//#define APP_COMPANY_IDENTIFIER        		0x004C                           			 			 /**< Company identifier for Apple Inc. as per www.bluetooth.org. */

//#define DEVICE_NAME                     "Koalla xxx"                           	/**< Name of device. Will be included in the advertising data. */
//#define MANUFACTURER_NAME               "GindaTek"                       			/**< Manufacturer. Will be passed to Device Information Service. */
//#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
//#define APP_ADV_TIMEOUT_IN_SECONDS      180                                     /**< The advertising timeout in units of seconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

//#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
//#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
//#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
//#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
//#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF						0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_500MS_INTERVAL				16384																					//0.5s
#define APP_1MS_INTERVAL				33																					//1ms
#define APP_TIMER_PRESCALER				0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE			20                                           /**< Size of timer operation queues. */
#define SYSTEM_TIMER_INTERVAL				32768																				//1s
#define SWAP(x) ((((x)&0xFF)<<8)|(((x)>>8)&0xFF))


//extern uint8_t erase_time;
void		ble_srv_bat_send(ble_cds_t *m_cds);
void		stop_recording(void);
void		flash_clear_status(void); 
void 	show_bat_level(void);
int mpu_run_6500_self_test(long *gyro, long *accel);
void read_device_name(void);


extern dmp_s dmp; 
extern uint8_t			m_buffer[];
extern uint16_t			w_current_page_no;
extern uint16_t		w_current_column_addr;				//each page has 2k bytes for write
extern uint8_t	 		motion_sampling_rate;

extern uint16_t   motion_sampling_time_cnt;		//motion sampling time count for each rate (unit is 1ms)
extern uint8_t	 motion_notification;
extern uint8_t	 motion_interrupt_mode;
extern uint8_t		motion_dmp_on;
//extern long			timestamp;
//extern uint16_t		adv_interval;	//unit 0.625ms
//extern uint8_t		adv_period;		//unit sec
extern bool			m_led_on_state;
extern bool 	cds_enabled;
extern	uint16_t			int_threshold_val;	
extern	uint8_t 			dmp_lp_quat;
extern	uint8_t 			dmp_gyro_cal;
extern	uint8_t 			dmp_get_gyro;
extern	uint8_t 			dmp_get_accel;
extern 	uint8_t				dmp_mode;
extern 	uint8_t				count_data_on;
extern 	uint16_t			cnt_threshold_val;
extern 	uint8_t major_ver_no;
extern 	uint8_t minor_ver_no;
extern 	uint8_t minor1_ver_no;

extern 	bsp_indication_t m_stable_state;

extern uint8_t cmd_45;
extern uint8_t				accel_fsr_val;	
extern uint8_t				gyro_fsr_val;

bool			over_threshold;
uint16_t			cur_count;
uint16_t			total_count;
uint8_t 			blink_led_color;
//UART
#define UART_BUF_SIZE                128                                         /**< UART buffer size. */
extern uint8_t tx_buf[];
extern uint8_t rx_buf[];
uint8_t tx_len = 0;
uint8_t rx_len = 0;

uint8_t accel_fsr_tbl[4]={2,4,8,16};
uint8_t accel_fsr_tblx[4]={4,8,16,32};
uint32_t start_page_no;
uint32_t * p_start_addr;
uint32_t * p_end_addr;

uint8_t		device_name[13]={"NAXSEN 6x"};
uint8_t		device_name_len=9;
uint8_t		test_result;
uint8_t		init_count;
sys_flag_t sys_flag;
//static uint8_t  default_beacon_uuid[16] 		= {0x15, 0x34, 0x51, 0x64, 0x67, 0xAB, 0x3E, 0x49, 
//																							 0xF9, 0xD6, 0xE2, 0x90, 0x00, 0x00, 0x00, 0x07};		/**< beacon service UUID  */
/* Beacon parameter */
beacon_t   								beacon;									 /**< beacon struct  */
uint16_t										beacon_major;
uint16_t										beacon_minor;
uint16_t										beacon_interval;
uint16_t										beacon_period;
uint16_t										new_bat_chrg_max_value;
uint16_t		max_bat_val=0x250;
//uint16_t		t_max_bat_val;
uint16_t		cur_bat_level;
uint16_t 		bat_full_cnt;
uint16_t		cur_bat_val=0;
uint16_t 		prev_bat_val=0;
uint16_t		max_chrg_cnt=0;
uint16_t		max_chrg_val=0;
uint16_t		cur_chrg_cnt=0;
uint16_t		cur_chrg_val=0;
uint8_t			chrg_state=0;

uint16_t			t_time_cnt;						//time count for 1ms
static ble_srv_motion_meas_t 				ble_motion_measurement;

app_gpiote_user_id_t		m_gpiote_mpu_int_user_id=0;            	/**< GPIOTE user id for mpu-sensor module. */
nrf_drv_wdt_channel_id m_channel_id;
//static uint8_t new_mtu_len=240;
#define SAADC_OVERSAMPLE NRF_SAADC_OVERSAMPLE_4X  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_SAMPLES_IN_BUFFER 4                 //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
nrf_saadc_value_t     m_buffer_pool[SAADC_SAMPLES_IN_BUFFER];
uint8_t test_bat_cnt;
bool test_bat_f;


uint8_t bsp_event;
uint8_t  battery_level;

bool print_f=true;
//bool print_f=false;
uint16_t test_cnt;

uint16_t       m_conn_handle = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static nrf_ble_gatt_t m_gatt;			/**< GATT module instance. */


/* YOUR_JOB: Declare all services structure your application is using
 */
static	ble_bas_t	m_bas;                                   /**< Structure used to identify the battery service. */
static	ble_tps_t	m_tps;
ble_mts_t	m_mts;
static	ble_advs_t	m_advs;
static	ble_cds_t	m_cds;

APP_TIMER_DEF(m_system_timer_id);                        /**< system timer. 1s */
APP_TIMER_DEF(m_1ms_timer_id);
APP_TIMER_DEF(m_500ms_timer_id);


// YOUR_JOB: Use UUIDs for service(s) used in your application.
/*
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_TX_POWER_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_MTS_SERVICE, BLE_UUID_TYPE_BLE},
                                   {BLE_UUID_ADVS_SERVICE, BLE_UUID_TYPE_BLE},
									{BLE_UUID_CDS_SERVICE, BLE_UUID_TYPE_BLE}};
//								{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; //< Universally unique service identifiers.
*/
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_MTS_SERVICE, BLE_UUID_TYPE_BLE}};


uint16_t cvt_to_uint(uint8_t hb, uint8_t lb); 
static void advertising_start(void);
void app_uart_send_data(uint8_t length);


//static void advertising_stop(void);
//void nrf_delay_ms(uint16_t delay_t);									

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#define MAX_CTRL_POINT_RESP_PARAM_LEN   3
#define IRQ_ENABLED                     0x01                        /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS           32                          /**< Maximum number of interrupts available. */
#define BOOTLOADER_DFU_START            0xB1
/**@brief Function for disabling all interrupts before jumping from bootloader to application.
 */
static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];

    // Loop from interrupt 0 for disabling of all interrupts.
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            // The interrupt was enabled, hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

/**@brief Function for preparing the reset, disabling SoftDevice, and jumping to the bootloader.
 *
 */
static uint32_t bootloader_start(void)
{
    uint32_t err_code;
		//SEGGER_RTT_WriteString(0, "bootloader_start\r\n");

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    VERIFY_SUCCESS(err_code);

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    VERIFY_SUCCESS(err_code);

    err_code = sd_softdevice_disable();
    VERIFY_SUCCESS(err_code);

    err_code = sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
    VERIFY_SUCCESS(err_code);
	
//		NRF_POWER->GPREGRET=0xB1;

    NVIC_ClearPendingIRQ(SWI2_IRQn);
    interrupts_disable();
	
    NVIC_SystemReset();
    return NRF_SUCCESS;
}

/**@brief Function for preparing for system reset.
*
* @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
* @ref dfu_app_handler.c before entering the bootloader/DFU.
* This allows the current running application to shut down gracefully.
*/

static void reset_prepare(void)
{
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
		{
		// Disconnect from peer.
		//SEGGER_RTT_WriteString(0, "reset_prepare gap_disconnect\r\n");
			sd_ble_gap_disconnect(m_conn_handle,  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			bsp_indication_set(BSP_INDICATE_IDLE);
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
		}
}


/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
//    bsp_board_leds_off();
		SEGGER_RTT_WriteString(0, "Wdt INT\r\n");

}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
				//sys_flag.saadc_int_f=true;
				nrf_gpio_pin_clear(TESTBAT);		//set low to save power
			//SEGGER_RTT_printf(0, "%x\r\n", m_buffer_pool[0]);
}
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    //uint32_t err_code;
    //uint8_t  battery_level;
		uint16_t dif_val,full_value,t_max_bat_val;
		uint16_t t_max_time;
		uint16_t div_res;
	
		if (m_buffer_pool[0]<100)
			return;
		//SEGGER_RTT_printf(0,"bat value= %d\r\n", m_buffer_pool[0]);
		
		if (max_bat_val<m_buffer_pool[0]){
			max_bat_val=m_buffer_pool[0];
			//SEGGER_RTT_printf(0,"bat value= %d\r\n", t_max_bat_val);	
		}

		if (prev_bat_val==0) {
				prev_bat_val=m_buffer_pool[0];
		}
		cur_bat_val=m_buffer_pool[0];
		if (max_bat_val>0x250) t_max_bat_val=0x250;
		else t_max_bat_val=max_bat_val;
		if (sys_flag.charging) {
				full_value=t_max_bat_val-430;
				if (cur_bat_val<prev_bat_val) 
					cur_bat_val=prev_bat_val;
				else
					prev_bat_val=cur_bat_val;
		} else {
					full_value=t_max_bat_val-445;
				if (cur_bat_val>prev_bat_val) 
					cur_bat_val=prev_bat_val;
				else
					prev_bat_val=cur_bat_val;
		}
		if (cur_bat_val<0x1e0) full_value+=20;	//3.5V
		else
		if (cur_bat_val<0x1d0) full_value+=40;	//3.3V
		if (cur_bat_val>430) dif_val=cur_bat_val-430;
		else dif_val=0;
		div_res=(dif_val*100)/full_value;;
		battery_level=div_res;
		if (battery_level>100) battery_level=100;
/*
		if (cur_bat_level!=battery_level) {
			cur_bat_level=battery_level;
			SEGGER_RTT_printf(0, "bat level=%d\r\n", battery_level);			
		}
*/
		//SEGGER_RTT_printf(0, "%d dif_val=%d,full_vlaue=%d,bat level=%d\r\n",cur_bat_val,dif_val,full_value, battery_level);			
		if (sys_flag.chk_bat) {
			SEGGER_RTT_printf(0, "%d  ", cur_bat_val);
			SEGGER_RTT_printf(0, "%d\r\n", battery_level);			
			if (cds_enabled) {
				ble_srv_bat_send(&m_cds);
			}
			
		}

		if (sys_flag.charging) {
			if (sys_flag.chrg_full) {
			} else {
					//debug charging battery
					if (cur_chrg_val==m_buffer_pool[0]) {
						cur_chrg_cnt++;
					} else {
						if (cur_chrg_cnt>max_chrg_cnt) {
							max_chrg_cnt=cur_chrg_cnt;
							max_chrg_val=cur_chrg_val;
						}
						cur_chrg_cnt=0;
						cur_chrg_val=m_buffer_pool[0];
					}
					//
					if (cur_bat_val> new_bat_chrg_max_value){
						new_bat_chrg_max_value=cur_bat_val;
						bat_full_cnt=0;
					} else {
						bat_full_cnt++;
						if (new_bat_chrg_max_value>=0x250) t_max_time=60;
						else t_max_time=300;
						if (bat_full_cnt>t_max_time) {
						//x minutes
						//charging full
							chrg_state=2;
							sys_flag.chrg_full=true;
							max_bat_val=new_bat_chrg_max_value;
							if (!(sys_flag.advertising||sys_flag.connected)) 
							bsp_indication_set(BSP_INDICATE_CHRG_FULL);
						}
					}
			}
		}
		if (sys_flag.battery_low) {
			if (battery_level>25) {
					sys_flag.battery_low=false;
					bsp_indication_set(BSP_INDICATE_IDLE);
			}
		} else {
			if (battery_level<=15) {
					sys_flag.chrg_full=false;
					sys_flag.battery_low=true;
					bsp_indication_set(BSP_INDICATE_BATTERY_LOW);
					if (sys_flag.advertising) {
							sys_flag.advertising=false;
							sd_ble_gap_adv_stop();
					} else
					if (sys_flag.connected) {
						sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
						sys_flag.connected=false;
					}
			}
		}
		ble_bas_battery_level_update(&m_bas, battery_level);
}

void cal_count()
{
	uint16_t accn;
	uint32_t tdd;
	uint8_t accel_data[6];
	double t_count,td;
		if (motion_dmp_on) 
			memcpy(accel_data,m_buffer+16,6);		//accel data 
		 else
			memcpy(accel_data,ble_motion_measurement.inst_accel,6);		//accel data 
		
				accn=(uint16_t)(accel_data[0]<<8)+accel_data[1];
			if (accn>=0x8000)
				accn=-accn+1;
				td=(accn*accel_fsr_tbl[accel_fsr_val])/32768.0;
				t_count=td*td;
				accn=(uint16_t)(accel_data[2]<<8)+accel_data[3];
			if (accn>=0x8000)
				accn=-accn+1;
				td=(accn*accel_fsr_tbl[accel_fsr_val])/32768.0;
				t_count+=td*td;
				accn=(uint16_t)(accel_data[4]<<8)+accel_data[5];
			if (accn>=0x8000)
				accn=-accn+1;
				td=(accn*accel_fsr_tbl[accel_fsr_val])/32768.0;
				t_count+=td*td;
/*	
	for (uint8_t i=0;i<6;i++)
	SEGGER_RTT_printf(0," %02x",accel_data[i]);
	SEGGER_RTT_printf(0,"\r\n");
*/	
				if (t_count!=0) {
					accn=sqrt(t_count)*1000;
	//SEGGER_RTT_printf(0,"accn= %x\r\n", accn);
					if (accn>cnt_threshold_val && !over_threshold) {
						cur_count++;
						total_count++;
						over_threshold = true;
					}else{
						over_threshold = false;
					}
				}
}

void process_1s_int(void)
{
	uint32_t ret_code;
	
			nrf_drv_wdt_channel_feed(m_channel_id);		//reset WDT
//	if (motion_notification){
/*
	if (motion_dmp_on) {
	SEGGER_RTT_printf(0,"test_cnt= %x\r\n", test_cnt);
	test_cnt=0;
	}
*/
		if (sys_flag.init_f) {
			init_count++;
				nrf_gpio_pin_set(TESTBAT);		//set high to detect battery voltage
				nrf_drv_saadc_buffer_convert(m_buffer_pool, 1);
				nrf_drv_saadc_sample();
				battery_level_update();
			if (init_count>3) {
					sys_flag.init_f=false;
//					show_bat_level();
			}
			return;
		}
			if (sys_flag.charging) {
				if (sys_flag.chrg_full) {
				} else {
					if (nrf_gpio_pin_read(CHRG_STATE))
					{
					//charging full
						chrg_state=1;
						sys_flag.chrg_full=true;
						max_bat_val=new_bat_chrg_max_value;
		SEGGER_RTT_WriteString(0, "chrg full\r\n");
							if (!(sys_flag.advertising||sys_flag.connected)) 
							bsp_indication_set(BSP_INDICATE_CHRG_FULL);
			SEGGER_RTT_printf(0,"chg full value= %d\r\n", m_buffer_pool[0]);	
		SEGGER_RTT_WriteString(0, "chrg full\r\n");
					
					}
				
				}

				if (nrf_gpio_pin_read(CHRG_DETECT)==0) {
			//charging cable removed
					sys_flag.pc_connected=false;
					sys_flag.charging=false;
					sys_flag.chrg_full=false;
					bsp_indication_set(BSP_INDICATE_IDLE);
					if (sys_flag.sys_data_updated)
					{
						sys_flag.sys_data_updated=false;
						save_system_data();
					}
				}
			} else {
			if (nrf_gpio_pin_read(CHRG_DETECT)) {
		//t_bat_f=false;
		SEGGER_RTT_WriteString(0, "chrg detect\r\n");
				sys_flag.charging=true;
				sys_flag.chrg_full=false;
				
				max_chrg_cnt=0;
				max_chrg_val=cur_bat_val;
				cur_chrg_cnt=0;
				cur_chrg_val=cur_bat_val;
				chrg_state=0;
				
				bat_full_cnt=0;
				new_bat_chrg_max_value=0;
				if (!(sys_flag.advertising||sys_flag.connected)) {
					bsp_indication_set(BSP_INDICATE_CHRGING);
				}
			}
			}
			if ((sys_flag.charging)||(sys_flag.connected)) {
					test_bat_cnt++;
					if (test_bat_cnt>=10) {
						test_bat_cnt=0;
						test_bat_f=true;
					}
			} else
			if (sys_flag.idle) {
					test_bat_cnt++;
					if (test_bat_cnt>=10) {
						test_bat_cnt=0;
						test_bat_f=true;
					}
			}
			battery_level_update();
			if (test_bat_f) {
				test_bat_f=false;
				//nrf_saadc_enable();
				nrf_gpio_pin_set(TESTBAT);		//set high to detect battery voltage
		//SEGGER_RTT_WriteString(0, "set TESTBAT\r\n");
				//nrf_delay_us(10);
				nrf_drv_saadc_buffer_convert(m_buffer_pool, 1);
				nrf_drv_saadc_sample();
			//SEGGER_RTT_printf(0, "%04x  \r\n", m_buffer_pool[0]);
		//SEGGER_RTT_printf(0, "%x\r\n  ", ret_code);
			}
			if (sys_flag.buttons_disabled) {
				sys_flag.buttons_disabled=false;
				bsp_buttons_enable();
			}

			if (sys_flag.start_blink_led) {
				sys_flag.start_blink_led=false;
				sys_flag.blinking_led=true;
				app_timer_start(m_500ms_timer_id, APP_500MS_INTERVAL, NULL);
				bsp_indication_set(BSP_INDICATE_IDLE);
			}
			if (sys_flag.stop_blink_led) {
				sys_flag.stop_blink_led=false;
				sys_flag.blinking_led=false;
				app_timer_stop(m_500ms_timer_id);
				bsp_indication_set(m_stable_state);
			}
			if (sys_flag.counting) {
				if (!sys_flag.pc_send_data&!motion_notification&!motion_dmp_on)
				{
					ret_code=mpu_sensor_get_accel_reg(ble_motion_measurement.inst_accel);
					memcpy(tx_buf , ble_motion_measurement.inst_accel, 6);
					cal_count();
				}
				
			}
		
}
static uint8_t motion_data_encode()

{
    uint8_t len  = 1;
	uint8_t i;
	uint16_t accn;
	uint32_t t_count;
	
		if (motion_dmp_on)
		{		
			memcpy(tx_buf+1 , m_buffer, dmp.packet_length);
			len=dmp.packet_length+1;
			
		}else
		{
				memcpy(tx_buf +len , ble_motion_measurement.inst_accel, 6);
				len+=6;
				memcpy(tx_buf +len , ble_motion_measurement.inst_gyro, 6);
				len+=6;
			if (count_data_on) {
				cal_count();				
				tx_buf[len++]=cur_count>>8;
				tx_buf[len++]=cur_count&0xFF;
			}
		}
		tx_buf[0]=len-1;
		if (print_f) {
			//print_f=false;
		for (i=0;i<len;i++) {
			SEGGER_RTT_printf(0, "%02x ",tx_buf[i]);
		}
		SEGGER_RTT_WriteString(0, "\r\n");
	}
    return len;
}

void process_1ms_int(void)
{
//	uint8_t ent;
	//test_cnt++;
		if (sys_flag.pc_connected) {
				if (sys_flag.pc_send_data) {
							if ( mpu_sensor_get_accel_reg(ble_motion_measurement.inst_accel) ){
							// Error!
							}
							if ( mpu_sensor_get_gyro_reg(ble_motion_measurement.inst_gyro) ){
							// Error!
							}
							tx_len=motion_data_encode();
							app_uart_send_data(tx_len);
				}
		} else {
			if (m_conn_handle!=BLE_CONN_HANDLE_INVALID) {
				if (motion_notification){
							if ( mpu_sensor_get_accel_reg(ble_motion_measurement.inst_accel) ){
							// Error!
							}
							if ( mpu_sensor_get_gyro_reg(ble_motion_measurement.inst_gyro) ){
							// Error!
							}
						ble_srv_motion_measurement_send(&m_mts, &ble_motion_measurement);
					
				}
			}
		}
}
void process_mpu_int(void)
{
	//uint8_t i;
				uint8_t more;
	//test_cnt++;
		//SEGGER_RTT_WriteString(0, "mpu int\r\n");
		if (sys_flag.pc_connected) {
			if (sys_flag.pc_send_data) {
					if (mpu_sensor_read_fifo_stream(dmp.packet_length, m_buffer, &more))
					{ //error
					} else
					{
							tx_len=motion_data_encode();
							app_uart_send_data(tx_len);
					}
				}
		} else {
			if (m_conn_handle!=BLE_CONN_HANDLE_INVALID) {
				if (motion_interrupt_mode) {
				//uint8_t ret_code;
					//ret_code=twi_read_byte(0x68, 0x3A, &data);
					if ( mpu_sensor_get_accel_reg(ble_motion_measurement.inst_accel) ){
							// Error!
					} else 
					{
							ble_srv_motion_measurement_send(&m_mts, &ble_motion_measurement);
					}
				} else
				if (motion_dmp_on) {
		//SEGGER_RTT_printf(0, "DMP=%d\r\n",dmp.packet_length);
					if (mpu_sensor_read_fifo_stream(dmp.packet_length, m_buffer, &more))
					{ //error
					} else
					{
					ble_srv_motion_measurement_send(&m_mts, &ble_motion_measurement);
					}
				}
			}
		}
}
void process_bsp_int(void)
{
}
/**@brief Function for handling the 1 second  timer timeout.
 *
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 * 1s
 */
static void app_1s_timeout_haldler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		sys_flag.timer_1s_f=true;
}


static void app_500ms_timeout_haldler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
	if (sys_flag.blinking_led) {
		if (blink_led_color==0) {
    nrf_gpio_pin_toggle(LED_GREEN);
		} else
		if (blink_led_color==1) {
			nrf_gpio_pin_toggle(LED_RED);
		} else {
			nrf_gpio_pin_toggle(LED_GREEN);
			nrf_gpio_pin_toggle(LED_RED);
		}
	} else {
			nrf_gpio_pin_clear(LED_GREEN);
			nrf_gpio_pin_clear(LED_RED);
	}
}

/**@brief Function for handling the APP timer timeout.
 *
 * @details This function will be called each time the APP timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 * 1ms per INT
 */
static void app_1ms_timeout_haldler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		t_time_cnt++;
		if (t_time_cnt>=motion_sampling_time_cnt) {
				t_time_cnt=0;
				sys_flag.timer_1ms_f=true;
		}
}
static void mpu_int_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	sys_flag.mpu_int_f=true;
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
        uint32_t err_code;
   // Initialize timer module.
    err_code = app_timer_init();
   APP_ERROR_CHECK(err_code);
//    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

       err_code = app_timer_create(&m_system_timer_id, APP_TIMER_MODE_REPEATED, app_1s_timeout_haldler);
       APP_ERROR_CHECK(err_code); 
       err_code = app_timer_create(&m_1ms_timer_id, APP_TIMER_MODE_REPEATED, app_1ms_timeout_haldler);
       APP_ERROR_CHECK(err_code); 
       err_code = app_timer_create(&m_500ms_timer_id, APP_TIMER_MODE_REPEATED, app_500ms_timeout_haldler);
       APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	/*
			SEGGER_RTT_printf(0, "%d\r\n",device_name_len);
		for (uint8_t i=0;i<device_name_len;i++)
			SEGGER_RTT_printf(0, "%02x",device_name[i]);
		SEGGER_RTT_printf(0, "\r\n");
*/
    err_code = sd_ble_gap_device_name_set(&sec_mode,(uint8_t *)device_name,device_name_len);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    switch (p_evt->evt_id)
    {
        case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
        {
//            m_mtu_exchanged = true;
            NRF_LOG_INFO("ATT MTU exchange completed. MTU set to %u bytes.\r\n",
                         p_evt->params.att_mtu_effective);
        } break;

        case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
        {
//            m_data_length_updated = true;
            NRF_LOG_INFO("Data length updated to %u bytes.\r\n", p_evt->params.data_length);
        } break;
    }

//    nrf_ble_amts_on_gatt_evt(&m_amts, p_evt);
}
static void conn_evt_len_ext_set()
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = true;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
   ble_opt_t  opt;
	ret_code_t err_code;
//    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_BLE_MAX_MTU_SIZE);
//    NRF_LOG_INFO("nrf_ble_gatt_att_mtu_periph_set error code=%x\r\n",err_code);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_BLE_MAX_MTU_SIZE+4);
//    err_code = nrf_ble_gatt_data_length_set(&m_gatt, CONN_CFG_TAG, new_mtu_len+4);
//    NRF_LOG_INFO("nrf_ble_gatt_data_length_set error code=%x\r\n",err_code);
    APP_ERROR_CHECK(err_code);
    opt.common_opt.conn_evt_ext.enable = true;
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
		conn_evt_len_ext_set();
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.
		*/
       uint32_t                           err_code;
       ble_tps_init_t                     tps_init;
       ble_bas_init_t					bas_init;
//		ble_dis_init_t 					dis_init;
		ble_mts_init_t					mts_init;
		ble_advs_init_t					advs_init;
		ble_cds_init_t					cds_init;

       // Initialize TPS Service.
       memset(&tps_init, 0, sizeof(tps_init));

		tps_init. initial_tx_power_level= 0;
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tps_init.tps_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tps_init.tps_attr_md.write_perm);
		err_code=ble_tps_init(&m_tps, &tps_init);
       APP_ERROR_CHECK(err_code);
			err_code=ble_tps_tx_power_level_set(&m_tps,20); 
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = false;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    // Initialize the Device Firmware Update Service.

// Initialize Measure Data Service.
    memset(&mts_init, 0, sizeof(mts_init));
    err_code = ble_mts_init(&m_mts, &mts_init);
    APP_ERROR_CHECK(err_code);
	
    // Initialize Beacon Data Service.
    memset(&advs_init, 0, sizeof(advs_init));
    err_code = ble_advs_init(&m_advs, &advs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize communication Data Service.
    memset(&cds_init, 0, sizeof(cds_init));
    err_code = ble_cds_init(&m_cds, &cds_init);
    APP_ERROR_CHECK(err_code);
    // Initialize Device Information Service.
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
void application_timers_start(void)
{
       uint32_t err_code;
		err_code = app_timer_start(m_system_timer_id, SYSTEM_TIMER_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
       //err_code = app_timer_start(m_1ms_timer_id, APP_1MS_INTERVAL, NULL);

}

void app_1ms_timer_stop(void)
{
       uint32_t err_code;
       err_code = app_timer_stop(m_1ms_timer_id);
       APP_ERROR_CHECK(err_code);

}
void app_1ms_timer_start(void)
{
       uint32_t err_code;
       err_code = app_timer_start(m_1ms_timer_id, APP_1MS_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code = NRF_SUCCESS;
		uint8_t eff_size;
		//SEGGER_RTT_printf(0, "evt_id=%x\r\n",p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
	case BLE_GAP_EVT_CONN_PARAM_UPDATE:
		//err_code=sd_ble_gattc_exchange_mtu_request(m_conn_handle,32);
     //       APP_ERROR_CHECK(err_code);
	break;
        case BLE_GAP_EVT_DISCONNECTED:
				//SEGGER_RTT_WriteString(0, "BLE_GAP_EVT_DISCONNECTED\r\n");
				if (sys_flag.sys_data_updated)
				{
						sys_flag.sys_data_updated=false;
						save_system_data();
				}
				if (cmd_45==0x83) {
				//reset (DFU)
					bootloader_start();
				}
				m_conn_handle = BLE_CONN_HANDLE_INVALID;
				sys_flag.connected=false;
				sys_flag.advertising=false;
				m_led_on_state=false;
				if (sys_flag.charging)
					bsp_indication_set(BSP_INDICATE_CHRGING);
				else if (sys_flag.chrg_full)
							bsp_indication_set(BSP_INDICATE_CHRG_FULL);
				else	
							bsp_indication_set(BSP_INDICATE_IDLE);
        break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			sd_ble_gattc_exchange_mtu_request(m_conn_handle,NRF_BLE_MAX_MTU_SIZE);
			sys_flag.connected=true;
			sys_flag.advertising=false;
			m_led_on_state=false;
			sys_flag.sys_data_updated=false;
      bsp_indication_set(BSP_INDICATE_CONNECTED);
	/*
			if (sys_flag.charging) {
					power_off_usb();
			}
			power_on_flash();
	*/
//     err_code = nrf_ble_gatt_data_length_get(&m_gatt, m_conn_handle, &eff_size);
//		SEGGER_RTT_printf(0, "data_length_get %02x\r\n",eff_size);
           break; // BLE_GAP_EVT_CONNECTED
				case BLE_GAP_EVT_TIMEOUT:
						sys_flag.advertising=false;
						m_led_on_state=false;
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
					break;
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT
        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_GATT_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
         break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);

    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_cds_on_ble_evt(&m_cds, p_ble_evt);
    ble_mts_on_ble_evt(&m_mts, p_ble_evt);
    ble_advs_on_ble_evt(&m_advs, p_ble_evt);
    ble_tps_on_ble_evt(&m_tps, p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;
//    ble_enable_params_t ble_enable_params;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
		err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum ATT MTU.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
//    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = 251;
		err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
		NRF_LOG_INFO("NRF_BLE_GATT_MAX_MTU_SIZE err code=%x\r\n",err_code);
//    APP_ERROR_CHECK(err_code);

/*
    // Configure the maximum event length.
    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                     = CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gap_conn_cfg.event_length = 320;
    ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   = BLE_GAP_CONN_COUNT_DEFAULT;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
*/
    // Configure the number of custom UUIDs.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 0;
//    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
/*
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
*/
}

void 	show_bat_level()
{
	//battery_level=29;
		if (battery_level < 30) {
			nrf_gpio_pin_set(BSP_LED_1);
		} else
		if (battery_level > 70) {
			nrf_gpio_pin_set(BSP_LED_0);
		} else {
			nrf_gpio_pin_set(BSP_LED_0);
			nrf_gpio_pin_set(BSP_LED_1);
		}
			nrf_delay_ms(1000);
			nrf_gpio_pin_clear(BSP_LED_0);
			nrf_gpio_pin_clear(BSP_LED_1);

}
void disable_ble()
{
						if (sys_flag.advertising) {
							sys_flag.advertising=false;
							sd_ble_gap_adv_stop();
							if (sys_flag.charging)
								bsp_indication_set(BSP_INDICATE_CHRGING);
							else if (sys_flag.chrg_full)
								bsp_indication_set(BSP_INDICATE_CHRG_FULL);
						}
						else if (sys_flag.connected) {
							sys_flag.connected=false;
							if (sys_flag.charging)
								bsp_indication_set(BSP_INDICATE_CHRGING);
							else if (sys_flag.chrg_full)
								bsp_indication_set(BSP_INDICATE_CHRG_FULL);
							sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
							bsp_board_led_on(BSP_BOARD_LED_0);
							nrf_delay_ms(100);
							bsp_board_led_off(BSP_BOARD_LED_0);
						}
}
//
//UART
//

void app_uart_send_data(uint8_t length)
{
    uint32_t err_code;
	//SEGGER_RTT_printf(0, "length %d\n\r",length);
	
    for (uint8_t i = 0; i < length; i++)
    {
        do
        {
            err_code = app_uart_put(tx_buf[i]);
	//SEGGER_RTT_printf(0, "%x",tx_buf[i]);
            if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed  Error 0x%x. \r\n", err_code);
                APP_ERROR_CHECK(err_code);
            }
        } while (err_code == NRF_ERROR_BUSY);
    }
	//SEGGER_RTT_printf(0, "\n\r");
}

void uart_tx_test()
{
	uint8_t i;
	for (i=0;i<16;i++)
	tx_buf[i]=i+10;
	app_uart_send_data(16);
}
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint32_t       err_code;
	uint8_t cmd,len,cmd_len;
	uint8_t hb,lb;
	uint8_t i,tch;
	uint16_t tint,max_val;
	bool err_f;
			//SEGGER_RTT_printf(0, "uart_event_handle %x\n\r",rx_len);

    switch (p_event->evt_type)
    {
			case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&rx_buf[rx_len]));
			//SEGGER_RTT_printf(0, "rx_buf=%02x",rx_buf[rx_len]);
            rx_len++;

            if (rx_buf[rx_len - 1] == 0x0a)
            {
								memset(&tx_buf, 0, 64);
								cmd=rx_buf[0];
								tx_buf[0]=cmd;
								len=1;

			SEGGER_RTT_printf(0, "rx_len=%02x\r\n",rx_len);
			SEGGER_RTT_printf(0, "cmd=%02x\r\n",cmd);
/*
			for (i=0;i<rx_len;i++)
			SEGGER_RTT_printf(0, "%02x",rx_buf[i]);
			SEGGER_RTT_printf(0, "\r\n");
*/							
								switch (cmd) {
									case 0x30:										
										disable_ble();
										sys_flag.pc_connected=true;
										break;
									case 0x31:										
										sys_flag.pc_connected=false;
										if (sys_flag.pc_send_data) {
											sys_flag.pc_send_data=false;
											cmd_45=10;
										}
										if (sys_flag.sys_data_updated)
										{
												sys_flag.sys_data_updated=false;
												save_system_data();
										}
										break;
									case 0x32:										
										sys_flag.pc_send_data=true;
										over_threshold = false;
										cmd_45=10;
										break;
									case 0x33:										
										sys_flag.pc_send_data=false;
										cmd_45=10;
										break;
									case 0x36:
										//reset accumulated count
										total_count=0;
										break;
									case 0x37:
									//blinking leds
										if (rx_buf[1]==1) {
											sys_flag.start_blink_led=true;
										} else {
											sys_flag.stop_blink_led=true;
										}
										blink_led_color=rx_buf[2];
									break;
									case 0x42:
									//set device name
										cmd_len=rx_len-1;
										if (cmd_len>12) cmd_len=12;
										memcpy(device_name,rx_buf+1,cmd_len);
										device_name_len=cmd_len;
										sys_flag.sys_data_updated=true;
										sys_flag.device_name_chged=true;
								SEGGER_RTT_printf(0, "device_name_len=%x\r\n",device_name_len);
									for (i=0;i<device_name_len;i++)
								SEGGER_RTT_printf(0, "%x",device_name[i]);
								SEGGER_RTT_printf(0, "\r\n");
									break;
									case 0x45:
								//set configuration
										for (i=1;i<rx_len-1;i++)
								SEGGER_RTT_printf(0, "%02x ",rx_buf[i]);
								SEGGER_RTT_printf(0, "\r\n");
											err_f=false;
									tch=rx_buf[1];
										if (tch <4){
											if (accel_fsr_val!=tch) {
												sys_flag.sys_data_updated=true;
												accel_fsr_val=tch;
											}
										} else {
											err_f=true;
										}
										tch=rx_buf[2];
										if (tch <4){
											if (gyro_fsr_val!=tch) {
												sys_flag.sys_data_updated=true;
												gyro_fsr_val=tch;	
											}
										} else {
											err_f=true;
										}
										hb=rx_buf[3];
										lb=rx_buf[4];
										tint=cvt_to_uint(hb, lb);
										if (tint <1020) {
											if (int_threshold_val != tint) {
												sys_flag.sys_data_updated=true;
												int_threshold_val = tint;		
											}
										} else {
											err_f=true;
										}
										tch = rx_buf[5];		
										if ((tch==1)||(tch==2)) {
											if (dmp_lp_quat!=tch) {
												sys_flag.sys_data_updated=true;
												dmp_lp_quat=tch;
											}
										} else {
											err_f=true;
										}
										tch = rx_buf[6];		
										if (tch<2) {
											if (dmp_gyro_cal!=tch) {
												sys_flag.sys_data_updated=true;
												dmp_gyro_cal=tch;
											}
										} else {
											err_f=true;
										}
										tch = rx_buf[7];		
										if (tch<2) {
											if (dmp_get_gyro!=tch){
												sys_flag.sys_data_updated=true;
												dmp_get_gyro=tch;
											}
										} else {
											err_f=true;
										}
										tch = rx_buf[8];		
										if (tch<2) {
											if (dmp_get_accel!=tch) {
												sys_flag.sys_data_updated=true;
												dmp_get_accel=tch;
											}
										} else {
												err_f=true;
										}
										tch = rx_buf[9];
										if ((tch <3)||((tch>=7)&(tch<=9))) {
											if (motion_sampling_rate != tch){
												sys_flag.sys_data_updated=true;
												motion_sampling_rate = tch;	
											}
										} else {
						SEGGER_RTT_WriteString(0, "fail 9\r\n");
											err_f=true;
										}
										tch = rx_buf[10];		
										if (tch<2) {
											if (dmp_mode!=tch) {
												sys_flag.sys_data_updated=true;
												dmp_mode=tch;
											}
										} else {
											err_f=true;
										}
										tch = rx_buf[11];		
										if ((tch>0)&&(tch<4)) {
											cmd_45=tch;
										} else {
											cmd=0xff;
										}
										tch = rx_buf[12];		
										if (tch<2) {
											if (count_data_on!=tch) {
												sys_flag.sys_data_updated=true;
												count_data_on=tch;
											}
										} else {
							SEGGER_RTT_WriteString(0, "fail 12\r\n");
											err_f=true;
										}
										hb=rx_buf[13];
										lb=rx_buf[14];
										tint=cvt_to_uint(hb, lb);
										max_val=accel_fsr_tbl[accel_fsr_val]*1000;
										if (tint > max_val)
											tint=max_val;
										if (cnt_threshold_val != tint) {
											sys_flag.sys_data_updated=true;
											cnt_threshold_val = tint;							
										}
										if (err_f) {
											tx_buf[0]=0xc5;
										}
										//len=1;
									break;
									case 0x47:
										//len=1;
										tx_buf[len++]=major_ver_no;
										tx_buf[len++]=minor_ver_no;
										tx_buf[len++]=minor1_ver_no;
				
									break;
								case 0x49:
									tx_buf[1]=accel_fsr_val;
									tx_buf[2]=gyro_fsr_val;
									tx_buf[3]=(uint8_t)(int_threshold_val>>8);
									tx_buf[4]=int_threshold_val&0xff;
									tx_buf[5]=dmp_lp_quat;
									tx_buf[6]=dmp_gyro_cal;
									tx_buf[7]=dmp_get_gyro;
									tx_buf[8]=dmp_get_accel;
									tx_buf[9]=motion_sampling_rate;
									tx_buf[10]=dmp_mode;
									tx_buf[11]=0;
									tx_buf[12]=count_data_on;
									tx_buf[13]=(uint8_t)(cnt_threshold_val>>8);
									tx_buf[14]=cnt_threshold_val&0xff;
									len=15;
								break;
								default:
									tx_buf[0]=cmd|0x80;
								//len=1;
								break;
							}
							app_uart_send_data(len);	
              rx_len = 0;
						}
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        .rx_pin_no    = UART_RX,
        .tx_pin_no    = UART_TX,
        //.rts_pin_no   = RTS_PIN_NUMBER,
        //.cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_BUF_SIZE,
                       UART_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;
		ble_advdata_manuf_data_t manuf_specific_data;
	
    //beacon.major = SWAP(beacon_major );								
		//beacon.minor = SWAP(beacon_minor);
    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data        = (uint8_t *) &beacon;
    manuf_specific_data.data.size          = sizeof(beacon);								

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    //advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //advdata.uuids_complete.p_uuids  = m_adv_uuids;
    advdata.p_manuf_specific_data   = &manuf_specific_data;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = beacon_interval;
    options.ble_adv_fast_timeout  = beacon_period;
    //options.ble_adv_fast_timeout  = 0;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

void process_bsp_event()
{
    ret_code_t ret_code;
		m_led_on_state=false;
    switch (bsp_event)
    {
				case BSP_EVENT_BATVAL:
				SEGGER_RTT_WriteString(0, "BSP_BATVAL_KEY\r\n");
						sys_flag.show_batval=true;
					if (sys_flag.idle) {
						show_bat_level();
					}
				break;
        case BSP_EVENT_COUNTING:
				//start/stop counting
				SEGGER_RTT_WriteString(0, "BSP_COUNTING_KEY\r\n");
				//uart_tx_test();
					if (sys_flag.test_device) {
						if (sys_flag.connected) {
						//bsp_indication_set(BSP_INDICATE_IDLE);
		//SEGGER_RTT_WriteString(0, "self test button pressed\r\n");
							test_result=0;
							bsp_board_led_on(BSP_BOARD_LED_1);
							nrf_delay_ms(1000);
							bsp_board_led_off(BSP_BOARD_LED_1);							
							ret_code=mpu_sensor_init();
							if (ret_code!=0) test_result|=2;
							
							long gyro[3], accel[3];
							ret_code = mpu_run_6500_self_test(gyro, accel);
							if (ret_code!=0) test_result|=4;
							
					//test_result=2;
						sys_flag.test_end=true;
		//SEGGER_RTT_WriteString(0, "self test button pressed\r\n");
						}
					} else {
							bsp_board_led_on(BSP_BOARD_LED_1);
							nrf_delay_ms(500);
							bsp_board_led_off(BSP_BOARD_LED_1);							
						if (sys_flag.counting) {
				SEGGER_RTT_WriteString(0, "stop count\r\n");
							sys_flag.counting=false;
							if (sys_flag.charging)
								bsp_indication_set(BSP_INDICATE_CHRGING);
							else if (sys_flag.chrg_full)
								bsp_indication_set(BSP_INDICATE_CHRG_FULL);
							else
								bsp_indication_set(BSP_INDICATE_IDLE);
							save_count_data();
						} else {
				SEGGER_RTT_WriteString(0, "start count\r\n");
							//start counting
							over_threshold=false;
							sys_flag.counting=true;
							cur_count=0;
							bsp_indication_set(BSP_INDICATE_COUNTING);
						}
					}
            break; // BSP_EVENT_RECORD
        case BSP_EVENT_ADVERTISING:
				SEGGER_RTT_WriteString(0, "BSP_ADVERTISING_KEY\r\n");
				//sys_flag.battery_low=false;
					if (sys_flag.show_batval) {
							sys_flag.show_batval=false;
					} else {
					if (sys_flag.pc_connected) {
				SEGGER_RTT_WriteString(0, "pc_connected\r\n");
					} else {
						if (sys_flag.advertising) {
				SEGGER_RTT_WriteString(0, "advertising, stop it\r\n");
							sys_flag.advertising=false;
							sd_ble_gap_adv_stop();
							if (sys_flag.charging)
								bsp_indication_set(BSP_INDICATE_CHRGING);
							else if (sys_flag.chrg_full)
								bsp_indication_set(BSP_INDICATE_CHRG_FULL);
							else
								bsp_indication_set(BSP_INDICATE_IDLE);
						}
						else if (sys_flag.connected) {
							sys_flag.connected=false;
							sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
							bsp_board_led_on(BSP_BOARD_LED_0);
							nrf_delay_ms(100);
							bsp_board_led_off(BSP_BOARD_LED_0);
							if (sys_flag.charging)
								bsp_indication_set(BSP_INDICATE_CHRGING);
							else if (sys_flag.chrg_full)
								bsp_indication_set(BSP_INDICATE_CHRG_FULL);
							else
								bsp_indication_set(BSP_INDICATE_IDLE);
						}
						else	if (sys_flag.battery_low) {
								if (sys_flag.charging) {
									sys_flag.advertising=true;
									bsp_board_led_off(BSP_LED_1);
									advertising_start();
								}
							} else {
				SEGGER_RTT_WriteString(0, "start advertising\r\n");
								sys_flag.advertising=true;
								bsp_board_led_off(BSP_LED_1);
								
								
								//read_device_name();
								/*
								if (sys_flag.device_name_chged) {
										sys_flag.device_name_chged=false;
										gap_params_init();
										advertising_init();									
								}
								*/
								advertising_start();
							}
						}							
						}
            break; // BSP_EVENT_ADV

        default:
            break;
    }
}
/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
		if (sys_flag.init_f) {
		} else {
			sys_flag.bsp_event_f=true;
			bsp_event=event;
		}
}


/**@brief Function for initializing buttons and leds.
 *
 */
static void buttons_leds_init()
{
	uint32_t ret_code;
//    ret_code_t err_code;
//    bsp_event_t startup_event;

    bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS , bsp_event_handler);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    bsp_btn_ble_init(NULL, NULL);

	nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_SIMPLE(NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_drv_gpiote_out_init(TESTBAT, &config);
	nrf_drv_gpiote_out_init(PWR_CTRL, &config);
	nrf_drv_gpiote_out_config_t config1 = GPIOTE_CONFIG_OUT_SIMPLE(NRF_GPIOTE_INITIAL_VALUE_HIGH);
	
		nrf_gpio_cfg_input(SENSOR_INT, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(CHRG_DETECT, NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(CHRG_STATE, NRF_GPIO_PIN_NOPULL);

	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	nrf_drv_gpiote_in_init(SENSOR_INT, &in_config, mpu_int_event_handler);
    //SEGGER_RTT_printf(0,"error_code=%02x\r\n",err_code);
	nrf_drv_gpiote_in_event_enable(SENSOR_INT,false);
		ret_code=NRF_POWER->RESETREAS;
		//SEGGER_RTT_printf(0,"NRF_POWER->RESETREAS= %x\r\n", ret_code);
			if (ret_code&0x04) {
				sys_flag.init_f=false;
			}

}
/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing system varaibles.
 */
static void sys_var_init(void)
{
//	uint8_t i;
//sys_flag.test_device=true;
	cmd_45=0xff;
	init_count=0;
		sys_flag.idle=true;
		sys_flag.pc_connected=false;
		sys_flag.init_f=true;
		sys_flag.buttons_disabled=false;
		sys_flag.test_device=false;
		sys_flag.timer_1s_f=false;
		sys_flag.timer_1ms_f=false;
		sys_flag.mpu_int_f=false;
		sys_flag.mem_full=false;
		m_led_on_state=false;
		sys_flag.device_error=false;
		sys_flag.advertising=false;
		sys_flag.connected=false;
		sys_flag.battery_low=false;
		sys_flag.sys_data_updated=false;
//		sys_flag.dmp_recording=false;
		sys_flag.device_name_chged=false;
		sys_flag.show_batval=false;
			beacon_interval=500;		//unit 0.625ms
			beacon_period=30;				//unit sec
		beacon.device_type = 0x02;
//		beacon.device_type = 0x01;
		beacon.adv_data_length = 0x07;
		beacon.uuid[0]=0x16;
		beacon.uuid[1]=0x00;
		beacon.measured_rssi=0xb9;
}



/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
	uint8_t ret_code;

		if (sys_flag.timer_1ms_f) {
			sys_flag.timer_1ms_f=false;
			process_1ms_int();
		}
		if (sys_flag.timer_1s_f) {
			sys_flag.timer_1s_f=false;
			process_1s_int();
		}
		if (sys_flag.mpu_int_f) {
			sys_flag.mpu_int_f=false;
			process_mpu_int();
		}
		if (sys_flag.bsp_event_f) {
			sys_flag.bsp_event_f=false;
			process_bsp_event();
		}
		
		if (sys_flag.device_name_chged) {
			if (!sys_flag.advertising&&!sys_flag.connected) {
				sys_flag.device_name_chged=false;
				gap_params_init();
				advertising_init();
			}
		}
		
	
		if (m_conn_handle==BLE_CONN_HANDLE_INVALID) {
			
//			if (sys_flag.dmp_recording) {
//				sys_flag.dmp_recording=false;
//			}
			
		}


		if (cmd_45==2) {
		//power off
			cmd_45=0x82;
					sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		}
		if (cmd_45==3) {
		//reset (DFU)
			if (sys_flag.battery_low) {
				//do nothing
				cmd_45=0xff;
			} else {
				cmd_45=0x83;
				reset_prepare();
			}
		}
		if (cmd_45==10) {
			cmd_45=0;
			if (dmp_mode) {
				if (sys_flag.pc_send_data) {
					motion_interrupt_mode = 0;			
					if (!sys_flag.battery_low) {
						motion_dmp_on = 1 ;
						mpu_sensor_set_sleep_mode(0);
						set_sampling_rate(motion_sampling_rate);
						//err_code=mpu_sensor_dmp_set_fifo_rate(motion_sampling_rate);	
						mpu_sensor_dmp_enable_feature(0);
						mpu_sensor_set_dmp_state(1);			//enable DMP
						mpu_int_enable();
					}
				} else {
					motion_dmp_on = 0;
					mpu_sensor_set_dmp_state(0);			//disable DMP
					mpu_sensor_set_sleep_mode(1);
					mpu_int_disable();
				}
			} else {
				if (sys_flag.pc_send_data) {
					motion_interrupt_mode = 0;			
					mpu_int_disable();
					if (!sys_flag.battery_low) {
						motion_notification = 1;
						ret_code=start_sensor();
			//SEGGER_RTT_printf(0, "start_sensor=%x\r\n", ret_code);
						t_time_cnt=0;				//reset time counter
						app_1ms_timer_start();
					}
				} else {
					motion_notification = 0;				
					mpu_sensor_set_sleep_mode(1);
					app_1ms_timer_stop();
				}
			}
		}
		if ((!sys_flag.timer_1ms_f)&(!sys_flag.mpu_int_f)) {
			sd_app_evt_wait();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start()
{
    ret_code_t    err_code;

			if (sys_flag.device_name_chged) {
										sys_flag.device_name_chged=false;
										gap_params_init();
										advertising_init();									
			}
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		//SEGGER_RTT_printf(0, "ble_advertising_start ret code=%x",err_code);

        APP_ERROR_CHECK(err_code);

}

static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


void saadc_init(void)
{
   ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    //nrf_saadc_channel_config_t channel_config;

    saadc_config.low_power_mode = true;                                                   //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
		saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);

    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
		NRF_SAADC->CH[4].CONFIG |= 0x01000000;
    nrf_saadc_buffer_init(m_buffer_pool, 4);
	
    //err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], 1);
				nrf_gpio_pin_set(TESTBAT);		//set high to detect battery voltage
		//		nrf_delay_ms(10);
				//nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], 1);
				//nrf_drv_saadc_sample();
	
				err_code=nrf_drv_saadc_sample_convert(0,m_buffer_pool);
			//SEGGER_RTT_printf(0, "%04x  \r\n", m_buffer_pool[0]);
				battery_level_update();
				//nrf_delay_ms(10);
				nrf_gpio_pin_clear(TESTBAT);		//set low to save power	

}

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

void init_fstorage()
{
	uint32_t uicr_data;
//	uint8_t dev_name_len,dev_name[20];
//	uint32_t temp_buf[20];
//	uint8_t tt[10];

		
    uicr_data=nrf5_flash_end_addr_get();
	//NRF_LOG_INFO("uicr_data=%4x",uicr_data);
	p_end_addr=(uint32_t*)(uicr_data-1);
	p_start_addr=(uint32_t*)(uicr_data-0x2000);
	//SEGGER_RTT_printf(0,"start_addr=%4x\n",(uint32_t)p_start_addr);
	start_page_no=(uint32_t)p_start_addr;
	start_page_no=start_page_no>>12;
	//NRF_LOG_INFO("page_no=%x\n",start_page_no);
	//SEGGER_RTT_printf(0,"page_no=%x\n",start_page_no);
	read_system_data();
	
}

/**@brief Function for application main entry.
 */
int main(void)
{
//    bool erase_bonds;
    ret_code_t err_code;
		uint32_t ret_code;


    // Initialize.
    log_init();
    timers_init();
    //

    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER 
    lfclk_config();
		sys_var_init();
		twi_config();
		
		buttons_leds_init();
		//spi_config();
		init_fstorage();
		uart_init();
	
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();

    // Initialize sensors.
		sensor_init();

    // Start execution.
    application_timers_start();
		SEGGER_RTT_WriteString(0, "Init\r\n");

    saadc_init();
		/*
		if (sys_flag.device_error)
				bsp_indication_set(BSP_INDICATE_ERROR);
		else
			bsp_indication_set(BSP_INDICATE_IDLE);
		*/
			//bsp_indication_set(BSP_INDICATE_TEST);
/*		
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
		nrf_drv_wdt_enable();
*/
//advertising_start();
   // Enter main loop.
    for (;;)
    {

        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
