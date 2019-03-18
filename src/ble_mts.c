
#include "SEGGER_RTT.h"
#include "math.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#define NRF_LOG_MODULE_NAME "MTS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_mts.h"
#include "device.h"
#include "nrf_delay.h"
#include "app_gpiote.h"
#include "mpu_sensor.h"
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "mpu_sensor_dmp.h"


#define MTS_BASE16_UUID					0x1600 				/**< Used vendor specific UUID. */
#define BLE_UUID_MTS_MOTION_CHAR		0x1601				/**< The UUID of the Motion Characteristic. */
#define BLE_UUID_MTS_MOTION_DMP_CHAR	0x1602				/**< The UUID of the Motion DMP Characteristic. */
#define BLE_UUID_MTS_RATE_CHAR			0x1603				/**< The UUID of the Motion Rate Characteristic. */

#define BLE_UUID_MTS_ACCEL_FSR_CHAR		0x1604				/**< The UUID of the Motion Accelerator FSR (Full Scale Range) Characteristic. */
#define BLE_UUID_MTS_GYRO_FSR_CHAR		0x1605				/**< The UUID of the Motion Gyrometer FSR Characteristic. */
#define BLE_UUID_MTS_ACCEL_INT_CHAR		0x1606				/**< The UUID of the Motion Accelerator INT Characteristic. */
#define BLE_UUID_MTS_ACCEL_INT_NTF_CHAR	0x1607				/**< The UUID of the Motion Accelerator INT Notify Characteristic. */
#define BLE_UUID_MTS_LP_QUAT_CHAR	0x1608				/**< The UUID of the Motion LP QUAT Characteristic. */
#define BLE_UUID_MTS_GYRO_CAL_CHAR		0x1609				/**< The UUID of the Gyro CAL Characteristic. */
#define BLE_UUID_MTS_GET_GYRO_CAL_CHAR		0x160A				/**< The UUID of the get Gyro CAL Characteristic. */
#define BLE_UUID_MTS_GET_ACCEL_RAW_CHAR		0x160B				/**< The UUID of the get Accel RAW Characteristic. */
#define BLE_UUID_MTS_GET_ACCU_CNT_CHAR		0x160C				/**< The UUID of the get accumulated count */

extern struct mpu_state_s mpu_st;
extern dmp_s dmp; 
extern 	sys_flag_t sys_flag;
extern 	uint16_t			cur_count;
extern 	uint16_t			total_count;
extern 	bool			over_threshold;


//extern uint8_t       motion_sampling_time_cnt;		//motion sampling time count for each rate (unit is 1ms)
extern uint16_t       t_time_cnt;						//time count for 1ms
//extern mpu_sensor_init_t mpu_fsr_val;

extern uint8_t			m_buffer[];
extern uint16_t			motion_sampling_time_cnt;		//motion sampling time count for each rate (unit is 1ms)
extern bool				sampling_data_on;
extern ble_mts_t	m_mts;

//static int twi_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t tmp);
//int twi_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t * tmp);

//static struct app_motion_s app_motion = {0};
/* Motion data */          	
uint8_t	 motion_notification          = 0;
uint8_t	 motion_dmp_on          			= 0;
//static uint8_t	 motion_timeout_flag          = 0;
//static uint16_t  motion_interrupt_thresh			= 5;										/**< Motion threshold in mg. */
uint8_t	 motion_interrupt_mode        = 0;

//static uint8_t	 motion_dmp_send_prescaler    = 0;
uint8_t	 			motion_sampling_rate;
uint8_t				accel_fsr_val;	
uint8_t				gyro_fsr_val;
uint16_t			int_threshold_val;	
uint8_t 			dmp_lp_quat;
uint8_t 			dmp_gyro_cal;
uint8_t 			dmp_get_gyro;
uint8_t 			dmp_get_accel;
uint8_t				dmp_mode;
uint8_t				count_data_on;
uint16_t			cnt_threshold_val;

void				cal_count(void);
void app_1ms_timer_start(void);
void app_1ms_timer_stop(void);

uint8_t twi_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data, uint8_t len);
uint8_t twi_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t tmp);

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the SoftDevice.
 *
 * @param[in] p_mts     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_mts_t * p_mts, ble_evt_t * p_ble_evt)
{
    p_mts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//		sys_flag.sys_data_updated=false;
   
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the SoftDevice.
 *
 * @param[in] p_mts     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_mts_t * p_mts, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_mts->conn_handle = BLE_CONN_HANDLE_INVALID;
				motion_notification = 0;				
				motion_interrupt_mode = 0;			
				mpu_int_disable();
				mpu_sensor_set_sleep_mode(1);
				motion_dmp_on = 0;
				mpu_sensor_set_dmp_state(0);			//disable DMP
}

void set_sampling_rate(uint8_t rate)
{
	uint16_t sample_rate;
	uint8_t ret_code;
			motion_sampling_rate=rate;
			switch (rate)
			{
				case 0:
					motion_sampling_time_cnt=100;	//10hz
					sample_rate=10;
				break;
				case 1:
					motion_sampling_time_cnt=50;	//20hz
					sample_rate=20;
				break;
				case 2:
					motion_sampling_time_cnt=20;	//50hz
					sample_rate=50;
				break;
				case 7:
					motion_sampling_time_cnt=1000;	//1hz
					sample_rate=1;
				break;
				case 8:
					motion_sampling_time_cnt=200;	//5hz
					sample_rate=5;
				break;
				case 9:
					motion_sampling_time_cnt=25;	//40hz
					sample_rate=40;
				break;
			}
			/*
			if (motion_notification) {
					if (sample_rate>20) {
						motion_sampling_time_cnt=50;	//20hz
						sample_rate=20;
					}
			} else
			*/
			if (motion_dmp_on) {
					sample_rate=100;
					if (sample_rate>20) {
						motion_sampling_time_cnt=50;	//20hz
					}
			} else
			if (dmp_mode) {
				sample_rate=100;
			}
			ret_code=mpu_sensor_set_sample_rate(sample_rate);
			//SEGGER_RTT_printf(0, "sample_rate=%d time cnt=%d \r\n", sample_rate,motion_sampling_time_cnt);
}

//bool print_mts=true;
bool print_mts=false;
/**@brief Function for encoding a RSCS Measurement.
 *
 * @param[in]   p_rscs              Running Speed and Cadence Service structure.
 * @param[in]   p_rsc_measurement   Measurement to be encoded.
 * @param[out]  p_encoded_buffer    Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t motion_measurement_encode(ble_mts_t * p_mts, ble_srv_motion_meas_t * p_motion_measurement,
										uint8_t * p_encoded_buffer)
{
    uint8_t len  = 0;
//	uint8_t i;
	uint16_t accn;
	uint32_t t_count;
	
		if (motion_dmp_on)
		{		
			memcpy(&p_encoded_buffer[len] , m_buffer, dmp.packet_length);
				len=dmp.packet_length;
			
		}else
		if (motion_interrupt_mode)
		{
				memcpy(&p_encoded_buffer[len] , p_motion_measurement->inst_accel, 6);
				len+=6;
		} else
		{
				memcpy(&p_encoded_buffer[len] , p_motion_measurement->inst_accel, 6);
				len+=6;
				memcpy(&p_encoded_buffer[len] , p_motion_measurement->inst_gyro, 6);
				len+=6;
			if (count_data_on) {
				cal_count();
				p_encoded_buffer[len++]=cur_count>>8;
				p_encoded_buffer[len++]=cur_count&0xFF;
			}
		}
		if (print_mts) {
			//print_mts=false;
		for (uint8_t i=0;i<len;i++) {
			SEGGER_RTT_printf(0, "%02x ",p_encoded_buffer[i]);
		}
		SEGGER_RTT_WriteString(0, "\r\n");
		}
    return len;
}
uint32_t ble_srv_motion_measurement_send(ble_mts_t * p_mts, ble_srv_motion_meas_t * p_measurement)
{
    uint32_t err_code;
//	uint8_t i;

    // Send value if connected and notifying.
    if ((p_mts->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        uint16_t               hvx_len;
			  uint8_t                hvx_data[BLE_FRAME_CHAR_LEN_MAX];
				//uint8_t                hvx_dmp_data[BLE_FRAME_MEAS_DMP_CHAR_LEN];
				ble_gatts_hvx_params_t 				hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

				// Setting the notification handle of specfic mode  
		if (motion_dmp_on)
		{
				hvx_params.handle   = p_mts->measure_dmp_handles.value_handle;
		} else
		if (motion_interrupt_mode)
		{
				hvx_params.handle   = p_mts->accel_int_ntf_handles.value_handle;
		} else
		{
				hvx_params.handle   = p_mts->measure_handles.value_handle;
		}
				hvx_len = motion_measurement_encode(p_mts, p_measurement, hvx_data);
				hvx_params.p_data   = hvx_data;
				hvx_params.p_len    = &hvx_len;
			  hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset   = 0;
				err_code = sd_ble_gatts_hvx( p_mts->conn_handle, &hvx_params);
    }
		else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_mts     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_mts_t * p_mts, ble_evt_t * p_ble_evt)
{
//	uint8_t tch;
		ble_srv_motion_evt_t evt;
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
//		uint8_t * p_val = p_evt_write->data;
//    NRF_LOG_INFO("MTS Write handle=%x\r\n",p_evt_write->handle);
   if (
        (p_evt_write->handle == p_mts->measure_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
       if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
                evt.evt_type = BLE_MPU_EVT_NOTIFICATION_ENABLED;
		}
		else
		{
                evt.evt_type = BLE_MPU_EVT_NOTIFICATION_DISABLED;
		}
            
		p_mts->evt_handler(p_mts, &evt);
    } else
   if (
        (p_evt_write->handle == p_mts->measure_dmp_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
			if (ble_srv_is_notification_enabled(p_evt_write->data))
			{
                evt.evt_type = BLE_MPU_EVT_NOTIFICATION_DMP_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_MPU_EVT_NOTIFICATION_DMP_DISABLED;
            }
            
            p_mts->evt_handler(p_mts, &evt);
    } else
   if (
        (p_evt_write->handle == p_mts->accel_int_ntf_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
				if (ble_srv_is_notification_enabled(p_evt_write->data))
				{
                evt.evt_type = BLE_MPU_EVT_INT_NOTIFICATION_ENABLED;
            } else {
                evt.evt_type = BLE_MPU_EVT_INT_NOTIFICATION_DISABLED;
            }
            p_mts->evt_handler(p_mts, &evt);
    }
    else
    {
        // Do Nothing. This event is no relevant for this service.
    }
 
}
void mpu_int_enable(void)
{
		//uint8_t err_code;
		nrf_drv_gpiote_in_event_enable(SENSOR_INT,true); 
}

void mpu_int_disable(void)
{
		//uint32_t err_code;
		nrf_drv_gpiote_in_event_enable(SENSOR_INT,false); 
}
static void motion_interrupt_services_enable(void)
{
	
		mpu_sensor_fast_accel_init();
	
	
		nrf_delay_ms(4);
   //err_code=twi_read_bytes(0x68, 0x19, reg_data,8);
		mpu_int_enable();

}

/**@brief Function for handle the event from motion ble service.
 */
static void ble_srv_motion_evt_handler(ble_mts_t * p_mts, ble_srv_motion_evt_t * p_evt)
{
	uint8_t ret_code;
		if (p_evt->evt_type == BLE_MPU_EVT_NOTIFICATION_ENABLED)
		{
				motion_interrupt_mode = 0;			
				mpu_int_disable();
			if (!sys_flag.battery_low) {
				motion_notification = 1;
				if (motion_dmp_on == 0) {
						ret_code=start_sensor();
			//SEGGER_RTT_printf(0, "start_sensor=%x\r\n", ret_code);
						t_time_cnt=0;				//reset time counter
						app_1ms_timer_start();
						over_threshold = false;
				}
			}
		}
		else if (p_evt->evt_type == BLE_MPU_EVT_NOTIFICATION_DISABLED)
		{
				motion_notification = 0;				
				mpu_sensor_set_sleep_mode(1);
				app_1ms_timer_stop();
				
				//nrf_drv_twi_disable(&m_app_twi);
				//nrf_drv_twi_uninit(&m_app_twi);
		}
		else if (p_evt->evt_type == BLE_MPU_EVT_NOTIFICATION_DMP_ENABLED)
		{
				motion_interrupt_mode = 0;			
			if (!sys_flag.battery_low) {
				uint8_t err_code;
			  motion_dmp_on = 1 ;
				mpu_sensor_set_sleep_mode(0);
				set_sampling_rate(motion_sampling_rate);
				//err_code=mpu_sensor_dmp_set_fifo_rate(motion_sampling_rate);	
				mpu_sensor_dmp_enable_feature(0);
				mpu_sensor_set_dmp_state(1);			//enable DMP
				mpu_int_enable();
						over_threshold = false;
			}
		}
		else if (p_evt->evt_type == BLE_MPU_EVT_NOTIFICATION_DMP_DISABLED)
		{
				motion_dmp_on = 0;
				mpu_sensor_set_dmp_state(0);			//disable DMP
				mpu_sensor_set_sleep_mode(1);
				mpu_int_disable();
			
		} else if (p_evt->evt_type == BLE_MPU_EVT_INT_NOTIFICATION_ENABLED)	{
			
			if (!sys_flag.battery_low) {
				if (motion_dmp_on == 0)	{
					mpu_sensor_set_sleep_mode(0);
					motion_interrupt_services_enable();
					motion_interrupt_mode = 1;
				}
			}
		} else if (p_evt->evt_type == BLE_MPU_EVT_INT_NOTIFICATION_DISABLED)	{
			
			if (motion_dmp_on == 0)	{
				motion_interrupt_mode = 0;			
				mpu_int_disable();
				mpu_sensor_set_sleep_mode(1);
			
			}
		}
}
/**@brief Authorize READ request event handler.
 *
 * @details Handles READ events from the BLE stack.
 *
 * @param[in]   p_mts Data Service structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_mts_t * p_mts, ble_evt_t * p_ble_evt)
{
	uint8_t send_data[2];
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
		//SEGGER_RTT_printf(0, "mts handle=%x\r\n", evt_rw_auth->request.read.handle);

    if (evt_rw_auth->type != BLE_GATTS_AUTHORIZE_TYPE_READ)
    {
        // Unexpected operation
        return;
    }
    auth_reply.type=BLE_GATTS_AUTHORIZE_TYPE_READ;
    auth_reply.params.read.gatt_status=BLE_GATT_STATUS_SUCCESS;
    auth_reply.params.read.offset=0;
		auth_reply.params.read.update=true;
		auth_reply.params.read.p_data=send_data;
       auth_reply.params.read.len=2;
    if(evt_rw_auth->request.read.handle == p_mts->cnt_handles.value_handle)
    {
			send_data[0]=total_count>>8;
			send_data[1]=total_count&0xFF;
			sd_ble_gatts_rw_authorize_reply(p_mts->conn_handle, &auth_reply);
    }
}

void ble_mts_on_ble_evt(ble_mts_t * p_mts, ble_evt_t * p_ble_evt)
{
		//SEGGER_RTT_printf(0, "evt_id=%x\r\n", p_ble_evt->header.evt_id);
     if ((p_mts == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_mts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_mts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_mts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_mts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding motion all Characteristic.
 *
 * @param[in] p_mts       MTS Data Service structure.
 * @param[in] p_mts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t motion_char_add(ble_mts_t * p_mts, const ble_mts_init_t * p_mts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
//
//add motion measure Characteristic
//
    memset(&char_md, 0, sizeof(char_md));

    //char_md.char_props.read= 1;
    char_md.char_props.read= 0;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_mts->uuid_type;
    ble_uuid.uuid = BLE_UUID_MTS_MOTION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    //attr_md.rd_auth = 1;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 18;

    sd_ble_gatts_characteristic_add(p_mts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mts->measure_handles);
 //
//add motion measure DMP Characteristic

    attr_char_value.max_len   = 28;
    ble_uuid.uuid = BLE_UUID_MTS_MOTION_DMP_CHAR;
    sd_ble_gatts_characteristic_add(p_mts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mts->measure_dmp_handles);
//
//add motion ACC INT Notify Characteristic
//
    attr_char_value.max_len   = 6;
    ble_uuid.uuid = BLE_UUID_MTS_ACCEL_INT_NTF_CHAR;
    sd_ble_gatts_characteristic_add(p_mts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mts->accel_int_ntf_handles);
//
//add read accumulated CNT Characteristic
//

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    char_md.char_props.read= 1;
    char_md.char_props.notify = 0;
    attr_md.rd_auth = 1;
    attr_char_value.max_len   = 2;
    ble_uuid.uuid = BLE_UUID_MTS_GET_ACCU_CNT_CHAR;
    sd_ble_gatts_characteristic_add(p_mts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mts->cnt_handles);


		return 0;
}

uint32_t ble_mts_init(ble_mts_t * p_mts, const ble_mts_init_t * p_mts_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
//    ble_uuid128_t mts_base_uuid = {MTS_BASE_UUID};
//    ble_uuid_t mts_base_uuid;
		//mts_base_uuid.uuid= MTS_BASE16_UUID;

    if ((p_mts == NULL) || (p_mts_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    memset(p_mts,0,sizeof(ble_mts_t));
    
    // Initialize the service structure.
		p_mts->conn_handle = BLE_CONN_HANDLE_INVALID;
		p_mts->evt_handler = ble_srv_motion_evt_handler;
		p_mts->uuid_type=1;


    // Add a custom base UUID.

//    err_code = sd_ble_uuid_vs_add(&mts_base_uuid, &p_mts->uuid_type);
//		SEGGER_RTT_printf(0, "sd_ble_uuid_vs_add %02x\r\n",err_code);
/*
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
*/
    ble_uuid.type = p_mts->uuid_type;
		ble_uuid.uuid = BLE_UUID_MTS_SERVICE;
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MTS_SERVICE);
		//SEGGER_RTT_printf(0, "ble_uuid.type %02x\r\n",ble_uuid.type);

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_mts->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the motion Characteristic.
    err_code = motion_char_add(p_mts, p_mts_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
        return 0;

}




