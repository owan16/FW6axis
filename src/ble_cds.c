
#include "SEGGER_RTT.h"

#include "ble_cds.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#define NRF_LOG_MODULE_NAME "CDS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "device.h"
#include "mpu_sensor.h"
#include "mpu_sensor_dmp.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "bsp.h"

//#define PRINT_FLASH_DATA 1

#define BLE_UUID_CDS_TX_CHAR		0xFFF6				/**< The UUID of the TX  Characteristic. */
#define BLE_UUID_CDS_RX_CHAR		0xFFF7				/**< The UUID of the RX Characteristic. */



//#define CDS_BASE_UUID                  {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x6D, 0xA5, 0xE5, 0x11, 0xCE, 0x13, 0x00, 0x00, 0x7A, 0x19}} /**< Used vendor specific UUID. */
#define CDS_BASE16_UUID                  0xFFF0 /**< Used vendor specific UUID. */
extern uint16_t       t_time_cnt;						//time count for 1ms
extern nrf_drv_wdt_channel_id m_channel_id;
extern uint8_t 		m_rx_buf[];
extern 	sys_flag_t sys_flag;
extern	uint8_t	 			motion_sampling_rate;
extern uint16_t		max_bat_val;
//extern  nrf_saadc_value_t     m_buffer_pool[1];
extern uint16_t 		bat_full_cnt;
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


extern nrf_saadc_value_t     m_buffer_pool[];

extern uint16_t		cur_bat_val;
extern uint16_t		max_chrg_cnt;
extern uint16_t		max_chrg_val;
extern uint16_t		cur_chrg_cnt;
extern uint16_t		cur_chrg_val;
extern uint8_t		chrg_state;
extern uint8_t		test_result;
extern uint8_t		device_name[];
extern uint8_t		device_name_len;
extern uint8_t 		blink_led_color;
extern 	uint16_t			total_count;
extern uint8_t accel_fsr_tbl[];

uint8_t major_ver_no=1;
uint8_t minor_ver_no=1;
uint8_t minor1_ver_no=1;
bool cmd_46_nodata;
bool cds_enabled;
uint8_t cmd_45;

#define CDS_BUF_LEN 128
uint8_t tx_buf[CDS_BUF_LEN];
uint8_t rx_buf[CDS_BUF_LEN];
uint8_t test_dev_cnt;



/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the  SoftDevice.
 *
 * @param[in] p_cds     communication Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_cds_t * p_cds, ble_evt_t * p_ble_evt)
{
    p_cds->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
   
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the  SoftDevice.
 *
 * @param[in] p_cds     communication Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_cds_t * p_cds, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cds->conn_handle = BLE_CONN_HANDLE_INVALID;
		sys_flag.test_device=false;
}


uint16_t cvt_to_uint(uint8_t hb, uint8_t lb) 
{
	uint16_t result;
	uint8_t tch,tch1;
	tch=(hb>>4)&0x0f;
	tch1=hb&0x0f;
	result=(uint16_t)(tch<<12)+(uint16_t)(tch1<<8);
	tch=(lb>>4)&0x0f;
	tch1=lb&0x0f;
	result+=(uint16_t)(tch<<4)+tch1;
	SEGGER_RTT_printf(0, "result=%d\r\n",result);
	return result;
}
/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_cds     communication Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_cds_t * p_cds, ble_evt_t * p_ble_evt)
{
	bool err_f;
	uint16_t len;
	uint8_t hb,lb;
	uint8_t ret_code;
	uint8_t i,tch,cmd,cmd_len;
	uint16_t tint,max_val;
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
//    NRF_LOG_INFO("CDS Write handle=%x\r\n",p_evt_write->handle);
   if (
        (p_evt_write->handle == p_cds->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
       if (ble_srv_is_notification_enabled(p_evt_write->data))
       {
                cds_enabled=true;
				}
				else
				{
                cds_enabled=false;
				}
			}
    if (p_evt_write->handle == p_cds->tx_handles.value_handle)
    {
			SEGGER_RTT_printf(0, "write len=%x\r\n",p_evt_write->len);
//    ble_gatts_rw_authorize_reply_params_t auth_reply;
			memset(&tx_buf, 0, sizeof(tx_buf));
			cmd=p_evt_write->data[0];
			tx_buf[0]=cmd;
			len=1;
			err_f=false;
			SEGGER_RTT_printf(0, "cmd=%02x\r\n",cmd);
			switch (cmd) {
				case 0x36:
				//reset accumulated count
					total_count=0;
				break;
				case 0x37:
				//blinking leds
					if (p_evt_write->data[1]==1) {
						sys_flag.start_blink_led=true;
					} else {
						sys_flag.stop_blink_led=true;
					}
					blink_led_color=p_evt_write->data[2];
				break;
				case 0x42:
				//set device name
					cmd_len=p_evt_write->len;
					if (cmd_len>13) cmd_len=13;
					device_name_len=cmd_len-1;
					for (i=0;i<device_name_len;i++) {
						tch=p_evt_write->data[i+1];
						device_name[i]=tch;
						if (tch==0x0) {
							cmd_len--;
							break;
						}
					}
					sys_flag.sys_data_updated=true;
					sys_flag.device_name_chged=true;
				break;
				case 0x45:
				//set configuration
						tch=p_evt_write->data[1];
						if (tch <4){
								if (accel_fsr_val!=tch) {
								sys_flag.sys_data_updated=true;
								accel_fsr_val=tch;
								}
						} else {
		SEGGER_RTT_WriteString(0, "fail 1\r\n");
								err_f=true;
						}
						tch=p_evt_write->data[2];
						if (tch <4){
								if (gyro_fsr_val!=tch) {
								sys_flag.sys_data_updated=true;
								gyro_fsr_val=tch;	
								}
						} else {
		SEGGER_RTT_WriteString(0, "fail 2\r\n");
								err_f=true;
						}
						
						hb=p_evt_write->data[3];
						lb=p_evt_write->data[4];
						tint=cvt_to_uint(hb, lb);
						if (tint <1020) {
							if (int_threshold_val != tint) {
								sys_flag.sys_data_updated=true;
								int_threshold_val = tint;		
							}
							} else {
		SEGGER_RTT_WriteString(0, "fail 3\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[5];		
						if ((tch==1)||(tch==2)) {
							if (dmp_lp_quat!=tch) {
								sys_flag.sys_data_updated=true;
								dmp_lp_quat=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 5\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[6];		
						if (tch<2) {
							if (dmp_gyro_cal!=tch) {
								sys_flag.sys_data_updated=true;
								dmp_gyro_cal=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 6\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[7];		
						if (tch<2) {
							if (dmp_get_gyro!=tch){
								sys_flag.sys_data_updated=true;
								dmp_get_gyro=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 7\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[8];		
						if (tch<2) {
							if (dmp_get_accel!=tch) {
								sys_flag.sys_data_updated=true;
								dmp_get_accel=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 8\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[9];
						if ((tch <3)||((tch>=7)&(tch<=9))) {
							if (motion_sampling_rate != tch){
								sys_flag.sys_data_updated=true;
								motion_sampling_rate = tch;	
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 9\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[10];		
						if (tch<2) {
							if (dmp_mode!=tch) {
								sys_flag.sys_data_updated=true;
								dmp_mode=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 10\r\n");
								err_f=true;
						}
						tch = p_evt_write->data[11];		
						if ((tch>0)&&(tch<4)) {
							cmd_45=tch;
						} else {
								cmd=0xff;
						}
						tch = p_evt_write->data[12];		
						if (tch<2) {
							if (count_data_on!=tch) {
								sys_flag.sys_data_updated=true;
								count_data_on=tch;
							}
						} else {
		SEGGER_RTT_WriteString(0, "fail 12\r\n");
								err_f=true;
						}
						hb=p_evt_write->data[13];
						lb=p_evt_write->data[14];
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
						//len=0;
							tx_buf[len++]=major_ver_no;
							tx_buf[len++]=minor_ver_no;
							tx_buf[len++]=minor1_ver_no;
				
					break;
				case 0x49:


					 SEGGER_RTT_printf(0,"on_write,case 0x49:\r\n");
     //#if H176V2_ENABLE
            /*                        
      tx_buf[1]=accel_fsr_val;
      tx_buf[2]=gyro_fsr_val;
      tx_buf[3]=0x00;
      tx_buf[4]=(uint8_t)(int_threshold_val>>8);
      tx_buf[5]=int_threshold_val&0xff;

      
      tx_buf[6]=dmp_lp_quat;
      tx_buf[7]=dmp_gyro_cal;
      tx_buf[8]=dmp_get_gyro;
      tx_buf[9]=dmp_get_accel;
      tx_buf[10]=motion_sampling_rate;
      tx_buf[11]=dmp_mode;
	  //Owan open it
     // tx_buf[12]=count_data_on;
      //tx_buf[13]=(uint8_t)(cnt_threshold_val>>8);
      //tx_buf[14]=cnt_threshold_val&0xff;
	  //***************************
      len=15;

	  */
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
/*
				
					tx_buf[3]=0;
					tx_buf[4]=(uint8_t)(int_threshold_val>>8);
					tx_buf[5]=int_threshold_val&0xff;
					tx_buf[6]=dmp_lp_quat;
					tx_buf[7]=dmp_gyro_cal;
					tx_buf[8]=dmp_get_gyro;
					tx_buf[9]=dmp_get_accel;
					tx_buf[10]=motion_sampling_rate;
					tx_buf[11]=0;
					len=12;
				
					tx_buf[12]=count_data_on;
					tx_buf[13]=(uint8_t)(cnt_threshold_val>>8);
					tx_buf[14]=cnt_threshold_val&0xff;
					len=15;
				*/
					break;
				default:
					tx_buf[0]=cmd|0x80;
					//len=1;
				break;
				case 0x9:
				//self test
						sys_flag.test_device=true;
						sys_flag.test_end=false;
						test_dev_cnt=0;
						//len=1;
						//return;
					break;
				case 0x10:
						sys_flag.chk_bat=!sys_flag.chk_bat;
					break;
				case 0x11:
						tch = p_evt_write->data[1];		
						if ((tch <3)||((tch>=7)&(tch<=9))) {
								motion_sampling_rate=tch;
		//SEGGER_RTT_printf(0, "motion_sampling_rate=%x\r\n",motion_sampling_rate);
						}
					break;
				case 0x12:
						tch = p_evt_write->data[1];		
						if (tch<2) {
								dmp_mode=tch;
						}
					break;
				case 0x92:
					//power off
						cmd_45=2;
					break;
				case 0x93:
					//reset
						cmd_45=3;
					break;
			
		}
		
		SEGGER_RTT_printf(0, "len=%x\r\n",len);
		for (i=0;i<len;i++) {
			SEGGER_RTT_printf(0, "%02x ",tx_buf[i]);
		}
		SEGGER_RTT_WriteString(0, "\r\n");
		
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
						uint16_t * p_length;
						//uint16_t len;
						//len=CDS_BUF_LEN;
						//len=10;
						p_length=&len;
            hvx_params.handle = p_cds->rx_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            //hvx_params.offset = 0;
            hvx_params.p_len  = p_length;
            hvx_params.p_data = tx_buf;
            sd_ble_gatts_hvx(p_cds->conn_handle, &hvx_params);

    }
    else
    {
        // Do Nothing. This event is no relevant for this service.
    }
 
}
void		ble_srv_bat_send(ble_cds_t *p_cds)
{
	uint16_t len;
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
						uint16_t * p_length;
						tx_buf[0]=m_buffer_pool[0]>>8;
						tx_buf[1]=m_buffer_pool[0]&0xff;
						tx_buf[2]=max_bat_val>>8;
						tx_buf[3]=max_bat_val&0xff;
						tx_buf[4]=chrg_state;
						tx_buf[5]=max_chrg_val>>8;
						tx_buf[6]=max_chrg_val&0xff;
						tx_buf[7]=max_chrg_cnt>>8;
						tx_buf[8]=max_chrg_cnt&0xff;
						tx_buf[9]=cur_chrg_cnt>>8;
						tx_buf[10]=cur_chrg_cnt&0xff;
						len=11;
						p_length=&len;
            hvx_params.handle = p_cds->rx_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.p_len  = p_length;
            hvx_params.p_data = tx_buf;
            sd_ble_gatts_hvx(p_cds->conn_handle, &hvx_params);
	
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_HVN_TX_COMPLETE event from the SoftDevice.
 *
 * @param[in] p_cds     communication Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_notify(ble_cds_t * p_cds, ble_evt_t * p_ble_evt)
{

		//ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		//SEGGER_RTT_WriteString(0, "cds on_notify\r\n");
		ble_gatts_hvx_params_t hvx_params;
		memset(&hvx_params, 0, sizeof(hvx_params));
		uint16_t * p_length;
		uint16_t len;
		len=0;
	if (tx_buf[0]==0x9) {
			if (test_dev_cnt==1) return;
			if (sys_flag.test_end) {
				test_dev_cnt=1;
				sys_flag.test_device=false;
				tx_buf[1]=test_result;
				len=2;
				SEGGER_RTT_printf(0, "self test_result=%02x\r\n",test_result);
			} else {
				len=1;
			}
	}
		p_length=&len;
		hvx_params.handle = p_cds->rx_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		//hvx_params.offset = 0;
		hvx_params.p_len  = p_length;
		hvx_params.p_data = tx_buf;
		sd_ble_gatts_hvx(p_cds->conn_handle, &hvx_params);
	
}


void ble_cds_on_ble_evt(ble_cds_t * p_cds, ble_evt_t * p_ble_evt)
{
		//SEGGER_RTT_printf(0, "ble_cds_on_ble_evt=%02x\r\n",p_ble_evt->header.evt_id);
     if ((p_cds == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cds, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cds, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            //on_rw_authorize_request(p_cds, p_ble_evt);
            break;
				
        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
						if (tx_buf[0]==0x09)
							on_notify(p_cds, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding  all Characteristic.
 *
 * @param[in] p_cds       CDS Data Service structure.
 * @param[in] p_cds_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cds_char_add(ble_cds_t * p_cds,const ble_cds_init_t * p_cds_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
//
//add  TX Characteristic
//
    memset(&char_md, 0, sizeof(char_md));

    //char_md.char_props.read= 1;
    char_md.char_props.notify= 0;
    char_md.char_props.write= 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_cds->uuid_type;
    ble_uuid.uuid = BLE_UUID_CDS_TX_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth = 0;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_CDS_MAX_DATA_LEN;

    sd_ble_gatts_characteristic_add(p_cds->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cds->tx_handles);
 //
//add RX Characteristic
//
/*
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read= 1;
//    char_md.char_props.write= 0;
    char_md.char_props.notify= 1;

//    char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;


    ble_uuid.type = p_cds->uuid_type;


    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_CDS_MAX_DATA_LEN;
*/

    char_md.char_props.read= 0;
    char_md.char_props.write= 0;
    char_md.char_props.notify= 1;
//		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
    ble_uuid.uuid = BLE_UUID_CDS_RX_CHAR;

    sd_ble_gatts_characteristic_add(p_cds->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cds->rx_handles);

		return 0;
}

uint32_t ble_cds_init(ble_cds_t * p_cds, const ble_cds_init_t * p_cds_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
//    ble_uuid128_t cds_base_uuid = CDS_BASE_UUID;
//    ble_uuid_t cds_base_uuid;
//		cds_base_uuid.uuid= CDS_BASE16_UUID;

    if ((p_cds == NULL) || (p_cds_init == NULL))
    {
        return NRF_ERROR_NULL;
    }
    memset(p_cds,0,sizeof(ble_cds_t));
    
    // Initialize the service structure.
    p_cds->conn_handle             = BLE_CONN_HANDLE_INVALID;
//		p_cds->uuid_type=BLE_UUID_TYPE_BLE;
		p_cds->uuid_type=1;

    // Add a custom base UUID.
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CDS_SERVICE);

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cds->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the CDS Characteristic.
//    err_code = cds_char_add(p_cds);
    err_code = cds_char_add(p_cds, p_cds_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
        return 0;

}




