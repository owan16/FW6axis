
#include "SEGGER_RTT.h"

#include "ble_advs.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#define NRF_LOG_MODULE_NAME "ADVS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "device.h"

extern 	sys_flag_t sys_flag;
/*
uint8_t		system_major=0;
uint8_t		system_minor=0;
uint8_t		system_rssi=0xb9;
uint8_t		app_interval=50;	//unit 1ms
//uint16_t	adv_interval=80;	//unit 0.625ms
uint16_t	adv_interval=300;	//unit 0.625ms
uint8_t		adv_period=30;		//unit sec
*/
extern 	beacon_t   							beacon;														/**< beacon struct  */
extern 	uint16_t								beacon_major;
extern 	uint16_t								beacon_minor;
extern 	uint16_t								beacon_interval;
extern 	uint16_t								beacon_period;


/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the  SoftDevice.
 *
 * @param[in] p_advs     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_advs_t * p_advs, ble_evt_t * p_ble_evt)
{
    p_advs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
   
}


/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the  SoftDevice.
 *
 * @param[in] p_advs     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_advs_t * p_advs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_advs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the  SoftDevice.
 *
 * @param[in] p_advs     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_advs_t * p_advs, ble_evt_t * p_ble_evt)
{
	uint16_t ti;
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	if (p_evt_write->handle == p_advs->major_handles.value_handle)
    {
			ti=p_evt_write->data[0]+(uint16_t)(p_evt_write->data[1]<<8);
			if (ti!=beacon_major) {
				sys_flag.sys_data_updated=true;
				beacon_major=ti;
			}
		}
	else if (p_evt_write->handle == p_advs->minor_handles.value_handle)
    {
			ti=p_evt_write->data[0]+(uint16_t)(p_evt_write->data[1]<<8);
 			if (ti!=beacon_minor) {
				sys_flag.sys_data_updated=true;
				beacon_minor=ti;
			}

    }	else if (p_evt_write->handle == p_advs->intv_handles.value_handle)
    {
			ti=p_evt_write->data[0]+(uint16_t)(p_evt_write->data[1]<<8);
			if (ti!=beacon_interval) {
				if (ti > 50) {
				sys_flag.sys_data_updated=true;
				beacon_interval=ti;
				}
			}
    }
	else if (p_evt_write->handle == p_advs->period_handles.value_handle)
    {

		ti=p_evt_write->data[0]+(uint16_t)(p_evt_write->data[1]<<8);
		if (ti!=beacon_period) {
			if (ti>30)	{
				sys_flag.sys_data_updated=true;
				beacon_period=ti;
			}
		}
    }
    else
    {
        // Do Nothing. This event is no relevant for this service.
    }
 
}
/**@brief Authorize READ request event handler.
 *
 * @details Handles READ events from the BLE stack.
 *
 * @param[in]   p_advs  Waveform Data Service structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_advs_t * p_advs, ble_evt_t * p_ble_evt)
{
	uint8_t send_data[2];
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
		//SEGGER_RTT_printf(0, "advs handle=%x\r\n", evt_rw_auth->request.read.handle);

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
    if(evt_rw_auth->request.read.handle == p_advs->major_handles.value_handle)
    {
			send_data[0]=beacon_major&0xff;
			send_data[1]=beacon_major>>8;
			sd_ble_gatts_rw_authorize_reply(p_advs->conn_handle, &auth_reply);
    }
    else if(evt_rw_auth->request.read.handle == p_advs->minor_handles.value_handle)
    {
			send_data[0]=beacon_minor&0xff;
			send_data[1]=beacon_minor>>8;
		sd_ble_gatts_rw_authorize_reply(p_advs->conn_handle, &auth_reply);
    }
    else if(evt_rw_auth->request.read.handle == p_advs->rssi_handles.value_handle)
    {
       auth_reply.params.read.len=1;
				send_data[0]=	beacon.measured_rssi;
		sd_ble_gatts_rw_authorize_reply(p_advs->conn_handle, &auth_reply);
    }    else if(evt_rw_auth->request.read.handle == p_advs->intv_handles.value_handle)
    {
			send_data[0]=beacon_interval&0xff;
			send_data[1]=beacon_interval>>8;
		sd_ble_gatts_rw_authorize_reply(p_advs->conn_handle, &auth_reply);
    }
    else if(evt_rw_auth->request.read.handle == p_advs->period_handles.value_handle)
    {
			send_data[0]=beacon_period&0xff;
			send_data[1]=beacon_period>>8;
		sd_ble_gatts_rw_authorize_reply(p_advs->conn_handle, &auth_reply);
    }
}


void ble_advs_on_ble_evt(ble_advs_t * p_advs, ble_evt_t * p_ble_evt)
{
		//SEGGER_RTT_printf(0, "evt_id=%x\r\n", p_ble_evt->header.evt_id);
     if ((p_advs == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_advs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_advs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_advs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_advs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding Beacon all Characteristic.
 *
 * @param[in] p_advs       ADVS Data Service structure.
 * @param[in] p_advs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t all_char_add(ble_advs_t * p_advs, const ble_advs_init_t * p_advs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
//
//add major Characteristic
//
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read= 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_advs->uuid_type;
    ble_uuid.uuid = BLE_UUID_ADVS_MAJOR_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
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
    attr_char_value.max_len   = 2;

    sd_ble_gatts_characteristic_add(p_advs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_advs->major_handles);
 
    ble_uuid.uuid = BLE_UUID_ADVS_MINOR_CHAR;
    sd_ble_gatts_characteristic_add(p_advs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_advs->minor_handles);
										   

/*
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read= 1;
    char_md.char_props.write =0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_advs->uuid_type;
*/
		char_md.char_props.write =0;
/*
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 1;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;
*/
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
/*
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_ADVS_MAX_DATA_LEN;    ble_uuid.uuid = BLE_UUID_ADVS_RSSI_CHAR;
*/

     ble_uuid.uuid = BLE_UUID_ADVS_RSSI_CHAR;
   sd_ble_gatts_characteristic_add(p_advs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_advs->rssi_handles);
																					 
    char_md.char_props.write =1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    ble_uuid.uuid = BLE_UUID_ADVS_INTV_CHAR;
    sd_ble_gatts_characteristic_add(p_advs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_advs->intv_handles);
    ble_uuid.uuid = BLE_UUID_ADVS_PERIOD_CHAR;
    sd_ble_gatts_characteristic_add(p_advs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_advs->period_handles);

		return 0;
}

uint32_t ble_advs_init(ble_advs_t * p_advs, const ble_advs_init_t * p_advs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
//    ble_uuid128_t advs_base_uuid = ADVS_BASE_UUID;
//    ble_uuid_t advs_base_uuid;
//		advs_base_uuid.uuid= ADVS_BASE16_UUID;

    if ((p_advs == NULL) || (p_advs_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    memset(p_advs,0,sizeof(ble_advs_t));
    
    // Initialize the service structure.
    p_advs->conn_handle             = BLE_CONN_HANDLE_INVALID;
//		p_advs->uuid_type=BLE_UUID_TYPE_BLE;
		p_advs->uuid_type=1;

    // Add a custom base UUID.
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ADVS_SERVICE);

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_advs->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add all Characteristic.
    err_code = all_char_add(p_advs, p_advs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    return 0;

}




