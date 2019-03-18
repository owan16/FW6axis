/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_TPS)
#include "ble_tps.h"
#include <string.h>
#include "ble_srv_common.h"
#include "SEGGER_RTT.h"
const int8_t tx_power_tbl[7]={4,0,-4,-8,-12,-16,-20};

uint8_t ble_tx_power_level=1;

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_tps       TX Power Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_tps_t * p_tps, ble_evt_t * p_ble_evt)
{
    p_tps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the  SoftDevice.
 *
 * @param[in] p_tps     motion Data Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_tps_t * p_tps, ble_evt_t * p_ble_evt)
{
	uint8_t tch;
	int8_t t8;
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	if (p_evt_write->handle == p_tps->tx_power_level_handles.value_handle)
    {
			tch=p_evt_write->data[0];
		}
		if (tch<7) {
			if (tch!=ble_tx_power_level) {
				t8=tx_power_tbl[tch];
				ble_tx_power_level=tch;
				//SEGGER_RTT_printf(0, "tx power level=%d\r\n",t8);
				ble_tps_tx_power_level_set(p_tps,t8); 
			}
		}
}

/**@brief Authorize READ request event handler.
 *
 * @details Handles READ events from the BLE stack.
 *
 * @param[in]   p_tps  Waveform Data Service structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_tps_t * p_tps, ble_evt_t * p_ble_evt)
{
	uint8_t send_data[2];
    ble_gatts_evt_rw_authorize_request_t * evt_rw_auth = &p_ble_evt->evt.gatts_evt.params.authorize_request;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

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
    auth_reply.params.read.len=1;
		send_data[0]=ble_tx_power_level;
		sd_ble_gatts_rw_authorize_reply(p_tps->conn_handle, &auth_reply);
}
void ble_tps_on_ble_evt(ble_tps_t * p_tps, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_tps, p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_tps, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_tps, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding TX Power Level characteristics.
 *
 * @param[in]   p_tps        TX Power Service structure.
 * @param[in]   p_tps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_power_level_char_add(ble_tps_t            * p_tps,
                                        const ble_tps_init_t * p_tps_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TX_POWER_LEVEL_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_tps_init->tps_attr_md.read_perm;
    attr_md.write_perm = p_tps_init->tps_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    //memset(&attr_char_value, 0, sizeof (attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (int8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = (uint8_t*)&p_tps_init->initial_tx_power_level;

    return sd_ble_gatts_characteristic_add(p_tps->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_tps->tx_power_level_handles);
}


uint32_t ble_tps_init(ble_tps_t * p_tps, const ble_tps_init_t * p_tps_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TX_POWER_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_tps->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add TX Power Level characteristic
    return tx_power_level_char_add(p_tps, p_tps_init);
}


uint32_t ble_tps_tx_power_level_set(ble_tps_t * p_tps, int8_t tx_power_level)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&tx_power_level;
	
		sd_ble_gap_tx_power_set(tx_power_level);

    // Update database
    return sd_ble_gatts_value_set(p_tps->conn_handle,
                                  p_tps->tx_power_level_handles.value_handle,
                                  &gatts_value);
}
#endif // NRF_MODULE_ENABLED(BLE_TPS)
