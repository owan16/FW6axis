/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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

#include "bsp_btn_ble.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ble.h"
#include "bsp.h"
#define NRF_LOG_MODULE_NAME "BSP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define BTN_ID_ADV					0  /**< ID of button used to start advertising */
#define BTN_ID_BATVAL				0  /**< ID of button used to show battery level */
#define BTN_ID_COUNT				1  /**< ID of button used to  start/stop count */
//#define BTN_ID_POWER				0  /**< ID of button used to turn off power */
//
#define BTN_ACTION_ADV				BSP_BUTTON_ACTION_RELEASE		/**< Button action used to start advertising */
#define BTN_ACTION_COUNT			BSP_BUTTON_ACTION_RELEASE		/**< Button action used to start/stop count */
//#define BTN_ACTION_SYSOFF			BSP_BUTTON_ACTION_LONG_PUSH		/**< Button action used to turn off power on long press. */
#define BTN_ACTION_BATVAL			BSP_BUTTON_ACTION_LONG_PUSH		/**< Button action used to show battery level. */



/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS.
 */
#define RETURN_ON_ERROR(err_code)  \
do                                 \
{                                  \
    if ((err_code) != NRF_SUCCESS) \
    {                              \
        return err_code;           \
    }                              \
}                                  \
while (0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_INVALID_PARAM.
 */
#define RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_INVALID_PARAM)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will return from the current function if err_code
 *        is not NRF_SUCCESS or NRF_ERROR_NOT_SUPPORTED.
 */
#define RETURN_ON_ERROR_NOT_NOT_SUPPORTED(err_code)                             \
do                                                                              \
{                                                                               \
    if (((err_code) != NRF_SUCCESS) && ((err_code) != NRF_ERROR_NOT_SUPPORTED)) \
    {                                                                           \
        return err_code;                                                        \
    }                                                                           \
}                                                                               \
while (0)


/**@brief This macro will call the registered error handler if err_code
 *        is not NRF_SUCCESS and the error handler is not NULL.
 */
#define CALL_HANDLER_ON_ERROR(err_code)                           \
do                                                                \
{                                                                 \
    if (((err_code) != NRF_SUCCESS) && (m_error_handler != NULL)) \
    {                                                             \
        m_error_handler(err_code);                                \
    }                                                             \
}                                                                 \
while (0)


//static bsp_btn_ble_error_handler_t m_error_handler = NULL; /**< Error handler registered by the user. */
//static uint32_t                    m_num_connections = 0;  /**< Number of connections the device is currently in. */


/**@brief Function for configuring the buttons 
 *
 * @retval NRF_SUCCESS  Configured successfully.
 * @return A propagated error code.
 */
static uint32_t buttons_configure()
{
    uint32_t err_code;

    err_code = bsp_event_to_button_action_assign(BTN_ID_ADV,
                                                 BTN_ACTION_ADV,
                                                 BSP_EVENT_ADVERTISING);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
    err_code = bsp_event_to_button_action_assign(BTN_ID_COUNT,
                                                 BTN_ACTION_COUNT,
                                                 BSP_EVENT_COUNTING);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);
	
    err_code = bsp_event_to_button_action_assign(BTN_ID_BATVAL,
                                                 BTN_ACTION_BATVAL,
                                                 BSP_EVENT_BATVAL);
    RETURN_ON_ERROR_NOT_INVALID_PARAM(err_code);

	
    return NRF_SUCCESS;
}



void bsp_btn_ble_on_ble_evt(ble_evt_t * p_ble_evt)
{
    //uint32_t err_code;

}


uint32_t bsp_btn_ble_init(bsp_btn_ble_error_handler_t error_handler, bsp_event_t * p_startup_bsp_evt)
{
    uint32_t err_code = NRF_SUCCESS;

	err_code = buttons_configure();

    return err_code;
}
