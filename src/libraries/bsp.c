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
#include "SEGGER_RTT.h"
#include "bsp.h"
#include <stddef.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "bsp_config.h"
#include "boards.h"
#include "device.h"

#ifndef BSP_SIMPLE
#include "app_timer.h"
#endif // BSP_SIMPLE
#include "app_button.h"
#define NRF_LOG_MODULE_NAME "BSP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

extern sys_flag_t sys_flag;
bsp_indication_t m_stable_state        = BSP_INDICATE_IDLE;
static bool             m_leds_clear          = false;
static uint32_t         m_indication_type     = 0;
//static bool             m_alert_on            = false;
bool							m_led_on_state		= false;
#define LEDS_TIMER_INTERVAL				16384								//500ms
APP_TIMER_DEF(m_leds_timer_id);
//APP_TIMER_DEF(m_alert_timer_id);

static bsp_event_callback_t   m_registered_callback         = NULL;
static bsp_button_event_cfg_t m_events_list[BUTTONS_NUMBER] = {{BSP_EVENT_NOTHING, BSP_EVENT_NOTHING}};
APP_TIMER_DEF(m_button_timer_id);
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action);

static const app_button_cfg_t app_buttons[BUTTONS_NUMBER] =
{
    #ifdef BSP_BUTTON_0
    {BSP_BUTTON_0, false, BUTTON_PULL, bsp_button_event_handler},
//    {BSP_BUTTON_0, APP_BUTTON_ACTIVE_HIGH, NRF_GPIO_PIN_NOPULL , bsp_button_event_handler},
    #endif // BUTTON_0

    #ifdef BSP_BUTTON_1
    {BSP_BUTTON_1, false, BUTTON_PULL, bsp_button_event_handler},
    #endif // BUTTON_1


};

bool bsp_button_is_pressed(uint32_t button)
{
    return bsp_board_button_state_get(button);
}

/**@brief Function for handling button events.
 *
 * @param[in]   pin_no          The pin number of the button pressed.
 * @param[in]   button_action   Action button.
 */
static void bsp_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    bsp_event_t        event  = BSP_EVENT_NOTHING;
    uint32_t           button = 0;
    uint32_t           err_code;
    static uint8_t     current_long_push_pin_no;              /**< Pin number of a currently pushed button, that could become a long push if held long enough. */
    static bsp_event_t release_event_at_push[BUTTONS_NUMBER]; /**< Array of what the release event of each button was last time it was pushed, so that no release event is sent if the event was bound after the push of the button. */

    button = bsp_board_pin_to_button_idx(pin_no);
		//SEGGER_RTT_printf(0,"button= %d action=%x\r\n", button,button_action);	

    if (button < BUTTONS_NUMBER)
    {
        switch (button_action)
        {
            case APP_BUTTON_PUSH:
               event = m_events_list[button].push_event;
                if (m_events_list[button].long_push_event != BSP_EVENT_NOTHING)
                {
										nrf_gpio_pin_clear(BSP_LED_1);
										//nrf_gpio_pin_set(BSP_LED_0);
                    err_code = app_timer_start(m_button_timer_id, APP_TIMER_TICKS(BSP_LONG_PUSH_TIMEOUT_MS), (void*)&current_long_push_pin_no);
                    if (err_code == NRF_SUCCESS)
                    {
                        current_long_push_pin_no = pin_no;
                    }
                }
                release_event_at_push[button] = m_events_list[button].release_event;
								event=BSP_EVENT_NOTHING;
                break;
            case APP_BUTTON_RELEASE:
               (void)app_timer_stop(m_button_timer_id);
                if (release_event_at_push[button] == m_events_list[button].release_event)
                {
                    event = m_events_list[button].release_event;
		//SEGGER_RTT_printf(0,"event= %x\r\n", event);	
                }
                break;
            case BSP_BUTTON_ACTION_LONG_PUSH:
                event = m_events_list[button].long_push_event;
						break;
        }
    }

    if ((event != BSP_EVENT_NOTHING) && (m_registered_callback != NULL))
    {
				sys_flag.buttons_disabled=true;
				bsp_buttons_disable();
        m_registered_callback(event);
    }
}

/**@brief Handle events from button timer.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void button_timer_handler(void * p_context)
{
//Power button long push -- turn off power
/*
		nrf_gpio_pin_set(BSP_LED_0);
		nrf_gpio_pin_clear(PWR_CTRL);
	while(1) {
	}
*/
    bsp_button_event_handler(*(uint8_t *)p_context, BSP_BUTTON_ACTION_LONG_PUSH);
}

static void leds_off(void)
{
        bsp_board_leds_off();
}

/**@brief       Configure leds to indicate required state.
 * @param[in]   indicate   State to be indicated.
 */
static uint32_t bsp_led_indication(bsp_indication_t indicate)
{
    uint32_t err_code   = NRF_SUCCESS;
    uint32_t next_delay = 0;
    if(m_leds_clear)
    {
        m_leds_clear = false;
        leds_off();
    }
		//SEGGER_RTT_printf(0,"bsp_led_indication= %d\r\n", indicate);	
		if (sys_flag.blinking_led) return 0;

    switch (indicate)
    {
        case BSP_INDICATE_IDLE:
					/*
            if (m_led_on_state)
            {
								m_led_on_state=false;
                bsp_board_led_off(BSP_BOARD_LED_0);
                next_delay = IDLE_LED_OFF_INTERVAL  ;
            }
            else
            {
 								m_led_on_state=true;
								//bsp_board_led_on(BSP_BOARD_LED_0);
                next_delay = IDLE_LED_ON_INTERVAL;
            }
								//err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);
				*/
            //m_stable_state = indicate;
            break;

        case BSP_INDICATE_ADVERTISING:
            // in advertising blink LED_0
            if (m_led_on_state)
            {
								m_led_on_state=false;
                bsp_board_led_off(BSP_BOARD_LED_0);
                next_delay = ADVERTISING_LED_OFF_INTERVAL ;
            }
            else
            {
 								m_led_on_state=true;
               bsp_board_led_on(BSP_BOARD_LED_0);
                next_delay = ADVERTISING_LED_ON_INTERVAL;
            }
            err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);

            m_stable_state = indicate;
            break;


        case BSP_INDICATE_CONNECTED:
            //bsp_board_led_on(BSP_LED_INDICATE_CONNECTED);
            if (m_led_on_state)
            {
								m_led_on_state=false;
                bsp_board_led_off(BSP_BOARD_LED_0);
                next_delay = CONNECTED_LED_OFF_INTERVAL ;
            }
            else
            {
 								m_led_on_state=true;
               bsp_board_led_on(BSP_BOARD_LED_0);
                next_delay = CONNECTED_LED_ON_INTERVAL;
            }
            err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);
            m_stable_state = indicate;
            break;


        case BSP_INDICATE_BATTERY_LOW:
		//battery low
            if (m_led_on_state)
            {
								m_led_on_state=false;
								bsp_board_led_off(BSP_BOARD_LED_1);
                next_delay = BAT_LOW_LED_OFF_INTERVAL;
            }
            else
            {
								m_led_on_state=true;
                bsp_board_led_on(BSP_BOARD_LED_1);
                next_delay = BAT_LOW_LED_ON_INTERVAL;
            }

            m_stable_state = indicate;
            err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);
            break;

        case BSP_INDICATE_CHRGING:
		//in charging
            if (m_led_on_state)
            {
								m_led_on_state=false;
								//bsp_board_leds_off();
								bsp_board_led_off(BSP_BOARD_LED_0);
								bsp_board_led_off(BSP_BOARD_LED_1);
                next_delay = CHRGING_LED_OFF_INTERVAL;
            }
            else
            {
								m_led_on_state=true;
								//bsp_board_leds_on();
                bsp_board_led_on(BSP_BOARD_LED_0);
                bsp_board_led_on(BSP_BOARD_LED_1);
								next_delay = CHRGING_LED_ON_INTERVAL;
            }

            m_stable_state = indicate;
            err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);
            break;

		//charging full
        case BSP_INDICATE_CHRG_FULL:
            bsp_board_led_on(BSP_BOARD_LED_0);
            m_stable_state = indicate;
            break;
				//device error
        case BSP_INDICATE_ERROR:
				/*
            bsp_board_led_on(BSP_BOARD_LED_1);
            m_stable_state = indicate;
            break;
				*/
        case BSP_INDICATE_COUNTING:
		//record data
            if (m_led_on_state)
            {
								m_led_on_state=false;
                bsp_board_led_off(BSP_BOARD_LED_1);
                next_delay = COUNTING_LED_OFF_INTERVAL;
            }
            else
            {
								m_led_on_state=true;
                bsp_board_led_on(BSP_BOARD_LED_1);
                next_delay = COUNTING_LED_ON_INTERVAL;
            }

            m_stable_state = indicate;
            err_code = app_timer_start(m_leds_timer_id, APP_TIMER_TICKS(next_delay), NULL);
            break;


        default:
            break;
    }

    return err_code;
}


/**@brief Handle events from leds timer.
 *
 * @note Timer handler does not support returning an error code.
 * Errors from bsp_led_indication() are not propagated.
 *
 * @param[in]   p_context   parameter registered in timer start function.
 */
static void leds_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    if (m_indication_type & BSP_INIT_LED)
    {
        UNUSED_VARIABLE(bsp_led_indication(m_stable_state));
    }
}


/**@brief Configure indicators to required state.
 */
uint32_t bsp_indication_set(bsp_indication_t indicate)
{
    uint32_t err_code = NRF_SUCCESS;

    if (m_indication_type & BSP_INIT_LED)
    {
				bsp_board_leds_off();
        err_code  = app_timer_stop(m_leds_timer_id);
				sys_flag.idle=false;
				if (sys_flag.blinking_led) return 0;
				switch (indicate)
				{
						case BSP_INDICATE_IDLE:
								if (sys_flag.counting) {
									indicate=BSP_INDICATE_COUNTING;
									bsp_board_led_on(BSP_BOARD_LED_1);
								} else
								if (sys_flag.connected) {
									indicate=BSP_INDICATE_CONNECTED;
									bsp_board_led_on(BSP_BOARD_LED_0);
								} else
								if (sys_flag.advertising) {
									indicate=BSP_INDICATE_ADVERTISING;
									bsp_board_led_on(BSP_BOARD_LED_0);
								} else
								if (sys_flag.device_error) {
									indicate=BSP_INDICATE_ERROR;
									bsp_board_led_on(BSP_BOARD_LED_1);
								
								} else
								if (sys_flag.chrg_full) {
									indicate=BSP_INDICATE_CHRG_FULL;
									bsp_board_led_on(BSP_BOARD_LED_0);
								} else
								if (sys_flag.charging) {
									indicate=BSP_INDICATE_CHRGING;
									bsp_board_led_on(BSP_BOARD_LED_0);
									bsp_board_led_on(BSP_BOARD_LED_1);
								} else
								if (sys_flag.battery_low) {
									indicate=BSP_INDICATE_BATTERY_LOW;
									bsp_board_led_on(BSP_BOARD_LED_1);
								} else {
									sys_flag.idle=true;
									//bsp_board_led_on(BSP_BOARD_LED_0);
								}
								
								m_led_on_state=false;
            break;
						case BSP_INDICATE_CONNECTED:
								if (sys_flag.counting) {
									indicate=BSP_INDICATE_COUNTING;
								}
								break;
				}
				
        err_code = bsp_led_indication(indicate);
    }
    return err_code;
}


uint32_t bsp_init(uint32_t type, bsp_event_callback_t callback)
{
    uint32_t err_code = NRF_SUCCESS;

    m_indication_type     = type;

    m_registered_callback = callback;

    // BSP will support buttons and generate events
    if (type & BSP_INIT_BUTTONS)
    {
//        uint32_t num;

        if (err_code == NRF_SUCCESS)
        {
            err_code = app_button_init((app_button_cfg_t *)app_buttons,
                                       BUTTONS_NUMBER,
                                       APP_TIMER_TICKS(50));
        }

        if (err_code == NRF_SUCCESS)
        {
           err_code = app_button_enable();
        }
        if (err_code == NRF_SUCCESS)
        {
            err_code = app_timer_create(&m_button_timer_id,
                                        APP_TIMER_MODE_SINGLE_SHOT,
                                        button_timer_handler);
        }

    }


    if (type & BSP_INIT_LED)
    {
        bsp_board_leds_init();
    }

    // timers module must be already initialized!
    if (err_code == NRF_SUCCESS)
    {
        err_code =
            app_timer_create(&m_leds_timer_id, APP_TIMER_MODE_SINGLE_SHOT, leds_timer_handler);
    }

    return err_code;
}


/**@brief Assign specific event to button.
 */
uint32_t bsp_event_to_button_action_assign(uint32_t button, bsp_button_action_t action, bsp_event_t event)
{
    uint32_t err_code = NRF_SUCCESS;

    if (button < BUTTONS_NUMBER)
    {
        if (event == BSP_EVENT_DEFAULT)
        {
            // Setting default action: BSP_EVENT_KEY_x for PUSH actions, BSP_EVENT_NOTHING for RELEASE and LONG_PUSH actions.
            event = (action == BSP_BUTTON_ACTION_PUSH) ? (bsp_event_t)(BSP_EVENT_ADVERTISING + button) : BSP_EVENT_NOTHING;
        }
		//SEGGER_RTT_printf(0,"button= %d action=%x\r\n", button,action);	
        switch (action)
        {
            case BSP_BUTTON_ACTION_PUSH:
                m_events_list[button].push_event = event;
                break;
            case BSP_BUTTON_ACTION_RELEASE:
                m_events_list[button].release_event = event;
                break;
            case BSP_BUTTON_ACTION_LONG_PUSH:
                m_events_list[button].long_push_event = event;
                break;
            default:
                err_code = NRF_ERROR_INVALID_PARAM;
                break;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }

    return err_code;
}


uint32_t bsp_buttons_enable()
{
    return app_button_enable();
}

uint32_t bsp_buttons_disable()
{
    return app_button_disable();
}

uint32_t bsp_wakeup_button_enable(uint32_t button_idx)
{
    nrf_gpio_cfg_sense_set(bsp_board_button_idx_to_pin(button_idx),
            BUTTONS_ACTIVE_STATE ? NRF_GPIO_PIN_SENSE_HIGH :NRF_GPIO_PIN_SENSE_LOW);
    return NRF_SUCCESS;
}

uint32_t bsp_wakeup_button_disable(uint32_t button_idx)
{
    nrf_gpio_cfg_sense_set(bsp_board_button_idx_to_pin(button_idx),
                           NRF_GPIO_PIN_NOSENSE);
    return NRF_SUCCESS;
}
