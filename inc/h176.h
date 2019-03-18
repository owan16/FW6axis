/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"
//definitions for Koalla on PCBA

#define LED_GREEN			25		//for power and BT state
#define LED_RED				26		//for recording state
#define BUTTON_BT		13		//button for advertisng and power on/off
#define BUTTON_CNT	27		//button for recording start/stop
#define SCL_PIN			17		// I2C SCL signal pin
#define SDA_PIN			16		// I2C SDA signal pin

/*
#define SPI_SCK_PIN		6		// SPI clock GPIO pin number.
#define SPI_MOSI_PIN	5		// SPI Master Out Slave In GPIO pin number.
#define SPI_MISO_PIN	8		// SPI Master In Slave Out GPIO pin number.
#define SPI_SS_PIN		7		// SPI Slave Select GPIO pin number.
*/

#define PWR_CTRL			4		// IO pin to turn system power ON/OFF (High-ON, Low-OFF)
#define CHRG_DETECT		31		// IO pin to detect charging start (High - start charging,Low - remove charging)
#define CHRG_STATE		3		// IO pin for battery charging state (High - charging full)
#define TESTBAT				30		// IO pin to enable read battery voltage (for power saving)
#define SENSOR_INT		14		//sensor data ready interrupt

#define LED_START      25
#define LED_1          25
#define LED_2          26


#define LEDS_NUMBER    2

   
#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  LEDS_MASK
#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BUTTONS_NUMBER 2

#define BUTTON_START   13
#define BUTTON_1       13
#define BUTTON_2       27
#define BUTTON_STOP    27
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0
//#define BUTTONS_ACTIVE_STATE 1

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2


#define UART_RX  		8
#define UART_TX  		7


// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

