
/**@file
 *
 * @defgroup ble_advs_srv ADV Data Service
 * @{
 * @ingroup  ble_advs_srv
 * @brief    Beacon Data Service implementation.
 *
 * @details The Beacon Data Service is a simple GATT-based service with Beacon setting and record characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the S130 SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate S130 SoftDevice events to the Beacon Data Service module
 *       by calling the ble_advs_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_advs_H__
#define BLE_advs_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>


#define BLE_UUID_ADVS_MAJOR_CHAR		0x1701				/**< The UUID of the Beacon Measurement Characteristic. */
#define BLE_UUID_ADVS_MINOR_CHAR		0x1702				/**< The UUID of the Beacon Measurement DMP Characteristic. */
#define BLE_UUID_ADVS_RSSI_CHAR			0x1703				/**< The UUID of the Beacon Rate Characteristic. */
#define BLE_UUID_ADVS_INTV_CHAR			0x1704				/**< The UUID of the Beacon Accelerator FSR (Full Scale Range) Characteristic. */
#define BLE_UUID_ADVS_PERIOD_CHAR		0x1705				/**< The UUID of the Beacon Gyrometer FSR Characteristic. */



//#define ADVS_BASE_UUID                  {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x6D, 0xA5, 0xE5, 0x11, 0xCE, 0x13, 0x00, 0x00, 0x7A, 0x19}} /**< Used vendor specific UUID. */
#define ADVS_BASE17_UUID                  0x1700 /**< Used vendor specific UUID. */

#define BLE_UUID_ADVS_SERVICE 0x1700                      /**< The UUID of Beacon Data Service. */

#define BLE_ADVS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Beacon Data Service module. */

#define BLE_ADVS_MAX_SETTING_CHAR_LEN        2        /**< Maximum length of the Setting Characteristic (in bytes). */






/* Forward declaration of the ble_advs_t type. */
typedef struct ble_advs_s ble_advs_t;


// ***internal use***
//flag of CCCD notification status
typedef struct 
{
    //internal use
    uint8_t reserve:4;
    uint8_t packet_number:4;
}ble_advs_notified_s;



/**@brief Beacon Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_advs_init
 *          function.
 */
typedef struct
{
    uint8_t beacon_setting[2];
} ble_advs_init_t;

/**@brief  Beacon Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_advs_s
{
    uint8_t                  uuid_type;					/**< UUID type for  Beacon Service Base UUID. */
    uint16_t                 service_handle;			/**< Handle of Beacon Setting  Service. */
    ble_gatts_char_handles_t major_handles;				/**< Handles related to the major Characteristic . */
    ble_gatts_char_handles_t minor_handles;				/**< Handles related to the minor Characteristic . */
    ble_gatts_char_handles_t rssi_handles;				/**< Handles related to the RSSI Characteristic . */
    ble_gatts_char_handles_t intv_handles;				/**< Handles related to the interval Characteristic . */
    ble_gatts_char_handles_t period_handles;			/**< Handles related to the period Characteristic . */
	uint16_t                 conn_handle;				/**< Handle of the current connection . BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_advs_notified_s		notification_enabled;		/**< Variable to indicate if the peer has enabled notification of the Beacon and Current Record characteristic.*/
};

/**@brief Function for initializing the Beacon Data Service.
 *
 * @param[out] p_advs      Beacon Data Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_advs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_advs or p_advs_init is NULL.
 */
uint32_t ble_advs_init(ble_advs_t * p_advs, const ble_advs_init_t * p_advs_init);

/**@brief Function for handling the Beacon Data Service's BLE events.
 *
 * @details The Beacon Data Service expects the application to call this function each time an
 * event is received from the  SoftDevice. This function processes the event if it
 * is relevant and calls the Beacon Data Service event handler of the
 * application if necessary.
 *
 * @param[in] p_advs       Beacon Data Service structure.
 * @param[in] p_ble_evt   Event received from the  SoftDevice.
 */
void ble_advs_on_ble_evt(ble_advs_t * p_advs, ble_evt_t * p_ble_evt);


#endif // BLE_advs_H__

/** @} */




