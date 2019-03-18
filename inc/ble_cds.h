
/**@file
 *
 * @defgroup ble_cds_srv Motion Data Service
 * @{
 * @ingroup  ble_cds_srv
 * @brief    Motion Data Service implementation.
 *
 * @details The Communication Data Service is a simple GATT-based service .
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value.
 *          This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the S130 SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate S130 SoftDevice events to the Communication Data Service module
 *       by calling the ble_cds_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_cds_H__
#define BLE_cds_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>



#define BLE_UUID_CDS_SERVICE 0xFFF0                      /**< The UUID of Communication Data Service. */

//#define BLE_CDS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by this Service module. */
#define BLE_CDS_MAX_DATA_LEN  BLE_GATT_ATT_MTU_DEFAULT  /**< Maximum length of data (in bytes) that can be transmitted to the peer by this Service module. */

#define BLE_CDS_MAX_SETTING_CHAR_LEN        2        /**< Maximum length of the Setting Characteristic (in bytes). */






/* Forward declaration of the ble_CDS_t type. */
typedef struct ble_cds_s ble_cds_t;

/**@brief Beacon Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_advs_init
 *          function.
 */
typedef struct
{
    uint8_t cds_setting[2];
} ble_cds_init_t;

/**@brief Communication Data Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_cds_s
{
    uint8_t                  uuid_type;               /**< UUID type for Communication Data Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Communication Data and Date Sync Service */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the Communication Data Characteristic  */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the Communication Data Characteristic  */
	uint16_t                 conn_handle;             /**< Handle of the current connection  BLE_CONN_HANDLE_INVALID if not in a connection. */
};

/**@brief Function for initializing the Communication Data Service.
 *
 * @param[out] p_cds      Communication Data Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_cds_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_cds or p_cds_init is NULL.
 */
//uint32_t ble_cds_init(ble_cds_t * p_cds);
uint32_t ble_cds_init(ble_cds_t * p_cds, const ble_cds_init_t * p_cds_init);
void ble_cds_on_ble_evt(ble_cds_t * p_cds, ble_evt_t * p_ble_evt);


#endif // BLE_cds_H__

/** @} */




