
/**@file
 *
 * @defgroup ble_mts_srv Motion Data Service
 * @{
 * @ingroup  ble_mts_srv
 * @brief    Motion Data Service implementation.
 *
 * @details The Motion Data Service is a simple GATT-based service with Motion setting and record characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the S130 SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note The application must propagate S130 SoftDevice events to the Motion Data Service module
 *       by calling the ble_mts_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_mts_H__
#define BLE_mts_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>
#include "mpu_sensor.h"



#define BLE_UUID_MTS_SERVICE 0x1600                      /**< The UUID of Motion Data Service. */

#define BLE_MTS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Motion Data Service module. */

#define BLE_MTS_MAX_SETTING_CHAR_LEN        2        /**< Maximum length of the Setting Characteristic (in bytes). */

#define BLE_FRAME_CHAR_LEN_MAX														28

typedef struct {
    void (*tap_cb)(uint8_t count, uint8_t direction);
    void (*android_orient_cb)(uint8_t orientation);
    uint16_t orient;
    uint16_t feature_mask;
    uint16_t fifo_rate;
    uint8_t packet_length;
}dmp_s;

typedef void (*ble_setting_handler_t) (uint8_t * p_data, uint16_t * p_length);

/**@brief Motion Data Service event handler type. */
typedef void (*ble_get_measure_handler_t) (uint8_t * p_data, uint16_t * p_length);

/**@brief Motion Data Service event handler type. */
typedef void (*ble_get_rate_handler_t) (uint8_t * p_data, uint16_t * p_length);

/**@brief Motion Data Service event handler type. */
typedef void (*ble_get_acc_fsr_handler_t) (uint8_t * p_data, uint16_t * p_length);

typedef void (*ble_get_gyro_fsr_handler_t) (uint8_t * p_data, uint16_t * p_length);


/**@brief Motion Process Unit Service event type. */
typedef enum
{
  BLE_MPU_EVT_NOTIFICATION_ENABLED,                  /**< Motion Process Unit notification enabled event. */
  BLE_MPU_EVT_NOTIFICATION_DISABLED,                   /**< Motion Process Unit notification disabled event. */
	BLE_MPU_EVT_NOTIFICATION_DMP_ENABLED,                  /**< Motion Process Unit dmp notification enabled event. */
  BLE_MPU_EVT_NOTIFICATION_DMP_DISABLED,               /**< Motion Process Unit dmp notification disabled event. */
	BLE_MPU_EVT_INT_NOTIFICATION_ENABLED,									/**< Motion Process Unit interrupt notification enabled event. */
	BLE_MPU_EVT_INT_NOTIFICATION_DISABLED,									/**< Motion Process Unit interrupt notification disabled event. */
} ble_srv_motion_evt_type_t;
/**@brief Motion Process Unit Service event. */
typedef struct
{
		ble_srv_motion_evt_type_t evt_type;									/**< Type of event. */
//		uint8_t evt_prescaler_val;
//		uint8_t evt_sensor_val;
//		uint8_t evt_scale_acc_val;
//		uint8_t evt_scale_gyro_val;
//		uint16_t evt_threshold_val;
} ble_srv_motion_evt_t;

// Forward declaration of the ble_srv_motion_s type. 
typedef struct ble_mts_s ble_srv_motion_t;

/**@brief Motion Process Unit Service event handler type. */
typedef void (*ble_srv_motion_evt_handler_t) (ble_srv_motion_t * p_motion, ble_srv_motion_evt_t * p_evt);


/**@brief Motion Data Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_mts_init
 *          function.
 */
typedef struct
{
    ble_setting_handler_t mts_setting_handler;
} ble_mts_init_t;

/**@brief Motion Data Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_mts_s
{
 	ble_srv_motion_evt_handler_t	evt_handler;			/**< Event handler to be called for handling events in the Motion Process Unit Service. */
	uint8_t  					uuid_type;  
    uint16_t             		service_handle;						/**< Handle of Motion Setting and Date Sync Service*/
    ble_gatts_char_handles_t	measure_handles;				/**< Handles related to the Motion Characteristic  */
    ble_gatts_char_handles_t	measure_dmp_handles;		/**< Handles related to the DMP Motion Characteristic */
    ble_gatts_char_handles_t	accel_int_ntf_handles;	/**< Handles related to the notify value while > threshold Characteristic  */
    ble_gatts_char_handles_t	cnt_handles;						/**< Handles related to the read accumulated CNT  */
	uint16_t 					conn_handle;			/**< Handle of the current connection  BLE_CONN_HANDLE_INVALID if not in a connection. */
};
/**@brief Motion Process Unit Service measurement structure. This contains a Gyro, Accelerometer, Compass and
 *        Quaternion measurement. The first byte is sequence number */
typedef struct ble_srv_motion_meas_s
{                                         
		// YOUR_JOB: Build own data format
		short sensors;
		uint8_t 	sequence;
		uint8_t   inst_accel[6];                                               			/**< Instantaneous Accel. */
		uint8_t   inst_gyro[6];                                               			/**< Instantaneous Gyro. */
		//uint8_t   inst_compass[6];                                               		/**< Instantaneous Compass. */
		uint8_t   inst_quaternion[16];                                               /**< Instantaneous Quaternion. */

} ble_srv_motion_meas_t;
struct app_motion_s {
		uint8_t		measuring;
		uint8_t   motion_int_enable;
		uint8_t		motion_level;
		uint8_t		motion_seq;
		uint8_t		pair_req;
		uint16_t	tag_id;
		uint8_t 	gyro_fsr;
		uint8_t 	accel_fsr;
		uint16_t 	sample_rate;
		uint8_t 	beacon_timer_flag;
		uint8_t 	mpu_int_flag;
};
/* Forward declaration of the ble_MTS_t type. */
typedef struct ble_mts_s ble_mts_t;

/**@brief Function for initializing the Motion Data Service.
 *
 * @param[out] p_mts      Motion Data Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_mts_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_mts or p_mts_init is NULL.
 */
uint32_t ble_mts_init(ble_mts_t * p_mts, const ble_mts_init_t * p_mts_init);

/**@brief Function for handling the Motion Data Service's BLE events.
 *
 * @details The Motion Data Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Motion Data Service event handler of the
 * application if necessary.
 *
 * @param[in] p_mts       Motion Data Service structure.
 * @param[in] p_ble_evt   Event received from the SoftDevice.
 */
void ble_mts_on_ble_evt(ble_mts_t * p_mts, ble_evt_t * p_ble_evt);

/**@brief Function for sending Motion packet.
 *
 * @details The application calls this function when having new Motion packet. If
 *          notification has been enabled, the Motion Characteristic is sent to the client.
 *
 * @param[in]   p_mts          Pointer to the Motion Data Service structure.
 * @param[in] p_data    Pointer to new data packet of Motion.
 * @param[in] p_length    Pointer to length of data packet.
 *
 */
uint32_t ble_mts_motion_send(ble_mts_t * p_mts, uint8_t * p_data, uint16_t * p_length);


/**@brief Function for sending back last date record,step information,and sleep information in flash memory,
 *         and updating the date record anduser information with new input date.
 *
 * @details The application calls this function when having date record and user information synchronization. 
 *         If notification has been enabled, the Motion Characteristic is sent to the client.
 *
 * @param[in]   p_mts          Pointer to the Motion Data Service structure.
 * @param[in] p_data    Pointer to data of pd setting.
 * @param[in] length    length of pd setting.
 *
 */
uint8_t twi_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data);
uint32_t ble_srv_motion_measurement_send(ble_mts_t * p_mts, ble_srv_motion_meas_t * p_measurement);
//uint32_t ble_srv_motion_int_notify_send(ble_mts_t * p_mts, uint8_t data);
void motion_interrupt_services_enable(void);

#endif // BLE_mts_H__

/** @} */




