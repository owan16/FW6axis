/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
 /****************************************************************************
 * (C) KUASMIS WMC Lab. , NCTU & GAHO electric Inc. Co-workded - 2013 
 ****************************************************************************/

 /* This function gets new data from the FIFO when the DMP is in
		 * use. The FIFO can contain any combination of gyro, accel,
		 * quaternion, and gesture data. The sensors parameter tells the
		 * caller which data fields were actually populated with new data.
		 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		 * the FIFO isn't being filled with accel data.
		 * The driver parses the gesture data to determine if a gesture
		 * event has occurred; on an event, the application will be notified
		 * via a callback (assuming that a callback function was properly
		 * registered). The more parameter is non-zero if there are
		 * leftover packets in the FIFO.
		 */

/* Gyro and accel data are written to the FIFO by the DMP in chip
 * frame and hardware units. This behavior is convenient because it
 * keeps the gyro and accel outputs of dmp_read_fifo and
 * mpu_read_fifo consistent.
 */

/* Unlike gyro and accel, quaternions are written to the FIFO in
* the body frame, q30. The orientation is set by the scalar passed
* to dmp_set_orientation during initialization.
*/
	 

#ifndef MPU_SENSOR_DMP_H
#define MPU_SENSOR_DMP_H

#include <stdbool.h>
#include <stdint.h>

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)


uint8_t mpu_sensor_dmp_load_firmware(void);


uint8_t mpu_sensor_dmp_set_orientation(unsigned short orient);
uint8_t mpu_sensor_dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
uint8_t mpu_sensor_dmp_set_fifo_rate(unsigned short rate);

uint8_t mpu_sensor_dmp_enable_feature(unsigned short mask);
uint8_t mpu_sensor_dmp_enable_gyro_cal(unsigned char enable);
uint8_t mpu_sensor_dmp_enable_lp_quat(unsigned char enable);
uint8_t mpu_sensor_dmp_enable_6x_lp_quat(unsigned char enable);

uint8_t mpu_sensor_dmp_read_fifo(uint8_t *gyro, uint8_t *accel, uint8_t *quat_array, short *sensors, uint8_t *more);

#endif 				/* MPU_SENSOR_DMP_H */
