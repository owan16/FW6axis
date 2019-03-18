/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
 /****************************************************************************
 * (C) KUASMIS WMC Lab. , NCTU & GAHO electric Inc. Co-workded - 2013 
 ****************************************************************************/


#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H


#include <stdbool.h>
#include <stdint.h>


#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)


/**@brief MPU9250 Sensor Unit init structure. This contains auint8_t mpu_sensor_init(mpu_sensor_init_t * mpu_sensor_init);
ll options and data
 *        needed for initialization of the sensor. */
/*
typedef struct
{
		uint8_t											 	 accel_fsr_val;		
		uint8_t											 	 gyro_fsr_val;		
} mpu_sensor_init_t;
*/



/**
 * @brief Function for initializing Motion sensor and verifies it's on the bus.
 *
 * @param device_address Device TWI address in bits [6:0].
 * @return
 * @retval true sensor found on the bus and ready for operation.
 * @retval false sensor not found on the bus or communication failure.
 */
uint8_t twi_read_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t * data);
uint8_t mpu_sensor_fast_accel_init(void);
uint8_t mpu_sensor_set_sleep_mode(uint8_t enabled);
uint8_t mpu_sensor_init(void);
void set_sampling_rate(uint8_t rate);

/* Configiration */
uint8_t mpu_sensor_set_gyro_fsr(uint8_t fsr);
uint8_t mpu_sensor_set_accel_fsr(uint8_t fsr);
uint8_t mpu_sensor_set_lpf(uint16_t lpf);
uint8_t mpu_sensor_set_sample_rate(uint16_t rate);
uint8_t mpu_sensor_set_compass_sample_rate(uint16_t rate);
uint8_t mpu_sensor_set_bypass(uint8_t bypass_on);
uint8_t mpu_sensor_set_int_latched(uint8_t enable);
uint8_t mpu_sensor_set_sensors(void);
uint8_t mpu_sensor_set_lp_accel_mode(uint8_t rate);
uint8_t mpu_sensor_set_dmp_state(uint8_t enable);

/* FIFO Configiration*/
uint8_t mpu_sensor_configure_fifo(uint8_t sensors);
uint8_t mpu_sensor_get_fifo_count(uint16_t *count);
uint8_t mpu_sensor_set_fifo_clear(void);
uint8_t mpu_sensor_reset_fifo(void);


/* Get Configiration*/
uint8_t mpu_sensor_get_dmp_state(uint8_t *enabled);
uint8_t mpu_sensor_get_gyro_fsr(uint16_t *fsr);
uint8_t mpu_sensor_get_accel_fsr(uint8_t *fsr);
uint8_t mpu_sensor_get_lpf(uint16_t *lpf);
uint8_t mpu_sensor_get_fifo_config(uint8_t *sensors);
uint8_t mpu_sensor_get_sample_rate(uint16_t *rate);
uint8_t mpu_sensor_get_raw_data(uint8_t *data);
uint8_t mpu_sensor_get_accel_reg(uint8_t *data);
uint8_t mpu_sensor_get_gyro_reg(uint8_t *data);
uint8_t mpu_sensor_get_compass_reg(uint8_t *data);

uint8_t mpu_sensor_read_reg(uint8_t reg, uint8_t *data);
uint8_t mpu_sensor_read_fifo_stream(uint16_t length, uint8_t *data,uint8_t *more);

/* DMP Configiration*/
uint8_t mpu_sensor_set_memory_bank(uint8_t memory_bank);
uint8_t mpu_sensor_set_memory_address(uint8_t memory_addr);
uint8_t mpu_sensor_write_mem(uint16_t mem_addr, uint8_t length,uint8_t *data);
uint8_t mpu_sensor_read_mem(uint16_t mem_addr, uint8_t length, uint8_t *data);
uint8_t mpu_sensor_load_firmware(uint16_t length, const uint8_t *firmware,
																			uint16_t start_addr, uint16_t sample_rate);
uint8_t mpu_sensor_set_dmp_state(uint8_t enable);
uint8_t set_int_enable(uint8_t enable);
uint8_t start_sensor(void);
void mpu_int_enable(void);
void mpu_int_disable(void);
																		
#endif   /*  MPU_SENSOR_H  */
