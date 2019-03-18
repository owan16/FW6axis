 /****************************************************************************
 * (C) KUASMIS WMC Lab. , NCTU & GAHO electric Inc. Co-workded - 2013 
 ****************************************************************************/
 

#ifndef MPU_SENSOR_ERR_H
#define MPU_SENSOR_ERR_H


#define MPU_SUCCESS                           0  ///< Successful command
#define MPU_ERROR_WAKE_UP_WRITE					      1  ///< TWI wake up fails
#define MPU_ERROR_CHECK_PROD_REV_READ					2  ///< TWI check product revision fails
#define MPU_ERROR_SET_GYRO_FSR_WRITE					3  ///< TWI set the gyro full-scale range fails
#define MPU_ERROR_SET_ACCEL_FSR_WRITE					4  ///< TWI set the accel full-scale range fails
#define MPU_ERROR_SET_LPF_WRITE								5  ///< TWI set digital low pass filter fails
#define MPU_ERROR_SET_SAMP_RATE_WRITE					6  ///< TWI set sampling rate fails
#define MPU_ERROR_CONF_FIFO_WRITE							7	 ///< TWI config fifo setting fails
#define MPU_ERROR_SET_BYPASS_MODE							8 ///< TWI set bypass mode fails



/* SETUP COMPASS:	Error Table */
#define MPU_ERROR_SETUP_COMPASS							0x40 ///< TWI setup compass fails
/*
#define MPU_ERROR_COMPASS_NOT_FOUND					0x40 ///< TWI compass not found fails
#define MPU_ERROR_SET_AKM_POWER_DOWN_1			0x41 ///< TWI compass power down fails
#define MPU_ERROR_SET_AKM_FUSE_ROM_ACCESS		0x42 ///< TWI compass fuse rom access fails
#define MPU_ERROR_GET_AKM_ADJ_DATA					0x43 ///< TWI get sensitivity adjustment data from fuse ROM fails
#define MPU_ERROR_SET_AKM_POWER_DOWN_2			0x44 ///< TWI compass power down fails
#define MPU_ERROR_SET_MASTER_MODE						0x45 ///< TWI set up master mode, master clock, and ES bit fails
#define MPU_ERROR_SET_AKM_DATA							0x46 ///< TWI slave 0 reads from AKM data registers fails
#define MPU_ERROR_SET_AKM_READS_START_ADDR	0x47 ///< TWI compass reads start at this register fails
#define MPU_ERROR_SET_AKM_ENABLE_0					0x48 ///< TWI enable slave 0, 8-byte reads fails
#define MPU_ERROR_SET_AKM_MEAS_MODE					0x49 ///< TWI slave 1 changes AKM measurement mode fails
#define MPU_ERROR_SET_AKM_MEAS_MODE_REG			0x4A ///< TWI Measurement mode register fails
#define MPU_ERROR_SET_AKM_ENABLE_1					0x4B ///< TWI Enable slave 1, 1-byte writes fails
#define MPU_ERROR_SET_AKM_SLAVE_DATA_1			0x4C ///< TWI set slave 1 data fails
#define MPU_ERROR_SET_AKM_ACTION					  0x4D ///< TWI trigger slave 0 and slave 1 actions at each sample fails
#define MPU_ERROR_SET_AKM_VDD								0x4E ///< TWI the auxiliary I2C bus set to VDD fails
*/

// mpu_sensor_set_compass_sample_rate
#define MPU_ERROR_SET_COMPASS_SAMP_RATE			0x50 ///< TWI set the sample rate of compass fails

#define MPU_ERROR_GET_COMPASS_REG_DATA			0x60 ///< TWI get the compass data of ext register fails

#define MPU_ERROR_DMP_LOAD_FIRMWARE_BASE		0x70 ///< Load DMP firmware fails

#define MPU_ERROR_WRITE_MEMORY_BASE					0x80 ///< TWI fails of load the progtam to memory 

#define MPU_ERROR_DMP_READ_FIFO							0x90 ///< TWI get the quar and compass data 

#define MPU_ERROR_NOT_DEFINE								0xFE

#endif		/* MPU_SENSOR_ERR_H 	*/

