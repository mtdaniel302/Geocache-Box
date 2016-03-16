/*
 * myBNO055.h
 *
 * Created: 3/8/2016 7:32:30 AM
 *  Author: Riley
 */ 


#ifndef MYBNO055_H_
#define MYBNO055_H_

#include <avr/io.h>
#include <util/delay.h>		// for _delay_ms()

#define BNO055_API

#define BNO055_ID				0xA0
#define MY_BNO055_I2C_ADDDRESS	0x50 //left shift address 1, 
	//due to library masking a read/write bit on?? This was a MAJOR PITA ha


/*!
* @brief struct for sensor calibration values, of precision unsigned char
*/
struct bno055_calibration_unsigned_char_t {
	uint8_t sys; /**< system calibration u8 data */
	uint8_t gyro;/**< gyro calibration u8 data */
	uint8_t accel;/**< accelerometer calibration u8 data */
	uint8_t mag;/**< magnetometer calibration u8 data */
};


/*----------------------------------------------------------------------------*
 *  The following functions are used for reading and writing of
 *  sensor data using I2C communication
*----------------------------------------------------------------------------*/
#ifdef  BNO055_API
/*  \Brief: The function is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be read
 *  \param reg_data : This data read from the sensor, which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*  \Brief: The function is used as SPI bus write //I2C, not SPI
 *  \Return : Status of the SPI write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register, will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: I2C init routine
*/


/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek);

/* get calibration values (all 0-3, 2 bits */
void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag); 


/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
s32 bno055_data_readout_template(void);
/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/

#endif
/********************End of I2C function declarations***********************/






#endif /* MYBNO055_H_ */

/*
About Calibration

3.10 Calibration
Though the sensor fusion software runs the calibration algorithm of all the three sensors
(accelerometer, gyroscope and magnetometer) in the background to remove the offsets,
some preliminary steps had to be ensured for this automatic calibration to take place.
The accelerometer and the gyroscope are relatively less susceptible to external
disturbances, as a result of which the offset is negligible. Whereas the magnetometer is
susceptible to external magnetic field and therefore to ensure proper heading accuracy, the
calibration steps described below have to be taken.
Depending on the sensors been selected, the following simple steps had to be taken after
every ‘Power on Reset’ for proper calibration of the device.

3.10.1 Accelerometer Calibration
? Place the device in 6 different stable positions for a period of few seconds to allow the
accelerometer to calibrate.
? Make sure that there is slow movement between 2 stable positions
? The 6 stable positions could be in any direction, but make sure that the device is lying at
least once perpendicular to the x, y and z axis.
? The register CALIB_STAT can be read to see the calibration status of the accelerometer.

3.10.2 Gyroscope Calibration
? Place the device in a single stable position for a period of few seconds to allow the
gyroscope to calibrate
? The register CALIB_STAT can be read to see the calibration status of the gyroscope.

3.10.3 Magnetometer Calibration
Magnetometer in general are susceptible to both hard-iron and soft-iron distortions, but
majority of the cases are rather due to the former. And the steps mentioned below are to
calibrate the magnetometer for hard-iron distortions.
Nevertheless certain precautions need to be taken into account during the positioning of
the sensor in the PCB which is described in our HSMI (Handling, Soldering and Mounting
Instructions) application note to avoid unnecessary magnetic influences.

Compass, M4G & NDOF_FMC_OFF:
? Make some random movements (for example: writing the number ‘8’ on air) until the
CALIB_STAT register indicates fully calibrated.
? It takes more calibration movements to get the magnetometer calibrated than in the
NDOF mode.

*/