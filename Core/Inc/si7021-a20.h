/**
  ******************************************************************************
  * @file           : si7021-a20.h
  * @brief          : This file contains the headers of the humidity and
  * 				temperature sensor Si7021.
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

#ifndef INC_SI7021_A20_H_
#define INC_SI7021_A20_H_

/* Includes -------------------------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Defines --------------------------------------------------------------------------------------*/

/* I2C communication address */
#define SI7021_ADDRESS_READ          ((0x40 << 1) | 0x01)
#define SI7021_ADDRESS_WRITE         (0x40 << 1)

#define SI7021_MEASURE_HOLD           0xE5 		// Measure Relative Humidity, Hold Master Mode
#define SI7021_MEASURE_NOHOLD         0xF5 		// Measure Relative Humidity, No Hold Master Mode
#define SI7021_MEASURE_TEMP_HOLD      0xE3 		// Measure Temperature, Hold Master Mode
#define SI7021_MEASURE_TEMP_NOHOLD    0xF3 		// Measure Temperature, No Hold Master Mode
#define SI7021_READ_PREV_TEMP         0xE0 		// Read Temperature Value from Previous RH Measurement
#define SI7021_RESET                  0xFE		// Reset
#define SI7021_WRITE_USER_REG1        0xE6 		// Write RH/T User Register 1
#define SI7021_READ_USER_REG1         0xE7 		// Read RH/T User Register 1
#define SI7021_WRITE_HEATER_REG       0x51 		// Write Heater Control Register
#define SI7021_READ_HEATER_REG        0x11 		// Read Heater Control Register
#define SI7021_READ_ID1               0xFA0F 	// Read Electronic ID 1st Byte
#define SI7021_READ_ID2               0xFCC9 	// Read Electronic ID 2nd Byte
#define SI7021_FIRMWARE_VERSION       0x84B8 	// Read Firmware Revision

/* Heater modes */
/* Used in function si7021_set_config() parameter "heater" */
#define SI7021_HEATER_ON                 (0x1 << 2)
#define SI7021_HEATER_OFF                0

/* Sensors measurement resolution */
/* Used in function si7021_set_config() parameter "resolution" */
#define SI7021_RESOLUTION_RH12_TEMP14    0
#define SI7021_RESOLUTION_RH8_TEMP12     1
#define SI7021_RESOLUTION_RH10_TEMP13    (1 << 7)
#define SI7021_RESOLUTION_RH11_TEMP11    (1 << 7 | 1)

/* Built-in heater current configuration */
/* Used in function si7021_set_heater_power() parameter "power" */
#define SI7021_HEATER_POWER_3MA          0
#define SI7021_HEATER_POWER_9MA          0x01
#define SI7021_HEATER_POWER_15MA         0x02
#define SI7021_HEATER_POWER_27MA         0x04
#define SI7021_HEATER_POWER_51MA         0x08
#define SI7021_HEATER_POWER_94MA         0x0f

/* Functions declarations -----------------------------------------------------------------------*/

/**
 * @brief		Read serial number of an Si7021 device.
 * @param		hi2c: I2C bus handler.
 * @retval		HAL_OK: If ID is properly read.
 */
HAL_StatusTypeDef si7021_read_id(I2C_HandleTypeDef *hi2c, uint8_t * id_buffer);

/**
 * @brief		Set measurement resolutions and turns on/off built-in heater.
 * @param		hi2c: I2C bus handler.
 * 				heater: Heater mode.
 * 				resolution: Measurement resolution configuration.
 * @retval		HAL_OK
 * 				HAL_ERROR
 */
HAL_StatusTypeDef si7021_set_config(I2C_HandleTypeDef *hi2c, uint8_t heater, uint8_t resolution);

/**
 * @brief		Read User Register 1.
 * @param		hi2c: I2C bus handler.
 * @retval		CONFIG_ERROR (0x00): If reading doesn't work.
 * 				(bXX11_1X1X) User Register 1 value.
 */
uint8_t	si7021_read_config(I2C_HandleTypeDef *hi2c);

/**
 * @brief		Set measurement resolutions and turns on/off built-in heater.
 * @param		hi2c: I2C bus handler.
 * 				power: Heater current configuration.
 * @retval		HAL_OK
 * 				HAL_ERROR
 */
HAL_StatusTypeDef si7021_set_heater_power(I2C_HandleTypeDef *hi2c, uint8_t power);

/**
 * @brief		Read Heater Control Register.
 * @param		hi2c: I2C bus handler.
 * @retval		CONFIG_ERROR (0xF0): If reading doesn't work.
 * 				(b0000_XXXX) Heater Control Register value.
 */
uint8_t	si7021_read_heater_power(I2C_HandleTypeDef *hi2c);

/**
 * @brief		Starts humidity measurement. As a side effect temperature will be measure as well.
 * 				To get temperature value use "si7021_read_previous_temperature()" function.
 * @param		hi2c: I2C bus handler.
 * @retval		Relative humidity measured in %. Example: 55 = 55%
 * 				SI7021_MEASURE_FAILED
 */
uint8_t si7021_measure_humidity(I2C_HandleTypeDef *hi2c);

/**
 * @brief		Starts temperature measurement. This blocking call takes about 20ms to complete.
 * @param		hi2c: I2C bus handler.
 * @retval		Current temperature measured in Celsius multiplied by 100. Example: 2312 = +23.12°C
 * 				SI7021_MEASURE_FAILED
 */
int32_t si7021_measure_temperature(I2C_HandleTypeDef *hi2c);

/**
 * @brief		Reads previous temperature measurement done by prior call of "si7021_measure_humidity()".
 * @param		hi2c: I2C bus handler.
 * @retval		Current temperature measured in Celsius multiplied by 100. Example: 2312 = +23.12°C
 * 				SI7021_MEASURE_FAILED
 */
int32_t si7021_read_previous_temperature(I2C_HandleTypeDef *hi2c);

#endif /* INC_SI7021_A20_H_ */
