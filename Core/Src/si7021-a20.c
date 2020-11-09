/**
 ******************************************************************************
 * @file           : si7021-a20.c
 * @brief          : Humidity and temperature sensor operations library.
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

/* Includes -------------------------------------------------------------------------------------*/
#include "si7021-a20.h"
#include "main.h"

/*-----------------------------------------------------------------------------------------------*/

/* Defines --------------------------------------------------------------------------------------*/
/* Default communication value timeout */
#define DEFAULT_I2C_TIMEOUT			100

/* Configuration register error values */
#define CONFIG_ERROR				0x00	// Reset value is: 0x3A
#define HEAT_POWER_ERROR			0xF0	// Reset value is: 0x00

/* Measurement error value */
#define SI7021_HUMIDITY_MEASURE_FAILED       	0xFF	// 255% value of humidity
#define SI7021_TEMPERATURE_MEASURE_FAILED       0xFFFF 	// -327,66°C for temperature
/*-----------------------------------------------------------------------------------------------*/

/* Variables declarations -----------------------------------------------------------------------*/
static uint8_t	si7021_buffer[2];

/*-----------------------------------------------------------------------------------------------*/

/* Functions definitions ------------------------------------------------------------------------*/

HAL_StatusTypeDef si7021_read_id(I2C_HandleTypeDef *hi2c, uint8_t * id_buffer)
{
	uint8_t	si7021_id_buffer[15];
	uint8_t si7021_id[8];

	HAL_StatusTypeDef i2c_comm;

	si7021_buffer[0] = (SI7021_READ_ID1>>8);
	si7021_buffer[1] = (uint8_t)(SI7021_READ_ID1);

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_id_buffer, 8, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK)
		{
			si7021_buffer[0] = (SI7021_READ_ID2>>8);
			si7021_buffer[1] = (uint8_t)(SI7021_READ_ID2);
			i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
			if(i2c_comm == HAL_OK)
			{
				i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, &si7021_id_buffer[8], 6, DEFAULT_I2C_TIMEOUT);
				if(i2c_comm == HAL_OK)
				{
					si7021_id[0] = si7021_id_buffer[0];
					si7021_id[1] = si7021_id_buffer[2];
					si7021_id[2] = si7021_id_buffer[4];
					si7021_id[3] = si7021_id_buffer[6];
					si7021_id[4] = si7021_id_buffer[8];
					si7021_id[5] = si7021_id_buffer[10];
					si7021_id[6] = si7021_id_buffer[12];
					si7021_id[7] = si7021_id_buffer[14];
					id_buffer = si7021_id;

				}
			}
		}
	}
	return i2c_comm;
}

HAL_StatusTypeDef si7021_set_config(I2C_HandleTypeDef *hi2c, uint8_t heater, uint8_t resolution)
{
	si7021_buffer[0] = SI7021_WRITE_USER_REG1;
	si7021_buffer[1] = heater | resolution;

	return HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
}

uint8_t si7021_read_config(I2C_HandleTypeDef *hi2c)
{
	si7021_buffer[0] = SI7021_READ_USER_REG1;
	HAL_StatusTypeDef i2c_comm;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK)
			return si7021_buffer[0];
		else return CONFIG_ERROR;
	}
	else return CONFIG_ERROR;
}

HAL_StatusTypeDef si7021_set_heater_power(I2C_HandleTypeDef *hi2c, uint8_t power)
{
	si7021_buffer[0] = SI7021_WRITE_HEATER_REG;
	si7021_buffer[1] = power;

	return HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
}

uint8_t si7021_read_heater_power(I2C_HandleTypeDef *hi2c)
{
	si7021_buffer[0] = SI7021_READ_HEATER_REG;
	HAL_StatusTypeDef i2c_comm;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK)
			return si7021_buffer[0];
		else return HEAT_POWER_ERROR;
	}
	else return HEAT_POWER_ERROR;
}

uint8_t si7021_measure_humidity(I2C_HandleTypeDef *hi2c)
{
	si7021_buffer[0] = SI7021_MEASURE_NOHOLD;
	HAL_StatusTypeDef i2c_comm;

	/* Start measure */
	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		HAL_Delay(30);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);

		if(i2c_comm == HAL_OK)
			return (si7021_buffer[0] << 8 | si7021_buffer[1]) * 125 / 65536 - 6;
		else return SI7021_HUMIDITY_MEASURE_FAILED;
	}
	else return SI7021_HUMIDITY_MEASURE_FAILED;
}

int32_t si7021_read_previous_temperature(I2C_HandleTypeDef *hi2c)
{
	si7021_buffer[0] = SI7021_READ_PREV_TEMP;
	HAL_StatusTypeDef i2c_comm;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK)
		{
			float temp_code = (float)((si7021_buffer[0] << 8 | si7021_buffer[1]) * 175.72 / 65536 - 46.85);
			return (int32_t)(temp_code*100);
		}
		else return SI7021_TEMPERATURE_MEASURE_FAILED;
	}
	else return SI7021_TEMPERATURE_MEASURE_FAILED;
}

int32_t si7021_measure_temperature(I2C_HandleTypeDef *hi2c)
{
	si7021_buffer[0] = SI7021_MEASURE_TEMP_NOHOLD;
	HAL_StatusTypeDef i2c_comm;

	i2c_comm = HAL_I2C_Master_Transmit(hi2c, SI7021_ADDRESS_WRITE, si7021_buffer, 1, DEFAULT_I2C_TIMEOUT);
	if(i2c_comm == HAL_OK)
	{
		HAL_Delay(30);
		/* Read result */
		i2c_comm = HAL_I2C_Master_Receive(hi2c, SI7021_ADDRESS_READ, si7021_buffer, 2, DEFAULT_I2C_TIMEOUT);
		if(i2c_comm == HAL_OK)
		{
			float temp_code = (float)((si7021_buffer[0] << 8 | si7021_buffer[1]) * 175.72 / 65536 - 46.85);
			return (int32_t)(temp_code*100);
		}
		else return SI7021_TEMPERATURE_MEASURE_FAILED;
	}
	else return SI7021_TEMPERATURE_MEASURE_FAILED;
}
