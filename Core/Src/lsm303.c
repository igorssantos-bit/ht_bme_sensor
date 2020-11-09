/**
 ******************************************************************************
 * @file        :   lsm303.c
 * @brief		:	This file contains application functions and handlers to
 * 					operate lsm303agr_reg_addr.c andlsm303agr_reg_addr.h files.
 ******************************************************************************
 * @attention
 * This file will change according to each application.
 *
 * How to use it:
 *
 * Declare variables
 *
 *	static axis3bit16_t data_raw_acceleration;
 * 	static axis3bit16_t data_raw_magnetic;
 * 	static axis1bit16_t data_raw_temperature;
 *
 * 	static float 	acceleration_mg[3];
 * 	static float 	magnetic_mG[3];
 * 	static float 	temperature_degC;
 *
 * 	stmdev_ctx_t dev_ctx_xl;
 * 	stmdev_ctx_t dev_ctx_mg;
 *
 * Initialize sensor driver interface
 *
 * 	dev_ctx_xl.write_reg 	= accel_write;
 *	dev_ctx_xl.read_reg 	= accel_read;
 *	dev_ctx_xl.handle 		= (void*)LSM303AGR_I2C_ADD_XL;
 *
 *	dev_ctx_mg.write_reg 	= magnet_write;
 *	dev_ctx_mg.read_reg 	= magnet_read;
 *	dev_ctx_mg.handle 		= (void*)LSM303AGR_I2C_ADD_MG;
 *
 *	Checks sensor presence and identification:
 *
 *	lsm303agr_check_id(&dev_ctx_xl, &dev_ctx_mg);		// Must return HAL_OK.
 *
 *	Initialize default configurations:
 *
 *	lsm303agr_default_init(&dev_ctx_xl, &dev_ctx_mg);	// Must return HAL_OK.
 *														// Can be modified for each application.
 *
 *	Read output only if new value is available example:
 *
 *	 	// Declare register structure to check status.
 *		lsm303agr_reg_t reg;
 *
 *		lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
 *		if (reg.status_reg_a.zyxda)
 *		{
 *			// Read accelerometer raw data.
 *			memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
 *			lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw_acceleration.u8bit);
 *
 *			// Separates and converts to [mg] unit.
 *			acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[0] );
 *			acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[1] );
 *			acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[2] );
 *		}
 *
 *		lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
 *		if (reg.status_reg_m.zyxda)
 *		{
 *			// Read magnetic field data.
 *			memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
 *			lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic.u8bit);
 *
 *			// Separates and converts to [mgauss] unit.
 *			magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[0]);
 *			magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[1]);
 *			magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[2]);
 *		}
 *
 *		lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);
 *		if (reg.byte)
 *		{
 *			// Read temperature data.
 *			memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
 *			lsm303agr_temperature_raw_get(&dev_ctx_xl, data_raw_temperature.u8bit);
 *
 *			// Converts to [°C] unit.
 *			temperature_degC = lsm303agr_from_lsb_hr_to_celsius( data_raw_temperature.i16bit );
 *		}
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "lsm303.h"
#include "lsm303agr_reg.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "traffic.h"

/* Defines -------------------------------------------------------------------*/
#define DEFAULT_I2C_TIMEOUT		1000
#define DEFAULT_ACT_TIMEOUT		0x15
#define DEFAULT_ACT_TRESHOLD	0x07

/* Variables -----------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

uint16_t u16_strong_threshold[8] = {
		0x01ff, 		//0x03E8
		0x0622,    	//1570
		0x0933,    	//2355
		0x1244,    	//4676
		0x1555,    	//5461
		0x1866,    	//6246
		0x2177,    	//8567
		0x2488,    	//9352
};

/* Functions definitions -----------------------------------------------------*/

/**
 * @brief		Check the ID of accelerometer and magnetometer registers.
 * @param		ctx_xl: Address of accelerometer driver interface.
 * 				ctx_xl: Address of magnetometer driver interface.
 * @retval		HAL_OK: If IDs are properly read.
 * 				HAL_ERROR: If any of IDs are wrong.
 */
HAL_StatusTypeDef lsm303agr_check_id(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg)
{
	uint8_t whoamI = 0;

	lsm303agr_xl_device_id_get(ctx_xl, &whoamI);
	if ( whoamI != LSM303AGR_ID_XL ) return HAL_ERROR;

	whoamI = 0;
	lsm303agr_mag_device_id_get(ctx_mg, &whoamI);
	if ( whoamI != LSM303AGR_ID_MG ) return HAL_ERROR;

	return HAL_OK;
}

/**
 * @brief		Execute a series of set commands configuring the sensor operations.
 * 				Can be changed for each application and called before readings.
 * @param		ctx_xl: Address of accelerometer driver interface.
 * 				ctx_xl: Address of magnetometer driver interface.
 * @retval		HAL_OK: If all commands are rightly executed.
 * 				HAL_ERROR: If any command return execution error.
 */
HAL_StatusTypeDef lsm303agr_default_init(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg)
{
	uint8_t rst;

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(ctx_mg, PROPERTY_ENABLE);
	do {
		lsm303agr_mag_reset_get(ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	if(lsm303agr_xl_block_data_update_set(ctx_xl, PROPERTY_ENABLE)) return HAL_ERROR;
	if(lsm303agr_mag_block_data_update_set(ctx_mg, PROPERTY_ENABLE)) return HAL_ERROR;

	/* Set Output Data Rate */
	if(lsm303agr_xl_data_rate_set(ctx_xl, LSM303AGR_XL_ODR_1Hz)) return HAL_ERROR;
	if(lsm303agr_mag_data_rate_set(ctx_mg, LSM303AGR_MG_ODR_50Hz)) return HAL_ERROR;

	/* Set accelerometer full scale */
	if(lsm303agr_xl_full_scale_set(ctx_xl, LSM303AGR_2g)) return HAL_ERROR;

	/* High pass filter configuration */
	if(lsm303agr_xl_high_pass_on_outputs_set(ctx_xl, PROPERTY_DISABLE)) return HAL_ERROR;
	//	if(lsm303agr_xl_high_pass_on_outputs_set(ctx_xl, PROPERTY_ENABLE)) return HAL_ERROR;
	//	if(lsm303agr_xl_high_pass_bandwidth_set(ctx_xl, LSM303AGR_AGGRESSIVE)) return HAL_ERROR;

	/* Set accelerometer operation mode */
	//	if(lsm303agr_xl_operating_mode_set(ctx_xl, LSM303AGR_HR_12bit)) return HAL_ERROR;
	if(lsm303agr_xl_operating_mode_set(ctx_xl, LSM303AGR_LP_8bit)) return HAL_ERROR;

	/* Set / Reset magnetic sensor mode */
	if(lsm303agr_mag_set_rst_mode_set(ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR)) return HAL_ERROR;

	/* Enable temperature compensation on mag sensor */
	if(lsm303agr_mag_offset_temp_comp_set(ctx_mg, PROPERTY_ENABLE)) return HAL_ERROR;

	/* Enable temperature sensor */
	if(lsm303agr_temperature_meas_set(ctx_xl, LSM303AGR_TEMP_ENABLE)) return HAL_ERROR;
	//if(lsm303agr_temperature_meas_set(ctx_xl, LSM303AGR_TEMP_DISABLE)) return HAL_ERROR;

	/* Set magnetometer in continuous mode */
	if(lsm303agr_mag_operating_mode_set(ctx_mg, LSM303AGR_CONTINUOUS_MODE)) return HAL_ERROR;
	//if(lsm303agr_mag_operating_mode_set(ctx_mg, LSM303AGR_POWER_DOWN)) return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef lsm303agr_xl_init(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg)
{
	HAL_StatusTypeDef 	lsm303agr_hal;
	/* Checks sensor presence and identification: */
	do {
		lsm303agr_hal = lsm303agr_check_id(ctx_xl, ctx_mg);		// Must return HAL_OK.
	} while (lsm303agr_hal);

	lsm303agr_mag_reset_set(ctx_mg, PROPERTY_ENABLE);
	do {
		lsm303agr_mag_reset_get(ctx_mg, &lsm303agr_hal);
	} while (lsm303agr_hal);

	lsm303agr_hal = lsm303agr_xl_data_rate_set(ctx_xl, LSM303AGR_XL_ODR_50Hz);

	lsm303agr_hal = lsm303agr_xl_operating_mode_set(ctx_xl, LSM303AGR_NM_10bit);

	lsm303agr_hal = lsm303agr_mag_operating_mode_set(ctx_mg, LSM303AGR_POWER_DOWN);
	lsm303agr_hal = lsm303agr_temperature_meas_set(ctx_xl, LSM303AGR_TEMP_DISABLE);

	lsm303agr_ctrl_reg6_a_t val_reg6;
	lsm303agr_hal = lsm303agr_xl_pin_int2_config_get(ctx_xl, &val_reg6);
	val_reg6.p2_act = 1;
	lsm303agr_hal = lsm303agr_xl_pin_int2_config_set(ctx_xl, &val_reg6);

	lsm303agr_hal = lsm303agr_act_timeout_set(ctx_xl, DEFAULT_ACT_TIMEOUT);
	lsm303agr_hal = lsm303agr_act_threshold_set(ctx_xl, DEFAULT_ACT_TRESHOLD);

	lsm303agr_hal = lsm303agr_xl_high_pass_bandwidth_set(ctx_xl, LSM303AGR_AGGRESSIVE);
	lsm303agr_hal = lsm303agr_xl_high_pass_on_outputs_set(ctx_xl, PROPERTY_ENABLE);

	return HAL_OK;
}

HAL_StatusTypeDef lsm303agr_mg_init(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg){
	HAL_StatusTypeDef 	rst;

	/* Checks sensor presence and identification: */
	do {
		rst = lsm303agr_check_id(ctx_xl, ctx_mg);		// Must return HAL_OK.
		if(rst==HAL_ERROR) printf("erro\r\n");
	} while (rst);

	/* Restore default configuration for magnetometer */
	lsm303agr_mag_reset_set(ctx_mg, PROPERTY_ENABLE);
	do {
		lsm303agr_mag_reset_get(ctx_mg, &rst);
	} while (rst);

	/* Enable Block Data Update */
	if(lsm303agr_xl_block_data_update_set(ctx_xl, PROPERTY_ENABLE)) return HAL_ERROR;
	if(lsm303agr_mag_block_data_update_set(ctx_mg, PROPERTY_ENABLE)) return HAL_ERROR;

	/* Set Output Data Rate */
	if( lsm303agr_xl_data_rate_set(ctx_xl,LSM303AGR_XL_POWER_DOWN) ) return HAL_ERROR;
	if( lsm303agr_temperature_meas_set(ctx_xl, LSM303AGR_TEMP_DISABLE) ) return HAL_ERROR;
	if(lsm303agr_mag_data_rate_set(ctx_mg, LSM303AGR_MG_ODR_50Hz)) return HAL_ERROR;

	/* Set / Reset magnetic sensor mode */
	if( lsm303agr_mag_set_rst_mode_set(ctx_mg, LSM303AGR_SENS_OFF_CANC_EVERY_ODR) ) return HAL_ERROR;

	/* Enable temperature compensation on mag sensor */
	if( lsm303agr_mag_offset_temp_comp_set(ctx_mg, PROPERTY_ENABLE) ) return HAL_ERROR;

	/* Enable temperature sensor */
	if( lsm303agr_temperature_meas_set(ctx_xl, LSM303AGR_TEMP_ENABLE) ) return HAL_ERROR;

	/* Set magnetometer in continuous mode */
	if( lsm303agr_mag_operating_mode_set(ctx_mg, LSM303AGR_CONTINUOUS_MODE) ) return HAL_ERROR;

	/* Set magnetometer in low power mode */
	if( lsm303agr_mag_power_mode_set(ctx_mg, LSM303AGR_LOW_POWER) ) return HAL_ERROR;

	/* Set magnetometer threshold */
	if( lsm303agr_mag_int_gen_treshold_set(ctx_mg, (uint8_t*) &u16_strong_threshold[traffic.u8_strong_mag_sensivity] ) ) return HAL_ERROR;

	/* Disable DRDY and Enable Threshold interrupt event */
	if( lsm303agr_mag_drdy_on_pin_set(ctx_mg, 1) ) return HAL_ERROR;
	if( lsm303agr_mag_int_on_pin_set(ctx_mg, 1) ) return HAL_ERROR;

	/* Read/Clear generator configuration register */
	uint8_t res;
	lsm303agr_mag_int_gen_conf_get(ctx_mg, (lsm303agr_int_crtl_reg_m_t*) &res);

	/* Enable interrupt event */
	lsm303agr_int_crtl_reg_m_t crtl_reg_bits_conf = {
			.ien = 1,		// INT1 Enable
			.iel = 0,		// Latched mode
			.iea = 1,		// '0' active low mode, '1' active high mode
			.not_used_01 = 0,
			.zien = 1,		// int on z-axis
			.yien = 1,		// int on y-axis
			.xien = 1,		// int on x-axis
	};
	if( lsm303agr_mag_int_gen_conf_set(ctx_mg, (lsm303agr_int_crtl_reg_m_t*) &crtl_reg_bits_conf) ) return HAL_ERROR;

	return HAL_OK;
}

void lsm303agr_get_mems_data(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg, float *acceleration_mg, float *magnetic_mG, float temperature_degC)
{
	lsm303agr_reg_t reg;
	axis3bit16_t 	data_raw_acceleration;
	axis3bit16_t 	data_raw_magnetic;
	axis1bit16_t 	data_raw_temperature;

	lsm303agr_xl_status_get(ctx_xl, &reg.status_reg_a);
	if (reg.status_reg_a.zyxda)
	{
		// Read accelerometer raw data.
		memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
		lsm303agr_acceleration_raw_get(ctx_xl, data_raw_acceleration.u8bit);

		// Separates and converts to [mg] unit.
		acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[0] );
		acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[1] );
		acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg( data_raw_acceleration.i16bit[2] );
	}

	lsm303agr_mag_status_get(ctx_mg, &reg.status_reg_m);
	if (reg.status_reg_m.zyxda)
	{
		// Read magnetic field data.
		memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
		lsm303agr_magnetic_raw_get(ctx_mg, data_raw_magnetic.u8bit);

		// Separates and converts to [mgauss] unit.
		magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[0]);
		magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[1]);
		magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss( data_raw_magnetic.i16bit[2]);
	}

	lsm303agr_temp_data_ready_get(ctx_xl, &reg.byte);
	if (reg.byte)
	{
		// Read temperature data.
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lsm303agr_temperature_raw_get(ctx_xl, data_raw_temperature.u8bit);

		// Converts to [°C] unit.
		temperature_degC = lsm303agr_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
	}
	return;
}

/*************************************************************************************************/
/*    LOW LEVEL FUNCTIONS                                                                         */
/*************************************************************************************************/
int32_t accel_write(void *handle, uint8_t reg, uint8_t *buff, uint16_t len)
{
	// enable auto incremented in multiple read/write commands
	reg |= 0x80;
	return (HAL_I2C_Mem_Write( (I2C_HandleTypeDef *) handle, LSM303AGR_I2C_ADD_XL, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 1000));
}

int32_t accel_read(void *handle, uint8_t reg, uint8_t *buff, uint16_t len)
{
	// enable auto incremented in multiple read/write commands
	reg |= 0x80;
	return (HAL_I2C_Mem_Read((I2C_HandleTypeDef *) handle, LSM303AGR_I2C_ADD_XL, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 1000));
}

int32_t magnet_write(void *handle, uint8_t reg, uint8_t *buff, uint16_t len)
{
	// enable auto incremented in multiple read/write commands
	reg |= 0x80;
	return (HAL_I2C_Mem_Write((I2C_HandleTypeDef *) handle, LSM303AGR_I2C_ADD_MG, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 1000));
}


int32_t magnet_read(void *handle, uint8_t reg, uint8_t *buff, uint16_t len)
{

	// enable auto incremented in multiple read/write commands
	reg |= 0x80;
	return (HAL_I2C_Mem_Read((I2C_HandleTypeDef *) handle, LSM303AGR_I2C_ADD_MG, reg, I2C_MEMADD_SIZE_8BIT, buff, len, 1000));

}

/*************************************************************************************************/
/*    MATHEMATICCAL FUNCTIONS                                                                    */
/*************************************************************************************************/

float CalculateAverage(float *data_vector, uint16_t samples)
{
	float sum = 0.0;

	for(uint8_t i=0; i<samples; ++i)
	{
		sum += data_vector[i];
	}
	return (sum/samples);
}

float CalculateVariance(float *data_vector, uint16_t samples, float average)
{
	float differ, var_sum = 0.0;

	for(uint8_t i=0; i<samples; i++)
	{
		differ = data_vector[i] - average;
		var_sum = var_sum + pow(differ,2);
	}
	return (var_sum / samples);
}

float CalculateStandardDeviation (float variance)
{
	return sqrt(variance);
}

float CalculateSMA(float *data_vector_x, float *data_vector_y, float *data_vector_z, uint16_t samples)
{
	float sum_x = 0.0;
	float sum_y = 0.0;
	float sum_z = 0.0;

	for(uint8_t i=0; i<samples; ++i)
	{
		sum_x += abs(data_vector_x[i]);
		sum_y += abs(data_vector_y[i]);
		sum_z += abs(data_vector_z[i]);
	}
	return ((sum_x + sum_y + sum_z)/samples);
}

float CalculateSVM(float axis_value_x, float axis_value_y, float axis_value_z)
{
	return sqrt((axis_value_x * axis_value_x)+(axis_value_y * axis_value_y)+(axis_value_z * axis_value_z));
}

