/*
 * sig_sensors.c
 *
 *  Created on: Oct 8, 2020
 *      Author: igors
 */

#include "sig_sensors.h"

extern I2C_HandleTypeDef hi2c1;

void init_lsm303agr(void){
	/* Initialize LSM-accelerometer driver interface. */
	lsm.dev_ctx_xl.write_reg 	= accel_write;
	lsm.dev_ctx_xl.read_reg 	= accel_read;
	lsm.dev_ctx_xl.handle 	= (I2C_HandleTypeDef *) &hi2c1;
	/* Initialize LSM-magnetometer driver interface. */
	lsm.dev_ctx_mg.write_reg 	= magnet_write;
	lsm.dev_ctx_mg.read_reg 	= magnet_read;
	lsm.dev_ctx_mg.handle 	= (I2C_HandleTypeDef *) &hi2c1;

	//if( lsm303agr_mg_init((stmdev_ctx_t*) &lsm.dev_ctx_xl, (stmdev_ctx_t*) &lsm.dev_ctx_mg) != HAL_OK){
	if( lsm303agr_mg_init((stmdev_ctx_t*) &lsm.dev_ctx_xl, (stmdev_ctx_t*) &lsm.dev_ctx_mg) != HAL_OK){
		printf("LSM303AGR Initialization failed\r\n");
		Error_Handler();
	}
}

bool lsm_mag_ready(stmdev_ctx_t *dev_ctx_mg){
    lsm303agr_reg_t reg;

    lsm303agr_mag_status_get(dev_ctx_mg, &reg.status_reg_m);
    if (reg.status_reg_m.zyxda){
    	return true;
    }else{
    	return false;
    }
}

void lsm_read_mag_data(void){
	axis3bit16_t 	data_raw_magnetic;
	axis1bit16_t 	data_raw_temperature;

	// Leitura do magnetometro
	memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
	lsm303agr_magnetic_raw_get((stmdev_ctx_t*) &lsm.dev_ctx_mg, data_raw_magnetic.u8bit);
	lsm.mag_raw_data.i16_data_x = data_raw_magnetic.i16bit[0];
	lsm.mag_raw_data.i16_data_y = data_raw_magnetic.i16bit[1];
	lsm.mag_raw_data.i16_data_z = data_raw_magnetic.i16bit[2];

	// Leitura da temperatuna no acelerometro
	memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
	lsm303agr_temperature_raw_get((stmdev_ctx_t*) &lsm.dev_ctx_xl, data_raw_temperature.u8bit);
	lsm.acc_raw_data.f_temp = lsm303agr_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);

	// Limpa flag DRDY
	un_system_flags.flag.lsm_mg_int1_dataready = 0;

}

void read_lsm303agr_data(void){
	lsm303agr_reg_t reg;
	axis3bit16_t 	data_raw_acceleration;
	axis3bit16_t 	data_raw_magnetic;
	axis1bit16_t 	data_raw_temperature;

	lsm303agr_xl_status_get((stmdev_ctx_t*) &lsm.dev_ctx_xl, &reg.status_reg_a);
	if (reg.status_reg_a.zyxda)
	{
		lsm.acc_raw_data.data_ready = 1;
		// Read accelerometer raw data.
		memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
		lsm303agr_acceleration_raw_get((stmdev_ctx_t*) &lsm.dev_ctx_xl, data_raw_acceleration.u8bit);
		lsm.acc_raw_data.i16_data_x = data_raw_acceleration.i16bit[0];
		lsm.acc_raw_data.i16_data_y = data_raw_acceleration.i16bit[1];
		lsm.acc_raw_data.i16_data_z = data_raw_acceleration.i16bit[2];
		//printf("'acc_x': '%d', 'acc_y': '%d', 'acc_z': '%d'\r\n", lsm.acc_raw_data.i16_raw_x, lsm.acc_raw_data.i16_raw_y, lsm.acc_raw_data.i16_raw_z);
	}


	lsm303agr_mag_status_get((stmdev_ctx_t*) &lsm.dev_ctx_mg, &reg.status_reg_m);
	if (reg.status_reg_a.zyxda)
	{
		lsm.mag_raw_data.data_ready = 1;
		/* Read magnetic field data */
		memset(data_raw_magnetic.u8bit, 0x00, 3*sizeof(int16_t));
		lsm303agr_magnetic_raw_get((stmdev_ctx_t*) &lsm.dev_ctx_mg, data_raw_magnetic.u8bit);
		lsm.mag_raw_data.i16_data_x = data_raw_magnetic.i16bit[0];
		lsm.mag_raw_data.i16_data_y = data_raw_magnetic.i16bit[1];
		lsm.mag_raw_data.i16_data_z = data_raw_magnetic.i16bit[2];
		//printf("'mag_x': '%d', 'mag_y': '%d', 'mag_z': '%d'\r\n", lsm.mag_raw_data.i16_raw_x, lsm.mag_raw_data.i16_raw_y, lsm.mag_raw_data.i16_raw_z);
	}

	lsm303agr_temp_data_ready_get((stmdev_ctx_t*) &lsm.dev_ctx_xl, &reg.byte);
	if (reg.byte)
	{
		// Read temperature data.
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lsm303agr_temperature_raw_get((stmdev_ctx_t*) &lsm.dev_ctx_xl, data_raw_temperature.u8bit);
		lsm.acc_raw_data.f_temp = lsm303agr_from_lsb_hr_to_celsius(data_raw_temperature.i16bit);
		//lsm.acc_raw_data.i16_temp = data_raw_temperature.i16bit+25;
		//printf("'temp': '%f'\r\n", lsm.acc_raw_data.f_temp);
	}

}

void init_bme680(void){

	// SDO Grounded -> 0x76
	bme.gas_sensor.dev_id = BME680_I2C_ADDR_PRIMARY;
	bme.gas_sensor.intf = BME680_I2C_INTF;
	bme.gas_sensor.handle = (I2C_HandleTypeDef *) &hi2c1;
	bme.gas_sensor.read = user_i2c_read;
	bme.gas_sensor.write = user_i2c_write;
	bme.gas_sensor.delay_ms = user_delay_ms;
	/* amb_temp can be set to 25 prior to configuring the gas sensor
	 * or by performing a few temperature readings without operating the gas sensor.
	 */
	bme.gas_sensor.amb_temp = 25;


	int8_t rslt = BME680_OK;
	rslt = bme680_init( &bme.gas_sensor);
	if(rslt!=BME680_OK)
		printf("Erro ao inicializar BME = %d\r\n", rslt);
}
