/*
 * sig_sensors.h
 *
 *  Created on: Oct 8, 2020
 *      Author: igors
 */

#ifndef INC_SIG_SENSORS_H_
#define INC_SIG_SENSORS_H_

#include <stdbool.h>
#include "stm32l0xx_hal.h"
#include "i2c.h"
#include "adc.h"

#include "lsm303.h"
#include "lsm303agr_reg.h"
#include "si7021-a20.h"
#include "bme680.h"
#include "bme680_defs.h"

/*! Structure with variables related to LSM303 */
typedef struct {
	uint8_t data_ready;
   int16_t i16_data_x;
   int16_t i16_data_y;
   int16_t i16_data_z;
   float f_temp;
} st_lsm_data_t;

typedef struct{
	stmdev_ctx_t dev_ctx_xl;
	stmdev_ctx_t dev_ctx_mg;
	st_lsm_data_t	acc_raw_data;
	st_lsm_data_t	mag_raw_data;
} st_lsm303_t;

/*! Structure with variables related to SI7021 sensor.*/
typedef struct{
	int16_t temperature;
	uint16_t humidity;
} st_si70xx_t;

/*! Structure with variables related to HT-ADC sensor .*/
typedef struct{
	uint16_t voltage;
} st_battery_t;

/*! Structure with variables related to BME680 sensor .*/
typedef struct{
	struct bme680_dev gas_sensor;
	int16_t temperature;
	uint32_t pressure;
	uint32_t humidity;
	uint32_t gas_resistance;
} st_bme680_t;

volatile st_si70xx_t si;
volatile st_lsm303_t lsm;
volatile st_battery_t bat;
volatile st_bme680_t bme;

void init_si702(void);
void init_lsm303agr(void);
bool lsm_mag_ready(stmdev_ctx_t *dev_ctx_mg);
void lsm_read_mag_data(void);
void read_lsm303agr_data(void);
void init_bme680(void);

#endif /* INC_SIG_SENSORS_H_ */
