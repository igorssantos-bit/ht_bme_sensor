/**
 ******************************************************************************
 * @file           : lsm303.h
 * @brief          : Header for lsm303.c file.
 *                   This file contains the common defines and function
 *                   prototypes of the application.
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef INC_LSM303_H_
#define INC_LSM303_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "lsm303agr_reg.h"

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;

typedef union{
  int32_t i32bit[3];
  uint8_t u8bit[12];
} axis3bit32_t;

int32_t accel_write		(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t accel_read		(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t magnet_write	(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);
int32_t magnet_read		(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len);

HAL_StatusTypeDef lsm303agr_check_id		(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg);
HAL_StatusTypeDef lsm303agr_default_init	(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg);
HAL_StatusTypeDef lsm303agr_xl_init			(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg);
HAL_StatusTypeDef lsm303agr_mg_init			(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg);

void lsm303agr_get_mems_data		(stmdev_ctx_t *ctx_xl, stmdev_ctx_t *ctx_mg,
									 float *acceleration_mg, float *magnetic_mG, float temperature_degC);

float CalculateAverage				(float *data_vector, uint16_t samples);
float CalculateVariance				(float *data_vector, uint16_t samples, float average);
float CalculateStandardDeviation 	(float variance);
float CalculateSMA					(float *data_vector_x, float *data_vector_y, float *data_vector_z, uint16_t samples);
float CalculateSVM					(float axis_value_x, float axis_value_y, float axis_value_z);

#endif /* INC_LSM303_H_ */
