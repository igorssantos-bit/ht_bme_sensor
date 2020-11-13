/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "HT_mcu_api.h"
#include "HT_sigfox_api.h"
#include "HT_sigfox_api.h"
#include "sig_sensors.h"
#include "filtro.h"
#include "traffic.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TIM_HandleTypeDef  Tim6_Handler={.Instance=TIM6};

volatile un_system_flags_t un_system_flags;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ST_Init(void);
void fnAPP_Init_Standard_Values(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_ADC_Init();
	/* USER CODE BEGIN 2 */
	mcuConfig();

	/********** OPEN AND CONFIFIGURES SIGFOX LIBRARY IN RCZ2 *********************/
	/********** IN ORDER TO OPEN OTHER RCZS, SEE SIGFOX_API.h **/
	/********** BASICALLY CHANGES TO OTHER RC VALUE LIKE RCZ3 **/
	configRegion(RCZ2);

	//init_lsm303agr();
	init_bme680();
	printf("Sistema Inicializado\r\n");
	//traffic_init();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

   /* Get the total measurement duration so as to sleep or wait till the
    * measurement is complete */
   uint16_t meas_period;
   int8_t rslt;
   bme680_get_profile_dur(&meas_period, &bme.gas_sensor);

   struct bme680_field_data data;

	while(1){
		user_delay_ms(meas_period); /* Delay till the measurement is ready */

		rslt = bme680_get_sensor_data(&data, &bme.gas_sensor);

		printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
				data.pressure / 100.0f, data.humidity / 1000.0f );
		/* Avoid using measurements from an unstable heating setup */
		if(data.status & BME680_GASM_VALID_MSK)
			printf(", G: %d ohms", data.gas_resistance);

		printf("\r\n");

		/* Trigger the next measurement if you would like to read data out continuously */
		if (bme.gas_sensor.power_mode == BME680_FORCED_MODE) {
			rslt = bme680_set_sensor_mode(&bme.gas_sensor);
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
			|RCC_PERIPHCLK_RTC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

   HAL_SYSTICK_Config( (uint32_t )HAL_RCC_GetHCLKFreq() / 1000 );
   HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

}

/* USER CODE BEGIN 4 */

//uint16_t celsius2sigtemp(){
//}

//int16_t sigtemp2celsius(){
//}

void configRegion(rc_mask RCZ) {
	ST_SFX_ERR open_err = ST_SFX_ERR_NONE;

	switch(RCZ){
	case RCZ1:
		ST_RF_API_reduce_output_power(RCZ1_OUTPUT_POWER);
		open_err = St_Sigfox_Open_RCZ(RCZ1);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ2:
		//		ST_RF_API_reduce_output_power(RCZ2_OUTPUT_POWER);
		ST_RF_API_reduce_output_power(-32);
		open_err = St_Sigfox_Open_RCZ(RCZ2);

		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ3:
		open_err = St_Sigfox_Open_RCZ(RCZ3);
		ST_RF_API_reduce_output_power(RCZ3_OUTPUT_POWER);
		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ4:
		open_err = St_Sigfox_Open_RCZ(RCZ4);
		ST_RF_API_reduce_output_power(RCZ4_OUTPUT_POWER);
		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ5:
		open_err = St_Sigfox_Open_RCZ(RCZ5);
		ST_RF_API_reduce_output_power(RCZ5_OUTPUT_POWER);
		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ6:
		open_err = St_Sigfox_Open_RCZ(RCZ6);
		ST_RF_API_reduce_output_power(RCZ6_OUTPUT_POWER);
		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	case RCZ7:
		open_err = St_Sigfox_Open_RCZ(RCZ7);
		ST_RF_API_reduce_output_power(RCZ7_OUTPUT_POWER);
		if(open_err != 0)
			printf("Open rcz error: %X\n", open_err);

		break;
	default:
		break;
	}

}

//sfx_error_t sendFrame(e_uplink_frames_type_t frame) {
//
//	/********** SEND MESSAGE TO RCZ2 ****************************/
//
//	sfx_error_t err;
//
//	uint8_t DL_request = 0;
//	uint8_t DL_recvd = 0;
//
//	uint8_t i = 0;
//	uint8_t frame_len = 0;
//
//	switch(frame){
//	case DATA_INFO_2_FRAME:
//	case DATA_EVENT_2_FRAME:
//	case DATA_EVENT_1_FRAME:
//	case DATA_INFO_1_FRAME:
//		frame_len = 12;
//		err=SIGFOX_API_send_frame(uplink_frame.bytes, frame_len, downlink_frame.bytes, 3, 0);
//		break;
//
//	case DAILY_REQUEST_FRAME:
//		frame_len = 3;
//		DL_request = 1;
//		comm_timeout = 40;
//		err=SIGFOX_API_send_frame(uplink_frame.bytes, frame_len, downlink_frame.bytes, 3, 1);
//		comm_timeout = 0;
//		if( err == 0)
//			DL_recvd = 1;
//		break;
//
//	case CONFIG_REQUEST_FRAME:
//		frame_len = 2;
//		DL_request = 1;
//		comm_timeout = 40;
//		err=SIGFOX_API_send_frame(uplink_frame.bytes, frame_len, downlink_frame.bytes, 3, 1);
//		comm_timeout = 0;
//		if( err == 0)
//			DL_recvd = 1;
//		break;
//
//	case LOG_FRAME:
//		frame_len = 10;
//		err=SIGFOX_API_send_frame(uplink_frame.bytes, frame_len, downlink_frame.bytes, 3, 0);
//		break;
//
//	default:
//		break;
//	}
//	printf("!{\n\r");
//	printf("\"Label\": \"Uplink\",\n\r");
//	printf("\"State\": %u,\n\r", currentState);
//	printf("\"Timestamp\": %lu,\n\r", timestamp);
//	printf("\"Minutes\": %lu,\n\r", minutesCount);
//	printf("\"UL Message\": ");
//	printf("\"");
//	for(i = 0; i < frame_len; i++){
//		printf("%02X", uplink_frame.bytes[i]);
//	}
//	printf("\",\n\r");
//	printf("\"DL request\": %u,\n\r", DL_request);
//	printf("\"UL Error\": 0x%02X\n\r", (uint8_t)err);
//	printf("}!\n\r");
//
//
//	if(DL_recvd){
//		printf("!{\n\r");
//		printf("\"Label\": \"Downlink\",\n\r");
//		printf("\"State\": %u,\n\r", currentState);
//		printf("\"Timestamp\": %lu,\n\r", timestamp);
//		printf("\"Minutes\": %lu,\n\r", minutesCount);
//		printf("\"DL Message\": ");
//		printf("\"");
//		for(i = 0; i < 8; i++){
//			printf("%02X", downlink_frame.bytes[i]);
//		}
//		printf("\"\n\r}!\n\r");
//	}
//
//	//	FEM_Operation(FEM_TX);
//	//	HAL_Delay(500);
//	//	FEM_Operation(FEM_SHUTDOWN);
//	//	S2LPShutdownEnter();
//	//	HAL_Delay(500);
//
//	return err;
//}

sfx_error_t sendFrame(void) {

	/********** SEND MESSAGE TO RCZ2 ****************************/

	uint8_t customer_data[12]={0xAA, 0xAA, 0xAA, 0xAA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA, 0xBA};
	uint8_t customer_resp[8];
	sfx_error_t err;
	sfx_bool downlink_request = 0;

	/********** FUNCTION PARAMETERS  ****************************/
	/********** THE LAST ONE IS TO REQUEST DOWNLINK ************/
	/********** 1 - YES, 0 - NO	 ******************************/

	err=SIGFOX_API_send_frame(customer_data,sizeof(customer_data),customer_resp, 3, downlink_request);

	if(downlink_request) {
		printf("Customer resp: {");

		for(uint16_t i = 0; i < 7; i++)
			printf("0x%x,", customer_resp[i]);

		printf("0x%x}\n\r", customer_resp[7]);
	}

	printf("\nError Send Frame: %X\n", err);


	return err;
}

void mcuConfig(void) {
	ST_SFX_ERR stSfxRetErr;

	ST_Init();

	NVM_BoardDataType sfxConfiguration;
	stSfxRetErr = ST_Sigfox_Init(&sfxConfiguration, 0);

	if(stSfxRetErr != ST_SFX_ERR_NONE)
	{
		/* If an error occured reading Sigfox credentials (for example the board has never been registered)
		 * automatically set the test mode credentials. */
		if(stSfxRetErr == ST_SFX_ERR_CREDENTIALS)
		{
			sfxConfiguration.id = 0;
			memset(sfxConfiguration.pac, 0x00, 8);
			sfxConfiguration.rcz = 0;

		} else {
			printf("Error!\n");
		}
	}

	/* Calibrate RTC in case of STM32*/
#if  !(defined(BLUENRG2_DEVICE) || defined(BLUENRG1_DEVICE))
	/* The low level driver uses the internal RTC as a timer while the STM32 is in low power.
  This function calibrates the RTC using an auxiliary general purpose timer in order to
  increase its precision. */
	ST_MCU_API_TimerCalibration(500);
#endif
	printf("!{\n\r");
	printf("\"Label\": \"HT32SX\",\n\r");
	printf("\"ID\": \"%.8X\",\n\r", (unsigned int)sfxConfiguration.id);
	printf("\"PAC\": \"");
	//	printf("Sigfox iMCP HT32SX\n");

	//	printf("ID: %.8X - PAC: ", (unsigned int)sfxConfiguration.id);

	for(uint16_t i = 0; i < sizeof(sfxConfiguration.pac); i++)
	{
		printf("%.2X", sfxConfiguration.pac[i]);
	}

	printf("\",\n\r");

	/*			SET S2LP XTAL FREQUENCY														*/
	/*			DO NOT CHANGE IT																	*/

	ST_RF_API_set_xtal_freq(50000000); 

	/*			SET A PROPER FREQUENCY OFFSET											*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_freq_offset(sfxConfiguration.freqOffset);
	printf("\"Freq. Offset\": %d,\n\r", (int)sfxConfiguration.freqOffset);

	/*			SET LBT OFFSET																		*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_lbt_thr_offset(sfxConfiguration.lbtOffset);
	//	printf("LBT %d \n", (int)sfxConfiguration.lbtOffset);
	printf("\"LBT\": %li,\n\r", sfxConfiguration.lbtOffset);

	/*			SET RSSI OFFSET																		*/
	/*			THIS VALUE CAN BE FOUND IN CREDENTIALS 						*/

	ST_RF_API_set_rssi_offset(sfxConfiguration.rssiOffset);
	//	printf("RSSI %d \n", (int)sfxConfiguration.rssiOffset);
	printf("\"LBT\": %d,\n\r", (int)sfxConfiguration.rssiOffset);
	printf("}!\n\r");
}

void ST_Init(void)
{
	/* Put the radio off */
	S2LPShutdownInit();
	HAL_Delay(10);
	S2LPShutdownExit();

	/* Init TIM6 which will trigger TX */
	SdkEvalTimersState(&Tim6_Handler, ENABLE);

	/* Auto detect settings, if EEPROM is available */
#if EEPROM_PRESENT == EEPROM_YES
	/* Identify the S2-LP RF board reading some production data */
	S2LPManagementIdentificationRFBoard();
#elif EEPROM_PRESENT==EEPROM_NO
	/* Set XTAL frequency with offset */
	S2LPRadioSetXtalFrequency(XTAL_FREQUENCY+XTAL_FREQUENCY_OFFSET);

	/* Set the frequency base */
	S2LPManagementSetBand(BOARD_FREQUENCY_BAND);

	/* Configure PA availability */
#if S2LP_FEM_PRESENT == S2LP_FEM_NO
	S2LPManagementSetRangeExtender(0);
#else
	S2LPManagementSetRangeExtender(1);
#endif
#endif

	/* uC IRQ config and enable */

	S2LPIRQInit();
	S2LPIRQEnable(ENABLE, 0);

	/* FEM Initialization */
	FEM_Init();

	/* TCXO Initialization */
	//TCXO_Init();

}

void fnAPP_Init_Standard_Values(void){
	traffic.u8_strong_mag_sensivity = 5;
	traffic.u8_traffic_time2transmit_first_day_hour = 30;			// 30
	traffic.u8_unit_time2transmit_first_day_hour = 1;				//minutos
	traffic.u8_traffic_time2transmit_second_day_hour = 30;           // 30
	traffic.u8_unit_time2transmit_second_day_hour = 1;				//minutos
	traffic.u8_traffic_type  = 0;									// X e Y
	traffic.u8_traffic_reset_counters = 0;                          //
	traffic.u8_traffic_threshold = 25;                              //
	traffic.u8_traffic_threshold_inferior = 15;                     //
	traffic.u32_cnt_trafegoX = 0;
	traffic.u32_cnt_trafegoY = 0;
	traffic.u32_cnt_trafegoZ = 0;
	traffic.u32_cnt_trafegoXYZ = 0;
	traffic.u8_traffic_erro = 0;
	traffic.u8_npsat = 1; // 3 amostras
	traffic.u8_sfreq = 1;  //0 = 800, 1 = 400Hz, 2 = 200, 3 = 100
	traffic.u8_tmhbuf = 6; //350 amostras
	traffic.u8_nptd = 1;   //40 amostras
	traffic.u8_vatualizacao_media_detectado = 7;  // 0 a 7 sendo que 0 -> desabilita e depois incrementa em multiplos de 50

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error Handler\r\n");
	while(1);

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
