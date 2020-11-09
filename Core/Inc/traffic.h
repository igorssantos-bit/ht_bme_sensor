#ifndef _TRAFFIC_H
#define _TRAFFIC_H

/*************************************************************************************************/
/*    DEFINES                                                                                    */
/*************************************************************************************************/
#define SENSIBILIDADE_DETECCAO          	20
#define SENSIBILIDADE_LIBERACAO         	10

typedef enum {
	ESTADO_AGUARDANDO_DETECCAO = 1,
	ESTADO_VEICULO_DETECTADO,
	ESTADO_INICIAL,
	ESTADO_ERRO_AMPLITUDE
} st_estado_eixo;

typedef struct {

	/* Traffic Variables */
	uint8_t u8_strong_mag_sensivity;
	uint8_t u8_traffic_threshold;
	uint8_t u8_traffic_time2transmit_first_day_hour;
	uint8_t u8_unit_time2transmit_first_day_hour;
	uint8_t u8_traffic_time2transmit_second_day_hour;
	uint8_t u8_unit_time2transmit_second_day_hour;
	uint8_t u8_traffic_type; // 0 -> MagX e MagY, 1 -> MagX e MagZ, 2 -> MagY e MagZ, 3 -> MagXYZ e MagZ (2 bits apenas)
	uint8_t u8_traffic_reset_counters;
	uint32_t u32_cnt_trafegoX;
	uint32_t u32_cnt_trafegoY;
	uint32_t u32_cnt_trafegoZ;
	uint32_t u32_cnt_trafegoXYZ;
	uint8_t u8_traffic_erro;							// indica quando detectar um erro ou alarme no processo de contagem
	uint8_t u8_npsat;										// controla a quantidade de pontos sequenciais acima do threshold para entrar/sair da detecção
	uint8_t u8_sfreq;                           	// controla a frequencia da transmissão dos dados pelo magnetômetr
	uint8_t u8_tmhbuf;                          	// controla o tamanho do buffer da média
	uint8_t u8_nptd;                            	// controla a quantidade de amostras a esperar após entrar em um estado antes de aceitar novas variações no estado
	uint8_t u8_vatualizacao_media_detectado;    	// controla a velocidade de atualização da média quando no estado detectado. Vai de 0 a 7 com incrementos de 50 em 50.
	uint8_t u8_traffic_threshold_inferior;      	// threshold para sair do estado detectado

} st_system_status_t;

volatile st_system_status_t traffic;

/*************************************************************************************************/
/*    PROTOTYPES                                                                                    */
/*************************************************************************************************/

void traffic_init (void);
void calibracao(int16_t *valorRefX, int16_t *valorRefY, int16_t *valorRefZ, uint16_t);
void traffic_Process (void);

void filtraDadoMagnetometro(int16_t *valorMedioX, int16_t *valorMedioY, int16_t *valorMedioZ);
void atualizaMedia(int16_t *valorRef, int16_t rawData, uint8_t eixo);

uint8_t algoritmoContagemEixo (int16_t magnetometroValor, int16_t valorReferencia, uint8_t *pEstado, uint16_t linhaDeCorteSuperior, uint16_t linhaDeCorteInferior ,uint16_t *pignoreSamples, uint8_t *pDetectadoCount);
uint8_t algoritmoContagemEixoXYZ (int16_t magX, int16_t referenciaX, int16_t magY, int16_t referenciaY, int16_t magZ, int16_t referenciaZ, uint8_t *pEstado, uint16_t linhaDeCorte);
uint8_t algoritmoContagemEixoXYZ_2(uint8_t contx, uint8_t conty, uint8_t contz, uint8_t estadox, uint8_t estadoy, uint8_t estadoz);

void clearBuffer(uint8_t *pbuf, uint8_t len);

#endif
