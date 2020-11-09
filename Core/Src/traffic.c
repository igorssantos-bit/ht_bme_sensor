
//#include <asf.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "traffic.h"

#include "filtro.h"
#include "sig_sensors.h"

#define VALOR_MAXIMO_AMPLITUDE 250
#define SIZE_BUFFER_MEDIA 400

/*************************************************************************************************/
/*    VARIABLES                                                                                  */
/*************************************************************************************************/

// Variaveis do traffic para o filtro de kalman
float xantX = 1.0;
float pantX = 1.0;
float xantY = 1.0;
float pantY = 1.0;
float xantZ = 1.0;
float pantZ = 1.0;
float xantX_steady = 1.0;
float pantX_steady = 1.0;
float xantY_steady = 1.0;
float pantY_steady = 1.0;
float xantZ_steady = 1.0;
float pantZ_steady = 1.0;

// Buffer com dados brutos do magnetometro
int16_t bufferMediaX[SIZE_BUFFER_MEDIA];
int16_t bufferMediaY[SIZE_BUFFER_MEDIA];
int16_t bufferMediaZ[SIZE_BUFFER_MEDIA];

// Valores dos eixos
int16_t valorRefX = 0;
int16_t valorRefY = 0;
int16_t valorRefZ = 0;
int16_t valorMedioX = 0;
int16_t valorMedioY = 0;
int16_t valorMedioZ = 0;
int16_t amplitudeMaxX = 0;
int16_t amplitudeMaxY = 0;
int16_t amplitudeMaxZ = 0;

// Estado dos eixos do magnetometro
st_estado_eixo estadoX;
st_estado_eixo estadoY;
st_estado_eixo estadoZ;
st_estado_eixo estadoChangedX;
st_estado_eixo estadoChangedY;
st_estado_eixo estadoChangedZ;

// Debouncers
uint16_t ignoreSamplesX = 80;
uint16_t ignoreSamplesY = 80;
uint16_t ignoreSamplesZ = 80;
uint8_t  waitNumAmostrasX = 0;
uint8_t  waitNumAmostrasY = 0;
uint8_t  waitNumAmostrasZ = 0;

// contadores do traffic
uint16_t cont;
uint8_t contX, contY, contZ;
uint32_t periodoDetectadoX = 0;
uint32_t periodoDetectadoY = 0;
uint32_t periodoDetectadoZ = 0;


st_filter_buffer stTrafficBufferX;
st_filter_buffer stTrafficBufferY;
st_filter_buffer stTrafficBufferZ;

st_lsm_data_t dadoMagAnterior, dadoMagNovo, derivadaMag;
volatile st_system_status_t traffic;

/*************************************************************************************************/
/*    FUNCTIONS                                                                                  */
/*************************************************************************************************/

void traffic_init (void){
	// Variaveis do traffic_Process
	cont = 0, contX = 0, contY = 0, contZ = 0;

	estadoX = ESTADO_AGUARDANDO_DETECCAO;
	estadoY = ESTADO_AGUARDANDO_DETECCAO;
	estadoZ = ESTADO_AGUARDANDO_DETECCAO;
	estadoChangedX = ESTADO_AGUARDANDO_DETECCAO;
	estadoChangedY = ESTADO_AGUARDANDO_DETECCAO;
	estadoChangedZ = ESTADO_AGUARDANDO_DETECCAO;

	traffic.u32_cnt_trafegoX = 0;
	traffic.u32_cnt_trafegoY = 0;
	traffic.u32_cnt_trafegoZ = 0;
	traffic.u32_cnt_trafegoXYZ = 0;
	traffic.u8_traffic_reset_counters = 0;

	// Variaveis da calibracao
	dadoMagAnterior.i16_data_x = 0;
	dadoMagAnterior.i16_data_y = 0;
	dadoMagAnterior.i16_data_z = 0;
	dadoMagNovo.i16_data_x = 0;
	dadoMagNovo.i16_data_y = 0;
	dadoMagNovo.i16_data_z = 0;

	valorRefX = 0;
	valorRefY = 0;
	valorRefZ = 0;
	calibracao(&valorRefX,&valorRefY, &valorRefZ , 4000);     //calibracao com 4000 amostra

	for (int aux = 0; aux < ((traffic.u8_tmhbuf * 50) + 50); aux++){  //SIZE_BUFFER_MEDIA
		bufferMediaX[aux] = valorRefX;
		bufferMediaY[aux] = valorRefY;
		bufferMediaZ[aux] = valorRefZ;
	}

	//calibracao(2000);     //calibracao em 2000 amostras
	stTrafficBufferX.bufferSize = (traffic.u8_tmhbuf * 50) + 50;  // 50 a 400 -> SIZE_BUFFER_MEDIA;
	stTrafficBufferX.next_pos = 0;
	stTrafficBufferX.p16_bufAddress = (int16_t*) &bufferMediaX;
	stTrafficBufferX.media = bufferMediaX[0];

	stTrafficBufferY.bufferSize = (traffic.u8_tmhbuf * 50) + 50;  // 50 a 400 -> SIZE_BUFFER_MEDIA;
	stTrafficBufferY.next_pos = 0;
	stTrafficBufferY.p16_bufAddress = (int16_t*) &bufferMediaY;
	stTrafficBufferY.media = bufferMediaY[0];

	stTrafficBufferZ.bufferSize = (traffic.u8_tmhbuf * 50) + 50;  // 50 a 400 -> SIZE_BUFFER_MEDIA;
	stTrafficBufferZ.next_pos = 0;
	stTrafficBufferZ.p16_bufAddress = (int16_t*) &bufferMediaZ;
	stTrafficBufferZ.media = bufferMediaZ[0];

}

/*
 * Processo de calibra��o
 * considera calibrado em qnt amostras
 */
void calibracao(int16_t *valorRefX, int16_t *valorRefY, int16_t *valorRefZ, uint16_t qnt){
	uint16_t i;
	int16_t valorMedioX, valorMedioY, valorMedioZ = 0;

	xantX = 1.0;
	pantX = 1.0;
	xantY = 1.0;
	pantY = 1.0;
	xantZ = 1.0;
	pantZ = 1.0;

	printf("Calibrando\r\n");

	for (i = 0; i < qnt; i++){
		if (un_system_flags.flag.lsm_mg_int1_dataready == 1){
			lsm_read_mag_data();
			filtraDadoMagnetometro(&valorMedioX,&valorMedioY,&valorMedioZ);
		}else{
			HAL_Delay(1);
		}
	}

	(*valorRefX) = valorMedioX;
	(*valorRefY) = valorMedioY;
	(*valorRefZ) = valorMedioZ;

	// backup do xant e pant de cada eixo ap�s a calibra��o
	xantX_steady = xantX;
	pantX_steady = pantX;
	xantY_steady = xantY;
	pantY_steady = pantY;
	xantZ_steady = xantZ;
	pantZ_steady = pantZ;

	printf("Terminado\r\n");

	printf("Valor_Medio(x;y;z) = %d;", abs(valorMedioX));
	printf("%d;", abs(valorMedioY));
	printf("%d\r\n", abs(valorMedioZ));

}

void traffic_Process (void){

	if (un_system_flags.flag.lsm_mg_int1_dataready == 1){
		// Atualizando contador
		printf("Atualizando data\r\n");

		// Leitura e filtragem dos dados
		lsm_read_mag_data();
		filtraDadoMagnetometro(&valorMedioX,&valorMedioY,&valorMedioZ);

		// Contagem por eixos
		contX = 0;
		contY = 0;
		contZ = 0;
		contX = algoritmoContagemEixo (valorMedioX,valorRefX, &estadoX, traffic.u8_traffic_threshold, traffic.u8_traffic_threshold_inferior, &ignoreSamplesX, &waitNumAmostrasX);
		contY = algoritmoContagemEixo (valorMedioY,valorRefY, &estadoY, traffic.u8_traffic_threshold, traffic.u8_traffic_threshold_inferior, &ignoreSamplesY, &waitNumAmostrasY);
		contZ = algoritmoContagemEixo (valorMedioZ,valorRefZ, &estadoZ, traffic.u8_traffic_threshold, traffic.u8_traffic_threshold_inferior, &ignoreSamplesZ, &waitNumAmostrasZ);

		// Gravação dos dados por eixo
		traffic.u32_cnt_trafegoX += contX;
		traffic.u32_cnt_trafegoY += contY;
		traffic.u32_cnt_trafegoZ += contZ;
		traffic.u32_cnt_trafegoXYZ += algoritmoContagemEixoXYZ_2(contX, contY, contZ, estadoX, estadoY, estadoZ);

		// Lógica de estados eixo X
		switch (estadoX) {
		case ESTADO_AGUARDANDO_DETECCAO:
			// ATUALIZA MÉDIAS
			valorRefX =  filtroMediaSt(valorMedioX,&stTrafficBufferX);
			// EVENTOS A SEREM ANALISADOS OU EXECUTADOS NA TRANSIÇÃO DE ESTADO
			if (estadoChangedX == ESTADO_VEICULO_DETECTADO){
				estadoChangedX = ESTADO_AGUARDANDO_DETECCAO;
				ignoreSamplesX = (traffic.u8_nptd * 40);
			}
			// EVENTOS A SEREM ANALISADOS OU EXECUTADOS DURANTE TODO O ESTADO
			amplitudeMaxX = 0;
			break;

		case ESTADO_VEICULO_DETECTADO:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefX =  filtroMediaSt(valorMedioX,&stTrafficBufferX);
				}
			}
			if (estadoChangedX == ESTADO_AGUARDANDO_DETECCAO){
				estadoChangedX = ESTADO_VEICULO_DETECTADO;
				ignoreSamplesX = (traffic.u8_nptd * 40);
			}
			// ENCONTRA O VALOR MÁXIMO GERADO NO ESTADO DETECTADO
			if (abs(amplitudeMaxX) < abs(valorMedioX)){
				amplitudeMaxX = valorMedioX;
			}
			break;

		case ESTADO_ERRO_AMPLITUDE:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefX =  filtroMediaSt(valorMedioX,&stTrafficBufferX);
				}
			}
			traffic.u8_traffic_erro |= 0x01;
			if (estadoChangedX == ESTADO_VEICULO_DETECTADO){
				estadoChangedX = ESTADO_ERRO_AMPLITUDE;
			}
			break;
		default:
			break;
		}

		// Lógica de estados eixo Y
		switch (estadoY) {
		case ESTADO_AGUARDANDO_DETECCAO:
			valorRefY =  filtroMediaSt(valorMedioY,&stTrafficBufferY);
			if (estadoChangedY == ESTADO_VEICULO_DETECTADO){
				estadoChangedY = ESTADO_AGUARDANDO_DETECCAO;
				ignoreSamplesY = (traffic.u8_nptd * 40);
			}
			amplitudeMaxY = 0;
			break;

		case ESTADO_VEICULO_DETECTADO:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefY =  filtroMediaSt(valorMedioY,&stTrafficBufferY);
				}
			}
			// EVENTOS A SEREM ANALISADOS OU EXECUTADOS NA TRANSIÇÃO DE ESTADO
			if (estadoChangedY == ESTADO_AGUARDANDO_DETECCAO){
				estadoChangedY = ESTADO_VEICULO_DETECTADO;
				ignoreSamplesY = (traffic.u8_nptd * 40);
			}
			if (abs(amplitudeMaxY) < abs(valorMedioY)){
				amplitudeMaxY = valorMedioY;
			}
			break;

		case ESTADO_ERRO_AMPLITUDE:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefY =  filtroMediaSt(valorMedioY,&stTrafficBufferY);
				}
			}
			traffic.u8_traffic_erro |= 0x02;
			if (estadoChangedX == ESTADO_VEICULO_DETECTADO){
				estadoChangedX = ESTADO_ERRO_AMPLITUDE;
			}
			break;
		default:
			break;
		}

		// Lógica de estados eixo Z
		switch (estadoZ) {
		case ESTADO_AGUARDANDO_DETECCAO:
			valorRefZ =  filtroMediaSt(valorMedioZ,&stTrafficBufferZ);
			if (estadoChangedY == ESTADO_VEICULO_DETECTADO){
				estadoChangedY = ESTADO_AGUARDANDO_DETECCAO;
				ignoreSamplesZ = (traffic.u8_nptd * 40);
			}
			amplitudeMaxZ = 0;
			break;

		case ESTADO_VEICULO_DETECTADO:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefZ =  filtroMediaSt(valorMedioZ,&stTrafficBufferZ);
				}
			}
			// EVENTOS A SEREM ANALISADOS OU EXECUTADOS NA TRANSIÇÃO DE ESTADO
			if (estadoChangedZ == ESTADO_AGUARDANDO_DETECCAO){
				estadoChangedZ = ESTADO_VEICULO_DETECTADO;
				ignoreSamplesZ = (traffic.u8_nptd * 40);
			}
			if (abs(amplitudeMaxZ) < abs(valorMedioZ)){
				amplitudeMaxZ = valorMedioZ;
			}
			break;

		case ESTADO_ERRO_AMPLITUDE:
			// fora do estado aguardando detecção -> atualiza a média lentamente
			if (traffic. u8_vatualizacao_media_detectado){
				if ((cont % (traffic.u8_vatualizacao_media_detectado * 50)) == 0){
					valorRefZ =  filtroMediaSt(valorMedioZ,&stTrafficBufferZ);
				}
			}
			traffic.u8_traffic_erro |= 0x04;
			if (estadoChangedZ == ESTADO_VEICULO_DETECTADO){
				estadoChangedZ = ESTADO_ERRO_AMPLITUDE;
			}
			break;
		default:
			break;
		}
	}

	// Debug
	if (cont % 50 == 0){
		cont = 0;
		printf("contador (X;Y;Z;XYZ): %lu, ", traffic.u32_cnt_trafegoX);
		printf("%lu, ", traffic.u32_cnt_trafegoY);
		printf("%lu, ", traffic.u32_cnt_trafegoZ);
		printf("%lu\r\n", traffic.u32_cnt_trafegoXYZ);
	}
	cont++;
}


/*
 *  filtraDadoMagnetometro
 *  L� os dados do magnet�metro e efetua a filtragem do mesmo.
 *  Atualiza o valor medio em cada um dos eixos
 *  Retorna verdadeiro quando a leitura foi feita e falso caso contr�rio
 */
//TODO: esta fun��o corretamente
void filtraDadoMagnetometro(int16_t *valorMedioX, int16_t *valorMedioY, int16_t *valorMedioZ){
	(*valorMedioX) = (int16_t) kalmanFilterAtualizar(&pantX, &xantX, lsm.mag_raw_data.i16_data_x,0.2,20.0); //,0.0000002,15000.0
	(*valorMedioY) = (int16_t) kalmanFilterAtualizar(&pantY, &xantY, lsm.mag_raw_data.i16_data_y,0.2,20.0); //,0.0000002,15000.0
	(*valorMedioZ) = (int16_t) kalmanFilterAtualizar(&pantZ, &xantZ, lsm.mag_raw_data.i16_data_z,0.2,20.0); //,0.0000002,15000.0
}

/*
 * Atualiza media
 * Mant�m a media de calibra��o atualizada individualmente
 */
void atualizaMedia(int16_t *valorRef, int16_t rawData, uint8_t eixo){
	float valorRaw = 0.0;

	valorRaw = (float)rawData * 1.0;
	if (eixo == 0){
		(*valorRef) = (int16_t) kalmanFilterAtualizar(&pantX_steady, &xantX_steady, valorRaw,0.001,15000.0);
	}
	else{
		if (eixo == 1){
			(*valorRef) = (int16_t) kalmanFilterAtualizar(&pantY_steady, &xantY_steady, valorRaw,0.001,15000.0);
		}
		else{
			if (eixo == 2){
				(*valorRef) = (int16_t) kalmanFilterAtualizar(&pantZ_steady, &xantZ_steady, valorRaw,0.001,15000.0);
			}
		}
	}

}

/*
 *  Entrada:
 *	       magnetometroValor: valor da leitura do magnet�metro. O mais adequado � que este valor venha filtrado.
 *		   linhaDeCorte: valor que ser� usado como limiar de detec��o.
 *          pstEstado: ponteiro para o estado atual da detec��o.
 *  Altera valor:
 *				pstEstado: ponteiro para o estado atual da contagem
 *               pignoreSamples: ir� manter o sensor  em modo detectado pelo n�mero de leituras indicadas nessa vari�vel
 *                               Verificou-se que a influ�ncia da passagem de um ve�culo sobre o sensor dura de 200 a 350 amostras.
 *                               Bons resultados foram obtidos com a vari�vel pignoreSamples com valor de 200 amostras p/ amostragem de 800Hz.
 *
 *  Retorna:
 *		    1: se contou um ve�culo
 *		    0: caso n�o tenha contado
 *
 * OSB: Espera-se que o sinal comporte-se da seguinte forma:  ...00011110000....  ou  ... 0001< n_amostras = x >0000... ou
 *                                                            ... 0001< n_amostras = x >1100 ...
 *                                                            onde zero representa um valor abaixo da linha de corte e
 *                                                            um representa valores superiores da linha de corte.
 *      O eixo Z ter� um sinal + e ao passar um ve�culo o sinal � reduzido, mas n�o chega a valores menores que zero.
 * Modo de Instala��o:
 *	 eixo Z fica vertical ao solo de forma que o sinal dele seja +.
 * O algoritmo agora serve para qualquer um dos outros eixos tamb�m
 **/

uint8_t algoritmoContagemEixo (int16_t magnetometroValor, int16_t valorReferencia, uint8_t *pEstado, uint16_t linhaDeCorteSuperior, uint16_t linhaDeCorteInferior ,uint16_t *pignoreSamples, uint8_t *pDetectadoCount){
	uint8_t contadorTrafego=0;
	uint16_t diferenca = 0;

	diferenca = abs (abs(magnetometroValor) - abs(valorReferencia));
	contadorTrafego = 0;
	if (diferenca > VALOR_MAXIMO_AMPLITUDE ){
		(*pEstado) = ESTADO_ERRO_AMPLITUDE;
		return contadorTrafego;
	}

	if  (*pEstado == ESTADO_AGUARDANDO_DETECCAO){
		if ( (*pignoreSamples) > 0){
			(*pignoreSamples)--;
			(*pDetectadoCount) = 0;							//reset no contador
		}
		else{
			if (abs(linhaDeCorteSuperior) < abs(diferenca)){
				//fnDELAY_LOOP_ms(2);  // original era sem delay
				(*pDetectadoCount) = (*pDetectadoCount) + 1;
				if (*pDetectadoCount >= traffic.u8_npsat){  //if (*pDetectadoCount >=  NUM_AMOSTRAS_COUNT){
					contadorTrafego = 1;
					(*pEstado) = ESTADO_VEICULO_DETECTADO;
				}
			}
			else{
				contadorTrafego = 0;
				(*pDetectadoCount) = 0;								//reset no contador
			}
		}
	}
	else{
		if ( (*pignoreSamples) > 0){
			(*pignoreSamples)--;
			(*pDetectadoCount) = 0;								//reset no contador
		}
		else{
			if ( abs(linhaDeCorteInferior) > abs(diferenca)){			//  linhaDeCorte/2 = linhaDeCorteInferior -> p/ criar uma  histerese para sair da detec��o
				(*pDetectadoCount) = (*pDetectadoCount) + 1;
				if ( (*pDetectadoCount) >= traffic.u8_npsat){ //if ( (*pDetectadoCount) >= NUM_AMOSTRAS_COUNT){
					(*pEstado) = ESTADO_AGUARDANDO_DETECCAO;
				}
			}
			else{
				(*pDetectadoCount) = 0;							//reset no contador
			}
		}
		contadorTrafego = 0;
	}

	return contadorTrafego;
}

/*
 *  Entrada:
 *	       magnetometroValor: valor da leitura do magnet�metro. O mais adequado � que este valor venha filtrado.
 *		   linhaDeCorte: valor que ser� usado como limiar de detec��o.
 *          pstEstado: ponteiro para o estado atual da detec��o.
 *  Altera valor:
 *				pstEstado: ponteiro para o estado atual da contagem
 *
 *  Retorna:
 *		    1: se contou um ve�culo
 *		    0: caso n�o tenha contado
 *
 * OSB: Espera-se que o sinal comporte-se da seguinte forma:  ...00011110000....  ,
 *                                                            onde zero representa um valor abaixo da linha de corte e
 *                                                            um representa valores superiores da linha de corte.
 *      O eixo Z ter� um sinal + e ao passar um ve�culo o sinal � reduzido, mas n�o chega a valores menores que zero.
 * Modo de Instala��o:
 *	 eixo Z fica vertical ao solo de forma que o sinal dele seja +.
 * O algoritmo agora serve para qualquer um dos outros eixos tamb�m
 **/

uint8_t algoritmoContagemEixoXYZ (int16_t magX, int16_t referenciaX, int16_t magY, int16_t referenciaY, int16_t magZ, int16_t referenciaZ, uint8_t *pEstado, uint16_t linhaDeCorte){
	uint8_t contadorTrafego=0;
	int16_t diferencaX = 0;
	int16_t diferencaY = 0;
	int16_t diferencaZ = 0;
	int16_t diferenca = 0;

	diferencaX = abs(abs(magX) - abs(referenciaX));
	diferencaY = abs(abs(magY) - abs(referenciaY));
	diferencaZ = abs(abs(magZ) - abs(referenciaZ));
	diferenca = diferencaX + diferencaY + diferencaZ;

	if  ( (*pEstado) == ESTADO_AGUARDANDO_DETECCAO){
		if (abs(linhaDeCorte) < abs(diferenca)){
			contadorTrafego = 1;
			(*pEstado) = ESTADO_VEICULO_DETECTADO;
		}
	}
	else{
		if ( abs(linhaDeCorte) > abs(diferenca)){
			(*pEstado) = ESTADO_AGUARDANDO_DETECCAO;
			contadorTrafego = 0;
		}
	}

	return contadorTrafego;
}

uint8_t algoritmoContagemEixoXYZ_2(uint8_t contx, uint8_t conty, uint8_t contz, uint8_t estadox, uint8_t estadoy, uint8_t estadoz){


	if (contx && conty && contz){
		return 1;
	}
	else{
		if (contx && (conty || estadoy) && (contz || estadoz)){
			return 1;
		}
		else{
			if (conty && (contx || estadox) && (contz || estadoz)){
				return 1;
			}
			else{
				if (contz && (conty || estadoy) && (contx || estadox)){
					return 1;
				}

			}

		}

	}
	return 0;
}

/*
 * Zera todas as posi��es de zero a len de um buffer
 */
void clearBuffer(uint8_t *pbuf, uint8_t len){

	for (uint8_t cont = 0; cont<len; cont++){
		pbuf[cont] = 0;
	}
}

/*************************************************************************************************/
/*    END OF FILE                                                                                */
/*************************************************************************************************/
