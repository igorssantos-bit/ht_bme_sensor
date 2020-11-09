#include <stdint.h>
#include "filtro.h"

/*
*  Calcula o valor medio de um buffer após a inserção de um novo valor e a exclusão do valor mais antigo.
*  Entrada: 
*        pbuff: ponteiro para o buffer onde estão armazenados os valores para o filtro
*        numAmostras: numero de amostras utilizado no filtro. Espera-se que esse número seja igual ao tamanho do buffer de amostras
*        valorNovo: novo valor a ser inserido no buffer de amostras e com o qual será calculado o novo valor médio
* 
*/
int16_t filtroMedia( int16_t valorNovo, int16_t *pBuff, uint16_t numAmostras){
	int32_t valorMedio = 0;

     // Atualiza o buffer de amostras, sendo que 0 é o elemento mais antigo
	 // OBS: TROCAR ESSA MODO DE FAZER POR PONTEIROS PARA O ELEMENTO MAIS ANTIGO
	 // EVITANDO ASSIM PERDA DE TEMPO COM REALOCAÇÃO DE VALORES NO BUFFER
     for (uint16_t i = 0; i < (numAmostras - 1); i++){
		 pBuff[i] = pBuff[i+1];                                 // TROCAR CONFORME DESCRITO NA OBS ACIMA
		 valorMedio += pBuff[i];
	 }
	 pBuff[numAmostras-1] = valorNovo;
	 valorMedio += pBuff[numAmostras-1];
	 valorMedio /= numAmostras;
	 
	return valorMedio;
}


/*
*  Calcula o valor medio de um buffer após a inserção de um novo valor e a exclusão do valor mais antigo.
*  Input: 
*        st_traffic_buffer: estrutura com as caracteristicas do buffer e ponteiro para o buffer
*        valorNovo: novo valor a ser inserido no buffer de amostras e com o qual será calculado o novo valor médio
*  Output: 
*        st_traffic_buffer: estrutura atualizada após o filtro
*/
int16_t filtroMediaSt( int16_t valorNovo, st_filter_buffer *pStBuffer){
	int32_t valorMedio = 0;


	pStBuffer->p16_bufAddress[pStBuffer->next_pos] = valorNovo;      //*(pStBuffer->p16_bufAddress + pStBuffer->next_pos) = valorNovo;
    for (uint16_t i = 0; i < pStBuffer->bufferSize; i++){
	   valorMedio += pStBuffer->p16_bufAddress[i];
    }
	valorMedio /= pStBuffer->bufferSize;
		
    pStBuffer->media = valorMedio;
	pStBuffer->next_pos = pStBuffer->next_pos + 1;
    if ( ( pStBuffer->next_pos % pStBuffer->bufferSize ) == 0 ){
	   pStBuffer->next_pos = 0;
    }	

	return valorMedio;
}

float kalmanFilterAtualizar(float *pant, float *xant, float valorMedido, float Qw, float Qv){
	float pprox = 0;
	float xprox = 0;
	//float Qw = 0.2; float Qv = 20;     -> r�pido
	//float Qw = 0.002; float Qv = 1500; -> lento
	float KalmanGain = 0;

	//
	//
	//    Equa��o de Propaga��o
	//    xprox(i)=A*xant(i-1);                     % do estado
	//    pprox(i)=A*pant(i-1)*A + B*Qw*B;          % da covariancia do estado
	//
	//
	xprox = (*xant);
	pprox = (*pant) + Qw;

	//
	//      Equa��es de Atualiza��o:
	//      K(i)=pprox(i)*C/(C*pprox(i)*C + Qv);       % do ganho de Kalman
	//      xant(i)=xprox(i) + K(i)*(y(i)-C*xprox(i)); % do estado
	//      pant(i)=(1 - K(i)*C)*pprox(i);             % da covariancia do estado
	//
	KalmanGain = pprox / (pprox + Qv);
	(*xant) = xprox + KalmanGain * (valorMedido - xprox);
	(*pant) = (1 - KalmanGain) * pprox;

	return *xant; // valor calculado

}
	
/*************************************************************************************************/
/*    END OF FILE                                                                                */
/*************************************************************************************************/
