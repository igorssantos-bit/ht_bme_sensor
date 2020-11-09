#ifndef _FILTRO_H
#define _FILTRO_H

typedef struct {

	int16_t *p16_bufAddress;
	uint16_t bufferSize;
	uint16_t next_pos;
	int16_t media;

} st_filter_buffer;

/*************************************************************************************************/
/*    PROTOTYPES                                                                                 */
/*************************************************************************************************/

int16_t filtroMedia( int16_t valorNovo, int16_t *pBuff, uint16_t numAmostras);
int16_t filtroMediaSt( int16_t valorNovo, st_filter_buffer *pStBuffer);
float kalmanFilterAtualizar(float *pant, float *xant, float valorMedido, float Qw, float Qv);

#endif
