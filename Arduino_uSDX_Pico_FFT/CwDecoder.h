#ifndef __CWDECODER_H__
#define __CWDECODER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * CwDecoder.h
 *
 * Created: Oct 2024
 * Author: Klaus Fensterseifer
 *
 * See CwDecoder.c for more information 
 */



#define  MAX_CW_RX_CNT  40   // 40 * 
#define  MAX_CW_RX_INDEX  40   // 40 * 
extern uint16_t cw_rx[MAX_CW_RX_INDEX][2];
extern uint16_t cw_rx_array;
extern uint16_t cw_rx_index;
extern uint32_t cw_rx_avg;
extern uint16_t cw_rx_cnt;

void CwDecoder_InicTable(void);
void CwDecoder_Inic(void);
void CwDecoder_Exit(void);
void CwDecoder_array_in(void);


#ifdef __cplusplus
}
#endif
#endif
