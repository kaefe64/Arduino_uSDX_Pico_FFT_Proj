#ifndef __HMI_H__
#define __HMI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * hmi.h
 *
 * Created: Apr 2021
 * Author: Arjan te Marvelde
 *
 * See hmi.c for more information 
 */

extern bool ptt_active;
extern uint32_t hmi_freq;  

void hmi_init(void);
void hmi_evaluate(void);

#ifdef __cplusplus
}
#endif
#endif
