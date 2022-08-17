#ifndef __USDX_TX_PHASEAMPL_H__
#define __USDX_TX_PHASEAMPL_H__

#ifdef __cplusplus
extern "C" {
#endif






#define F_XTAL     27005000  // 27MHz SI5351 crystal
#define F_MCU      20000000  // 20MHz ATMEGA328P crystal
#define F_SAMP_TX      4800  //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#if(F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4800LL / 20000000);  // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX  F_SAMP_TX
#endif







extern void uSDX_TX_PhaseAmpl_setup(void);
extern void uSDX_TX_PhaseAmpl_loop(void);











#ifdef __cplusplus
}
#endif
#endif
