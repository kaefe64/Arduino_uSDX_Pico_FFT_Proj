#ifndef __USDX_TX_PHASEAMPL_H__
#define __USDX_TX_PHASEAMPL_H__

#ifdef __cplusplus
extern "C" {
#endif






#define F_SAMP_TX      5336   //4800U  //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#define _F_SAMP_TX     F_SAMP_TX



extern  I2C i2c;
extern  SI5351 si5351;




extern void uSDX_TX_PhaseAmpl_setup(void);
extern void uSDX_TX_PhaseAmpl_loop(void);











#ifdef __cplusplus
}
#endif
#endif
