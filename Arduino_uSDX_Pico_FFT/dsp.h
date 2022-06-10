#ifndef __DSP_H__
#define __DSP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * dsp.h
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 *
 * See dsp.c for more information 
 */



#define FSAMP 480000UL  // freq AD sample / 3 channels = 160kHz
#define FSAMP_AUDIO 16000UL  // audio freq sample   32kHz=critical time
#define ADC_CLOCK_DIV ((uint16_t)(48000000UL/FSAMP))  //48Mhz / 480Khz = 100 
#define FRES      500u    //Hz resolucao de frequencias desejado para cada bin
#define FFT_NSAMP      ((((uint16_t)((FSAMP / 3u) / FRES))+1u) & (~(uint16_t)1u))  // must be even  160k / 50 = 320
//FFT max freq = (FSAMP/3) / 2
#define FFT_NUMFREQ    (FFT_NSAMP/2)



/* 
 * DAC_RANGE defines PWM cycle, determining DAC resolution and PWM frequency.
 * DAC resolution = Vcc / DAC_RANGE
 * PWM frequency = Fsys / DAC_RANGE
 * A value of 250 means 125MHz/250=500kHz
 * ADC is 12 bit, so resolution is by definition 4096
 * To eliminate undefined behavior we clip off the upper 4 sample bits.
 */
#define DAC_RANGE  256u
#define DAC_BIAS  (DAC_RANGE/2u)
#define ADC_RANGE 4096u
#define ADC_BIAS  (ADC_RANGE/2u)




//extern volatile uint16_t adc_audio_count;
//extern volatile uint16_t adc_waterfall_count;



void dsp_setagc(int agc);
void dsp_setmode(int mode);
void dsp_setvox(int vox);
int dsp_getmode(void);

//extern volatile uint16_t adc_audio_ready;
extern volatile uint16_t tim_count;
//extern volatile uint16_t fft_samples_ready;
//extern volatile uint16_t fft_samp_pos;    //number of samples saved for FFT
extern volatile uint16_t fft_samples_ready;
extern volatile uint16_t fft_display_graf_new;

#define AUD_GRAPH_NUM_COLS  100

#define AUD_NUM_VAR    (6u)  // number of variables on buffer for low freq = audio graphic
#define AUD_NUM_SAMP   (AUD_GRAPH_NUM_COLS*3u)  // number of variables on buffer for low freq = audio graphic
#define AUD_SAMP_I     0u
#define AUD_SAMP_Q     1u
#define AUD_SAMP_MIC   2u
#define AUD_SAMP_A     3u
#define AUD_SAMP_PEAK  4u
#define AUD_SAMP_GAIN  5u
#define AUD_STATE_SAMP_IN      0
#define AUD_STATE_SAMP_RDY     10
//#define AUD_STATE_SAMP_GRAPH   20
//#define AUD_STATE_SAMP_DONE    30
extern volatile int16_t aud_samp[AUD_NUM_VAR][AUD_NUM_SAMP];  //samples buffer for FFT and waterfall    only 0-1 used for I and Q  (3=MIC)  [NL][NCOL]
extern volatile uint16_t aud_samples_state;

extern volatile bool tx_enabled;
#define DSP_SETPTT(on)			tx_enabled = (on)

#define FIFO_START_FFT  10
#define FIFO_FFT_READY  20
#define FIFO_IQ_SAMPLE  30


void dsp_init();
void dsp_loop();


#ifdef __cplusplus
}
#endif
#endif
