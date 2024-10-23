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
#define FSAMP_AUDIO 16000U  // audio freq sample   32kHz=critical time
#define ADC_CLOCK_DIV ((uint16_t)(48000000UL/FSAMP))  //48Mhz / 480Khz = 100 
#define FRES      500u    //Hz resolucao de frequencias desejado para cada bin
#define FFT_NSAMP      ((((uint16_t)((FSAMP / 3u) / FRES))+1u) & (~(uint16_t)1u))  // must be even  160k / 500 = 320
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
#define DAC_RANGE  255u
#define DAC_BIAS  (DAC_RANGE/2u)
#define ADC_RANGE 4095u
#define ADC_BIAS  (ADC_RANGE/2u)


#ifdef PY2KLA_setup
#define EXCHANGE_I_Q  1    //include or remove this #define in case the LSB/USB and the lower/upper frequency of waterfall display are reverted - hardware I and Q pin dependent
#endif


#define PHASE_AMPLITUDE  11
#define I_Q_QSE          22
//
//Here you can choose one of the two methods for uSDR_Pico transmission
//
#define TX_METHOD    I_Q_QSE            // uSDR_Pico original project generating I and Q signal to a QSE mixer
//#define TX_METHOD    PHASE_AMPLITUDE    // DO NOT USE - is not ready - used for Class E RF amplifier - see description at: uSDX_TX_PhaseAmpl.cpp

#define IQ_TX_ATTENUATION   2    //practical value after testing min value without saturate the IQ output for TX


#define  LOW_PASS_16KHZ_AVERAGE_SUM   11
#define  LOW_PASS_16KHZ_FIR  55
// choose with type of filter used for I and Q after  160kHz sampling  to deliever to the 16kHz audio process
//#define LOW_PASS_16KHZ  LOW_PASS_16KHZ_AVERAGE_SUM
#define LOW_PASS_16KHZ  LOW_PASS_16KHZ_FIR




//extern volatile uint16_t adc_audio_count;
//extern volatile uint16_t adc_waterfall_count;
extern volatile int16_t adc_result[3];   //

#define MAX_SMETER_DISPLAY_TIME      (150  *16)   // X ms / (1/16kHz) = number of 16kHz ints to get X ms
extern volatile int16_t max_a_sample;
extern volatile int16_t display_a_sample;
extern volatile int16_t smeter_display_time;

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

#define PEAK_AVG_SHIFT   2     //affects agc speed 
extern volatile int32_t peak_avg_shifted;     // signal level detector after AGC = average of positive values

#define AGC_GAIN_SHIFT  6        //shift corresponding to AGC_GAIN_MAX
#define AGC_GAIN_MAX    (1u<<AGC_GAIN_SHIFT)       //max attenuation agc can do   signal * agc_gain / AGC_GAIN_MAX
extern volatile uint16_t agc_gain;
#define FFT_GAIN_SHIFT   4  //gain = 1 to 16 / 16
extern volatile uint16_t fft_gain;

extern volatile uint16_t dac_iq, dac_audio;

//extern volatile uint32_t hmi_freq_fft;

#define FIFO_START_FFT  10
#define FIFO_FFT_READY  20
#define FIFO_IQ_SAMPLE  30



void dsp_init();
void dsp_loop();


#ifdef __cplusplus
}
#endif
#endif
