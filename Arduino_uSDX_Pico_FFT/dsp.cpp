/*
 * dsp.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Signal processing of RX and TX branch, to be run on the second processor core.
 * Each branch has a dedicated routine that must run on set times.
 * The period is determined by reads from the inter-core fifo, by the dsp_loop() routine. 
 * This fifo is written from core0 from a 16us timer callback routine (i.e. 62.5kHz)
 *
 * The RX branch:
 * - Sample I and Q QSD channels, and shift into I and Q delay line (62.5 kHz per channel)
 * - Low pass filter: Fc=4kHz
 * - Quarter rate (15.625 kHz) to improve low freq behavior of Hilbert transform
 * - Calculate 15 tap Hilbert transform on Q
 * - Demodulate, taking proper delays into account
 * - Push to Audio output DAC
 *
 * Always perform audio sampling (62.5kHz) and level detections, in case of VOX active
 *
 * The TX branch (if VOX or PTT):
 * - Low pass filter: Fc=3kHz
 * - Eight rate (7.8125 kHz) to improve low F behavior of Hilbert transform
 * - Generate Q samples by doing a Hilbert transform
 * - Push I and Q to QSE output DACs
 *
 */
/*
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
*/
#include "Arduino.h"
//#include "SPI.h"
//#include "TFT_eSPI.h"
//#include "dma.h"
#include "pwm.h"
#include "adc.h"
#include "irq.h"
//#include "time.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hmi.h"
//#include "hardware/gpio.h"
//#include "pico/stdlib.h"
#include "hardware/structs/bus_ctrl.h"
#include "dsp.h"
#include "uSDR.h"
#include "kiss_fftr.h"
#include "TFT_eSPI.h"
#include "display_tft.h"


#define LOW_PASS_FITER    1   //use the low pass for I q and MIC (otherwise only the DMA average low pass filter)


#define ADC0_IRQ_FIFO 		22		// FIFO IRQ number
#define GP_PTT				    15		// PTT pin 20 (GPIO 15)



volatile uint16_t tim_count = 0;





//local functions
bool rx(void);
bool tx(void);
bool vox(void);


/**************************************************************************************
 * AGC reference level is log2(0x40) = 6, where 0x40 is the MSB of half DAC_RANGE
 * 1/AGC_DECAY and 1/AGC_ATTACK are multipliers before agc_gain value integrator
 * These values should ultimately be set by the HMI.
 * The time it takes to a gain change is the ( (Set time)/(signal delta) ) / samplerate
 * So when delta is 1, and attack is 64, the time is 64/15625 = 4msec (fast attack)
 * The decay time is about 100x this value
 * Slow attack would be about 4096
 **************************************************************************************/
#define AGC_REF		6
#define AGC_DECAY	8192
#define AGC_FAST	64
#define AGC_SLOW	4096
#define AGC_OFF		32766
volatile uint16_t agc_decay  = AGC_OFF;
volatile uint16_t agc_attack = AGC_OFF;
void dsp_setagc(int agc)
{
	switch(agc)
	{
	case 1:		//SLOW, for values see hmi.c
		agc_attack = AGC_SLOW;
		agc_decay  = AGC_DECAY;
		break;
	case 2:		//FAST
		agc_attack = AGC_FAST;
		agc_decay  = AGC_DECAY;
		break;
	default: 	//OFF
		agc_attack = AGC_OFF;
		agc_decay  = AGC_OFF;
		break;
	}
}

/**************************************************************************************
 * MODE is modulation/demodulation 
 * This setting steers the signal processing branch chosen
 **************************************************************************************/
volatile uint16_t dsp_mode;				// For values see hmi.c, assume {USB,LSB,AM,CW}
void dsp_setmode(int mode)
{
	dsp_mode = (uint16_t)mode;
}

int dsp_getmode(void)
{
  return(dsp_mode);
}


/**************************************************************************************
 * VOX LINGER is the number of 16us cycles to wait before releasing TX mode
 * The level of detection is related to the maximum ADC range.
 **************************************************************************************/
#define VOX_LINGER		500000/16
#define VOX_HIGH		ADC_BIAS/2
#define VOX_MEDIUM		ADC_BIAS/4
#define VOX_LOW			ADC_BIAS/16
#define VOX_OFF			0
volatile uint16_t vox_count;
volatile uint16_t vox_level = VOX_OFF;
void dsp_setvox(int vox)
{
	switch(vox)
	{
	case 1:
		vox_level = VOX_LOW;
		break;
	case 2:
		vox_level = VOX_MEDIUM;
		break;
	case 3:
		vox_level = VOX_HIGH;
		break;
	default: 
		vox_level = VOX_OFF;
		vox_count = 0;
		break;
	}
}


/* 
 * Low pass filters Fc=3, 7 and 15 kHz (see http://t-filter.engineerjs.com/)
 * Settings: sample rates 62500, 31250 or 15625 Hz, stopband -40dB, passband ripple 5dB
 * Note: 8 bit precision, so divide sum by 256 (this could be improved when 32bit accumulator)
 */
//int16_t lpf3_62[15] =  {  3,  3,  5,  7,  9, 10, 11, 11, 11, 10,  9,  7,  5,  3,  3};	// Pass: 0-3000, Stop: 6000-31250
                                        // Calculate only every fourth sample.  So effective sample rate will be 15625Hz


#if 0
/**************************************************************************************

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

**************************************************************************************/

#define LPF_TAP_NUM 13

static int16_t lpf_taps[LPF_TAP_NUM] = {
  1326,
  2635,
  460,
  -3277,
  -360,
  10353,
  16726,
  10353,
  -360,
  -3277,
  460,
  2635,
  1326
};

#endif




#if 0


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 2000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 3200 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define LPF_TAP_NUM 15

static int lpf_taps[LPF_TAP_NUM] = {
  -695,
  -1627,
  -2278,
  -1611,
  949,
  4846,
  8431,
  9888,
  8431,
  4846,
  949,
  -1611,
  -2278,
  -1627,
  -695
};



#endif


//#if 0



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 2400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 3600 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define LPF_TAP_NUM 17

static int lpf_taps[LPF_TAP_NUM] = {
  398,
  344,
  -574,
  -2167,
  -2882,
  -895,
  3935,
  9201,
  11491,
  9201,
  3935,
  -895,
  -2882,
  -2167,
  -574,
  344,
  398
};



//#endif



#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 2800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 3800 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define LPF_TAP_NUM 17

static int lpf_taps[LPF_TAP_NUM] = {
  843,
  1742,
  1498,
  -624,
  -2970,
  -2315,
  2745,
  9374,
  12440,
  9374,
  2745,
  -2315,
  -2970,
  -624,
  1498,
  1742,
  843
};


#endif






// Obs.:  LPF time critical, use max 25 filter taps
// Obs.:  The input signal ADC should be previous filthered to < half sampling frequency





#define LPF_SHIFT 16  // 16 bits coef 

volatile uint16_t dac_iq, dac_audio;
volatile bool tx_enabled; //, vox_active;


volatile uint16_t tim_count_loc;


/**************************************************************************************
 * Some macro's
 * See Alpha Max plus Beta Min algorithm for MAG (vector length)
 **************************************************************************************/
#define ABS(x)		((x)<0?-(x):(x))
#define MAG(i,q)	(ABS(i)>ABS(q) ? ABS(i)+((3*ABS(q))>>3) : ABS(q)+((3*ABS(i))>>3))







/*
#define TAMSUB  8u
#define TAMSUB_EXTRA  2u
int16_t vetsub[TAMSUB+TAMSUB_EXTRA] = {  0, -15, -30, -15, 0, 15, 30, 15, 0, -15 };
uint16_t possub = 0;
*/
/*
#define TAMSUB  10u
#define TAMSUB_EXTRA  3u
int16_t vetsub[TAMSUB+TAMSUB_EXTRA] = {  0, -5, -10, -10, -5, 0, 5, 10, 10, 5, 0, -5, -10 };
uint16_t possub = 0;
*/
#define TAMSUB  4u
#define TAMSUB_EXTRA  1u
int16_t vetsub[TAMSUB+TAMSUB_EXTRA] = {  0, -5, 0, 5,  0};
uint16_t possub = 1;



int dma_chan;

//the AD sample frequency is as high as possible to get information for FFT bandwidth
//there shoud be a hardware low pass filter 
//the audio sample is as high as possible to make easy to filter the undesirable (sample) freq on output hardware low pass filter (only RC filter)
//with audio sample freq too much high, there is no time to process each sample (and the low pass FIR need more taps)
//the DMA copy a block of samples from AD to  fft_samp[]
//the size of the block is how much the sample frequency must be divided to get the audio samples frequency
//ADC sample frequency = 480kHz, but for 3 samples = I, Q and MIC   resulting 160kHz of sample freq for each one
//the audio sample freq could rise to 32kHz   with critical time conditions (care to be taken when including code)
// 160kHz / 16kHz  = BLOCK_NSAMP = 30 samples / block
//the DMA will interrupt @16kHz with 30 samples saved on adc_smap[]
//the last samples from fft_samp[] will be summed to generate an audio sample. This is also a low pass filter for audio samples.
//   the number of samples summed to generate the audio sample = 16   to make a low pass <8kHz and make easy to divide >>4  (the division takes more time than shift)
//all samples have a bias value (half of Vref) and for digital filter an FFT they will be shifted to zero, removing the bias value
//the audio samples are taken always, every time DMA interrupt, 
//the samples for FFT are taken from time to time when FFT to water fall graphic is ready with last samples
//when waiting for FFT, the samples are taken on separate area of fft_samp[] to not override the samples been used in FFT
//there are some extra previous samples for FFT/Hilbert/Graph because to calculate the first result, it needs some previous samples
#define BLOCK_NSAMP    (FSAMP/FSAMP_AUDIO)    //block = 480k / 16k = 30 samples
#define BLOCK_NSET     (BLOCK_NSAMP/3)        //block = 10 sets of 3 samples
#define NBLOCK       ((FFT_NSAMP+(BLOCK_NSET-1)) / BLOCK_NSET)  // number of blocks necessary for FFT  320 / 30 = 10.666  =11
#define ADC_NUM_BLOCK  (2u)  //save last 1 blocks + 1 partial = average for 10 + 6 = 16 samples
#define ADC_NUM_BLOCK_MASK  (1u)  // 0 - 1
volatile int16_t adc_samp[ADC_NUM_BLOCK][BLOCK_NSAMP];  //samples buffer    0-1 used for I and Q  3=MIC=VOX  [NL][NCOL]
volatile uint16_t adc_samp_block_pos = 0;       //actual sample block reading by ADC and DMA
volatile uint16_t adc_samp_last_block_pos = 0;  //last sample block read
volatile int16_t adc_samp_sum[ADC_NUM_BLOCK][3];  //save the sum of each block  12 bits = 0-4095 * 10  must fit in 16 bits

//bias = samples average = DC component, used to remome the DC from the samples
#define AVG_BIAS_SHIFT  8  //16   
volatile int32_t adc_result_bias[3] = { (ADC_BIAS << AVG_BIAS_SHIFT), (ADC_BIAS << AVG_BIAS_SHIFT), (ADC_BIAS << AVG_BIAS_SHIFT) };
volatile int16_t adc_result[3];   //

#define FFT_NUM_BLOCK   (NBLOCK + 2u)  // number of blocks FFT + HILBERT_TAP_NUM = 15 = fit in 2*blocks=20
volatile int16_t fft_samp[FFT_NUM_BLOCK][BLOCK_NSAMP];  //samples buffer for FFT and waterfall    only 0-1 used for I and Q  (3=MIC)  [NL][NCOL]
volatile uint16_t fft_samp_block_pos = 0;    
volatile uint16_t fft_samples_ready = 0;  //all buffer filled
volatile uint16_t display_graf_new = 0;   //new data for graphic ready

volatile uint16_t i_int, j_int;
/************************************************************************************** 
 * CORE1:  DMA IRQ
 * dma handler - IRQ when a block of samples was read
 * take a block o samples, calculate average for I Q MIC and store for FFT
 * 
 **************************************************************************************/
void __not_in_flash_func(dma_handler)(void)
//void dma_handler() __attribute__ ((section (".scratch_x.")));
//void dma_handler() 
{
  //*** the next DMA instructions must happen as fast as possible
  //*** do not include anything here

  // Clear the interrupt request.
  dma_hw->ints0 = 1u << dma_chan;
  // Give the channel a new wave table entry to read from, and re-trigger it
  dma_channel_set_write_addr(dma_chan, &adc_samp[adc_samp_block_pos][0], true);
//  dma_channel_set_write_addr(dma_chan, &adc_samp[adc_samp_block_pos][0], true);
  

  gpio_set_mask(1<<14);


#if 0

//generating triangle wave for test

  j_int = 0;
  for(i_int=0; i_int<BLOCK_NSAMP; )
  {  
    adc_samp[adc_samp_last_block_pos][i_int] = (vetsub[j_int]) + ADC_BIAS;
    i_int++;
    adc_samp[adc_samp_last_block_pos][i_int] = (vetsub[j_int+TAMSUB_EXTRA]) + ADC_BIAS;
    i_int++;
    adc_samp[adc_samp_last_block_pos][i_int] = j_int; //adc_samp_last_block_pos;   //(ADC_BIAS - (BLOCK_NSAMP/2)) + i_sampl1;
    i_int++;
    j_int++;
  }


#endif

#if 0

  
  // 5 sets of 3 samples @ 160kHz
  j_int = 0;
  for(i_int=0; i_int<BLOCK_NSAMP; )
  {  
    adc_samp[adc_samp_last_block_pos][i_int] = (vetsub[possub]) + ADC_BIAS;
    i_int++;
    adc_samp[adc_samp_last_block_pos][i_int] = (vetsub[possub+TAMSUB_EXTRA]) + ADC_BIAS;
    i_int++;
    adc_samp[adc_samp_last_block_pos][i_int] = possub; //adc_samp_last_block_pos;   //(ADC_BIAS - (BLOCK_NSAMP/2)) + i_sampl1;
    i_int++;

    //generating triangle wave for test
    if(++possub >= TAMSUB)  possub = 0;
  }
  

#endif





  //prepare I Q and MIC audio samples
  //sum last samples to make a low frequency average sample (160kHz to 16kHz sample freq)


  // average method used for BIAS:
  // example:  average result = average value x4 = ave_x4
  //           ave_x4 += new_value - (ave_x4/4)


  //init first sum = value  
  i_int=0;

  // bias = samples average = DC value from the samples  (it is always positive)
  adc_result_bias[0] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[0] >> AVG_BIAS_SHIFT));
  // remove bias (avg) from samples
  adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[0] >> AVG_BIAS_SHIFT);
  // sum of last 10 samples = all block
  adc_samp_sum[adc_samp_last_block_pos][0] = adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
  i_int++;


  // bias = samples average = DC value from the samples  (it is always positive)
  adc_result_bias[1] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[1] >> AVG_BIAS_SHIFT));
  // remove bias (avg) from samples
  adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[1] >> AVG_BIAS_SHIFT);
  // sum of last 10 samples = all block
  adc_samp_sum[adc_samp_last_block_pos][1] = adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
  i_int++;


  // bias = samples average = DC value from the samples  (it is always positive)
  adc_result_bias[2] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[2] >> AVG_BIAS_SHIFT));
  // remove bias (avg) from samples
  adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[2] >> AVG_BIAS_SHIFT);
  // sum of last 10 samples = all block
  adc_samp_sum[adc_samp_last_block_pos][2] = adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
  i_int++;



  //sum += value
  //save the samples sum for each block to make easy to get the last 16 samples average (10=full block,  6=partial)
  for(; i_int<BLOCK_NSAMP; )
  {
    // bias = samples average = DC value from the samples  (it is always positive)
    adc_result_bias[0] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[0] >> AVG_BIAS_SHIFT));
    // remove bias (avg) from samples
    adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[0] >> AVG_BIAS_SHIFT);
    // sum of last 10 samples = all block
    adc_samp_sum[adc_samp_last_block_pos][0] += adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
    i_int++;


    // bias = samples average = DC value from the samples  (it is always positive)
    adc_result_bias[1] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[1] >> AVG_BIAS_SHIFT));
    // remove bias (avg) from samples
    adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[1] >> AVG_BIAS_SHIFT);
    // sum of last 10 samples = all block
    adc_samp_sum[adc_samp_last_block_pos][1] += adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
    i_int++;


    // bias = samples average = DC value from the samples  (it is always positive)
    adc_result_bias[2] += (int16_t)(adc_samp[adc_samp_last_block_pos][i_int] - (adc_result_bias[2] >> AVG_BIAS_SHIFT));
    // remove bias (avg) from samples
    adc_samp[adc_samp_last_block_pos][i_int] -= (adc_result_bias[2] >> AVG_BIAS_SHIFT);
    // sum of last 10 samples = all block
    adc_samp_sum[adc_samp_last_block_pos][2] += adc_samp[adc_samp_last_block_pos][i_int]; //block samples sum to subsample at lower frequency (it is a low pass filter too)
    i_int++;
  }
 
  // result = sum of last samples = average = low pass filter
  for(i_int=0; i_int<3; i_int++)
  {
    // low pass filter with the last samples average    4096 * 10  fits on  16 bits
    // (the signal should have freqs only < 8kHz  for use in the FIR low pass filter @16kHz sample freq)
    adc_result[i_int] = adc_samp_sum[adc_samp_last_block_pos][i_int] >> 3u;  // /8 instead of /10 = little gain
  }


/*
possub++;
possub &= TAMSUB_MASK;
adc_result[0] = vetsub[possub];
adc_result[1] = vetsub[(possub+(TAMSUB/4u))&TAMSUB_MASK];
*/


  // invoque FIFO IRQ on Core0 to use the adc_result[] audio sample (there is no time for all in one core)
  multicore_fifo_push_blocking(FIFO_IQ_SAMPLE);




  
  if(fft_samples_ready == 0)  //receiving the samples
  {
    //copy new samples to FFT buffer
    for(i_int=0; i_int<BLOCK_NSAMP; )
    {  
      fft_samp[fft_samp_block_pos][i_int] = adc_samp[adc_samp_last_block_pos][i_int];
      i_int++;
      fft_samp[fft_samp_block_pos][i_int] = adc_samp[adc_samp_last_block_pos][i_int];
      i_int++;
      fft_samp[fft_samp_block_pos][i_int] = adc_samp[adc_samp_last_block_pos][i_int]; 
      i_int++;
    }
    
    fft_samp_block_pos++;
    if(fft_samp_block_pos >= FFT_NUM_BLOCK)
    {
      fft_samples_ready = 1;
    }
  }
  else if(fft_samples_ready == 1)  //waiting FFT to end
  {

  }
  else // fft_samples_ready == 2  ready with graphic
  {
  //start filling FFT samples buffer over again
  fft_samp_block_pos = 0;
  fft_samples_ready = 0;
  }




  //prepare next block position
  //avg_num_block_pos++;
  //avg_num_block_pos&=AVG_NUM_BLOCK_MASK;

  adc_samp_last_block_pos = adc_samp_block_pos;
  adc_samp_block_pos++;   // = avg_num_block_pos;
  adc_samp_block_pos&=ADC_NUM_BLOCK_MASK;
/*  
  adc_samp_block_pos++;  
  if(adc_samp_block_pos >= ADC_NUM_BLOCK)
  {
    adc_samp_block_pos = 0;
  }
*/



  //time counter
  if(++tim_count_loc >= (FSAMP_AUDIO/1000u))   // DMA 16kHz / 16 = 1kHz = 1ms
  {
    tim_count++;
    tim_count_loc = 0;
  }

   
  gpio_clr_mask(1<<14);
  
}









/************************************************************************************** 
 * CORE0:  FIFO IRQ
 * FIFO IRQ handler - IRQ when FIFO push from Core1
 * 
 * 
 **************************************************************************************/
// 
void core0_irq_handler() 
{
            
  gpio_set_mask(1<<LED_BUILTIN);


  //after handling the interrupt, we need to clear it
  //do it at the begin in case this handler takes more time,
  //the new irq will be treated just after this one
  multicore_fifo_clear_irq();


  
  // check if there is data in FIFO (should have)
  if(multicore_fifo_rvalid()) 
  {
    // pop the data from FIFO stack
    (void)multicore_fifo_pop_blocking();


    //run the application for vox, rx and tx here in the irq
    //it must be treated as soon as possible
  
    //use audio samples
    tx_enabled = ptt_active || vox();     // Sample audio and check level 
  
    if (tx_enabled)
    {
      if (vox_level != VOX_OFF)         // Only when vox is enabled
        gpio_put(GP_PTT, false);      //     drive PTT low (active)
      tx();
    }
    else
    {
      if (vox_level != VOX_OFF)         // Only when vox is enabled
        gpio_put(GP_PTT, true);       //     drive PTT high (inactive)
      rx();
    }

  }






          
  gpio_clr_mask(1<<LED_BUILTIN);

}








#define HILBERT_TAP_NUM  15



/************************************************************************************** 
 * CORE0: inside DMA IRQ 
 * rx
 * Execute RX branch signal processing, max time to spend is <16us, i.e. rate is 62.5 kHz
 * No ADC sample interleaving, read both I and Q channels.
 * The delay is only 2us per conversion, which causes less distortion than interpolation of samples.
 **************************************************************************************/
volatile int16_t i_s_raw[LPF_TAP_NUM], q_s_raw[LPF_TAP_NUM];			// Raw I/Q samples minus DC bias
volatile uint16_t peak=0;							// Peak detector running value
volatile int16_t agc_gain=0;	       				// AGC gain (left-shift value)
volatile int16_t agc_accu=0;	       				// Log peak level integrator
volatile int16_t i_s[HILBERT_TAP_NUM], q_s[HILBERT_TAP_NUM];					// Filtered I/Q samples
volatile int16_t i_dc, q_dc; 						// DC bias for I/Q channel
//volatile int rx_cnt=0;								// Decimation counter
//bool rx() __attribute__ ((section (".scratch_x.")));
bool rx(void) 
{
	int16_t q_sample, i_sample, a_sample;
	int32_t q_accu, i_accu;
	int16_t qh;
	uint16_t i;
	int16_t k;

//  gpio_set_mask(1<<LED_BUILTIN);

  
  /*
   * Low pass filter + decimation
   */
//  rx_cnt = (rx_cnt+1)&3;              // Calculate only every fourth sample
//  if (rx_cnt>0) return (true);          // So net sample time is 64us or 15.625 kHz


	/*** SAMPLING ***/
	
	q_sample = adc_result[0];						// Take last ADC 0 result, connected to Q input
	i_sample = adc_result[1];						// Take last ADC 1 result, connected to I input



/*

  if(subindo == 1)
  {
    if(++contsub >= 26)
    {
      subindo = 0;
    }
  }
  else
  {
    if(--contsub <= -26)
    {
      subindo = 1;
    }    
  }
  q_sample = (contsub*1);  // + DAC_BIAS;
  i_sample = 0;


*/


/*
possub++;
possub &= TAMSUB_MASK;
q_sample = vetsub[possub];

i_sample = vetsub[(possub+(TAMSUB/4u))&TAMSUB_MASK];

*/



    
	/*
	 * Remove DC and store new sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 * Amplitude of samples should fit inside [-2048, 2047]
	 */
//samples already subtracted from bias at DMA IRQ
//	q_sample = (q_sample&0x0fff) - ADC_BIAS;		// Clip to 12 bits and subtract mid-range
//	q_dc += q_sample/128 - q_dc/128;				//   then IIR running average
//	q_sample -= q_dc;								//   and subtract DC
//	i_sample = (i_sample&0x0fff) - ADC_BIAS;		// Same for I sample
//	i_dc += i_sample/128 - i_dc/128;
//	i_sample -= i_dc;

	/*
	 * Shift with AGC feedback from AUDIO GENERATION stage
	 * Note: bitshift does not work with negative numbers, so need to MPY/DIV
	 * This behavior in essence is exponential, complementing the logarithmic peak detector
	 */
/*
	if (agc_gain > 0)
	{
		q_sample = q_sample * (1<<agc_gain);
		i_sample = i_sample * (1<<agc_gain);
	}
	else if (agc_gain < 0)
	{
		q_sample = q_sample / (1<<(-agc_gain));
		i_sample = i_sample / (1<<(-agc_gain));
	}
*/

#ifdef LOW_PASS_FITER
	/* 
	 * Shift-in I and Q raw samples 
	 */
	for (i=0; i<(LPF_TAP_NUM-1); i++)
	{
		q_s_raw[i] = q_s_raw[i+1];					// Q raw samples shift register
		i_s_raw[i] = i_s_raw[i+1];					// I raw samples shift register
	}
	q_s_raw[(LPF_TAP_NUM-1)] = q_sample;							// Store in shift registers
	i_s_raw[(LPF_TAP_NUM-1)] = i_sample;


	q_accu = 0;										// Initialize accumulators
	i_accu = 0;
	for (i=0; i<LPF_TAP_NUM; i++)							// Low pass FIR filter
	{
		q_accu += (int32_t)q_s_raw[i]*lpf_taps[i];
		i_accu += (int32_t)i_s_raw[i]*lpf_taps[i];
	}
	q_accu = q_accu >> LPF_SHIFT;
	i_accu = i_accu >> LPF_SHIFT;
#else
  q_accu = q_sample;
  i_accu = i_sample;
#endif

  for (i=0; i<(HILBERT_TAP_NUM-1); i++)               // Shift decimated samples
  {
    q_s[i] = q_s[i+1];
    i_s[i] = i_s[i+1];
  }
	q_s[(HILBERT_TAP_NUM-1)] = q_accu;
	i_s[(HILBERT_TAP_NUM-1)] = i_accu;
	

	/*** DEMODULATION ***/
	switch (dsp_mode)
	{
	case 0:											//USB
		/* 
		 * USB demodulate: I[7] - Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu >> 12;  // / 4096L;	
		a_sample = i_s[7] - qh;
		break;
	case 1:											//LSB
		/* 
		 * LSB demodulate: I[7] + Qh,
		 * Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[ 8])*2202L;
		qh = q_accu >> 12;  // / 4096L;	
		a_sample = i_s[7] + qh;
		break;
	case 2:											//AM
		/*
		 * AM demodulate: sqrt(sqr(i)+sqr(q))
		 * Approximated with MAG(i,q)
		 */
		a_sample = MAG(i_s[(HILBERT_TAP_NUM-1)], q_s[(HILBERT_TAP_NUM-1)]);
		break;
	default:
		break;
	}



  
	/*** AUDIO GENERATION ***/
	/*
	 * AGC, peak detector
	 * Sample speed is 15625 per second
	 */
  //peak += (ABS(a_sample))/128 - peak/128;     // Running average level detect, a=1/128
	peak += ((ABS(a_sample))>>7) - (peak>>7);			// Running average level detect, a=1/128
	k=0; i=peak;									// Logarithmic peak detection
	if (i&0xff00) {k+=8; i>>=8;}					// k=log2(peak), find highest bit set
	if (i&0x00f0) {k+=4; i>>=4;}
	if (i&0x000c) {k+=2; i>>=2;}
	if (i&0x0002) {k+=1;}
	agc_accu += (k - AGC_REF);						// Add difference with target to integrator (Acc += Xn - R)
	if (agc_accu > agc_attack)						// Attack time, gain correction in case of high level
	{
		agc_gain--;									// Decrease gain
		agc_accu -= agc_attack;						// Reset integrator
	} else if (agc_accu < -(agc_decay))				// Decay time, gain correction in case of low level
	{
		agc_gain++;									// Increase gain
		agc_accu += agc_decay;						// Reset integrator
	}

	/*
	 * Scale and clip output,  
	 * Send to audio DAC output
	 */
	a_sample += DAC_BIAS;							// Add bias level
	if (a_sample > DAC_RANGE)						// Clip to DAC range
		a_sample = DAC_RANGE;
	else if (a_sample<0)
		a_sample = 0;





	pwm_set_chan_level(dac_audio, PWM_CHAN_A, a_sample);
  //pwm_set_gpio_level(22, a_sample);



//  gpio_clr_mask(1<<LED_BUILTIN);


	return true;
}









/************************************************************************************** 
 * CORE0: inside DMA IRQ 
 * The VOX function is called separately every cycle, to check audio level.
 * Execute TX branch signal processing when tx enabled
 **************************************************************************************/
volatile int16_t a_s_raw[LPF_TAP_NUM]; 						// Raw samples, minus DC bias
volatile int16_t a_level=0;							// Average level of raw sample stream
volatile int16_t a_s[HILBERT_TAP_NUM];							// Filtered and decimated samples
volatile int16_t a_dc;								// DC level
//volatile int tx_cnt=0;								// Decimation counter
//bool vox() __attribute__ ((section (".scratch_x.")));
bool vox(void)
{

	int16_t a_sample;
	int i;

	/*
	 * Get sample and shift into delay line
	 */
	a_sample = adc_result[2];						// Get latest ADC 2 result


 

	/*
	 * Remove DC and store new raw sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 */
//samples already subtracted from bias
//	a_sample = (a_sample&0x0fff) - ADC_BIAS;		// Clip and subtract mid-range
//	a_dc += (a_sample - a_dc)/128;					//   then IIR running average
//	a_sample -= a_dc;								//   subtract DC

#ifdef LOW_PASS_FITER  
	for (i=0; i<(LPF_TAP_NUM-1); i++) 							//   and store in shift register
		a_s_raw[i] = a_s_raw[i+1];
#endif
	a_s_raw[14] = a_sample;


	/*
	 * Detect level of audio signal
	 * Return true if VOX enabled and:
	 * - Audio level higher than threshold 
	 * - Linger time sill active 
	 */
	//if (a_sample<0) a_sample = -a_sample;			// Absolute value
  a_sample = ABS(a_sample);   // Absolute value
	//a_level += (a_sample - a_level)/128;			//   running average, 16usec * 128 = 2msec
  a_level += (a_sample - a_level)>>7u;      //   running average, 16usec * 128 = 2msec

	if (vox_level != VOX_OFF)						// Only when VOX is enabled
	{
		if (a_level > vox_level)
		{
			vox_count = VOX_LINGER;					// While audio present, reset linger timer
			return(true);							//  and keep TX active
		}
		if (vox_count>0)
		{
			vox_count--;							// No audio; decrement linger timer
			return(true);							//  but keep TX active
		}
	}

 
	return(false);									// All other cases: no TX
}



/************************************************************************************** 
 * CORE0: inside DMA IRQ
 * Tx 
 **************************************************************************************/
//bool tx() __attribute__ ((section (".scratch_x.")));
bool tx(void) 
{
  int32_t a_accu, q_accu;
  int16_t qh;
  int i;
  uint16_t i_dac, q_dac;
    
  /*** RAW Audio SAMPLES from VOX function ***/
  /*** Low pass filter + decimation ***/
//  tx_cnt = (tx_cnt+1)&3;              // Calculate only every fourth sample
//  if (tx_cnt>0) return true;            //   So effective sample rate will be 15625Hz

#ifdef LOW_PASS_FITER
  //sample already saved at vox()
  a_accu = 0;                   // Initialize accumulator
  for (i=0; i<LPF_TAP_NUM; i++)              // Low pass FIR filter, using raw samples
    a_accu += (int32_t)a_s_raw[i]*lpf_taps[i];    
#endif
  for (i=0; i<(HILBERT_TAP_NUM-1); i++)              // Shift decimated samples
    a_s[i] = a_s[i+1];
#ifdef LOW_PASS_FITER
  a_s[(HILBERT_TAP_NUM-1)] = a_accu >> LPF_SHIFT;             // Store rescaled accumulator
#else
  a_s[(HILBERT_TAP_NUM-1)] = a_s_raw[14];
#endif

	/*** MODULATION ***/
	switch (dsp_mode)
	{
	case 0:											// USB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = -(q_accu / 4096L);						// USB: sign is negative
		break;
	case 1:											// LSB
		/* 
		 * qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
		 */	
		q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
		qh = q_accu / 4096L;						// LSB: sign is positive
		break;
	case 2:											// AM
		/*
		 * I and Q values are identical
		 */
		qh = a_s[7];
		break;
	default:
		break;
	}

	/* 
	 * Write I and Q to QSE DACs, phase is 7 samples back.
	 * Need to multiply AC with DAC_RANGE/ADC_RANGE (appr 1/8)
	 * Any case: clip to range
	 */
	a_accu = DAC_BIAS - (qh/8);
	if (a_accu<0)
		q_dac = 0;
	else if (a_accu>(DAC_RANGE-1))
		q_dac = DAC_RANGE-1;
	else
		q_dac = a_accu;
	
	a_accu = DAC_BIAS + (a_s[7]/8);
	if (a_accu<0)
		i_dac = 0;
	else if (a_accu>(DAC_RANGE-1))
		i_dac = DAC_RANGE-1;
	else
		i_dac = a_accu;
		
	// pwm_set_both_levels(dac_iq, q_dac, i_dac);		// Set both channels of the IQ slice simultaneously
	// pwm_set_chan_level(dac_iq, PWM_CHAN_A, q_dac);
	// pwm_set_chan_level(dac_iq, PWM_CHAN_B, i_dac);
	pwm_set_gpio_level(21, i_dac);
	pwm_set_gpio_level(20, q_dac);
	
	
	return true;
}









int16_t fft_i_s[HILBERT_TAP_NUM], fft_q_s[HILBERT_TAP_NUM];          // Filtered I/Q samples
kiss_fft_scalar fft_in_minus[FFT_NSAMP]; // kiss_fft_scalar is a float
kiss_fft_scalar fft_in_plus[FFT_NSAMP]; // kiss_fft_scalar is a float
kiss_fft_cpx fft_out[FFT_NSAMP];
kiss_fftr_cfg fft_cfg; // = kiss_fftr_alloc(FFT_NSAMP,false,0,0);
int16_t qh;  
uint16_t block_num;
uint16_t block_pos;
uint16_t aux = 0;
/************************************************************************************** 
 * CORE1: 
 * Timing loop, triggered through inter-core fifo 
 **************************************************************************************/
//void dsp_core1_setup_and_loop() __attribute__ ((section (".scratch_x.")));
void dsp_core1_setup_and_loop()
{
uint16_t i,j;


  //Core1 setup

  // Configure a channel to write the same word (32 bits) repeatedly to PIO0
  // SM0's TX FIFO, paced by the data request signal from that peripheral.
  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);
  //channel_config_set_dreq(&cfg, DREQ_PIO0_TX0);
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(
      dma_chan,
      &cfg,
      //&pio0_hw->txf[0], // Write address (only need to set this once)
      &adc_samp[adc_samp_block_pos][0],
      //NULL,             // Don't provide a read address yet
      &adc_hw->fifo,    // src
      BLOCK_NSAMP,        // Write the same value many times, then halt and interrupt
      //false             // Don't start yet
      true              // start immediately
  );

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  dma_channel_set_irq0_enabled(dma_chan, true);

  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);

  // Manually call the handler once, to trigger the first transfer
  //dma_handler();






  //analogReadResolution(12);
   
  // Initialize ADCs 
  adc_gpio_init(26);                // GP26 is ADC 0  Q
  adc_gpio_init(27);                // GP27 is ADC 1  I
  adc_gpio_init(28);                // GP28 is ADC 2  MIC
  adc_init();                       // Initialize ADC to known state
  adc_select_input(0);              // Start with ADC0
  
  adc_set_round_robin(0x01+0x02+0x04);      // Sequence ADC 0-1-2 (GP 26, 27, 28) free running
  adc_fifo_setup(     // IRQ for every result (fifo threshold = 1)
    true,    // Write each completed conversion to the sample FIFO
    //false,    // disable DMA data request (DREQ)
    true,    // disable DMA data request (DREQ)
    1,       // DREQ (and IRQ) asserted when at least 1 sample present
    false,   // We won't see the ERR bit because of 8 bit reads; disable.
    false     // Keep full 12 bits of each sample
    //true     // Shift each sample to 8 bits when pushing to FIFO
    );
  adc_set_clkdiv(ADC_CLOCK_DIV);    // 480k / 3 channels = 160 kSps
//  adc_next = 0;




  //irq_set_exclusive_handler(ADC0_IRQ_FIFO, adc_handler);
  //adc_irq_set_enabled(true);
  irq_set_enabled(ADC0_IRQ_FIFO, true);
  adc_run(true);

  



  //bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; // Set Core 1 prio high
  // Grant high bus priority to the DMA, so it can shove the processors out
  // of the way. This should only be needed if you are pushing things up to
  // >16bits/clk here, i.e. if you need to saturate the bus completely.
  bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;



  
  
  //fft setup
  fft_cfg = kiss_fftr_alloc(FFT_NSAMP,false,0,0);




/*
  block_num = 0;
  block_pos = 0;
  
  Serialx.print("\n====== FFT =======\n");
  for(j=0; j<FFT_NSAMP; j++)
  {

    fft_samp[block_num][block_pos] = 10;
    fft_samp[block_num][block_pos+1] = block_num;
    fft_samp[block_num][block_pos+2] = block_pos;
   
 
    block_pos+=3;
    if(block_pos >= BLOCK_NSAMP)
    {
      block_num++;
      block_pos = 0; 
    }          
  }


  block_num = 0;
  block_pos = 0;
  
  Serialx.print("\n====== FFT =======\n");
  for(j=0; j<FFT_NSAMP; j++)
  {
    
    Serialx.print((int)j, DEC);
    Serialx.print(",");
    Serialx.print((int)(fft_samp[block_num][block_pos]), DEC);
    Serialx.print(",");
    Serialx.print((int)(fft_samp[block_num][block_pos+1]), DEC);
    Serialx.print(",");
    Serialx.print((int)(fft_samp[block_num][block_pos+2]), DEC);
    Serialx.print(" ");
    Serialx.print("\n");    
  
  
  
    block_pos+=3;
    if(block_pos >= BLOCK_NSAMP)
    {
      block_num++;
      block_pos = 0; 
    }          
  }
  
  Serialx.print("====== fim =======\n");

*/





	//Core1 loop
 
  while(1) 
	{
  
//    gpio_set_mask(1<<14);


    //wait for FFT input data to be processed
    if((fft_samples_ready == 1) && //ready to start FFT with last samples
       (display_graf_new == 0))
    {


//#if 0

          
          if(++aux == 20)
          {
          
          
            
            Serialx.print("\n====== FFT =======\n");
          
            //fft_samp[NBLOCK_TOT][BLOCK_NSAMP]
            Serialx.print("FFT_NUM_BLOCK=");
            Serialx.print(FFT_NUM_BLOCK, DEC);
            Serialx.print(" BLOCK_NSAMP=");
            Serialx.print(BLOCK_NSAMP, DEC); 
            Serialx.print("\n");
              
            for(j=0; j<FFT_NUM_BLOCK; j++)
            {
              for (i=0; i<BLOCK_NSAMP; i+=3) 
                  {
                  Serialx.print((int)j, DEC);
                  Serialx.print(",");
                  Serialx.print((int)i, DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i]), DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i+1]), DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i+2]), DEC);
                  Serialx.print(" ");
                  Serialx.print("\n");    
                  }
            }
            
            Serialx.print("====== fim =======\n");
          
          /* 
            
            for(j=0; j<FFT_NUM_BLOCK; j++)
            {
              for (i=0; i<BLOCK_NSAMP; i+=3) 
                  {
                  Serialx.print((int)j, DEC);
                  Serialx.print(",");
                  Serialx.print((int)i, DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i]), DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i+1]), DEC);
                  Serialx.print(",");
                  Serialx.print((int)(fft_samp[j][i+2]), DEC);
                  Serialx.print(" ");
                  Serialx.print("\n");    
                  }
            }
          
          */
          
           
          
          }



//        display_graf_new = 0;  
//        fft_samples_ready = 2;  //ready to start new sample collect
  



//#endif





//#if 0


//#if 0
      block_num = 0;   
      block_pos = 0;

      // Hilbert H(Q)
      // fill first samples to calculate the first Hilbert value
      for(j=0; j<HILBERT_TAP_NUM; j++)
      {
        fft_q_s[j] = fft_samp[block_num][block_pos];
        fft_i_s[j] = fft_samp[block_num][block_pos+1];  

        block_pos+=3;
        if(block_pos >= BLOCK_NSAMP)
        {
          block_num++;
          block_pos = 0; 
        }
      }
      for(j=0; j<FFT_NSAMP; j++)
      {
        for (i=0; i<(HILBERT_TAP_NUM-1); i++)   // Shift decimated samples
        {
          fft_q_s[i] = fft_q_s[i+1];
          fft_i_s[i] = fft_i_s[i+1];
        }
        fft_q_s[(HILBERT_TAP_NUM-1)] = fft_samp[block_num][block_pos];
        fft_i_s[(HILBERT_TAP_NUM-1)] = fft_samp[block_num][block_pos+1];
     
        qh = ((int32_t)(fft_q_s[0]-fft_q_s[14])*315L + (int32_t)(fft_q_s[2]-fft_q_s[12])*440L + 
              (int32_t)(fft_q_s[4]-fft_q_s[10])*734L + (int32_t)(fft_q_s[6]-fft_q_s[ 8])*2202L) >> 12;  // / 4096L
        fft_in_minus[j] = fft_i_s[7] - qh;  //USB
        fft_in_plus[j] = fft_i_s[7] + qh;   //LSB


        block_pos+=3;
        if(block_pos >= BLOCK_NSAMP)
        {
          block_num++;
          block_pos = 0; 
        }          
      }
//#endif

#if 0
      block_num = 0;   
      block_pos = 0;
      
      for(j=0; j<FFT_NSAMP; j++)  //320
      {
        fft_in_minus[j] = fft_samp[block_num][block_pos];
        fft_in_plus[j] = fft_samp[block_num][block_pos+1]; 
        
        block_pos+=3;
        if(block_pos >= BLOCK_NSAMP)
        {
          block_num++;
          block_pos = 0; 
        }
      }
#endif

 


      // FFT  I - H(Q)
      kiss_fftr(fft_cfg , fft_in_minus, fft_out);  // ***  about 30ms


      // fill line for graphic  -band to 0
      for(i=0; i<FFT_NUMFREQ; i++)
      {
        if(MAG(fft_out[i].r, fft_out[i].i) > 1u)
        {
          vet_graf_fft[(GRAPH_NUM_LINES-1)][(FFT_NUMFREQ-1)+i] = 1;
        }
        else
        {
          vet_graf_fft[(GRAPH_NUM_LINES-1)][(FFT_NUMFREQ-1)+i] = 0;
        }
      }

      
      // FFT  I - H(Q)
      kiss_fftr(fft_cfg , fft_in_plus, fft_out);  // ***  about 30ms


      // fill line for graphic  0 to +band
      for(i=0; i<FFT_NUMFREQ; i++)
      {
        if(MAG(fft_out[i].r, fft_out[i].i) > 1u)
        {
          vet_graf_fft[(GRAPH_NUM_LINES-1)][FFT_NUMFREQ-i] = 1;
        }
        else
        {
          vet_graf_fft[(GRAPH_NUM_LINES-1)][FFT_NUMFREQ-i] = 0;
        }

      }



#if 0

          if(++aux == 20)
          {
                  Serialx.print("\n FFT \n");  

  
                  //fft_samp[FFT_NUM_BLOCK][BLOCK_NSAMP]
                  Serialx.print("fft_samp[FFT_NUM_BLOCK][BLOCK_NSAMP]    FFT_NUM_BLOCK=");
                  Serialx.print(FFT_NUM_BLOCK, DEC);  //13
                  Serialx.print("   BLOCK_NSAMP=");
                  Serialx.print(BLOCK_NSAMP, DEC);   //30
                  Serialx.print("\n");
                  Serialx.print("fft_in_minus[FFT_NSAMP]    FFT_NSAMP=");
                  Serialx.print(FFT_NSAMP, DEC);   //320
                  Serialx.print("    fft_out[FFT_NSAMP]     FFT_NSAMP=");
                  Serialx.print(FFT_NSAMP, DEC);   //320
                  Serialx.print("    FFT_NUMFREQ=");
                  Serialx.print(FFT_NUMFREQ, DEC);   //160
                  Serialx.print("\n");                  

                  for(i=0; i<FFT_NUMFREQ; i++)
                  {
                  Serialx.print((int)i, DEC);
                  Serialx.print(",");
                  Serialx.print((int)fft_in_minus[i], DEC);
                  Serialx.print(",");  
                  Serialx.print((int)fft_out[i].r, DEC);
                  Serialx.print(",");                    
                  Serialx.print((int)fft_out[i].i, DEC);
                  Serialx.print(",");                    
                  Serialx.print((int)MAG(fft_out[i].r, fft_out[i].i), DEC);
                  
                  Serialx.print("\n");  
                  }   
          }


#endif


 
      //graphic data is ready for graphic plotting  
      display_graf_new = 1;

//#endif
  
    }




      
//    gpio_clr_mask(1<<14);
   
  

  }
  
}





/************************************************************************************** 
 * CORE0: 
 * Initialize dsp context and spawn CORE1 process 
 *
 * Some CORE1 code parts should not run from Flash, but be loaded in SRAM at boot time
 * See platform.h for function qualifier macro's
 * for example: 
 * void __not_in_flash_func(funcname)(int arg1, float arg2)
 * {
 * }
 *
 * Need to set BUS_PRIORITY of Core 1 to high
 * #include bus_ctrl.h
 * bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; // Set Core 1 prio high
 **************************************************************************************/
void dsp_init() 
{

  uint16_t slice_num;


  gpio_init_mask(1<<14);  
  gpio_set_dir(14, GPIO_OUT); 

  
  tx_enabled = false;

  //analogWriteResolution(12);


 
  //https://forums.raspberrypi.com/viewtopic.php?t=306321
  // Initialize DACs, default mode is free running, A and B pins are output 
  gpio_set_function(20, GPIO_FUNC_PWM);     // GP20 is PWM for Q DAC (Slice 2, Channel A)
  gpio_set_function(21, GPIO_FUNC_PWM);     // GP21 is PWM for I DAC (Slice 2, Channel B)
  dac_iq = pwm_gpio_to_slice_num(20);       // Get PWM slice for GP20 (Same for GP21)
  pwm_set_clkdiv_int_frac (dac_iq, 1, 0);     // clock divide by 1 = 125MHz
  pwm_set_wrap(dac_iq, DAC_RANGE-1);        // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_iq, true);            // Set the PWM running
  
  gpio_set_function(22, GPIO_FUNC_PWM);     // GP22 is PWM for Audio DAC (Slice 3, Channel A)
  dac_audio = pwm_gpio_to_slice_num(22);      // Find PWM slice for GP22
  pwm_set_clkdiv_int_frac (dac_audio, 1, 0);    // clock divide by 1 = 125MHz
  pwm_set_wrap(dac_audio, DAC_RANGE-1);     // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_audio, true);         // Set the PWM running




  //pwm_set_chan_level(dac_audio, PWM_CHAN_A, 10);
  //pwm_set_gpio_level(22, DAC_BIAS);



    
  delay(500);  //necessario para rodar o core1 - apos testes
  multicore_launch_core1(dsp_core1_setup_and_loop);        // Start processing on core1
  delay(50);  


  
  //after multicore_launch_core1  because it uses the FIFO
  //https://hackaday.io/page/9880-raspberry-pi-pico-multicore-adventures
  // We clear the interrupt flag, if it got set by a chance
  multicore_fifo_clear_irq();
  // set the SIO_IRQ_PROC1 (FIFO register set interrupt) ownership to only one core. Opposite to irq_set_shared_handler() function
  // We pass it the name of function that shall be executed when interrupt occurs
  irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_irq_handler);
  // enable interrupt
  irq_set_enabled(SIO_IRQ_PROC0, true);



}


/************************************************************************************** 
 * CORE0: 
 * Loop 
 *
 **************************************************************************************/
void dsp_loop()
{

//    gpio_set_mask(1<<14);
    
//gpio_xor_mask(1<<LED_BUILTIN);






/*
  if(subindo == 1)
  {
    if(++contsub >= 5)
    {
      subindo = 0;
    }
  }
  else
  {
    if(++contsub <= -5)
    {
      subindo = 1;
    }    
  }


  //pwm_set_chan_level(dac_audio, PWM_CHAN_A, a_sample);
  pwm_set_gpio_level(22, ((contsub*5) + DAC_BIAS));

*/







//   gpio_clr_mask(1<<14);  

}
