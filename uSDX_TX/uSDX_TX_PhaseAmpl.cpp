//  Based on QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Adapted by: Klaus Fensterseifer PY2KLA
//  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj


#include "Arduino.h"
#include "pwm.h"
#include "uSDX_I2C.h"
#include "uSDX_SI5351.h"
#include "uSDX_TX_PhaseAmpl.h"





#define DAC_RANGE  256
#define DAC_BIAS  (DAC_RANGE/2)
#define ADC_RANGE 4096
#define ADC_BIAS  (ADC_RANGE/2)
#define PWM_AMPL_OUT_PIN    21  // 21 = i_dac

//  pwm_set_chan_level(dac_audio, PWM_CHAN_A, (cw_tone[cw_tone_pos]>>8)+DAC_BIAS);    //audio side tone
//  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, i_dac);

volatile int16_t adc_result[3];  

volatile uint16_t dac_iq, dac_audio;




I2C i2c;
SI5351 si5351;



//enum mode_t { LSB, USB, CW, FM, AM };
enum { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile int32_t freq = 7100000;

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

//***********************************************************************
//
//
//***********************************************************************
inline void _vox(bool trigger)
{
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
}





#define _UA       600   //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#define MAX_DP    ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define CARRIER_COMPLETELY_OFF_ON_LOW    1    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define KEY_CLICK        1   // Reduce key clicks by envelope shaping

//***********************************************************************
//
//
//***********************************************************************
inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}






#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;
//#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

//***********************************************************************
//
//
//***********************************************************************
inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for(j = 0; j != 15; j++) v[j] = v[j + 1];
// MORE_MIC_GAIN  adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)

  int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)

  i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)


#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband


#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( _F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-_F_SAMP_TX / _UA);
}













#define PWM_MIN   29u     // PWM value for which PA reaches its minimum: 29 when C31 installed;   0 when C31 removed;   0 for biasing BS170 directly
#define PWM_MAX   255u    // PWM value for which PA reaches its maximum: 96 when C31 installed; 255 when C31 removed;

//refresh LUT based on pwm_min, pwm_max
//***********************************************************************
//
//
//***********************************************************************
void build_lut()   //lookup table to convert from calculated amplitude to pwm level
{
  for(uint16_t i = 0; i != 256; i++)    // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (PWM_MAX - PWM_MIN)) / 255 + PWM_MIN;
    //lut[i] = min(pwm_max, (float)106*log(i) + pwm_min);  // compressed microphone output: drive=0, pwm_min=115, pwm_max=220
}



/*

//#define log2(n) (log(n) / log(2))
//***********************************************************************
//
//
//***********************************************************************
uint8_t log2(uint16_t x){
  uint8_t y = 0;
  for(; x>>=1;) y++;
  return y;
}


*/





#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
//static int16_t _adc;
//***********************************************************************
//
//
//***********************************************************************
void dsp_tx_ssb()
{ // jitter dependent things first
// SSB with single ADC conversion:
  //ADC_AUDIO_MIC |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //TX_AMPL_PWM = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  //TX_AMPL_PWM = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, amp);       // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = adc_result[2];  //ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency


#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  if(tx == 1)
  {
    //TX_AMPL_PWM = 0;
    pwm_set_gpio_level(PWM_AMPL_OUT_PIN, 0);
    si5351.SendRegister(SI_CLK_OE, TX0RX0);    // disable carrier
  }
  if(tx == 255){ si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier
#endif

}




volatile uint16_t acc;
volatile uint32_t cw_offset;
volatile uint8_t cw_tone = 1;
//const uint32_t tones[] = { F_MCU * 700ULL / 20000000, F_MCU * 600ULL / 20000000, F_MCU * 700ULL / 20000000};
const uint32_t tones[] = { 700UL, 600UL, 700UL};

volatile int8_t p_sin = 0;     // initialized with A*sin(0) = 0
volatile int8_t n_cos = 448/4; // initialized with A*cos(t) = A
//***********************************************************************
//
//
//***********************************************************************
inline void process_minsky() // Minsky circle sample [source: https://www.cl.cam.ac.uk/~am21/hakmemc.html, ITEM 149]: p_sin+=n_cos*2*PI*f/fs; n_cos-=p_sin*2*PI*f/fs;
{
  int8_t alpha127 = tones[cw_tone]/*cw_offset*/ * 798 / _F_SAMP_TX;  // alpha = f_tone * 2 * pi / fs
  p_sin += alpha127 * n_cos / 127;
  n_cos -= alpha127 * p_sin / 127;
}




// CW Key-click shaping, ramping up/down amplitude with sample-interval of 60us. Tnx: Yves HB9EWY https://groups.io/g/ucx/message/5107
const uint8_t ramp[31] PROGMEM = { 255, 254, 252, 249, 245, 239, 233, 226, 217, 208, 198, 187, 176, 164, 152, 139, 127, 115, 102, 90, 78, 67, 56, 46, 37, 28, 21, 15, 9, 5, 2 }; // raised-cosine(i) = 255 * sq(cos(HALF_PI * i/32))
uint16_t i_ramp = 31;

//***********************************************************************
//
//
//***********************************************************************
void dsp_tx_cw()
{ // jitter dependent things first
#ifdef KEY_CLICK
/*
  if(TX_AMPL_PWM < lut[255]) { //check if already ramped up: ramp up of amplitude 
     for(uint16_t i = 31; i != 0; i--) {   // soft rising slope against key-clicks
        TX_AMPL_PWM = lut[pgm_read_byte_near(&ramp[i-1])];
        delayMicroseconds(60);
     }
  }
*/
  if(i_ramp > 0)
  {
  //TX_AMPL_PWM = lut[255]; // submit amplitude to PWM register (actually this is done in advance
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, lut[pgm_read_byte_near(&ramp[i_ramp-1])]);  // submit amplitude to PWM register (actually this is done in advance
  i_ramp--;
  }
  else
#endif // KEY_CLICK
  {
  //TX_AMPL_PWM = lut[255]; // submit amplitude to PWM register (actually this is done in advance
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, lut[255]);  // submit amplitude to PWM register (actually this is done in advance
  }
  
  process_minsky();
  //RX_AUDIO_PWM = (p_sin >> (16 - volume)) + 128;  // RX audio PWM - side tone - TX audio monitoring
  pwm_set_chan_level(dac_audio, PWM_CHAN_A, ((p_sin >> (16 - volume)) + 128));  // RX audio PWM - side tone - TX audio monitoring
}




//***********************************************************************
//
//
//***********************************************************************
void dsp_tx_am()
{ // jitter dependent things first
  //ADC_AUDIO_MIC |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //TX_AMPL_PWM = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, amp);  // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  int16_t adc = adc_result[2];  // - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive-4);
  //static int16_t dc;
  //dc += (in - dc) / 2;
  //in = in - dc;     // DC decoupling
  #define AM_BASE 32
  in=max(0, min(255, (in + AM_BASE)));
  amp=in;// lut[in];
}


  

//***********************************************************************
//
//
//***********************************************************************
void dsp_tx_fm()
{ // jitter dependent things first
  //ADC_AUDIO_MIC |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //TX_AMPL_PWM = lut[255];                   // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, lut[255]);    // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t adc = adc_result[2]; // - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
}









#define TIME_LOOP   208UL   // 1/4800 = 208us
unsigned long old_time;


//***********************************************************************
//
//
//***********************************************************************
void uSDX_TX_PhaseAmpl_setup(void)
{
  uint32_t t0, t1;
  uint16_t i;
      
  si5351.powerDown();  // disable all CLK outputs (especially needed for si5351 variants that has CLK2 enabled by default, such as Si5351A-B04486-GT)

  build_lut();  //create the table for ampl to pwm output conversion



  
  Serial.println("\nMeasure I2C Bit Error");

  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand random bulk PLLB writes
  si5351.freq(freq, 0, 90);  // freq needs to be set in order to use freq_calc_fast()

  si5351.pll_regs[4] = 0x55;
  si5351.SendPLLRegisterBulk();
  



  uint16_t i2c_error = 0;  // number of I2C byte transfer errors
  for(i = 0; i != 10000; i++)
  {

    si5351.freq_calc_fast(i);
    //for(int j = 0; j != 8; j++) si5351.pll_regs[j] = rand();
    si5351.SendPLLRegisterBulk();
    
    #define SI_SYNTH_PLL_A 26
    for(int j = 4; j != 8; j++) 
    {
      uint8_t RecReg = si5351.RecvRegister(SI_SYNTH_PLL_A + j);
      if(RecReg != si5351.pll_regs[j]) 
      {
        //Serial.println("regs diferentes: " + String(RecReg) + "  !=  " + String(si5351.pll_regs[j]));
        i2c_error++;
      }
    }
        
  }

  Serial.println("Result: " + String(i2c_error)); 




  si5351.iqmsa = 0;  // enforce PLL reset



  Serial.println("\nMeasure I2C Bus speed for Bulk Transfers");

  // Measure I2C Bus speed for Bulk Transfers
  //si5351.freq(freq, 0, 90);

  t0 = micros();
  for(i = 0; i != 1000; i++) 
    si5351.SendPLLRegisterBulk();
  t1 = micros();
  uint32_t speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  
  //if(false) {    fatal(F("i2cspeed"), speed, 'k');  }
  Serial.println("Result i2c speed: " + String(speed) + "k\n"); 

  



  si5351.freq(freq, 0, 90);  // freq needs to be set in order to use freq_calc_fast()





#define GP_PTT            15    // PTT pin 20 (GPIO 15)
  gpio_init_mask(1<<GP_PTT);  
  gpio_set_dir(GP_PTT, GPIO_IN); 
  gpio_pull_up(GP_PTT);


 
  // Initialize ADCs 
  //adc_gpio_init(26);                // GP26 is ADC 0  Q
  adc_gpio_init(27);                // GP27 is ADC 1  I
  //adc_gpio_init(28);                // GP28 is ADC 2  MIC
  adc_init();                       // Initialize ADC to known state
  adc_select_input(1);              // Start with ADC1 



  
  gpio_set_function(PWM_AMPL_OUT_PIN, GPIO_FUNC_PWM);     // GP20 is PWM for Q DAC (Slice 2, Channel A)
  gpio_set_function(PWM_AMPL_OUT_PIN, GPIO_FUNC_PWM);     // GP21 is PWM for I DAC (Slice 2, Channel B) - amp output
  dac_iq = pwm_gpio_to_slice_num(PWM_AMPL_OUT_PIN);       // Get PWM slice for GP20 (Same for GP21)
  pwm_set_clkdiv_int_frac (dac_iq, 1, 0);     // clock divide by 1 = 125MHz
  pwm_set_wrap(dac_iq, DAC_RANGE-1);        // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_iq, true);            // Set the PWM running
  
  gpio_set_function(22, GPIO_FUNC_PWM);     // GP22 is PWM for Audio DAC (Slice 3, Channel A) - used for CW side tone
  dac_audio = pwm_gpio_to_slice_num(22);      // Find PWM slice for GP22
  pwm_set_clkdiv_int_frac (dac_audio, 1, 0);    // clock divide by 1 = 125MHz
  pwm_set_wrap(dac_audio, DAC_RANGE-1);     // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_audio, true);         // Set the PWM running
    
  old_time = micros();

}










//600Hz seno sampled @ 4800Hz    12 bits = 4096
#define TAM_VET_AUDIO   8
int16_t vet_audio[TAM_VET_AUDIO] = { 0/4,  1447/4,  2047/4,  1447/4,  -1/4,  -1448/4, -2048/4, -1448/4 };


/*
//600Hz + 800Hz seno sampled @ 4800Hz    12 bits = 4096
#define TAM_VET_AUDIO   (8+6)
int16_t vet_audio[TAM_VET_AUDIO] = { 0,  1447,  2047,  1447,  -1,  -1448, -2048, -1448, 0,  1773,  1773,  -1, -1774, -1774 );
*/




//***********************************************************************
//
//
//***********************************************************************
void uSDX_TX_PhaseAmpl_loop(void)
{
static uint16_t pos_audio = 0;
static uint16_t ampl_shift = 0;
static uint16_t cont = 0;
static uint16_t ptt, old_ptt = 22;



  ptt = gpio_get(GP_PTT);  // Get PTT level
  if (ptt!=old_ptt)           
  {
    old_ptt = ptt;
    if(ptt==0)
    {
      si5351.SendRegister(SI_CLK_OE, TX1RX0);  // enable carrier

      Serial.println("TX ON  ***   MIC = " + String(adc_read())); 
    }
    else
    {
      pwm_set_gpio_level(PWM_AMPL_OUT_PIN, 0);    // ampl out = 0   no TX 
      si5351.SendRegister(SI_CLK_OE, TX0RX0);    // disable carrier    
        
      Serial.println("TX OFF       MIC = " + String(adc_read())); 
    }
  }  
  



  if(((unsigned long)(micros() - old_time)) > TIME_LOOP)
  {

    old_time += TIME_LOOP;
    cont++;
    if(cont > 100)
    {
      cont = 0;
      ampl_shift++;
      if(ampl_shift>3)
        ampl_shift = 0;
    }



gpio_set_mask(1<<LED_BUILTIN);  //high



    if (ptt==0)   // TX
    {
      tx = 254;
/*      
      adc_result[2] = (vet_audio[pos_audio]>>(4+(pos_audio&ampl_shift)));   //audio sample  10bits + modulacao ampl
//      adc_result[2] = vet_audio[pos_audio]>>4;   //audio sample  10bits + modulacao ampl
      if(++pos_audio >= TAM_VET_AUDIO)  pos_audio = 0;
*/

      //read MIC
      adc_result[2] = ((int16_t)adc_read() - ADC_BIAS);  //ADC MIC
      for(uint16_t i=1; i<10; i++)
      {
        adc_result[2] += ((int16_t)adc_read() - ADC_BIAS);  //ADC MIC
      }
      adc_result[2] >>= 3;  //MIC average
 


    
      dsp_tx_ssb();
      //dsp_tx_cw();
      //dsp_tx_am();
      //dsp_tx_fm();


      //pwm_set_gpio_level(PWM_AMPL_OUT_PIN, adc_result[2]/4 + DAC_BIAS);    // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)



/*
  amp = adc_result[2]/4 + DAC_BIAS;

  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  //TX_AMPL_PWM = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  pwm_set_gpio_level(PWM_AMPL_OUT_PIN, amp);       // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  //int16_t adc = adc_result[2];  //ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = adc_result[2]/4;  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
*/


 
    }
    else
    {
      
    }
    
gpio_clr_mask(1<<LED_BUILTIN);  //low

  }



}
