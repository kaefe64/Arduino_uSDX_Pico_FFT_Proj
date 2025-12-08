#ifndef __DSP_FILTERS_H__
#define __DSP_FILTERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*

 * dsp_filters.h
 *
 * Created: Apr 2025
 * Author:  Klaus Fensterseifer 
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj

   Digital filters taps arrays
   Values calculated from 
   http://t-filter.engineerjs.com/

*/



#if 0


/*

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 300 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 500 Hz - 800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 1000 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/
#define CW_BPF_TAP_NUM  57      // band pass filter for CW

static int16_t cw_bpf_taps[CW_BPF_TAP_NUM] = {
  125,
  -73,
  -79,
  -78,
  -56,
  -23,
  -5,
  -36,
  -133,
  -275,
  -397,
  -407,
  -222,
  180,
  731,
  1264,
  1564,
  1436,
  796,
  -269,
  -1499,
  -2525,
  -2982,
  -2643,
  -1517,
  133,
  1857,
  3153,
  3634,
  3153,
  1857,
  133,
  -1517,
  -2643,
  -2982,
  -2525,
  -1499,
  -269,
  796,
  1436,
  1564,
  1264,
  731,
  180,
  -222,
  -407,
  -397,
  -275,
  -133,
  -36,
  -5,
  -23,
  -56,
  -78,
  -79,
  -73,
  125
};



#endif



#if 0

/*

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 200 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 500 Hz - 800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 1100 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define CW_BPF_TAP_NUM 31

static int16_t cw_bpf_taps[CW_BPF_TAP_NUM] = {
  56,
  488,
  737,
  986,
  948,
  498,
  -370,
  -1475,
  -2479,
  -2985,
  -2693,
  -1544,
  225,
  2124,
  3577,
  4120,
  3577,
  2124,
  225,
  -1544,
  -2693,
  -2985,
  -2479,
  -1475,
  -370,
  498,
  948,
  986,
  737,
  488,
  56
};



#endif


//#if 0

/*

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 300 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 550 Hz - 750 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 1000 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define CW_BPF_TAP_NUM   49     // band pass filter for CW

static int16_t cw_bpf_taps[CW_BPF_TAP_NUM] = {
  99,
  27,
  20,
  -48,
  -197,
  -385,
  -515,
  -487,
  -245,
  194,
  734,
  1218,
  1461,
  1311,
  715,
  -240,
  -1317,
  -2196,
  -2570,
  -2259,
  -1285,
  119,
  1572,
  2658,
  3059,
  2658,
  1572,
  119,
  -1285,
  -2259,
  -2570,
  -2196,
  -1317,
  -240,
  715,
  1311,
  1461,
  1218,
  734,
  194,
  -245,
  -487,
  -515,
  -385,
  -197,
  -48,
  20,
  27,
  99
};




//#endif





#if 0
/**************************************************************************************

FIR filter designed with
http://t-filter.engineerjs.com/

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

#define SSB_LPF_TAP_NUM 13

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
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
http://t-filter.engineerjs.com/

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

#define SSB_LPF_TAP_NUM 15

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
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
http://t-filter.engineerjs.com/

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

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
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
http://t-filter.engineerjs.com/

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

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
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


#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4100 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  350,
  1651,
  2300,
  843,
  -2080,
  -2530,
  2406,
  10080,
  13832,
  10080,
  2406,
  -2530,
  -2080,
  843,
  2300,
  1651,
  350
};


#endif




#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4200 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 15

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  768,
  1137,
  -319,
  -3119,
  -3520,
  1558,
  9521,
  13439,
  9521,
  1558,
  -3520,
  -3119,
  -319,
  1137,
  768
};



#endif


#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4800 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  -767,
  -1080,
  686,
  2621,
  700,
  -2801,
  344,
  10756,
  16848,
  10756,
  344,
  -2801,
  700,
  2621,
  686,
  -1080,
  -767
};

#endif


#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5100 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  -657,
  -2322,
  -1700,
  1495,
  1446,
  -3040,
  -1503,
  10273,
  17926,
  10273,
  -1503,
  -3040,
  1446,
  1495,
  -1700,
  -2322,
  -657
};

#endif


#if 0

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5600 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 17

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  796,
  -361,
  -3240,
  -1483,
  2258,
  -1600,
  -3557,
  9661,
  20242,
  9661,
  -3557,
  -1600,
  2258,
  -1483,
  -3240,
  -361,
  796
};


#endif





/*

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define AM_LPF_TAP_NUM 19

static int16_t am_lpf_taps[AM_LPF_TAP_NUM] = {
  422,
  -380,
  -2325,
  -1917,
  1341,
  1276,
  -3105,
  -1395,
  10293,
  17806,
  10293,
  -1395,
  -3105,
  1276,
  1341,
  -1917,
  -2325,
  -380,
  422
};



#if 0


/*

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 5000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 6000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define AM_LPF_TAP_NUM 19

static int16_t am_lpf_taps[AM_LPF_TAP_NUM] = {
  -1214,
  -751,
  2140,
  866,
  -1137,
  2544,
  447,
  -4150,
  9255,
  22194,
  9255,
  -4150,
  447,
  2544,
  -1137,
  866,
  2140,
  -751,
  -1214
};

#endif





#if 0   // LP filter 16kHz @160Khz inside the interrupt
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 160000 Hz

fixed point precision: 16 bits

* 0 Hz - 6000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 16000 Hz - 80000 Hz
  gain = 0
  desired attenuation = -42 dB  = 0.8% Vin
  actual attenuation = -42.49 dB  = 0.75% Vin

*/

#define FILTER_TAP_NUM 19

static const int16_t lpf_16khz_taps[FILTER_TAP_NUM] = {
  304,
  523,
  897,
  1366,
  1901,
  2454,
  2972,
  3394,
  3671,
  3768,
  3671,
  3394,
  2972,
  2454,
  1901,
  1366,
  897,
  523,
  304
};





/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 160000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 13000 Hz - 80000 Hz
  gain = 0
  desired attenuation = -63 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM 33

static int16_t filter_taps[FILTER_TAP_NUM] = {
  29,
  59,
  114,
  195,
  307,
  453,
  635,
  849,
  1091,
  1353,
  1622,
  1885,
  2127,
  2334,
  2493,
  2593,
  2627,
  2593,
  2493,
  2334,
  2127,
  1885,
  1622,
  1353,
  1091,
  849,
  635,
  453,
  307,
  195,
  114,
  59,
  29
};









#endif










// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************

//#define SSB_LPF_TAP_NUM 17

//static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {




/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 700 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 1900 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_700Hz 17

static int16_t filter_taps_700Hz[FILTER_TAP_NUM_700Hz] = {
  102,
  568,
  985,
  1676,
  2449,
  3241,
  3925,
  4391,
  4556,
  4391,
  3925,
  3241,
  2449,
  1676,
  985,
  568,
  102
};



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 2000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_800Hz 15

static int16_t filter_taps_800Hz[FILTER_TAP_NUM_800Hz] = {
  362,
  919,
  1534,
  2492,
  3347,
  4224,
  4738,
  4989,
  4738,
  4224,
  3347,
  2492,
  1534,
  919,
  362
};







/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 1200 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 2500 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_1200Hz 17

static int16_t filter_taps_1200Hz[FILTER_TAP_NUM_1200Hz] = {
  -330,
  -374,
  -199,
  476,
  1733,
  3426,
  5181,
  6511,
  7007,
  6511,
  5181,
  3426,
  1733,
  476,
  -199,
  -374,
  -330
};






/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 1600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 2800 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_1600Hz 17

static int16_t filter_taps_1600Hz[FILTER_TAP_NUM_1600Hz] = {
  -458,
  -952,
  -1317,
  -1021,
  327,
  2697,
  5506,
  7794,
  8677,
  7794,
  5506,
  2697,
  327,
  -1021,
  -1317,
  -952,
  -458
};








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

#define FILTER_TAP_NUM_2000Hz 15

static int16_t filter_taps_2000Hz[FILTER_TAP_NUM_2000Hz] = {
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

#define FILTER_TAP_NUM_2400Hz 17

static int16_t filter_taps_2400Hz[FILTER_TAP_NUM_2400Hz] = {
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




/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 2800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_2800Hz 17

static int16_t filter_taps_2800Hz[FILTER_TAP_NUM_2800Hz] = {
  505,
  1586,
  1763,
  147,
  -2278,
  -1996,
  2931,
  9883,
  13167,
  9883,
  2931,
  -1996,
  -2278,
  147,
  1763,
  1586,
  505
};




/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3200 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4300 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_3200Hz 17

static int16_t filter_taps_3200Hz[FILTER_TAP_NUM_3200Hz] = {
  35,
  1230,
  2519,
  1708,
  -1531,
  -3018,
  1660,
  10252,
  14658,
  10252,
  1660,
  -3018,
  -1531,
  1708,
  2519,
  1230,
  35
};




/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 4700 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_3600Hz 17

static int16_t filter_taps_3600Hz[FILTER_TAP_NUM_3600Hz] = {
  -705,
  -650,
  1376,
  3004,
  572,
  -2998,
  294,
  10548,
  16457,
  10548,
  294,
  -2998,
  572,
  3004,
  1376,
  -650,
  -705
};



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5100 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_4000Hz 17

static int16_t filter_taps_4000Hz[FILTER_TAP_NUM_4000Hz] = {
  -657,
  -2322,
  -1700,
  1495,
  1446,
  -3040,
  -1503,
  10273,
  17926,
  10273,
  -1503,
  -3040,
  1446,
  1495,
  -1700,
  -2322,
  -657
};



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5450 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_4400Hz 17

static int16_t filter_taps_4400Hz[FILTER_TAP_NUM_4400Hz] = {
  158,
  -1553,
  -3080,
  -95,
  2442,
  -2093,
  -2920,
  9933,
  19509,
  9933,
  -2920,
  -2093,
  2442,
  -95,
  -3080,
  -1553,
  158
};



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4800 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5800 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define FILTER_TAP_NUM_4800Hz 17

static int16_t filter_taps_4800Hz[FILTER_TAP_NUM_4800Hz] = {
  1680,
  2611,
  -705,
  -1331,
  2426,
  -713,
  -4050,
  9377,
  21055,
  9377,
  -4050,
  -713,
  2426,
  -1331,
  -705,
  2611,
  1680
};




// **************************************************************
// **************************************************************
// **************************************************************
// **************************************************************



#define SSB_AUDIO_LPF_NUM  11

/*
static int16_t audio_lpf_freq[SSB_AUDIO_LPF_NUM] = {
//700,
800,
1200,
1600,
2000,
2400,
2800,
3200,
3600,
4000,
4400,
4800
};
*/




static int16_t audio_lpf_taps_num[SSB_AUDIO_LPF_NUM] = {
//FILTER_TAP_NUM_700Hz,
FILTER_TAP_NUM_800Hz,
FILTER_TAP_NUM_1200Hz,
FILTER_TAP_NUM_1600Hz,
FILTER_TAP_NUM_2000Hz,
FILTER_TAP_NUM_2400Hz,
FILTER_TAP_NUM_2800Hz,
FILTER_TAP_NUM_3200Hz,
FILTER_TAP_NUM_3600Hz,
FILTER_TAP_NUM_4000Hz,
FILTER_TAP_NUM_4400Hz,
FILTER_TAP_NUM_4800Hz
};



static int16_t* audio_lpf[SSB_AUDIO_LPF_NUM] = {
//filter_taps_700Hz,
filter_taps_800Hz,
filter_taps_1200Hz,
filter_taps_1600Hz,
filter_taps_2000Hz,
filter_taps_2400Hz,
filter_taps_2800Hz,
filter_taps_3200Hz,
filter_taps_3600Hz,
filter_taps_4000Hz,
filter_taps_4400Hz,
filter_taps_4800Hz
};






#ifdef __cplusplus
}
#endif
#endif
