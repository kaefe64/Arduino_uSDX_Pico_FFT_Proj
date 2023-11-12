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



/* Menu definitions */
#define HMI_S_TUNE			0
#define HMI_S_MODE			1
#define HMI_S_AGC			2
#define HMI_S_PRE			3
#define HMI_S_VOX			4
#define HMI_S_BPF			5
#define HMI_S_DFLASH   6
#define HMI_NMENUS			7  //number of possible menus

/* Event definitions */
#define HMI_E_NOEVENT		0
#define HMI_E_INCREMENT		1
#define HMI_E_DECREMENT		2
#define HMI_E_ENTER			3
#define HMI_E_ESCAPE		4
#define HMI_E_LEFT			5
#define HMI_E_RIGHT			6
#define HMI_E_PTTON			7
#define HMI_E_PTTOFF		8
#define HMI_PTT_ON      9
#define HMI_PTT_OFF     10
#define HMI_NEVENTS			11
//#define HMI_NEVENTS      9

/* Sub menu option string sets */
#define HMI_NUM_OPT_TUNE	7  // = num pos cursor
#define HMI_NUM_OPT_MODE	4
#define HMI_NUM_OPT_AGC	3
#define HMI_NUM_OPT_PRE	5
#define HMI_NUM_OPT_VOX	4
#define HMI_NUM_OPT_BPF	5
#define HMI_NUM_OPT_DFLASH	2


//"USB","LSB","AM","CW"
#define MODE_USB  0
#define MODE_LSB  1
#define MODE_AM   2
#define MODE_CW   3



/*
 * Some macros
 */
#ifndef MIN
#define MIN(x, y)        ((x)<(y)?(x):(y))  // Get min value
#endif
#ifndef MAX
#define MAX(x, y)        ((x)>(y)?(x):(y))  // Get max value
#endif



#define  band0_hmi_freq_default     1820000L
#define  band1_hmi_freq_default     3700000L
#define  band2_hmi_freq_default     7050000L
#define  band3_hmi_freq_default    14200000L
#define  band4_hmi_freq_default    28400000L

#define b0_0 (uint8_t)(band0_hmi_freq_default >> 24)
#define b0_1 (uint8_t)((band0_hmi_freq_default >> 16)&0xff)
#define b0_2 (uint8_t)((band0_hmi_freq_default >> 8)&0xff)
#define b0_3 (uint8_t)(band0_hmi_freq_default&0xff)

#define b1_0 (uint8_t)(band1_hmi_freq_default >> 24)
#define b1_1 (uint8_t)((band1_hmi_freq_default >> 16)&0xff)
#define b1_2 (uint8_t)((band1_hmi_freq_default >> 8)&0xff)
#define b1_3 (uint8_t)(band1_hmi_freq_default&0xff)

#define b2_0 (uint8_t)(band2_hmi_freq_default >> 24)
#define b2_1 (uint8_t)((band2_hmi_freq_default >> 16)&0xff)
#define b2_2 (uint8_t)((band2_hmi_freq_default >> 8)&0xff)
#define b2_3 (uint8_t)(band2_hmi_freq_default&0xff)

#define b3_0 (uint8_t)(band3_hmi_freq_default >> 24)
#define b3_1 (uint8_t)((band3_hmi_freq_default >> 16)&0xff)
#define b3_2 (uint8_t)((band3_hmi_freq_default >> 8)&0xff)
#define b3_3 (uint8_t)(band3_hmi_freq_default&0xff)

#define b4_0 (uint8_t)(band4_hmi_freq_default >> 24)
#define b4_1 (uint8_t)((band4_hmi_freq_default >> 16)&0xff)
#define b4_2 (uint8_t)((band4_hmi_freq_default >> 8)&0xff)
#define b4_3 (uint8_t)(band4_hmi_freq_default&0xff)



#define BAND_VARS_SIZE   (HMI_NMENUS + 4)   //must be less than DATA_BLOCK_SIZE

//extern uint8_t  hmi_sub[HMI_NMENUS];							// Stored option selection per state
extern uint32_t hmi_freq;  
extern uint8_t  hmi_band;	
extern bool ptt_active;
extern bool vox_active;

//#define BAND_INDEX   HMI_S_BPF    // = 5
extern uint8_t  band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE];


void Setup_Band(uint8_t band);
void hmi_init0(void);
void hmi_init(void);
void hmi_evaluate(void);


#ifdef __cplusplus
}
#endif
#endif
