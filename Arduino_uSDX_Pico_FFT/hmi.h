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



//#define   SMETER_TEST   10     //uncomment this line to see the audio level direct on display (used to generate the S Meter levels)


/* Menu definitions (band vars array position) */
#define HMI_S_TUNE			0
#define HMI_S_MODE			1
#define HMI_S_AGC			2
#define HMI_S_PRE			3
#define HMI_S_VOX			4
#define HMI_S_BPF			5
#define HMI_S_SAVE   6
#define HMI_S_AUDIO   7
#define HMI_NMENUS			8  //number of possible menus
#define HMI_NMENUS_DFLASH			7   //number of menus saved on DFLASH (only the first ones from the array of menus are saved on dflash)
                                  //(HMI_NMENUS_DFLASH + 4) must be less than DFLASH DATA_BLOCK_SIZE (16)

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
#define HMI_E_ENTER_RELEASE			11
#define HMI_NEVENTS			12  //number of events


/* Sub menu number of options (string sets) */
#define HMI_NUM_OPT_TUNE	7  // = num pos cursor
#define HMI_NUM_OPT_MODE	4
#define HMI_NUM_OPT_AGC	3
#define HMI_NUM_OPT_PRE	5
#define HMI_NUM_OPT_VOX	4
#define HMI_NUM_OPT_BPF	5
//#define HMI_NUM_OPT_DFLASH	2
#define HMI_NUM_OPT_MEMORY	16u
#define HMI_NUM_OPT_MEMORY_SAVING   	(HMI_NUM_OPT_MEMORY + 1u)
#define HMI_NUM_OPT_AUDIO 4


//"USB","LSB","AM","CW"
#define MODE_USB  0
#define MODE_LSB  1
#define MODE_AM   2
#define MODE_CW   3


// hmi_o_audio "Rec from TX", "Rec from RX", "Play to TX", "Play to Speaker"
#define AUDIO_REC_TX   0
#define AUDIO_REC_RX   1
#define AUDIO_PLAY_TX   2
#define AUDIO_PLAY_SPK   3



#define BAND_VARS_SIZE   (HMI_NMENUS + 4)   //menus + frequency
#define BAND_VARS_SIZE_DFLASH   (HMI_NMENUS_DFLASH + 4)   //menus + frequency saved on dflash
//#define BAND_INDEX   HMI_S_BPF    // = 5
extern uint8_t  band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE];



//#define MEMORY_BAND_NUM   HMI_NUM_OPT_MEMORY
#define FREQ_SIZE         4u

union un_uint32
  {
    uint32_t u32;
    uint8_t u8[FREQ_SIZE];
  };

/*
  uint8_t mem_cursor;
  uint8_t mem_mode;
  uint8_t mem_agc;
  uint8_t mem_pre;
  uint8_t mem_vox;
  uint8_t mem_bpf;
*/
struct st_memory_band
  {
  uint8_t vars[HMI_NMENUS];
  un_uint32 mem_freq;       //FREQ_SIZE = 4 * uint8_t
  };

extern st_memory_band  memory_band[HMI_NUM_OPT_MEMORY];



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

#define b0_0 (uint8_t)((band0_hmi_freq_default >> 24)&0xff)
#define b0_1 (uint8_t)((band0_hmi_freq_default >> 16)&0xff)
#define b0_2 (uint8_t)((band0_hmi_freq_default >> 8)&0xff)
#define b0_3 (uint8_t)(band0_hmi_freq_default&0xff)

#define b1_0 (uint8_t)((band1_hmi_freq_default >> 24)&0xff)
#define b1_1 (uint8_t)((band1_hmi_freq_default >> 16)&0xff)
#define b1_2 (uint8_t)((band1_hmi_freq_default >> 8)&0xff)
#define b1_3 (uint8_t)(band1_hmi_freq_default&0xff)

#define b2_0 (uint8_t)((band2_hmi_freq_default >> 24)&0xff)
#define b2_1 (uint8_t)((band2_hmi_freq_default >> 16)&0xff)
#define b2_2 (uint8_t)((band2_hmi_freq_default >> 8)&0xff)
#define b2_3 (uint8_t)(band2_hmi_freq_default&0xff)

#define b3_0 (uint8_t)((band3_hmi_freq_default >> 24)&0xff)
#define b3_1 (uint8_t)((band3_hmi_freq_default >> 16)&0xff)
#define b3_2 (uint8_t)((band3_hmi_freq_default >> 8)&0xff)
#define b3_3 (uint8_t)(band3_hmi_freq_default&0xff)

#define b4_0 (uint8_t)((band4_hmi_freq_default >> 24)&0xff)
#define b4_1 (uint8_t)((band4_hmi_freq_default >> 16)&0xff)
#define b4_2 (uint8_t)((band4_hmi_freq_default >> 8)&0xff)
#define b4_3 (uint8_t)(band4_hmi_freq_default&0xff)


#define GP_PTT		15

//extern uint8_t  hmi_sub[HMI_NMENUS];							// Stored option selection per state
//extern uint32_t hmi_freq;  
#define  hmi_freq   memory_band[hmi_mem].mem_freq.u32
extern uint8_t  hmi_mem;     // actual memory
extern bool tx_enabled;
extern bool tx_enable_changed;
extern bool ptt_internal_active;    //PTT output = true for vox, mon and mem
extern bool ptt_external_active;
extern bool ptt_vox_active;	
extern bool ptt_mon_active;
extern bool ptt_aud_active;



#define AUDIO_BUF_MAX    160000  //160k bytes(memory used) * (1 / 16khz(sample freq)) = 10s
//#define AUDIO_BUF_MAX    128000  //128k bytes(memory used) * (1 / 16khz(sample freq)) = 8s
extern uint8_t audio_buf[AUDIO_BUF_MAX];
extern uint32_t audio_rec_pos;
extern uint32_t audio_play_pos;

#define AUDIO_STOPPED   0
#define AUDIO_START     1
#define AUDIO_RUNNING   2

extern uint16_t Aud_Rec_Tx;
extern uint16_t Aud_Rec_Rx;
extern uint16_t Aud_Play_Tx;
extern uint16_t Aud_Play_Spk;


void Setup_Band(uint8_t band);
void hmi_init0(void);
void hmi_init(void);
void hmi_evaluate(void);


//#define TST_MAX_SMETER_SWR  1



#ifdef __cplusplus
}
#endif
#endif
