/*
 * hmi.cpp
 *
 * Created: Apr 2021
 * Author: Arjan te Marvelde
 * May2022: adapted by Klaus Fensterseifer 
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
 * This file contains the HMI driver, processing user inputs.
 * It will also do the logic behind these, and write feedback to the LCD.
 *
 * The 4 auxiliary buttons have the following functions:
 * GP6 - Enter, confirm : Used to select menu items or make choices from a list
 * GP7 - Escape, cancel : Used to exit a (sub)menu or cancel the current action
 * GP8 - Left           : Used to move left, e.g. to select a digit
 * GP9 - Right			: Used to move right, e.g. to select a digit
 *
 * The rotary encoder (GP2, GP3) controls an up/down counter connected to some field. 
 * It may be that the encoder has a bushbutton as well, this can be connected to GP4.
 *     ___     ___
 * ___|   |___|   |___  A
 *   ___     ___     _
 * _|   |___|   |___|   B
 *
 * Encoder channel A triggers on falling edge. 
 * Depending on B level, count is incremented or decremented.
 * 
 * The PTT is connected to GP15 and will be active, except when VOX is used.
 *
 */

#include "Arduino.h"
#include "uSDR.h"
#include "relay.h"
#include "si5351.h"
#include "hmi.h"
#include "dsp.h"
#include "pico/multicore.h"
#include "SPI.h"
#include "TFT_eSPI.h"
#include "display_tft.h"
#include "Eeprom.h"
#include "CwDecoder.h"



//#define HMI_debug    10    //to release some serial print outs for debug

/*
 * GPIO assignments
 */
#define GP_ENC_A	2               // Encoder clock
#define GP_ENC_B	3               // Encoder direction
#define GP_AUX_0_Enter	6								// Enter, Confirm
#define GP_AUX_1_Escape	7								// Escape, Cancel
#define GP_AUX_2_Left	8								// Left move
#define GP_AUX_3_Right	9								// Right move
#define GP_MASK_IN	((1<<GP_ENC_A)|(1<<GP_ENC_B)|(1<<GP_AUX_0_Enter)|(1<<GP_AUX_1_Escape)|(1<<GP_AUX_2_Left)|(1<<GP_AUX_3_Right)|(1<<GP_PTT))
//#define GP_MASK_PTT	(1<<GP_PTT)


#define ENCODER_FALL              10    //increment/decrement freq on falling of A encoder signal
#define ENCODER_FALL_AND_RISE     22    //increment/decrement freq on falling and rising of A encoder signal
#ifdef PY2KLA_setup
#define ENCODER_TYPE              ENCODER_FALL_AND_RISE      //choose to trigger the encoder step on fall and on rise of A signal
#else
#define ENCODER_TYPE              ENCODER_FALL      //choose to trigger the encoder step only on fall of A signal
#endif

#define ENCODER_CW_A_FALL_B_LOW   10    //encoder type clockwise step when B low at falling of A
#define ENCODER_CW_A_FALL_B_HIGH  22    //encoder type clockwise step when B high at falling of A
#ifdef PY2KLA_setup
#define ENCODER_DIRECTION         ENCODER_CW_A_FALL_B_LOW    //direction related to B signal level when A signal is triggered
#else
#define ENCODER_DIRECTION         ENCODER_CW_A_FALL_B_HIGH   //direction related to B signal level when A signal is triggered
#endif

/*
 * Event flags
 */
//#define GPIO_IRQ_ALL		    (GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)
#define GPIO_IRQ_EDGE_ALL	  (GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

/*
 * Display layout:
 *   +----------------+
 *   |USB 14074.0 R920| --> mode=USB, freq=14074.0kHz, state=Rx,S9+20dB
 *   |      Fast -10dB| --> ..., AGC=Fast, Pre=-10dB
 *   +----------------+
 * In this HMI state only tuning is possible, 
 *   using Left/Right for digit and ENC for value, Enter to commit change.
 * Press ESC to enter the submenu states (there is only one sub menu level):
 *
 * Submenu	Values								ENC		Enter			Escape	Left	Right
 * -----------------------------------------------------------------------------------------------
 * Mode		USB, LSB, AM, CW					change	commit			exit	prev	next
 * AGC		Fast, Slow, Off						change	commit			exit	prev	next
 * Pre		+10dB, 0, -10dB, -20dB, -30dB		change	commit			exit	prev	next
 * Vox		NoVOX, Low, Medium, High			change	commit			exit	prev	next
 *
 * --will be extended--
 */

 

//char hmi_o_menu[HMI_NMENUS][8] = {"Tune","Mode","AGC","Pre","VOX"};	// Indexed by hmi_menu  not used - menus done direct in Evaluate()
const char hmi_o_mode[HMI_NUM_OPT_MODE][8] = {"USB","LSB","AM ","CW "};			// Indexed by memory_band[hmi_mem].vars[HMI_S_MODE]  MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
const char hmi_o_agc [HMI_NUM_OPT_AGC][8] = {"NoAGC","Slow ","Fast "};					// Indexed by memory_band[hmi_mem].vars[HMI_S_AGC]
const char hmi_o_pre [HMI_NUM_OPT_PRE][8] = {"-30dB","-20dB","-10dB","0dB  ","+10dB"};	// Indexed by memory_band[hmi_mem].vars[HMI_S_PRE]
const char hmi_o_vox [HMI_NUM_OPT_VOX][8] = {"NoVOX","VOX-L","VOX-M","VOX-H"};		// Indexed by memory_band[hmi_mem].vars[HMI_S_VOX]
#define NoVOX_pos_menu  0   //index for NoVOX option
const char hmi_o_bpf [HMI_NUM_OPT_BPF][8] = {"<2.5","2-6","5-12","10-24","20-40"};
//const char hmi_o_dflash [HMI_NUM_OPT_DFLASH][8] = {"Save", "Saving"};  //only save is visible  (saving is used to start the dflash write)
const char hmi_o_memory [HMI_NUM_OPT_MEMORY_SAVING][8] = {"00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "SAVING"}; 
const char hmi_o_audio [HMI_NUM_OPT_AUDIO][20] = {"Rec from TX", "Rec from RX", "Play to TX", "Play to Speaker"};

//const uint8_t  hmi_num_opt[HMI_NMENUS] = { HMI_NUM_OPT_TUNE, HMI_NUM_OPT_MODE, HMI_NUM_OPT_AGC, HMI_NUM_OPT_PRE, HMI_NUM_OPT_VOX, HMI_NUM_OPT_BPF, HMI_NUM_OPT_DFLASH, HMI_NUM_OPT_AUDIO };	 // number of options for each menu


// Map option to setting
const uint8_t hmi_pre[5] = {REL_ATT_30, REL_ATT_20, REL_ATT_10, REL_ATT_00, REL_PRE_10};
const uint8_t hmi_bpf[5] = {REL_LPF2, REL_BPF6, REL_BPF12, REL_BPF24, REL_BPF40};

uint8_t  hmi_menu;     // menu section 0=Tune/cursor 1=Mode 2=AGC 3=Pre 4=VOX 5=Band 6=Mem  (old hmi_state)
uint8_t  hmi_menu_opt_display;	 // current menu option showing on display (old hmi_option)
        // on <enter>, it will be copied to band vars
        // on starting (or come back to) the menu, hmi_menu_opt_display = band vars, show the actual band vars
uint8_t  hmi_mem;     // actual memory
uint8_t  hmi_mem_to_save;  //memory to save the actual setup
bool hmi_tune_used = false;

//                              { cursor, mode, agc, pre, vox, band, mem ok }
//uint8_t  hmi_sub[HMI_NMENUS] = {      4,    1,   2,   3,   0,    2,      0 };							// Stored option selection per state

/*
arr[row][col]
int arr[3][5] = { {10, 0, 0, 0, 0},
                  {8, 0, 0, 0, 0},
                  {9, 0, 0, 0, 0 }};
*/
//                         0=Tune/cursor 1=Mode 2=AGC 3=Pre 4=VOX 5=Band 6=Mem  ( 8 = HMI_NMENUS ,9,10,11 = freq )
/*
uint8_t  band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE] =  { {4,1,2,3,0,2,0, 0, b0_0, b0_1, b0_2, b0_3},
                                                         {4,1,2,3,0,2,0, b1_0, 0, b1_1, b1_2, b1_3},
                                                         {4,1,2,3,0,2,0, b2_0, 0, b2_1, b2_2, b2_3},
                                                         {4,1,2,3,0,3,0, b3_0, 0, b3_1, b3_2, b3_3},
                                                         {4,1,2,3,0,4,0, b4_0, 0, b4_1, b4_2, b4_3} };
*/


st_memory_band  memory_band[HMI_NUM_OPT_MEMORY] =  {0};




//uint32_t hmi_freq;														// Frequency from Tune state
const uint32_t hmi_step[HMI_NUM_OPT_TUNE] = {10000000L, 1000000L, 100000L, 10000L, 1000L, 100L, 50L};	// Frequency digit increments (tune option = cursor position)
const uint32_t hmi_maxfreq[HMI_NUM_OPT_BPF] = {2500000L, 6000000L, 12000000L, 24000000L, 40000000L};	// max freq for each band from pass band filters
const uint32_t hmi_minfreq[HMI_NUM_OPT_BPF] = {1000000L, 2000000L,  5000000L, 10000000L, 20000000L};	  // min freq for each band from pass band filters

#ifdef PY2KLA_setup
#define HMI_MULFREQ          4			// Factor between HMI and actual frequency
#else
#define HMI_MULFREQ          1      // Factor between HMI and actual frequency
																		// Set to 1, 2 or 4 for certain types of mixer
#endif

															
char s[32];   //aux to print to the screen



/*
<Aud Rec TX> = Record the transmission audio
 - When menu option <Aud Rec Tx> pressed:
   Clear the audio_rec_pos
 - and then
   When put audio I/Q PWM output to transmission, store at the vet_audio

<Aud Rec RX> = Record the reception audio
 - When menu option <Aud Rec Rx> pressed:
   Clear the audio_rec_pos
 - and then
   When put audio PWM to speaker, store at the vet_audio

<Aud Play TX> = Transmit the audio in memory
 - When menu option <Aud Play Tx> pressed:
   Clear the audio_play_pos
   Set ptt_aud_active = TRUE (ptt active)
 - and then
   for each audio memory I calculate the hilbert Q = H(I)
   Put the audio memory I/Q PWM output to transmission 
 - Set ptt_aud_active = FALSE (ptt active)

<Aud Play SPK> = Play the memory audio to the speaker
 - When menu option <Aud Play SPK> pressed:
   Clear the audio_play_pos
 - and then
   Put the audio memory PWM to the speaker


*/
uint8_t audio_buf[AUDIO_BUF_MAX] = {0};  //160k bytes(memory used) * (1 / 16khz(sample freq)) = 10s
uint32_t audio_rec_pos = 0;
uint32_t audio_play_pos = 0;

#define AUDIO_TIME_AFTER  (1*(1000/LOOP_MS))      //1s * (1000/LOOP_MS) = 10
#define AUDIO_TIME_MAIN   ((AUDIO_BUF_MAX/FSAMP_AUDIO)*(1000/LOOP_MS))    //160k/16k * 1000/100 = 100

bool tx_enabled = false;
bool tx_enable_changed = true;
bool ptt_internal_active = false;    //PTT output = true for vox, mon and mem
bool ptt_external_active = false;    //external = from mike
//these inputs will generate the tx_enabled to transmit
bool ptt_vox_active = false;	  //if vox whants to transmit
bool ptt_mon_active = false;
bool ptt_aud_active = false;


uint16_t Aud_Rec_Tx = AUDIO_STOPPED;
uint16_t Aud_Rec_Rx = AUDIO_STOPPED;
uint16_t Aud_Play_Tx = AUDIO_STOPPED;
uint16_t Aud_Play_Spk = AUDIO_STOPPED;



//***********************************************************************
//
//  Audio_Rec_Play - checks if audio menu started
//                   sets the correspondent command
//                   shows a counter on display
// 
//***********************************************************************
void Audio_Rec_Play(void)
{
  static uint16_t time_main = 0;
  static uint16_t time_after = 0;


  if(Aud_Rec_Tx == AUDIO_RUNNING)
  {
    if(ptt_external_active == false)
    {
      Aud_Rec_Tx = AUDIO_STOPPED;
    }
  }
/*
Serialx.println("Time main=" + String(time_main) +
                "   Aud_Rec_Tx=" + String(Aud_Rec_Tx) +
                "   Aud_Rec_Rx=" + String(Aud_Rec_Rx) +
                "   Aud_Play_Tx=" + String(Aud_Play_Tx) +
                "   Aud_Play_Spk=" + String(Aud_Play_Spk) +
                "   ptt_ext_act=" + (ptt_external_active ? "true" : "false"));
*/

  if(time_main > 0)     //if counting time 10s
  {
    if( (Aud_Rec_Tx == AUDIO_STOPPED) &&   //<escape> condition  stops audio
        (Aud_Rec_Rx == AUDIO_STOPPED) &&
        (Aud_Play_Tx == AUDIO_STOPPED) &&
        (Aud_Play_Spk == AUDIO_STOPPED) )
    {
      time_main = 0;
      time_after = AUDIO_TIME_AFTER;
      ptt_aud_active = false;
    }
    else
    {
      time_main--;
      if(time_main == 0)
      {
        time_after = AUDIO_TIME_AFTER;
        ptt_aud_active = false;

        Aud_Rec_Tx = AUDIO_STOPPED;
        Aud_Rec_Rx = AUDIO_STOPPED;
        Aud_Play_Tx = AUDIO_STOPPED;
        Aud_Play_Spk = AUDIO_STOPPED;
      }
      //if <enter> on any Audio menu, draw a count down window from 0 to 10s
      display_tft_countdown(true, (AUDIO_TIME_MAIN-time_main)/10);
    }
  }
  else if(time_after > 0)     //if time after 10s, 1s more to clear the count down window
  {
    time_after--;
    if(time_after == 0)
    {
      display_tft_countdown(false, 0);     //close count down window
    }
    else
    {
      //display_tft_countdown(true, AUDIO_TIME_MAIN-time_main);  //not necessary?  already on screen
    }
  }
  else if(Aud_Rec_Tx == AUDIO_START) 
    {
      display_tft_countdown(true, 0);
      if(ptt_external_active == true)
      {
        audio_rec_pos = 0;
        time_main = AUDIO_TIME_MAIN;
        Aud_Rec_Tx = AUDIO_RUNNING;
      }
    }
  else if(Aud_Rec_Rx == AUDIO_START)
  {
    audio_rec_pos = 0;
    time_main = AUDIO_TIME_MAIN;
    Aud_Rec_Rx = AUDIO_RUNNING;
  }
  else if(Aud_Play_Tx == AUDIO_START)
  {
    audio_play_pos = 0;
    time_main = AUDIO_TIME_MAIN;
    Aud_Play_Tx = AUDIO_RUNNING;
    ptt_aud_active = true;
  }
  else if(Aud_Play_Spk == AUDIO_START)
  {
    audio_play_pos = 0;
    time_main = AUDIO_TIME_MAIN;
    Aud_Play_Spk = AUDIO_RUNNING;
  }
}



//***********************************************************************
//
// get info from actual band = freq -> and store it at band_vars
// when switching back to that band, it will be at the same freq and setup
// it does not save at DFLASH, just to band_vars (it will lose the changes at power off)
// 
//***********************************************************************
void Store_Last_Band(uint8_t band)
{
/*    
  uint16_t j;

  for(j = 0; j < HMI_NMENUS; j++)
    {
      band_vars[band][j] = hmi_sub[j];
    }
*/
/*
  band_vars[band][HMI_NMENUS] = (uint8_t)(hmi_freq >> 24);
  band_vars[band][HMI_NMENUS+1] = (uint8_t)((hmi_freq >> 16)&0xff);
  band_vars[band][HMI_NMENUS+2] = (uint8_t)((hmi_freq >> 8)&0xff);
  band_vars[band][HMI_NMENUS+3] =  (uint8_t)(hmi_freq&0xff);
*/
  //memory_band[hmi_mem].mem_freq.u32 = hmi_freq;
}


//***********************************************************************
//
// get band info from band_vars -> and set  freq
// 
//***********************************************************************
void Setup_Band(uint8_t band)
{
/*    
  uint16_t j;

  for(j = 0; j < HMI_NMENUS; j++)
    {
    if(band_vars[band][j] < hmi_num_opt[j])   //checking boudaries
      {
      hmi_sub[j] = band_vars[band][j];
      }
    else
      {
      hmi_sub[j] = 0;    //hmi_num_opt[j]-1;
      }
    }
*/
	//hmi_freq = 7050000UL;							// Initial frequency
  //get freq from DFLASH band data
/*  
  hmi_freq = band_vars[band][HMI_NMENUS];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+1];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+2];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+3];
*/
  //hmi_freq = memory_band[hmi_mem].mem_freq.u32;

  if(hmi_freq > hmi_maxfreq[band])  //checking boudaries
    {
      hmi_freq = hmi_maxfreq[band];
    }
  else if(hmi_freq < hmi_minfreq[band])
    {
      hmi_freq = hmi_minfreq[band];
    }

/*
  Serialx.print("Setup_Band   freq = " + String(band_vars[band][HMI_NMENUS]));
  Serialx.print(" " + String(band_vars[band][HMI_NMENUS+1]));
  Serialx.print(" " + String(band_vars[band][HMI_NMENUS+2]));
  Serialx.print(" " + String(band_vars[band][HMI_NMENUS+2]));
  Serialx.println("   = " + String(hmi_freq));
*/

  //set the new band to display and freq

	SI_SETFREQ(0, HMI_MULFREQ*hmi_freq);			// Set freq to hmi_freq (MULFREQ depends on mixer type)
	SI_SETPHASE(0, 1);								// Set phase to 90deg (depends on mixer type)
	
	//ptt_state = 0;
	ptt_external_active = false;
	
	dsp_setmode(memory_band[hmi_mem].vars[HMI_S_MODE]);  //MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
	dsp_setvox(memory_band[hmi_mem].vars[HMI_S_VOX]);
	dsp_setagc(memory_band[hmi_mem].vars[HMI_S_AGC]);	
	relay_setattn(hmi_pre[memory_band[hmi_mem].vars[HMI_S_PRE]]);
	relay_setband(hmi_bpf[memory_band[hmi_mem].vars[HMI_S_BPF]]);
	

}






/*
 * HMI State Machine,
 * Handle event according to current state
 * Code needs to be optimized
 */
void hmi_handler(uint8_t event)
{
  static uint8_t hmi_menu_last = HMI_S_BPF;    //last menu when <escape>, used to come back to the same menu


    if ((event==HMI_PTT_ON) && (ptt_internal_active == false))  //if internal is taking the ptt control, not from mike, ignores mike
    {
      ptt_external_active = true;
    }
    else if (event==HMI_PTT_OFF)   
    {
      ptt_external_active = false;
    }

	/* Special case for TUNE state */
	if (hmi_menu == HMI_S_TUNE)  //on main tune
	{
		if (event==HMI_E_ESCAPE)										// Enter submenus
		{
			//memory_band[hmi_mem].vars[hmi_menu] = hmi_menu_opt_display;							// Store cursor position on TUNE
			hmi_menu = hmi_menu_last;										// go to last menu selected before TUNE
			hmi_menu_opt_display = memory_band[hmi_mem].vars[hmi_menu];							// Restore selection of new menu
		}
		else if (event==HMI_E_INCREMENT)
		{
      if(!gpio_get(GP_AUX_0_Enter))  //in case Enter is pressed
      {
        if(fft_gain[hmi_mem]<(1<<FFT_GAIN_SHIFT))
          {
            fft_gain[hmi_mem]++;
          }
        hmi_tune_used = true;
      }
      else if(!gpio_get(GP_AUX_2_Left))  //in case Left is pressed
      {
        if(hmi_mem<(HMI_NUM_OPT_MEMORY-1))
          {
            hmi_mem++;
          }
        hmi_tune_used = true;
      }
      else
      {
			  if (hmi_freq < (hmi_maxfreq[memory_band[hmi_mem].vars[HMI_S_BPF]] - hmi_step[hmi_menu_opt_display]))		// Boundary check HMI_MAXFREQ
			  	hmi_freq += hmi_step[hmi_menu_opt_display];						// Increment selected digit
      }
		}
		else if (event==HMI_E_DECREMENT)
		{
      if(!gpio_get(GP_AUX_0_Enter))  //in case Escape is pressed
      {
        if(fft_gain[hmi_mem]>1)
          {
            fft_gain[hmi_mem]--;
          }
        hmi_tune_used = true;
      }
      else if(!gpio_get(GP_AUX_2_Left))  //in case Left is pressed
      {
        if(hmi_mem>0)
          {
            hmi_mem--;
          }
        hmi_tune_used = true;
      }
      else
      {
        if (hmi_freq > (hmi_step[hmi_menu_opt_display] + hmi_minfreq[memory_band[hmi_mem].vars[HMI_S_BPF]]))		// Boundary check HMI_MINFREQ
				  hmi_freq -= hmi_step[hmi_menu_opt_display];						// Decrement selected digit
      }
		}
		if (event==HMI_E_RIGHT)
      {
			hmi_menu_opt_display = (hmi_menu_opt_display<(HMI_NUM_OPT_TUNE-1))?hmi_menu_opt_display+1:(HMI_NUM_OPT_TUNE-1);					// Digit to the right
      memory_band[hmi_mem].vars[HMI_S_TUNE] = hmi_menu_opt_display;
      }      
		if (event==HMI_E_LEFT)
      {      
			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;					// Digit to the left
      memory_band[hmi_mem].vars[HMI_S_TUNE] = hmi_menu_opt_display;
      }      
	}
  else  //in submenus
  {
  	/* Submenu states */
  	switch(hmi_menu)
  	{
  	case HMI_S_MODE:
  		if (event==HMI_E_INCREMENT)
      {
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_MODE-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_MODE-1;
      }
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_AGC:
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_AGC-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_AGC-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_PRE:
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_PRE-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_PRE-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_VOX:
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_VOX-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_VOX-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_BPF:
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_BPF-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_BPF-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_SAVE: //show only 0 position = save
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_MEMORY-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_MEMORY-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	case HMI_S_AUDIO:
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_AUDIO-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_AUDIO-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
  		break;
  	}
  	
  	/* General actions for all submenus */
  	if (event==HMI_E_ENTER)
    {
      if(hmi_menu == HMI_S_AUDIO)
      {
        switch(hmi_menu_opt_display)
        {
        case AUDIO_REC_TX:
          Aud_Rec_Tx = AUDIO_START;
          break;
        case AUDIO_REC_RX:
          Aud_Rec_Rx = AUDIO_START;
          break;
        case AUDIO_PLAY_TX:
          Aud_Play_Tx = AUDIO_START;
          break;
        case AUDIO_PLAY_SPK:
          Aud_Play_Spk = AUDIO_START;
          break;
        }        
      }
      else if(hmi_menu == HMI_S_BPF)
      {
        memory_band[hmi_mem].vars[HMI_S_BPF] = hmi_menu_opt_display;  //band changed
      }
      else if(hmi_menu == HMI_S_SAVE)
      {
        hmi_mem_to_save = hmi_menu_opt_display;
        memory_band[hmi_mem].vars[hmi_menu] = HMI_NUM_OPT_MEMORY_SAVING;  //saving to indicate save event
      }
      else
      {
  		  memory_band[hmi_mem].vars[hmi_menu] = hmi_menu_opt_display;				// Store selected option	
/*
        if((hmi_menu == HMI_S_VOX) && (hmi_menu_opt_display == NoVOX_pos_menu))  //if switching to NoVOX
        {
          gpio_set_dir(GP_PTT, false);          // PTT pin input
        }
*/
      }
  	}
  	else if (event==HMI_E_ESCAPE)
  	{
      if((hmi_menu == HMI_S_AUDIO) &&
         ((Aud_Rec_Tx != AUDIO_STOPPED) ||   //audio active
          (Aud_Rec_Rx != AUDIO_STOPPED) ||
          (Aud_Play_Tx != AUDIO_STOPPED) ||
          (Aud_Play_Spk != AUDIO_STOPPED)))
      {
        Aud_Rec_Tx = AUDIO_STOPPED;   //stops audio
        Aud_Rec_Rx = AUDIO_STOPPED;
        Aud_Play_Tx = AUDIO_STOPPED;
        Aud_Play_Spk = AUDIO_STOPPED;
      }
      else
      {
        hmi_menu_last = hmi_menu;
        hmi_menu = HMI_S_TUNE;										// Leave submenus
        hmi_menu_opt_display = memory_band[hmi_mem].vars[hmi_menu];							// Restore selection of new state
      }
  	}
  	else if (event==HMI_E_RIGHT)
  	{
  		hmi_menu = (hmi_menu<HMI_NMENUS-1)?(hmi_menu+1):1;		// Change submenu
  		hmi_menu_opt_display = memory_band[hmi_mem].vars[hmi_menu];							// Restore selection of new state
  	}
  	else if (event==HMI_E_LEFT)
  	{
  		hmi_menu = (hmi_menu>1)?(hmi_menu-1):HMI_NMENUS-1;		// Change submenu
  		hmi_menu_opt_display = memory_band[hmi_mem].vars[hmi_menu];							// Restore selection of new state
  	}

  }
  
}

/*
 * GPIO IRQ callback routine
 * Sets the detected event and invokes the HMI state machine
 */
void hmi_callback(uint gpio, uint32_t events)
{
	uint8_t evt=HMI_E_NOEVENT;

	switch (gpio)
	{
/*
    GPIO_IRQ_LEVEL_LOW = 0x1u,
    GPIO_IRQ_LEVEL_HIGH = 0x2u,
    GPIO_IRQ_EDGE_FALL = 0x4u,
    GPIO_IRQ_EDGE_RISE = 0x8u,
*/    
	case GP_ENC_A:									// Encoder
		if (events&GPIO_IRQ_EDGE_FALL)
    {
#if ENCODER_DIRECTION == ENCODER_CW_A_FALL_B_HIGH
			evt = gpio_get(GP_ENC_B)?HMI_E_INCREMENT:HMI_E_DECREMENT;
#else
      evt = gpio_get(GP_ENC_B)?HMI_E_DECREMENT:HMI_E_INCREMENT;
#endif
    } 
#if ENCODER_TYPE == ENCODER_FALL_AND_RISE
    else if (events&GPIO_IRQ_EDGE_RISE)
    {  
#if ENCODER_DIRECTION == ENCODER_CW_A_FALL_B_HIGH
      evt = gpio_get(GP_ENC_B)?HMI_E_DECREMENT:HMI_E_INCREMENT;
#else
      evt = gpio_get(GP_ENC_B)?HMI_E_INCREMENT:HMI_E_DECREMENT;
#endif
    }
#endif
		break;
	case GP_AUX_0_Enter:									// Enter
		if ((events&GPIO_IRQ_EDGE_RISE) && (hmi_tune_used == false))
    {
			evt = HMI_E_ENTER;
    }
		break;
	case GP_AUX_1_Escape:									// Escape
		if ((events&GPIO_IRQ_EDGE_RISE) && (hmi_tune_used == false))
    {
			evt = HMI_E_ESCAPE;
    }
		break;
	case GP_AUX_2_Left:									// Previous
		if ((events&GPIO_IRQ_EDGE_RISE) && (hmi_tune_used == false))
		//if (events&GPIO_IRQ_EDGE_RISE) 
    {
			evt = HMI_E_LEFT;
    }
		break;
	case GP_AUX_3_Right:									// Next
		if ((events&GPIO_IRQ_EDGE_RISE) && (hmi_tune_used == false))
    {
			evt = HMI_E_RIGHT;
    }
		break;

  case GP_PTT:                  // PTT TX
/*
    if (events&GPIO_IRQ_EDGE_ALL)
    {
      evt = gpio_get(GP_PTT)?HMI_PTT_OFF:HMI_PTT_ON;
    }
*/
    if (events&GPIO_IRQ_EDGE_FALL)
    {
      evt = HMI_PTT_ON;
    }    
    if (events&GPIO_IRQ_EDGE_RISE)
    {
      evt = HMI_PTT_OFF;
    }    

    break;

	default:
		return;
	}
	hmi_tune_used = false;
	hmi_handler(evt);								// Invoke state machine
}



/*
 * Initialize the User interface
 * It could take some time to read all DFLASH hmi data, so make it when display is showing title
 */
void hmi_init0(void)
{
	// Initialize LCD and set VFO
  //Init_HMI_data(&memory_band[hmi_mem].vars[HMI_S_BPF]);  //read data from DFLASH
  Eeprom_setup();  //read memories band data from arduino pro mini eeprom I2C
  hmi_mem = 0; //start with the first memory after reset
  //memory_band[hmi_mem].vars[HMI_S_BPF] = memory_band[hmi_mem].vars[HMI_S_BPF]; 
  //Setup_Band(memory_band[hmi_mem].vars[HMI_S_BPF]);
  //  menu position = Tune  and  cursor position = hmi_menu_opt_display
	hmi_menu = HMI_S_TUNE;
	hmi_menu_opt_display = memory_band[hmi_mem].vars[HMI_S_TUNE];  // option on Tune is the cursor position
#ifdef HMI_debug
  Serialx.println("TUNE " + String(memory_band[hmi_mem].vars[HMI_S_BPF]));
  Serialx.println("hmi_menu  " + String(hmi_menu));
  Serialx.println("hmi_menu_opt_display " + String(hmi_menu_opt_display));
#endif	

  CwDecoder_InicTable();   //fill table on running time
}


// dummy function to receive the init interrupts I could not avoid
void dummy_callback(uint gpio, uint32_t events) 
{
    //gpio_acknowledge_irq(gpio, events);
}


/*
 * Initialize the User interface
 */
void hmi_init(void)
{
	/*
	 * Notes on using GPIO interrupts: 
	 * The callback handles interrupts for all GPIOs with IRQ enabled.
	 * Level interrupts don't seem to work properly.
	 * For debouncing, the GPIO pins should be pulled-up and connected to gnd with 100nF.
	 * PTT has separate debouncing logic
	 */

/*
  // force charge the external capacitor/track 
  // used to avoid interrupt on RISE when capacitor is charging
  gpio_set_dir_out_masked(GP_MASK_IN); //GPIO out
  gpio_set_mask(GP_MASK_IN); // GPIO high
  sleep_us(50); // time for acomodation (maybe not needed)
*/

	// Init input GPIOs as input
	gpio_init_mask(GP_MASK_IN);   //	Initialise multiple GPIOs (enabled I/O and set func to GPIO_FUNC_SIO)
                                //  Clear the output enable (i.e. set to input). Clear any output value.

	// Enable pull-ups on input pins
	gpio_pull_up(GP_ENC_A);
	gpio_pull_up(GP_ENC_B);
	gpio_pull_up(GP_AUX_0_Enter);
	gpio_pull_up(GP_AUX_1_Escape);
	gpio_pull_up(GP_AUX_2_Left);
	gpio_pull_up(GP_AUX_3_Right);
	gpio_pull_up(GP_PTT);
/*	
  gpio_set_dir_in_masked(GP_MASK_IN);   //don't need,  already input by  gpio_init_mask()

for(;;)
{
      gpio_put(GP_PTT, 0);      //drive PTT low (active)
      gpio_set_dir(GP_PTT, GPIO_OUT);   // PTT output
      delay(2000);

      //gpio_put(GP_PTT, 0);      //drive PTT low (active)
      gpio_set_dir(GP_PTT, GPIO_IN);   // PTT output
      delay(2000);
}
*/
/*
// 4. LIMPEZA TOTAL - O passo que falta para parar o disparo inicial
// Isso limpa os bits que foram setados durante o tempo que o pino estava "solto"
  for(uint8_t i=0; i<32; i++) 
  {
      if(GP_MASK_IN & (1 << i)) 
      {
          gpio_acknowledge_irq(i, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
      }
  }
*/
// clear previous interrupt
//  gpio_acknowledge_irq(GP_AUX_1_Escape, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

// Enable interrupt on GPIO rise and fall
	gpio_set_irq_enabled(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_0_Enter, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_1_Escape, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_2_Left, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_3_Right, GPIO_IRQ_EDGE_ALL, true);
//	gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, false);
  gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, true);


// I got some interrupt during init that I could not clear, 
// so I will use a dummy function to receive them
  gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, dummy_callback);
  delay(10);  //there could be some time after the interrupt to call the callback, 
              //so wait a little to change the callback

	// Set callback, one for all GPIO, not sure about correctness!
	gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, hmi_callback);

}




#define x_RT  10      //position for R+smeter and T+power
#define y_RT  (31-7)

#define x_xGain ((1*X_CHAR1))  //position for fft gain
#define y_yGain ((3*Y_CHAR1)+8-8)

#define x_plus1  (x_RT+(2*X_CHAR2)) //position for the first +
#define y_plus1  (y_RT+6)

#define x_plus2  (x_RT+(2*X_CHAR2)+13)  //position for the second +
#define y_plus2  (y_RT-3)









/*
S Meter  	Antenna input
Reading   uVrms @ 50R
S9+20	    500
S9+10	    160
S9 	       50
S8 	       25
S7 	       12,5
S6 	       6,25
S5 	       3,125
S4 	       1,5625
S3 	       0,78125
S2 	       0,39063
S1 	       0,19531
*/
//                                             S  1  2  3  4   5   6   7    8    9   9+  9++
int16_t Smeter_table_level[MAX_Smeter_table] = {  1, 2, 4, 9, 18, 35, 75, 150, 300, 400, 600 };  //audio signal value after filters for each antenna level input



// SMeter adjust for "-30dB","-20dB","-10dB","0dB","+10dB"
//                    x22     x8.48   x2.8    x1     x2
const int32_t  smeter_pre_mult[HMI_NUM_OPT_PRE] =  { 353, 136, 46,  1,  2 };  // S level =  (max_a_sample * smeter_pre_mult) >> smeter_pre_shift
const int16_t  smeter_pre_shift[HMI_NUM_OPT_PRE] = {   4,   4,  4,  0,  0 };  // it makes "shift" instead of "division", tries to save processing
int16_t rec_level;
int16_t rec_level_old = 1;
int16_t fft_gain_old = 0;

/**************************************************************************************
    hmi_smeter - writes the S metr value on display
**************************************************************************************/
void hmi_smeter(void)
{
//  static int16_t agc_gain_old = 1;
  int16_t Smeter_index_new;
  //static int16_t Smeter_index = 0;  //smeter table index = number of blocks to draw on smeter bar graph


/*
    if(agc_gain_old != agc_gain)
    {
      rec_level = AGC_GAIN_MAX - agc_gain;
      sprintf(s, "%d", rec_level);
      tft_writexy_(2, TFT_GREEN, TFT_BLACK, 1,2,(uint8_t *)s);
      agc_gain_old = agc_gain;
    }
*/
    if(smeter_display_time >= MAX_SMETER_DISPLAY_TIME)  //new value ready to display, and avoid to write variable at same time on int
    {
      //correcting input ADC value with attenuators
      max_a_sample = (max_a_sample * smeter_pre_mult[memory_band[hmi_mem].vars[HMI_S_PRE]]) >> smeter_pre_shift[memory_band[hmi_mem].vars[HMI_S_PRE]];

      //look for smeter table index
      for(Smeter_index_new=(MAX_Smeter_table-1);  Smeter_index_new>0; Smeter_index_new--)
      {
        if(max_a_sample > Smeter_table_level[Smeter_index_new])
          {
          break;
          }
      }
      Smeter_bargraph(Smeter_index_new);   

      rec_level = Smeter_index_new + 1;  // S level = index + 1

#ifdef TST_MAX_SMETER_SWR
      rec_level = 11;
      rec_level_old = 6;
            //tft.fillRect(x_plus1, y_plus1, X_CHAR1-1, Y_CHAR1-4, TFT_LIGHTGREY); //TFT_BLACK);
            //tft.fillRect(x_plus2, y_plus2, X_CHAR1-1, Y_CHAR1-4, TFT_LIGHTGREY); //TFT_BLACK);
#endif

      if(tx_enable_changed == true)  //if changed tx-rx = display clear
      {
        rec_level_old = 0;  //print all
        fft_gain_old = 0;
      }

/*
      rec_level = rec_level_old;
      if(++contk > 5)
      {
        rec_level = rec_level_old + 1;
        if (rec_level > 11)  rec_level = 1;
        contk = 0;
      }
*/
#ifdef SMETER_TEST  //used to get the audio level for a RF input signal -> to fill the Smeter_table_level[] 
      //prints the audio level to display, 
      sprintf(s, "%3d", max_a_sample);
      //s[3] = 0;  //remove the low digit
      tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 1, 5, 3, 5, (uint8_t *)s);   
#else
      if(rec_level != rec_level_old)  //try to save some processing if the level is the same
      {
        if(rec_level <= 9)  // S1 .. S9
        {
          sprintf(s, "%d", rec_level);
          tft_writexy_plus(2, TFT_GREEN, TFT_BLACK, 0, x_RT+(1*X_CHAR2), 0, y_RT, (uint8_t *)s);

          if(rec_level_old > 9)  //erase the +
          {
            //erase the area for the font 1 = font used for "+"
            tft.fillRect(x_plus1, y_plus1, X_CHAR1-1, Y_CHAR1-4, TFT_BLACK);
            if(rec_level_old > 10)  //erase the other +
            {
            tft.fillRect(x_plus2, y_plus2, X_CHAR1-1, Y_CHAR1-4, TFT_BLACK);
            }
          }
        }
        else   // S9+ or S9++
        {
          if(rec_level_old < 9)  //try to save some processing, if was S9, don't need to write again
          {
            tft_writexy_plus(2, TFT_GREEN, TFT_BLACK, 0, x_RT+(1*X_CHAR2), 0, y_RT, (uint8_t *)"9");
          }
          if((rec_level > 9) && (rec_level_old < 10))  //try to save some processing if already +
          {
            tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 0, x_plus1, 0, y_plus1, (uint8_t *)"+");
          }
          if((rec_level == 10) && (rec_level_old == 11))  //erase the second +
          {
          tft.fillRect(x_plus2, y_plus2, X_CHAR1-1, Y_CHAR1-4, TFT_BLACK);
          }
          if((rec_level == 11) && (rec_level_old < 11))  //try to save some processing if already ++
          {
            tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 0, x_plus2, 0, y_plus2, (uint8_t *)"+");
          }
        }
        rec_level_old = rec_level;
      }
#endif

      display_a_sample = max_a_sample;  //save the last value printed on display
      max_a_sample = 0;  //restart the search for big signal
      smeter_display_time = 0;
    }
     
    if(fft_gain_old != fft_gain[hmi_mem])
    {
      sprintf(s, "%d  ",fft_gain[hmi_mem]);
      s[3]=0;
      tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 0, x_xGain+(1*X_CHAR1), 0, y_yGain, (uint8_t *)s);   
      fft_gain_old = fft_gain[hmi_mem];
    }

}

/*
  if(smeter_display_time < MAX_SMETER_DISPLAY_TIME)   //for some time, look for the bigger signal
  {
    if(avg_a_sample > max_a_sample)  //bigger than displayed (if the value is bigger, print on display right now)
    {
      max_a_sample = avg_a_sample;  //save the bigger audio signal received
      if(max_a_sample > display_a_sample)  //bigger than displayed (if the value is bigger, print on display right now)
      {
        smeter_display_time = MAX_SMETER_DISPLAY_TIME;  //indicate the new (bigger) value is ready to show at display
      }
    }
    else
    {
      smeter_display_time++;  //count time
    }
  }
*/






#if I2C_Arduino_Pro_Mini == 1    //only used when together with Arduino Pro Mini for relays control (and allow SWR reading)


#define TIME_SHOW   6   // x 100ms = "some time"

/**************************************************************************************
    hmi_power_show - used to show the recent bigger power value on the display for "some time"
**************************************************************************************/
bool hmi_power_show(int16_t pow) 
{
  static int16_t pow_big;
  static int16_t time = 0;
  bool ret;

  if(++time > TIME_SHOW)   //count every 100ms
  {
    time = 0;
    pow_big = 0;
    ret = true;  // indicates to show the actual value on display  as the "old" big value already stayed at display for "some time"
  }
  else
  {
    if(pow > pow_big)
    {
      pow_big = pow;   //holds the new bigger value
      time = 0;    // start the timer to keep the value some time
      ret = true;  // indicates to show the actual value on display right now  as it is bigger
    }
    else
    {
      ret = false;  // indicates to keep the value already showed
    }
  }
  return ret;
}


#define SWR_POW_MAX  10  //max power in watts on display (same number of ADC values at swr_pow table)
#define SWR_BARGRAPH_MAX  10  //number of steps for power on bragraph
const int16_t  power_adc[SWR_POW_MAX] = {  20,  40,  60,  80, 100, 120, 140, 160, 180, 200 };   //ADC 160 = 8W measured (values not calibraterd)


/**************************************************************************************
    hmi_power_swr - reads the swr and put on display
**************************************************************************************/
void hmi_power_swr(void)   //read the swr from Arduino Pro Mini I2C
{
	static uint8_t i2c_data[3];
  static uint8_t i2c_data0_old = 0;
  int16_t ret;
	int16_t pow;

	ret = i2c_read_blocking(i2c1, I2C_SWR, i2c_data, 3, false);  // get 3 bytes: SWR, FOR and REF
	if (ret==3)  //number os bytes received  (<0 if no answer from I2C slave)
  {

    if(tx_enable_changed == true)  //if changed tx-rx = display clear
    {
      i2c_data0_old = 0;  //print all
    }


/*
    //prints the SWR level to display, 
    sprintf(s, "%d %02x %02x %02x  ", ret, i2c_data[0], i2c_data[1], i2c_data[2]);
    tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 6, 5, 4, 5, (uint8_t *)s); 
*/
    // converts from AD reading to power using power_adc[] table
    for(pow=0; pow<SWR_POW_MAX; pow++)  //only 10 steps on the bargraph
      if(i2c_data[1] < power_adc[pow])
        break;
    //pow results 0-10W     


#ifdef TST_MAX_SMETER_SWR
    pow = 12;
    i2c_data[0] = 0xf9;// S level max = 15.9 = F9
#endif


    if(hmi_power_show(pow) == true)  //if it is time to show the new power (swr value follows the power moment)
    {

      if(pow < 10)
      {
        sprintf(s, "%d ", pow);
      }
      else
      {
        sprintf(s, "%d", pow);
      }
      //tft_writexy_(2, TFT_RED, TFT_BLACK, 1,2,(uint8_t *)s);
      tft_writexy_plus(2, TFT_RED, TFT_BLACK, 0, x_RT+(1*X_CHAR2), 0, y_RT, (uint8_t *)s);

      //TxPower_bargraph((int16_t)(((double)pow * ((double)SWR_POW_MAX / (double)SWR_BARGRAPH_MAX)+(double)0.5));  //division in case we have more power than bargraph steps
      TxPower_bargraph(pow);


      if(i2c_data0_old != i2c_data[0])
      {
  /*
        if((i2c_data[0]>>4) < 10)
        {
        sprintf(s, " %d.%d", (i2c_data[0]>>4),(i2c_data[0]&0x0f));
        }
        else
        {
        sprintf(s, "%d.%d", (i2c_data[0]>>4),(i2c_data[0]&0x0f));
        }
  */
        sprintf(s, "%d.%d ", (i2c_data[0]>>4),(i2c_data[0]&0x0f));
        tft_writexy_plus(1, TFT_RED, TFT_BLACK, 0, x_xGain+(1*X_CHAR1), 0, y_yGain, (uint8_t *)s);  //same position as "x" on RX
        i2c_data0_old = i2c_data[0];
      }
    }
  }

}

#endif




uint8_t  band_vars_old[HMI_NMENUS] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };          // Stored last option selection
uint32_t hmi_freq_old = 0xff;  //freq in use
uint8_t hmi_mem_old = 0xff;   //memory vars in use
uint8_t tx_enable_old = 3;  //no TX, no RX
uint8_t hmi_menu_old = 0xff;  //menu on display
uint8_t hmi_menu_opt_display_old = 0xff;  //menu option on display

void hmi_refresh(void)   //hmi loop
{
  //for(uint8_t i=0; i<HMI_NMENUS; i++)
  //  band_vars_old[i] = 0xff; 
  //hmi_freq_old = 0xff;
  hmi_mem_old = 0xff;
  tx_enable_old = 3;
  //hmi_menu_old = 0xff;
  hmi_menu_opt_display_old = 0xff;
  fft_gain_old = 0;
}


/*
 * Redraw the display, representing current state
 * This function is called regularly from the main loop.
 */
void hmi_evaluate(void)   //hmi loop
{
  // if band_var changed (after <enter>), set parameters accordingly
  if(hmi_mem_old != hmi_mem)
  {
    hmi_refresh();  //refresh the view of all hmi vars
    hmi_mem_old = hmi_mem;
    sprintf(s, "M%02d", hmi_mem);  // hmi_menu_opt_display = memory_band[hmi_mem].vars[HMI_S_BPF]
    tft_writexy_plus(1, TFT_BLUE, TFT_BLACK, 1,0,4,10,(uint8_t *)s); 
  }
  if((hmi_freq_old != hmi_freq) ||
     (band_vars_old[HMI_S_TUNE] != memory_band[hmi_mem].vars[HMI_S_TUNE]))    //cursor
  {
    SI_SETFREQ(0, HMI_MULFREQ*hmi_freq);
    //freq  (from encoder)
    sprintf(s, "%7.1f", (double)hmi_freq/1000.0);
    tft_writexy_plus(3, TFT_YELLOW, TFT_BLACK, 2,0,2,20,(uint8_t *)s);
    //cursor (writing the freq erase the cursor)
//    tft_cursor_plus(3, TFT_YELLOW, 2+(hmi_menu_opt_display>4?6:hmi_menu_opt_display), 0, 2, 20);
    tft_cursor_plus(3, TFT_YELLOW, 2+(memory_band[hmi_mem].vars[HMI_S_TUNE]>4?6:memory_band[hmi_mem].vars[HMI_S_TUNE]), 0, 2, 20);
    display_fft_graf_top();  //scale freqs
    hmi_freq_old = hmi_freq;
    band_vars_old[HMI_S_TUNE] = memory_band[hmi_mem].vars[HMI_S_TUNE];    //cursor
  }
  if(band_vars_old[HMI_S_MODE] != memory_band[hmi_mem].vars[HMI_S_MODE])    //mode (SSB AM CW)
  {
    dsp_setmode(memory_band[hmi_mem].vars[HMI_S_MODE]);  //MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
//    sprintf(s, "%s  ", hmi_o_mode[memory_band[hmi_mem].vars[HMI_S_MODE]]);  //removing LSB to create space for swr on display
//    tft_writexy_plus(2, TFT_GREEN, TFT_BLACK, 0, 0, 0, 22, (uint8_t *)s);
    display_fft_graf_top();  //scale freqs, mode changes the triangle

    if(memory_band[hmi_mem].vars[HMI_S_MODE] == MODE_CW)
    {
      CwDecoder_Inic();
    }
    if(band_vars_old[HMI_S_MODE] == MODE_CW)
    {
      CwDecoder_Exit();
    }

    band_vars_old[HMI_S_MODE] = memory_band[hmi_mem].vars[HMI_S_MODE];
  }
  if(band_vars_old[HMI_S_VOX] != memory_band[hmi_mem].vars[HMI_S_VOX])
  {
    dsp_setvox(memory_band[hmi_mem].vars[HMI_S_VOX]);
    band_vars_old[HMI_S_VOX] = memory_band[hmi_mem].vars[HMI_S_VOX];
  }
  if(band_vars_old[HMI_S_AGC] != memory_band[hmi_mem].vars[HMI_S_AGC])
  {
    dsp_setagc(memory_band[hmi_mem].vars[HMI_S_AGC]); 
    band_vars_old[HMI_S_AGC] = memory_band[hmi_mem].vars[HMI_S_AGC];
  }
  if(band_vars_old[HMI_S_BPF] != memory_band[hmi_mem].vars[HMI_S_BPF])
  {
    Setup_Band(memory_band[hmi_mem].vars[HMI_S_BPF]);  // = memory_band[hmi_mem].vars[HMI_S_BPF]  get the new band data 
    band_vars_old[HMI_S_BPF] = memory_band[hmi_mem].vars[HMI_S_BPF];  // = memory_band[hmi_mem].vars[HMI_S_BPF]  

    //printf(s, "B%d", memory_band[hmi_mem].vars[HMI_S_BPF]);  // hmi_menu_opt_display = memory_band[hmi_mem].vars[HMI_S_BPF]
    //tft_writexy_plus(2, TFT_BLUE, TFT_BLACK, 0,0,3,18,(uint8_t *)s);
    sprintf(s, "B%d", memory_band[hmi_mem].vars[HMI_S_BPF]);  // hmi_menu_opt_display = memory_band[hmi_mem].vars[HMI_S_BPF]
    tft_writexy_plus(1, TFT_YELLOW, TFT_BLACK, 1,0,5,10,(uint8_t *)s);
  }  
  if(band_vars_old[HMI_S_PRE] != memory_band[hmi_mem].vars[HMI_S_PRE])
  {  
    relay_setattn(hmi_pre[memory_band[hmi_mem].vars[HMI_S_PRE]]);
    band_vars_old[HMI_S_PRE] = memory_band[hmi_mem].vars[HMI_S_PRE];
  }
  if(memory_band[hmi_mem].vars[HMI_S_SAVE] == HMI_NUM_OPT_MEMORY_SAVING)  //mem save + enter = saving
  {
    //sprintf(s, "%s       ", hmi_o_dflash[hmi_menu_opt_display]);
    //tft_writexy_(1, TFT_RED, TFT_BLACK,8,0,(uint8_t *)s);  
#ifdef HMI_debug
    Serialx.println("Mem save   band " + String(memory_band[hmi_mem].vars[HMI_S_BPF]));
#endif

    memory_band[hmi_mem].vars[HMI_S_SAVE] = hmi_mem_to_save;  //menu actual = saved
    Save_Band_Eeprom(hmi_mem, hmi_mem_to_save);  //hmi_save( actual,  new one)
    hmi_mem = hmi_mem_to_save;  //actual = saved

    //sprintf(s, "%s       ", hmi_o_dflash[hmi_menu_opt_display]);
    //tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,8,0,(uint8_t *)s);  
  }




  //T or R  (using letters instead of arrow used on original project)
  if(tx_enable_old != (tx_enabled?1:0))
  {
    //erase the area for T or R, infos and the bar graph area
    tft.fillRect(x_RT, y_RT, (6*X_CHAR1), (3*Y_CHAR1), TFT_BLACK);   // TFT_LIGHTGREY);  TFT_BLACK); 

    if(tx_enabled == true)
    {
      //tft.drawRoundRect(x_RT-9, y_RT-8, (6*X_CHAR1)+17, (3*Y_CHAR1)+9, 10, TFT_RED);
      tft_writexy_plus(2, TFT_RED, TFT_BLACK, 0, x_RT, 0, y_RT, (uint8_t *)"T");
#if I2C_Arduino_Pro_Mini == 1    //using Arduino Pro Mini for relays control (and allow SWR reading)
      tft_writexy_plus(1, TFT_RED, TFT_BLACK, 0, x_xGain, 0, y_yGain, (uint8_t *)"#0.0");
#endif
    }
    else
    {
      //tft.drawRoundRect(x_RT-9, y_RT-8, (6*X_CHAR1)+17, (3*Y_CHAR1)+9, 10, TFT_GREEN);
      tft_writexy_plus(2, TFT_GREEN, TFT_BLACK, 0, x_RT, 0, y_RT, (uint8_t *)"R");
      tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 0, x_xGain, 0, y_yGain, (uint8_t *)"x");
    }
    rec_level_old = rec_level+1;

    tx_enable_changed = true;  //signal to init values at display

    tx_enable_old = (tx_enabled?1:0);
  }




  if(tx_enabled == false)  /* RX */
  {
    //Smeter rec level
    hmi_smeter();  //during RX, print Smeter on display

    if(memory_band[hmi_mem].vars[HMI_S_MODE] == MODE_CW)
    {
      CwDecoder_array_in();
    }
  }
  else  /* TX */
  {
#if I2C_Arduino_Pro_Mini == 1    //using Arduino Pro Mini for relays control (and allow SWR reading)
    hmi_power_swr();  //during TX, read the SWR, and print it on display
#endif
  }
  tx_enable_changed = false;  //signal to init values at display - already used


  // if menu changed, print new value

  if((hmi_menu_old != hmi_menu) || (hmi_menu_opt_display_old != hmi_menu_opt_display))
  {
#ifdef HMI_debug
  Serialx.println("evaluate Band " + String(memory_band[hmi_mem].vars[HMI_S_BPF]));
  Serialx.println("hmi_menu  " + String(hmi_menu_old) + " -> " + String(hmi_menu));
  Serialx.println("hmi_menu_opt_display " + String(hmi_menu_opt_display_old) + " -> "  + String(hmi_menu_opt_display));
#endif

    //erase the menu area
    tft.fillRect(0, 0, display_WIDTH, Y_CHAR1, TFT_BLACK);

  	//menu 
  	switch (hmi_menu)
  	{
  	case HMI_S_TUNE:
  		sprintf(s, "%s  %s  %s %s ", hmi_o_mode[memory_band[hmi_mem].vars[HMI_S_MODE]], hmi_o_vox[memory_band[hmi_mem].vars[HMI_S_VOX]], hmi_o_agc[memory_band[hmi_mem].vars[HMI_S_AGC]], hmi_o_pre[memory_band[hmi_mem].vars[HMI_S_PRE]]);
      tft_writexy_(1, TFT_BLUE, TFT_BLACK,0,0,(uint8_t *)s);  
      //cursor
      tft_cursor_plus(3, TFT_YELLOW, 2+(memory_band[hmi_mem].vars[HMI_S_TUNE]>4?6:memory_band[hmi_mem].vars[HMI_S_TUNE]), 0, 2, 20);    
  		break;
  	case HMI_S_MODE:
  		sprintf(s, "Set Mode: %s        ", hmi_o_mode[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_AGC:
  		sprintf(s, "Set AGC: %s        ", hmi_o_agc[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_PRE:
  		sprintf(s, "Set Pre: %s        ", hmi_o_pre[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_VOX:
  		sprintf(s, "Set VOX: %s        ", hmi_o_vox[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_BPF:
  		sprintf(s, "Band: B%d %sMHz    ", hmi_menu_opt_display, hmi_o_bpf[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_SAVE:
      //if(memory_band[hmi_mem].vars[HMI_S_SAVE] == HMI_NUM_OPT_MEMORY_SAVING) 
      {
    	//	sprintf(s, "Memory saving    ", hmi_o_memory[hmi_menu_opt_display]);
      }
      //else
      {
        //sprintf(s, "Memory: %s       ", hmi_o_dflash[hmi_menu_opt_display]);
        sprintf(s, "Memory save: %s  ", hmi_o_memory[hmi_menu_opt_display]);
      }
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	case HMI_S_AUDIO:
  		sprintf(s, "Audio: %s       ", hmi_o_audio[hmi_menu_opt_display]);
      tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,0,0,(uint8_t *)s);  
  		break;
  	default:
  		break;
  	}
   
    hmi_menu_old = hmi_menu;
    hmi_menu_opt_display_old = hmi_menu_opt_display;

  } 




  if (aud_samples_state == AUD_STATE_SAMP_RDY) //design a new graphic only when data is ready
  {
    //plot audio graphic     
    display_aud_graf();

    aud_samples_state = AUD_STATE_SAMP_IN;  
  }



  Audio_Rec_Play();  //check for audio rec play function

  CwDecoder_Loop();  //task on 100ms loop
}
