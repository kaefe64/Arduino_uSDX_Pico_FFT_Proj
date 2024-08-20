/*
 * hmi.c
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
/*
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
*/
#include "Arduino.h"
#include "uSDR.h"
#include "relay.h"
#include "si5351.h"
#include "dsp.h"
#include "hmi.h"
#include "dsp.h"
#include "pico/multicore.h"
#include "SPI.h"
#include "TFT_eSPI.h"
#include "display_tft.h"
#include "Dflash.h"



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
char hmi_o_mode[HMI_NUM_OPT_MODE][8] = {"USB","LSB","AM","CW"};			// Indexed by band_vars[hmi_band][HMI_S_MODE]  MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
char hmi_o_agc [HMI_NUM_OPT_AGC][8] = {"NoAGC","Slow","Fast"};					// Indexed by band_vars[hmi_band][HMI_S_AGC]
char hmi_o_pre [HMI_NUM_OPT_PRE][8] = {"-30dB","-20dB","-10dB","0dB","+10dB"};	// Indexed by band_vars[hmi_band][HMI_S_PRE]
char hmi_o_vox [HMI_NUM_OPT_VOX][8] = {"NoVOX","VOX-L","VOX-M","VOX-H"};		// Indexed by band_vars[hmi_band][HMI_S_VOX]
#define NoVOX_pos_menu  0   //index for NoVOX option
char hmi_o_bpf [HMI_NUM_OPT_BPF][8] = {"<2.5","2-6","5-12","10-24","20-40"};
char hmi_o_dflash [HMI_NUM_OPT_DFLASH][8] = {"Save", "Saving"};  //only save is visible  (saving is used to start the dflash write)
char hmi_o_audio [HMI_NUM_OPT_AUDIO][20] = {"Rec from TX", "Rec from RX", "Play to TX", "Play to Speaker"};

//const uint8_t  hmi_num_opt[HMI_NMENUS] = { HMI_NUM_OPT_TUNE, HMI_NUM_OPT_MODE, HMI_NUM_OPT_AGC, HMI_NUM_OPT_PRE, HMI_NUM_OPT_VOX, HMI_NUM_OPT_BPF, HMI_NUM_OPT_DFLASH, HMI_NUM_OPT_AUDIO };	 // number of options for each menu


// Map option to setting
uint8_t hmi_pre[5] = {REL_ATT_30, REL_ATT_20, REL_ATT_10, REL_ATT_00, REL_PRE_10};
uint8_t hmi_bpf[5] = {REL_LPF2, REL_BPF6, REL_BPF12, REL_BPF24, REL_BPF40};

uint8_t  hmi_menu;     // menu section 0=Tune/cursor 1=Mode 2=AGC 3=Pre 4=VOX 5=Band 6=Mem  (old hmi_state)
uint8_t  hmi_menu_opt_display;	 // current menu option showing on display (it will be copied to band vars on <enter>)  (old hmi_option)
uint8_t  hmi_band;     // actual band

//                              { cursor, mode, agc, pre, vox, band, mem ok }
//uint8_t  hmi_sub[HMI_NMENUS] = {      4,    1,   2,   3,   0,    2,      0 };							// Stored option selection per state

/*
arr[row][col]
int arr[3][5] = { {10, 0, 0, 0, 0},
                  {8, 0, 0, 0, 0},
                  {9, 0, 0, 0, 0 }};
*/
//                                                0=Tune/cursor 1=Mode 2=AGC 3=Pre 4=VOX 5=Band 6=Mem 7,8,9,10=freq
uint8_t  band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE] =  { {4,0,2,3,0,0,0, b0_0, b0_1, b0_2, b0_3},
                                                  {4,1,2,3,0,1,0, b1_0, b1_1, b1_2, b1_3},
                                                  {4,1,2,3,0,2,0, b2_0, b2_1, b2_2, b2_3},
                                                  {4,1,2,3,0,3,0, b3_0, b3_1, b3_2, b3_3},
                                                  {4,1,2,3,0,4,0, b4_0, b4_1, b4_2, b4_3} };







uint32_t hmi_freq;														// Frequency from Tune state
uint32_t hmi_step[HMI_NUM_OPT_TUNE] = {10000000, 1000000, 100000, 10000, 1000, 100, 50};	// Frequency digit increments (tune option = cursor position)
//#define HMI_MAXFREQ		30000000
//#define HMI_MINFREQ		     100
const uint32_t hmi_maxfreq[HMI_NUM_OPT_BPF] = {2500000, 6000000, 12000000, 24000000, 40000000};	// max freq for each band from pass band filters
const uint32_t hmi_minfreq[HMI_NUM_OPT_BPF] = {1000000, 2000000,  5000000, 10000000, 20000000};	  // min freq for each band from pass band filters

#ifdef PY2KLA_setup
#define HMI_MULFREQ          4			// Factor between HMI and actual frequency
#else
#define HMI_MULFREQ          1      // Factor between HMI and actual frequency
																		// Set to 1, 2 or 4 for certain types of mixer
#endif

															



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
  uint16_t j;
/*    
  for(j = 0; j < HMI_NMENUS; j++)
    {
      band_vars[band][j] = hmi_sub[j];
    }
*/

  band_vars[band][HMI_NMENUS] = (uint8_t)(hmi_freq >> 24);
  band_vars[band][HMI_NMENUS+1] = (uint8_t)((hmi_freq >> 16)&0xff);
  band_vars[band][HMI_NMENUS+2] = (uint8_t)((hmi_freq >> 8)&0xff);
  band_vars[band][HMI_NMENUS+3] =  (uint8_t)(hmi_freq&0xff);
}


//***********************************************************************
//
// get band info from band_vars -> and set  freq
// 
//***********************************************************************
void Setup_Band(uint8_t band)
{
  uint16_t j;
/*    
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
  hmi_freq = band_vars[band][HMI_NMENUS];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+1];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+2];
  hmi_freq <<= 8;        
  hmi_freq += band_vars[band][HMI_NMENUS+3];

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
	
	dsp_setmode(band_vars[band][HMI_S_MODE]);  //MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
	dsp_setvox(band_vars[band][HMI_S_VOX]);
	dsp_setagc(band_vars[band][HMI_S_AGC]);	
	relay_setattn(hmi_pre[band_vars[band][HMI_S_PRE]]);
	relay_setband(hmi_bpf[band_vars[band][HMI_S_BPF]]);
	


//	hmi_menu = HMI_S_BPF;  //changing band on band menu
//  hmi_band = band;
//	hmi_menu_opt_display = band_vars[hmi_band][hmi_menu];  //get the menu option = actual band
  
}



//***********************************************************************
//
// Save the actual band variables to DFLASH 
//    on first empty mem block = last_block+1 mem position
//
//
//***********************************************************************
void Save_Actual_Band_Dflash(void)
{
  uint8_t   data_block[DATA_BLOCK_SIZE];
  uint16_t  i;

  //copy data from band vars to dflash block (not all menus are saved at dflash)
  for (i=0; i<HMI_NMENUS_DFLASH; i++)
    {
    data_block[i] = band_vars[hmi_band][i];
    }

  // copy freq to dflash block and save freq to band vars (after menu data)
  data_block[HMI_NMENUS_DFLASH] = band_vars[hmi_band][HMI_NMENUS] = (uint8_t)(hmi_freq >> 24);  //uint32_t
  data_block[HMI_NMENUS_DFLASH+1] = band_vars[hmi_band][HMI_NMENUS+1] = (uint8_t)((hmi_freq >> 16) & 0xff); 
  data_block[HMI_NMENUS_DFLASH+2] = band_vars[hmi_band][HMI_NMENUS+2] = (uint8_t)((hmi_freq >> 8) & 0xff); 
  data_block[HMI_NMENUS_DFLASH+3] = band_vars[hmi_band][HMI_NMENUS+3] = (uint8_t)(hmi_freq & 0xff); 


  // write last menu configuration to data flash memory
//  if(Dflash_write_block(&band_vars[hmi_band][0]) == true)
  if(Dflash_write_block(data_block) == true)
  {
#ifdef HMI_debug
      Serialx.println("\nWrite block to DFLASH = OK");
      for(int ndata = 0; ndata < HMI_NMENUS; ndata++)
        {   
        Serialx.print(" " + String(band_vars[hmi_band][ndata]));
        }
      Serialx.println("\n");
  }
  else
  {
      Serialx.println("\nWrite block to DFLASH = NOT OK");
#endif
  }
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
			//band_vars[hmi_band][hmi_menu] = hmi_menu_opt_display;							// Store cursor position on TUNE
			hmi_menu = hmi_menu_last;										// go to last menu selected before TUNE
			hmi_menu_opt_display = band_vars[hmi_band][hmi_menu];							// Restore selection of new menu
		}
		else if (event==HMI_E_INCREMENT)
		{
      //if(!gpio_get(GP_AUX_1_Escape))  //in case Escape is pressed
      if(!gpio_get(GP_AUX_0_Enter))  //in case Escape is pressed
      {
        if(fft_gain<(1<<FFT_GAIN_SHIFT))
          {
            fft_gain++;
          }
      }
      else
      {
			  if (hmi_freq < (hmi_maxfreq[band_vars[hmi_band][HMI_S_BPF]] - hmi_step[hmi_menu_opt_display]))		// Boundary check HMI_MAXFREQ
			  	hmi_freq += hmi_step[hmi_menu_opt_display];						// Increment selected digit
      }
		}
		else if (event==HMI_E_DECREMENT)
		{
      //if(!gpio_get(GP_AUX_1_Escape))  //in case Escape is pressed
      if(!gpio_get(GP_AUX_0_Enter))  //in case Escape is pressed
      {
        if(fft_gain>1)
          {
            fft_gain--;
          }
      }
      else
      {
        if (hmi_freq > (hmi_step[hmi_menu_opt_display] + hmi_minfreq[band_vars[hmi_band][HMI_S_BPF]]))		// Boundary check HMI_MINFREQ
				  hmi_freq -= hmi_step[hmi_menu_opt_display];						// Decrement selected digit
      }
		}
		if (event==HMI_E_RIGHT)
      {
			hmi_menu_opt_display = (hmi_menu_opt_display<(HMI_NUM_OPT_TUNE-1))?hmi_menu_opt_display+1:(HMI_NUM_OPT_TUNE-1);					// Digit to the right
      band_vars[hmi_band][HMI_S_TUNE] = hmi_menu_opt_display;
      }      
		if (event==HMI_E_LEFT)
      {      
			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;					// Digit to the left
      band_vars[hmi_band][HMI_S_TUNE] = hmi_menu_opt_display;
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
  	case HMI_S_DFLASH: //show only 0 position = save
      /*
  		if (event==HMI_E_INCREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display<HMI_NUM_OPT_DFLASH-1)?hmi_menu_opt_display+1:HMI_NUM_OPT_DFLASH-1;
  		else if (event==HMI_E_DECREMENT)
  			hmi_menu_opt_display = (hmi_menu_opt_display>0)?hmi_menu_opt_display-1:0;
      */
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
        hmi_band = hmi_menu_opt_display;  //band changed
      }
      else if(hmi_menu == HMI_S_DFLASH)
      {
        band_vars[hmi_band][hmi_menu] = 1;  //saving to indicate save event
      }
      else
      {
  		  band_vars[hmi_band][hmi_menu] = hmi_menu_opt_display;				// Store selected option	
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
        hmi_menu_opt_display = band_vars[hmi_band][hmi_menu];							// Restore selection of new state
      }
  	}
  	else if (event==HMI_E_RIGHT)
  	{
  		hmi_menu = (hmi_menu<HMI_NMENUS-1)?(hmi_menu+1):1;		// Change submenu
  		hmi_menu_opt_display = band_vars[hmi_band][hmi_menu];							// Restore selection of new state
  	}
  	else if (event==HMI_E_LEFT)
  	{
  		hmi_menu = (hmi_menu>1)?(hmi_menu-1):HMI_NMENUS-1;		// Change submenu
  		hmi_menu_opt_display = band_vars[hmi_band][hmi_menu];							// Restore selection of new state
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
		if (events&GPIO_IRQ_EDGE_FALL)
    {
			evt = HMI_E_ENTER;
    }
		break;
	case GP_AUX_1_Escape:									// Escape
		if (events&GPIO_IRQ_EDGE_FALL)
    {
			evt = HMI_E_ESCAPE;
    }
		break;
	case GP_AUX_2_Left:									// Previous
		if (events&GPIO_IRQ_EDGE_FALL)
    {
			evt = HMI_E_LEFT;
    }
		break;
	case GP_AUX_3_Right:									// Next
		if (events&GPIO_IRQ_EDGE_FALL)
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
	
	hmi_handler(evt);								// Invoke state machine
}



/*
 * Initialize the User interface
 * It could take some time to read all DFLASH hmi data, so make it when display is showing title
 */
void hmi_init0(void)
{
	// Initialize LCD and set VFO
  Init_HMI_data(&hmi_band);  //read data from DFLASH
  //Setup_Band(hmi_band);
  //  menu position = Tune  and  cursor position = hmi_menu_opt_display
	hmi_menu = HMI_S_TUNE;
	hmi_menu_opt_display = band_vars[hmi_band][HMI_S_TUNE];  // option on Tune is the cursor position
#ifdef HMI_debug
  Serialx.println("TUNE " + String(hmi_band));
  Serialx.println("hmi_menu  " + String(hmi_menu));
  Serialx.println("hmi_menu_opt_display " + String(hmi_menu_opt_display));
#endif	
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

	// Init input GPIOs
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


// Enable interrupt on level low
	gpio_set_irq_enabled(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_0_Enter, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_1_Escape, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_2_Left, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_AUX_3_Right, GPIO_IRQ_EDGE_ALL, true);
//	gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, false);
  gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, true);

	// Set callback, one for all GPIO, not sure about correctness!
	gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, hmi_callback);


}



// SMeter adjust for "-30dB","-20dB","-10dB","0dB","+10dB"
//                    x22     x8.48   x2.8    x1     x2
int32_t  smeter_pre_mult[HMI_NUM_OPT_PRE] =  { 353, 136, 46, 1, 2 };  // S level =  (max_a_sample * smeter_pre_mult) >> smeter_pre_shift
int16_t  smeter_pre_shift[HMI_NUM_OPT_PRE] = {   4,   4,  4, 0, 0 };  // it makes "shift" instead of "division", tries to save processing
//int16_t contk = 0;
/*
 * Redraw the display, representing current state
 * This function is called regularly from the main loop.
 */
void hmi_evaluate(void)   //hmi loop
{
	char s[32];
  int16_t rec_level;
  static int16_t rec_level_old = 1;
  
  static uint8_t  band_vars_old[HMI_NMENUS] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };          // Stored last option selection
  static uint32_t hmi_freq_old = 0xff;
  static uint8_t hmi_band_old = 0xff;
  static bool tx_enable_old = true;
  static uint8_t hmi_menu_old = 0xff;
  static uint8_t hmi_menu_opt_display_old = 0xff;
//  static int16_t agc_gain_old = 1;
  static int16_t fft_gain_old = 0;
//  static uint32_t hmi_freq_fft;

#ifdef HMI_debug
  uint16_t ndata;
      for(ndata = 1; ndata < HMI_NMENUS-2; ndata++)
        {   
        if(band_vars[hmi_band][ndata] != band_vars_old[ndata])
           break;
        }
      if(ndata < HMI_NMENUS-2)
        {
        Serialx.print("evaluate Band " + String(hmi_band) + "  band vars changed   ");
        for(ndata = 0; ndata < HMI_NMENUS; ndata++)
          {   
          Serialx.print(" " + String(band_vars_old[ndata]));
          }
        Serialx.print("  ->  ");
        for(ndata = 0; ndata < HMI_NMENUS; ndata++)
          {   
          Serialx.print(" " + String(band_vars[hmi_band][ndata]));
          }
        Serialx.println("\n");
        }      
#endif

  // if band_var changed (after <enter>), set parameters accordingly

  if(hmi_freq_old != hmi_freq)
  {
    SI_SETFREQ(0, HMI_MULFREQ*hmi_freq);
    //freq  (from encoder)
    sprintf(s, "%7.1f", (double)hmi_freq/1000.0);
    tft_writexy_plus(3, TFT_YELLOW, TFT_BLACK, 2,0,2,20,(uint8_t *)s);
    //cursor (writing the freq erase the cursor)
//    tft_cursor_plus(3, TFT_YELLOW, 2+(hmi_menu_opt_display>4?6:hmi_menu_opt_display), 0, 2, 20);
    tft_cursor_plus(3, TFT_YELLOW, 2+(band_vars[hmi_band][HMI_S_TUNE]>4?6:band_vars[hmi_band][HMI_S_TUNE]), 0, 2, 20);
    display_fft_graf_top();  //scale freqs
    hmi_freq_old = hmi_freq;
  }
  if(band_vars_old[HMI_S_MODE] != band_vars[hmi_band][HMI_S_MODE])    //mode (SSB AM CW)
  {
    dsp_setmode(band_vars[hmi_band][HMI_S_MODE]);  //MODE_USB=0 MODE_LSB=1  MODE_AM=2  MODE_CW=3
    sprintf(s, "%s  ", hmi_o_mode[band_vars[hmi_band][HMI_S_MODE]]);
    tft_writexy_(2, TFT_GREEN, TFT_BLACK, 0,1,(uint8_t *)s);
    display_fft_graf_top();  //scale freqs, mode changes the triangle
    band_vars_old[HMI_S_MODE] = band_vars[hmi_band][HMI_S_MODE];
  }
  if(band_vars_old[HMI_S_VOX] != band_vars[hmi_band][HMI_S_VOX])
  {
    dsp_setvox(band_vars[hmi_band][HMI_S_VOX]);
    band_vars_old[HMI_S_VOX] = band_vars[hmi_band][HMI_S_VOX];
  }
  if(band_vars_old[HMI_S_AGC] != band_vars[hmi_band][HMI_S_AGC])
  {
    dsp_setagc(band_vars[hmi_band][HMI_S_AGC]); 
    band_vars_old[HMI_S_AGC] = band_vars[hmi_band][HMI_S_AGC];
  }
  if(hmi_band_old != hmi_band)
  {
    if(hmi_band_old < HMI_S_BPF)  //if not the first time;
      {
      Store_Last_Band(hmi_band_old);  // store data from old band (save freq to have it when back to this band)
      }
    //relay_setband(hmi_band);  // = hmi_band  
    sleep_ms(1);                  // I2C doesn't work without...
    Setup_Band(hmi_band);  // = hmi_band  get the new band data 
    hmi_band_old = hmi_band;  // = hmi_band  

    sprintf(s, "B%d", band_vars[hmi_band][HMI_S_BPF]);  // hmi_menu_opt_display = band_vars[hmi_band][HMI_S_BPF]
    //tft_writexy_plus(3, TFT_YELLOW, TFT_BLACK, 0,0,2,20,(uint8_t *)s);
    tft_writexy_plus(2, TFT_BLUE, TFT_BLACK, 0,0,3,18,(uint8_t *)s);

  }
  if(band_vars_old[HMI_S_PRE] != band_vars[hmi_band][HMI_S_PRE])
  {  
    relay_setattn(hmi_pre[band_vars[hmi_band][HMI_S_PRE]]);
    band_vars_old[HMI_S_PRE] = band_vars[hmi_band][HMI_S_PRE];
  }
  if(band_vars[hmi_band][HMI_S_DFLASH] == 1)  //mem save + enter = saving
  {
    //sprintf(s, "%s       ", hmi_o_dflash[hmi_menu_opt_display]);
    //tft_writexy_(1, TFT_RED, TFT_BLACK,8,0,(uint8_t *)s);  
#ifdef HMI_debug
    Serialx.println("Mem save   band " + String(hmi_band));
#endif
    band_vars[hmi_band][HMI_S_DFLASH] = 0;  //back to 0
    Save_Actual_Band_Dflash();

    //sprintf(s, "%s       ", hmi_o_dflash[hmi_menu_opt_display]);
    //tft_writexy_(1, TFT_MAGENTA, TFT_BLACK,8,0,(uint8_t *)s);  
  }




  //T or R  (using letters instead of arrow used on original project)
  if(tx_enable_old != tx_enabled)
  {
    if(tx_enabled == true)
    {
      //sprintf(s, "T   ");
      tft_writexy_(2, TFT_RED, TFT_BLACK, 0,2,(uint8_t *)"T   ");
    }
    else
    {
      //sprintf(s, "R");
      tft_writexy_(2, TFT_GREEN, TFT_BLACK, 0,2,(uint8_t *)"R");

      //sprintf(s, "x");
      tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 4, 9, 3, 5, (uint8_t *)"x");
    }
    rec_level_old = rec_level+1;

    tx_enable_old = tx_enabled;
  }



/*
  if (tx_enabled == false)  //waterfall only during RX
  {
    if (fft_display_graf_new == 1)    //design a new graphic only when a new line is ready from FFT
    {
      if(hmi_freq == hmi_freq_fft)
      {
        //plot waterfall graphic     
        display_fft_graf((uint16_t)(hmi_freq/500));  // warefall 110ms
      }
      else
      {
        //plot waterfall graphic     
        display_fft_graf((uint16_t)(hmi_freq_fft/500));  // warefall 110ms
        hmi_freq_fft = hmi_freq;
      }

      fft_display_graf_new = 0;  
      fft_samples_ready = 2;  //ready to start new sample collect
    }
  }

*/




   
  //Smeter rec level
  if(tx_enabled == false)
  {
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
      max_a_sample = (max_a_sample * smeter_pre_mult[band_vars[hmi_band][HMI_S_PRE]]) >> smeter_pre_shift[band_vars[hmi_band][HMI_S_PRE]];
      rec_level = Smeter(max_a_sample);
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
          tft_writexy_(2, TFT_GREEN, TFT_BLACK, 1,2,(uint8_t *)s);

          if(rec_level_old > 9)  //erase the +
          {
            //erase the area for the font 1 = font used for "+"
            tft.fillRect((3*X_CHAR1)+0, (3*Y_CHAR1)+5, X_CHAR1, Y_CHAR1, TFT_BLACK);
            if(rec_level_old > 10)  //erase the other +
            {
            tft.fillRect((3*X_CHAR1)+8, (2*Y_CHAR1)+14, X_CHAR1, Y_CHAR1, TFT_BLACK);
            }
          }
        }
        else   // S9+ or S9++
        {
          if(rec_level_old < 9)  //try to save some processing, if was S9, don't need to write again
          {
            tft_writexy_(2, TFT_GREEN, TFT_BLACK, 1,2,(uint8_t *)"9");
          }
          if((rec_level > 9) && (rec_level_old < 10))  //try to save some processing if already +
          {
            tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 3, 0, 3, 5, (uint8_t *)"+");
          }
          if((rec_level == 10) && (rec_level_old == 11))  //erase the second +
          {
          tft.fillRect((3*X_CHAR1)+8, (2*Y_CHAR1)+14, X_CHAR1, Y_CHAR1, TFT_BLACK);
          }
          if((rec_level == 11) && (rec_level_old < 11))  //try to save some processing if already ++
          {
            tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 3, 8, 2, 14, (uint8_t *)"+");
          }
        }
        rec_level_old = rec_level;
      }
#endif

      display_a_sample = max_a_sample;  //save the last value printed on display
      max_a_sample = 0;  //restart the search for big signal
      smeter_display_time = 0;
    }
     
    if(fft_gain_old != fft_gain)
    {
      sprintf(s, "%d  ",fft_gain);
      s[3]=0;
      tft_writexy_plus(1, TFT_GREEN, TFT_BLACK, 5, 9, 3, 5, (uint8_t *)s);   
      fft_gain_old = fft_gain;
    }       
  }


  // if menu changed, print new value

  if((hmi_menu_old != hmi_menu) || (hmi_menu_opt_display_old != hmi_menu_opt_display))
  {
#ifdef HMI_debug
  Serialx.println("evaluate Band " + String(hmi_band));
  Serialx.println("hmi_menu  " + String(hmi_menu_old) + " -> " + String(hmi_menu));
  Serialx.println("hmi_menu_opt_display " + String(hmi_menu_opt_display_old) + " -> "  + String(hmi_menu_opt_display));
#endif

  	//menu 
  	switch (hmi_menu)
  	{
  	case HMI_S_TUNE:
  		sprintf(s, "%s   %s   %s        ", hmi_o_vox[band_vars[hmi_band][HMI_S_VOX]], hmi_o_agc[band_vars[hmi_band][HMI_S_AGC]], hmi_o_pre[band_vars[hmi_band][HMI_S_PRE]]);
      tft_writexy_(1, TFT_BLUE, TFT_BLACK,0,0,(uint8_t *)s);  
      //cursor
      tft_cursor_plus(3, TFT_YELLOW, 2+(hmi_menu_opt_display>4?6:hmi_menu_opt_display), 0, 2, 20);    
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
  	case HMI_S_DFLASH:
  		sprintf(s, "Memory: %s       ", hmi_o_dflash[hmi_menu_opt_display]);
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
}
