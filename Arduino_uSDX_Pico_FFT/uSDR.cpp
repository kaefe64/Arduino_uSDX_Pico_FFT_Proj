/*
 * uSDR.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde 
 * May2022: adapted by Klaus Fensterseifer 
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
 * The application main loop
 * This initializes the units that do the actual work, and then loops in the background. 
 * Other units are:
 * - dsp.c, containing all signal processing in RX and TX branches. This part runs on the second processor core.
 * - si5351.c, containing all controls for setting up the si5351 clock generator.
 * - lcd.c, contains all functions to put something on the LCD
 * - hmi.c, contains all functions that handle user inputs
 */

#include "Arduino.h"

#include <Wire.h>
#include "uSDR.h"
#include "hmi.h"
#include "dsp.h"
#include "si5351.h"
#include "monitor.h"
#include "relay.h"
#include "TFT_eSPI.h"
#include "display_tft.h"





uint16_t tim_loc;     // local time 

void uSDR_setup0(void)  //main
{
  display_tft_setup0();

}



void uSDR_setup(void)  //main
{
  //gpio_init_mask(1<<14);  
  //gpio_set_dir(14, GPIO_OUT); 
  

	/*
	 * i2c0 is used for the si5351 interface
	 * i2c1 is used for the LCD and all other interfaces
	 */
#if TX_METHOD == I_Q_QSE
  Wire.begin();            //i2c0 master to Si5351
  //Wire.setClock(200000);   // Set i2c0 clock speed (default=100k)
#endif
  Wire1.begin();           //i2c1

  
	/* Initialize units */
	mon_init();										// Monitor shell on stdio
	si_init();										// VFO control unit
	relay_init();
  display_tft_setup();   //moved to setup0 to write into display from the beggining
	hmi_init();										// HMI user inputs
  dsp_init();                   // Signal processing unit

  tim_loc = tim_count;          // local time for main loop
  //digitalWrite(14, LOW);
}




#define LOOP_MS    100u  //miliseconds

void uSDR_loop(void)
{ 

  if((uint16_t)(tim_count - tim_loc) >= LOOP_MS)  //run the tasks every 100ms 
  {
    hmi_evaluate();               // Refresh HMI
    si_evaluate();                // Refresh VFO settings
    mon_evaluate();               // Check monitor input
    dsp_loop();  //spend more time here for FFT and graphic
    display_tft_loop();           // Refresh display
    //it takes 50ms for the tasks (most in hmi_evaluate() to plot the waterfall)
    
    tim_loc += LOOP_MS;
  }

}
