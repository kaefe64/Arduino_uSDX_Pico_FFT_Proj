//  Based on QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Adapted by: Klaus Fensterseifer PY2KLA
//  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj



//
//      uSDX_TX.ino
//
// Arduino sketch used to test Phase and Amplitude RF transmission method running at PICO (preparing to be included at uSDR_PICO_FFT project)
// It uses the TX code from uSDX Guido's project converted to run in RPI PICO in uSDX_PICO_FFT project  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
//
// - TX code from uSDX version "1.02x"
// - same Arduino IDE and library setup as uSDX_PICO_FFT
// - same uSDX_PICO_FFT connections:
//      Input: Microphone at GP28 = MIC
//      Input: PTT TX = LOW  at GP15 = PTT
//      Output: Phase at Si5351 CLK2
//      Output: Amplitude at GP21 = I TX 
//      Output: CW Side Tone at GP22 = Audio
// - it needs the 1K pullup change on SCL SDA I2C in Si5351 board, similar to "Modifying SI 5351 Module:"  at https://antrak.org.tr/blog/usdx-a-compact-sota-ssb-sdr-transceiver-with-arduino/
// - it needs to be connected by USB to PC with the Serial Monitor running
// - it just tests the transmission, no display, no reception.
//
// Consider that there are some uSDX Guido's comments mixed with mine for PICO. 
//


#include "uSDX_I2C.h"
#include "uSDX_SI5351.h"
#include "uSDX_TX_PhaseAmpl.h"




//***********************************************************************
//
//
//***********************************************************************
void setup()
{
  gpio_init_mask(1<<LED_BUILTIN);  
  gpio_set_dir(LED_BUILTIN, GPIO_OUT); 
  
  //uSDX.h -> Serialx = Serial1   //UART0  /dev/ttyUSB0
  Serial.begin(115200);  



  uint16_t tim = millis();
 
  // some delay required for Serial1 too
  for(int i=0; i<10; i++)
  {
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  gpio_set_mask(1<<LED_BUILTIN);
  delay(50);                       // wait
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  gpio_clr_mask(1<<LED_BUILTIN);
   delay(50);                       // wait
  }
    
  while (!Serial)  //Caution!!  with Serial, if no USB-serial open, it will be stuck here
  {  //wait for PC USB-serial to open
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  gpio_set_mask(1<<LED_BUILTIN);
  delay(250);
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  gpio_clr_mask(1<<LED_BUILTIN);
  delay(250);
  }

  Serial.println("Arduino uSDX TX");
  Serial.println("\nSerial took " + String((millis() - tim)) + "ms to start");



/*
Serial
pinos
entrada MIC
entrada PTT
saida I e Q
I2C

Timer 16kHz  ou DMA o mesmo do Pico FFT?
Handler = Dsp_tx() + ADC MIC
*/
  

uSDX_TX_PhaseAmpl_setup();

}




//***********************************************************************
//
//
//***********************************************************************
void loop()
{
  
uSDX_TX_PhaseAmpl_loop();
  
}
