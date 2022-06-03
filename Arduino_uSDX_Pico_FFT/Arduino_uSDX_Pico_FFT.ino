
/*
 * Arduino_uSDX_Pico.ino
 * uSDX_PICO running in Raspberry Pi Pico RP2040 with TFT Display compiled in Arduino IDE
 * 
 * Created: May 2022
 * Author: Klaus Fensterseifer
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj) 
 * 
 * 


Use  Boards Manager: Arduino Mbed OS RP2040 Boards
Do not use EarlePhilhower library (it is just conflitant with Mbed)

Lib used: TFT_eSPI by Bodmer


On Ubuntu, to allow program Pico direct from Arduino IDE, run once:
~/.arduino15/packages/arduino/hardware/mbed_rp2040/3.0.1$ sudo ./post_install.sh
File  ...ini.elf.uf2  generated at  /tmp/arduino_build_...


 
Mods in the Library files to fit to the project:
================================================


TFT_eSPI LIBRARY:
----------------------------------------------------
Change Arduino/libraries/TFT_eSPI/User_Setup_Select.h
Comment:
//#include <User_Setup.h>           // Default setup is root library folder
UnComment:
#include <User_Setups/Setup60_RP2040_ILI9341.h>    // Setup file for RP2040 with SPI ILI9341

----------------------------------------------------
On User_Setups/Setup60_RP2040_ILI9341.h
Uncomment:
#define ILI9341_DRIVER
#define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue
Choose the SPI pins (SPI1):
// For the Pico use these #define lines
#define TFT_SPI_PORT 1   // 0=SPI  1=SPI1
#define TFT_MISO  12  //0  RX
#define TFT_MOSI  11  //3  TX
#define TFT_SCLK  10  //2
#define TFT_CS   13  //20  // Chip select control pin
#define TFT_DC   4  //18  // Data Command control pin
#define TFT_RST  5  //19  // Reset pin (could connect to Arduino RESET pin)
//#define TFT_BL     // LED back-light
//#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen

Leave the fonts available:
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

// Comment out the #define below to stop the SPIFFS filing system and smooth font code being loaded
// this will save ~20kbytes of FLASH
#define SMOOTH_FONT

--------------------------------------------------------------
For ILI9341 + RP2040,  change  TFT_eSPI/TFT_Drivers/ILI9341_Defines.h
#define TFT_WIDTH  320  //240
#define TFT_HEIGHT 240  //320
(this was reported as a issue to TFT_eSPI github project, and can be solved on new releases, just put it back if your display looks turned 90 degrees)

--------------------------------------------------------------
For Wire/I2C (thanks to Bob W9RAN for corrections)
look pins at  pins_arduino.h
.arduino15/packages/arduino/hardware/mbed_rp2040/3.0.1/variants/RASPBERRY_PI_PICO
change the pins for I2C0 and include the pins for I2C1:
// Wire
#define PIN_WIRE_SDA        (16u)
#define PIN_WIRE_SCL        (17u)
#define PIN_WIRE_SDA1       (18u)
#define PIN_WIRE_SCL1       (19u)

#define WIRE_HOWMANY    (2)  //default was 1
#define I2C_SDA            (digitalPinToPinName(PIN_WIRE_SDA))
#define I2C_SCL            (digitalPinToPinName(PIN_WIRE_SCL))
#define I2C_SDA1            (digitalPinToPinName(PIN_WIRE_SDA))
#define I2C_SCL1            (digitalPinToPinName(PIN_WIRE_SCL))
*/

#include "uSDR.h"





//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  gpio_init_mask(1<<LED_BUILTIN);  
  gpio_set_dir(LED_BUILTIN, GPIO_OUT); 

  
  //uSDX.h -> Serialx = Serial1   //UART0  /dev/ttyUSB0
  Serialx.begin(115200);  


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
    
  while (!Serialx)  //Caution!!  with Serial, if no USB-serial open, it will be stuck here
  {  //wait for PC USB-serial to open
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  gpio_set_mask(1<<LED_BUILTIN);
  delay(250);
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  gpio_clr_mask(1<<LED_BUILTIN);
  delay(250);
  }

  Serialx.println("Arduino uSDX Pico FFT");
  Serialx.println("\nSerial took " + String((millis() - tim)) + "ms to start");




//  analogReadResolution(12);
//  analogWriteResolution(12);
//  analogWrite(DAC0, 0);
//  analogWrite(DAC1, 0);


//  display_setup();
//  adc_fft_setup();



  uSDR_setup();

}



//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void loop(void)
{

  uSDR_loop();
//gpio_xor_mask(1<<LED_BUILTIN);
}


 
