
/*
 * Arduino_uSDX_Pico.ino
 * uSDX_PICO running in Raspberry Pi Pico RP2040 with TFT Display compiled in Arduino IDE
 * 
 * Created: May 2022
 * Author: Klaus Fensterseifer
 * https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
 * 
 * 


>>Use  Boards Manager: Arduino Mbed OS RP2040 Boards
>>Do not use EarlePhilhower library (it is just conflitant with Mbed)

>>Lib used: TFT_eSPI by Bodmer


>>On Ubuntu, to allow program Pico direct from Arduino IDE, run once:
~/.arduino15/packages/arduino/hardware/mbed_rp2040/4.0.2$ sudo ./post_install.sh
>>Obs.: Compiled file  Arduino_uSDX_Pico_FFT.ino.uf2  generated at  /tmp/arduino-sketch-...


 
>>Mods in the Library files to fit to the project:
================================================


--------------------------------------------------------------
--------------------------------------------------------------
>>TFT_eSPI LIBRARY:
----------------------------------------------------
>>Change Arduino/libraries/TFT_eSPI/User_Setup_Select.h
>>Comment:
//#include <User_Setup.h>           // Default setup is root library folder
>>UnComment:
#include <User_Setups/Setup60_RP2040_ILI9341.h>    // Setup file for RP2040 with SPI ILI9341

----------------------------------------------------
>>On User_Setups/Setup60_RP2040_ILI9341.h
>>Uncomment:
#define ILI9341_DRIVER
#define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue
>>Choose the SPI pins (SPI1):
// For the Pico use these #define lines
#define TFT_MISO  12  //0  RX
#define TFT_MOSI  11  //3  TX
#define TFT_SCLK  10  //2
#define TFT_CS   13  //20  // Chip select control pin
#define TFT_DC   4  //18  // Data Command control pin
#define TFT_RST  5  //19  // Reset pin (could connect to Arduino RESET pin)
//#define TFT_BL     // LED back-light
//#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen

>>Leave the fonts available:
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

>>Choose SPI 1
#define TFT_SPI_PORT 1   // Set to 0 if SPI0 pins are used, or 1 if SPI1 pins used



--------------------------------------------------------------
>>In case your ILI9341 looks 90 degree view, or partially 90 degree view (as mine):
>>For ILI9341 + RP2040,  try change  TFT_eSPI/TFT_Drivers/ILI9341_Defines.h
#define TFT_WIDTH  320  //240
#define TFT_HEIGHT 240  //320
>>(this was reported as a issue to TFT_eSPI github project: https://github.com/Bodmer/TFT_eSPI/issues/1725)
>>(there are also options to set:  #if defined (ILI9341_DRIVER) || defined (ILI9341_2_DRIVER))
>>In combination, you can try to change display_tft.h  #define ROTATION_SETUP  0    to 1


--------------------------------------------------------------
--------------------------------------------------------------
>>The following mods will correct this beginner usual error:
>>Compilation error: 'Wire1' was not declared in this scope

>>For Wire/I2C, look the pins at  
.arduino15/packages/arduino/hardware/mbed_rp2040/4.0.2/variants/RASPBERRY_PI_PICO/pins_arduino.h
>>(.arduino15 is a hidden directory, set Windows/Linux to show hidden files/directories)
>>change the pins for I2C0 and include the pins for I2C1:
// Wire
#define PIN_WIRE_SDA        (16u)  //I2C0
#define PIN_WIRE_SCL        (17u)  //I2C0
#define PIN_WIRE_SDA1       (18u)  //included I2C1
#define PIN_WIRE_SCL1       (19u)  //included I2C1

#define WIRE_HOWMANY    (2)  //included I2C1   default was 1
#define I2C_SDA       (digitalPinToPinName(PIN_WIRE_SDA))   //I2C0
#define I2C_SCL       (digitalPinToPinName(PIN_WIRE_SCL))   //I2C0
#define I2C_SDA1      (digitalPinToPinName(PIN_WIRE_SDA1))  //included I2C1
#define I2C_SCL1      (digitalPinToPinName(PIN_WIRE_SCL1))  //included I2C1


--------------------------------------------------------------
>>Check on hmi.cpp:
#define HMI_MULFREQ          1      // Factor between HMI and actual frequency
                                    // Set to 1, 2 or 4 for certain types of mixer

--------------------------------------------------------------
>>Check on Si5351.cpp to set the correct Si5351 internal frequency:
#define SI_XTAL_FREQ  25001414UL  // Replace with measured crystal frequency of XTAL for CL = 10pF (default)


--------------------------------------------------------------
>>Check ENCODER selection at hmi.cpp (change if necessary)
#define ENCODER_TYPE             ENCODER_FALL      //choose what encoder is used
#define ENCODER_DIRECTION        ENCODER_CW_A_FALL_B_HIGH  //direction related to B signal level

--------------------------------------------------------------
>>Check the correct RX I and Q inputs at dsp.h  
#define EXCHANGE_I_Q  1    //include or comment this #define in case the LSB/USB and the lower/upper frequency of waterfall display are reverted - hardware dependent

--------------------------------------------------------------
>>Choose one TX method at uSDR.h
#define TX_METHOD    I_Q_QSE            // uSDR_Pico original project generating I and Q signal to a QSE mixer
//#define TX_METHOD    PHASE_AMPLITUDE    // DO NOT USE - is not ready - used for Class E RF amplifier - see description at: uSDX_TX_PhaseAmpl.cpp

--------------------------------------------------------------
>> Set if using Arduino Pro Mini instead of PCF8574 to control the filter relays at relay.h
#define I2C_Arduino_Pro_Mini  1    //=1 when I2C BPF and Atten is commanded with Arduino Pro Mini (allow SWR reading)

--------------------------------------------------------------
>> I made a #define PY2KLA_setup 1 at uSDR.h  to set my configuration on other files  (you can search for PY2KLA_setup on all files to look for configurations)
Comment out this #define and set your own configuration


>> Have fun
>> PY2KLA/PY4LL KLaus F.


*/


#include "uSDR.h"
#include "hmi.h"





//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void setup() {

  //RP2040 initialize the GPIOs as inputs with pulldown as default, and this is like PTT active 
  //it needs a strong pullup = 1K to 3v3 on pin GPIO15 (pin 20) to force high level during initialization
  gpio_pull_up(GP_PTT);             // PTT pullup  (it takes about 1s to reach this point after power up / reset)
  gpio_set_dir(GP_PTT, GPIO_IN);    // PTT input (just to confirm) - true for out, false for in 


  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  gpio_init_mask(1<<LED_BUILTIN);  
  gpio_set_dir(LED_BUILTIN, GPIO_OUT); 

  
  //uSDX.h -> Serialx = Serial1   //UART0  /dev/ttyUSB0
  //if you choose Serialx = Serial on uSDR.h - it will use Pico's USB and save the use of USB to serial converter and leave 2 spare pins
  Serialx.begin(115200);  

  uint16_t tim = millis();

  //special jobs while waiting initial display print
  uSDR_setup0();  //write something into display while waiting for the serial and DFLASH read
  hmi_init0();     //it could take some time to read all DFLASH data

  // some delay required for Serial to open
   while((millis() - tim) < 5000)   //try for 5s to connect to serial
  {
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  gpio_set_mask(1<<LED_BUILTIN);
  delay(50);                       // wait
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  gpio_clr_mask(1<<LED_BUILTIN);
  delay(50);                       // wait
  
  //if(Serial)  //serial open
  //  break;
  }  // If the serial is not open on 5s, it goes ahead and the serial print commands will be called but with no effect
  //fixed time for initial display - if the serial is not ok - consider no serial

//  Serialx.println("\n\n***  ARJAN-5  ***");
//  Serialx.println("\nArduino uSDX Pico FFT");
//  Serialx.println("\nSerial took " + String((millis() - tim)) + "ms to start");

  uSDR_setup();
}



//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
void loop(void)
{

  uSDR_loop();
//gpio_xor_mask(1<<LED_BUILTIN);
}


 
