//  Based on QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Adapted by: Klaus Fensterseifer PY2KLA
//  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj


#include "Arduino.h"
#include "uSDX_TX_PhaseAmpl.h"
#include "uSDX_I2C.h"
#include "uSDX_SI5351.h"




#define RS_HIGH_ON_IDLE   1   // Experimental LCD support where RS line is high on idle periods to comply with SDA I2C standard.



// I2C communication starts with a START condition, multiple single byte-transfers (MSB first) followed by an ACK/NACK and stops with a STOP condition;
// during data-transfer SDA may only change when SCL is LOW, during a START/STOP condition SCL is HIGH and SDA goes DOWN for a START and UP for a STOP.
// https://www.ti.com/lit/an/slva704/slva704.pdf
//***********************************************************************
//
//  class I2C
//
//***********************************************************************


//***********************************************************************
  I2C::I2C(){
    gpio_init_mask(1<<I2C_SDA);
    gpio_init_mask(1<<I2C_SCL);
    //I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    gpio_clr_mask(1<<I2C_SDA);  //low
    gpio_clr_mask(1<<I2C_SCL);  //low
    I2C_SCL_HI();  //input
    I2C_SDA_HI();  //input
#ifdef RS_HIGH_ON_IDLE
#else
    suspend();
#endif
  }
//***********************************************************************
  I2C::~I2C(){
    //I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    gpio_clr_mask(1<<I2C_SDA);  //low
    gpio_clr_mask(1<<I2C_SCL);  //low
    //I2C_DDR &= ~( I2C_SDA | I2C_SCL );  // 0 = input
    I2C_SCL_HI();  //input
    I2C_SDA_HI();  //input
  }  
//***********************************************************************
  void I2C::start(){
#ifdef RS_HIGH_ON_IDLE
    I2C_SDA_LO();
    DELAY(I2C_DELAY/2);
#else
    resume();  //prepare for I2C
#endif
    I2C_SCL_LO();
    //I2C_SDA_HI();
  }
//***********************************************************************
  void I2C::stop(){
    //DELAY(I2C_DELAY);
    I2C_SDA_LO();   // ensure SDA is LO so STOP-condition can be initiated by pulling SCL HI (in case of ACK it SDA was already LO, but for a delayed ACK or NACK it is not!)
    DELAY(I2C_DELAY/2);
    I2C_SCL_HI();
    I2C_SDA_HI();
    DELAY(I2C_DELAY/2);
    //I2C_SDA_HI();
    //I2C_DDR &= ~(I2C_SDA | I2C_SCL); // prepare for a start: pull-up both SDA, SCL
    //I2C_SCL_HI();  //input
    //I2C_SDA_HI();  //input
#ifdef RS_HIGH_ON_IDLE
#else
    suspend();
#endif
  }
//***********************************************************************
/*
  #define I2C::SendBit(data, mask) \
    if(data & mask){ \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();
*/
//***********************************************************************
  void I2C::SendByte(uint8_t data){
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI();  // recv ACK
    DELAY(I2C_DELAY/2);
    I2C_SCL_HI();
    I2C_SCL_LO();
    DELAY(I2C_DELAY/2);
  }
//***********************************************************************
  uint8_t I2C::RecvBit(uint8_t mask){
    I2C_SCL_HI();
    uint32_t i = 60000;  //60000;
    for(;!(I2C_SCL_GET()) && i; i--);  // wait util slave release SCL to HIGH (meaning data valid), or timeout at 3ms
    if(!i){ Serial.println("E07 I2C timeout"); }
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return (data) ? mask : 0;
  }
//***********************************************************************
  uint8_t I2C::RecvByte(uint8_t last){
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
    if(last){
      I2C_SDA_HI();  // NACK
    } else {
      I2C_SDA_LO();  // ACK
    }
    DELAY(I2C_DELAY/2);
    I2C_SCL_HI();
    I2C_SDA_HI();    // restore SDA for read
    I2C_SCL_LO();
    return data;
  }
//***********************************************************************
  void I2C::resume(){
//#ifdef LCD_RS_PORTIO
//    I2C_PORT &= ~I2C_SDA; // pin sharing SDA/LCD_RS mitigation
//#endif
  }
//***********************************************************************
  void I2C::suspend(){
    I2C_SDA_LO();         // pin sharing SDA/LCD_RS: pull-down LCD_RS; QCXLiquidCrystal require this for any operation
  }

  void I2C::begin(){};
  void I2C::beginTransmission(uint8_t addr){ start(); SendByte(addr << 1);  };
  bool I2C::write(uint8_t byte){ SendByte(byte); return 1; };
  uint8_t I2C::endTransmission(){ stop(); return 0; };








#if 0



//#define _I2C_DIRECT_IO    1 // Enables communications that is not using the standard I/O pull-down approach with pull-up resistors, instead I/O is directly driven with 0V/5V
class I2C_ { // Secundairy I2C class used by I2C LCD/OLED, uses alternate pins: PD2 (SDA) and PD3 (SCL)
public:
#if(F_MCU > 20900000)
  #ifdef OLED_SH1106
    #define _DELAY() for(uint8_t i = 0; i != 9; i++) asm("nop");
  #else
  #ifdef OLED_SSD1306
    #define _DELAY() for(uint8_t i = 0; i != 6; i++) asm("nop");
  #else // other (I2C_LCD)
    #define _DELAY() for(uint8_t i = 0; i != 7; i++) asm("nop");
  #endif
  #endif
#else // slow F_MCU
  #ifdef OLED_SH1106
    #define _DELAY() for(uint8_t i = 0; i != 8; i++) asm("nop");
  #else
  #ifdef OLED_SSD1306
    #define _DELAY() for(uint8_t i = 0; i != 4; i++) asm("nop"); // 4=731kb/s
  #else // other (I2C_LCD)
    #define _DELAY() for(uint8_t i = 0; i != 5; i++) asm("nop");
  #endif
  #endif
#endif // F_MCU
  #define _I2C_SDA (1<<2) // PD2
  #define _I2C_SCL (1<<3) // PD3
#ifdef _I2C_DIRECT_IO
  #define _I2C_INIT() _I2C_SDA_HI(); _I2C_SCL_HI(); DDRD |= (_I2C_SDA | _I2C_SCL);  // direct I/O (no need for pull-ups)
  #define _I2C_SDA_HI() PORTD |=  _I2C_SDA;
  #define _I2C_SDA_LO() PORTD &= ~_I2C_SDA;
  #define _I2C_SCL_HI() PORTD |=  _I2C_SCL; _DELAY();
  #define _I2C_SCL_LO() PORTD &= ~_I2C_SCL; _DELAY();
#else // !_I2C_DIRECT_IO
  #define _I2C_INIT()   PORTD &= ~_I2C_SDA; PORTD &= ~_I2C_SCL; _I2C_SDA_HI(); _I2C_SCL_HI();  // open-drain
  #define _I2C_SDA_HI() DDRD &= ~_I2C_SDA;
  #define _I2C_SDA_LO() DDRD |=  _I2C_SDA;
  #define _I2C_SCL_HI() DDRD &= ~_I2C_SCL; _DELAY();
  #define _I2C_SCL_LO() DDRD |=  _I2C_SCL; _DELAY();
#endif // !_I2C_DIRECT_IO
  #define _I2C_START() _I2C_SDA_LO(); _DELAY(); _I2C_SCL_LO(); // _I2C_SDA_HI();
  #define _I2C_STOP()  _I2C_SDA_LO(); _I2C_SCL_HI(); _I2C_SDA_HI();
  #define _I2C_SUSPEND() //_I2C_SDA_LO(); // SDA_LO to allow re-use as output port
  #define _SendBit(data, bit) \
    if(data & 1 << bit){ \
      _I2C_SDA_HI();  \
    } else {         \
      _I2C_SDA_LO();  \
    }                \
    _I2C_SCL_HI();    \
    _I2C_SCL_LO();
  inline void start(){ _I2C_INIT(); _I2C_START(); };
  inline void stop() { _I2C_STOP(); _I2C_SUSPEND(); };
  inline void SendByte(uint8_t data){
    _SendBit(data, 7);
    _SendBit(data, 6);
    _SendBit(data, 5);
    _SendBit(data, 4);
    _SendBit(data, 3);
    _SendBit(data, 2);
    _SendBit(data, 1);
    _SendBit(data, 0);
    _I2C_SDA_HI();  // recv ACK
    _DELAY(); //
    _I2C_SCL_HI();
    _I2C_SCL_LO();
  }
  void SendRegister(uint8_t addr, uint8_t* data, uint8_t n){
    start();
    SendByte(addr << 1);
    while(n--) SendByte(*data++);
    stop();
  }
  //void SendRegister(uint8_t addr, uint8_t val){ SendRegister(addr, &val, 1); }

  void begin(){};
  void beginTransmission(uint8_t addr){ start(); SendByte(addr << 1);  };
  bool write(uint8_t byte){ SendByte(byte); return 1; };
  uint8_t endTransmission(){ stop(); return 0; };
};
I2C_ Wire;



#endif
