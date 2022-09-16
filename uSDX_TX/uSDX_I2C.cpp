//  Based on QCX-SSB.ino - https://github.com/threeme3/QCX-SSB
//
//  Copyright 2019, 2020, 2021   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Adapted by: Klaus Fensterseifer PY2KLA
//  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj


#include "Arduino.h"
#include "uSDX_I2C.h"
#include "uSDX_SI5351.h"
#include "uSDX_TX_PhaseAmpl.h"







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
    gpio_clr_mask(1<<I2C_SDA);  //low
    gpio_clr_mask(1<<I2C_SCL);  //low
    I2C_SCL_HI();  //input  high on idle
    I2C_SDA_HI();  //input  high on idle
  }
//***********************************************************************
  I2C::~I2C(){
    gpio_clr_mask(1<<I2C_SDA);  //low
    gpio_clr_mask(1<<I2C_SCL);  //low
    I2C_SCL_HI();  //input  high on idle
    I2C_SDA_HI();  //input  high on idle
  }  
//***********************************************************************
  void I2C::start(){
    I2C_SDA_LO();
    DELAY(I2C_DELAY/2);
    I2C_SCL_LO();
    //I2C_SDA_HI();  //wait data to set SDA
  }
//***********************************************************************
  void I2C::stop(){
    I2C_SDA_LO();   // ensure SDA is LO so STOP-condition can be initiated by pulling SCL HI (in case of ACK it SDA was already LO, but for a delayed ACK or NACK it is not!)
    DELAY(I2C_DELAY/2);
    I2C_SCL_HI();
    I2C_SDA_HI();
    DELAY(I2C_DELAY/2);
  }
//***********************************************************************

  #define SendBit(data, mask) \
    if(data & mask){ \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();

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
    uint32_t i = 60000;  //need to be more then 60000 to be 3ms @ PICO
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
  
  
  
