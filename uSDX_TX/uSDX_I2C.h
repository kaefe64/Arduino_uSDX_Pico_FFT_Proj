#ifndef __USDX_I2C_H__
#define __USDX_I2C_H__

/*
#ifdef __cplusplus
extern "C" {
#endif
*/

/*
  gpio_init_mask(1<<LED_BUILTIN);  
  gpio_pull_up(GP_ENC_A);
  gpio_set_dir(LED_BUILTIN, GPIO_OUT); 
  gpio_set_dir(GP_PTT, true);           // PTT output
  gpio_xor_mask(1<<LED_BUILTIN);
  gpio_set_mask(1<<LED_BUILTIN);
  gpio_clr_mask(1<<LED_BUILTIN);
  gpio_get(GP_ENC_B)
*/  

  #define I2C_DELAY   20  //4    // Determines I2C Speed (2=939kb/s (too fast!!); 3=822kb/s; 4=731kb/s; 5=658kb/s; 6=598kb/s). Increase this value when you get I2C tx errors (E05); decrease this value when you get a CPU overload (E01). An increment eats ~3.5% CPU load; minimum value is 3 on my QCX, resulting in 84.5% CPU load
  #define DELAY(n)   for(uint8_t i = 0; i != n; i++) asm("nop");
/*
  #define I2C_DDR    DDRC     // Pins for the I2C bit banging
  #define I2C_PIN    PINC
  #define I2C_PORT   PORTC
  #define I2C_SDA    (1 << 4) // PC4
  #define I2C_SCL    (1 << 5) // PC5
  #define I2C_SDA_GET()   I2C_PIN & I2C_SDA
  #define I2C_SCL_GET()   I2C_PIN & I2C_SCL
  #define I2C_SDA_HI()    I2C_DDR &= ~I2C_SDA;  //DDRC = 0 input
  #define I2C_SDA_LO()    I2C_DDR |=  I2C_SDA;  //DDRC = 1 output
  #define I2C_SCL_HI()    I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
  #define I2C_SCL_LO()    I2C_DDR |=  I2C_SCL; DELAY(I2C_DELAY);
*/
  #define I2C_SDA    16   // GP16
  #define I2C_SCL    17   // GP17
  #define I2C_SDA_GET()   gpio_get(I2C_SDA)
  #define I2C_SCL_GET()   gpio_get(I2C_SCL)
  #define I2C_SDA_HI()    gpio_set_dir(I2C_SDA, GPIO_IN);  DELAY(I2C_DELAY/4)  //input
  #define I2C_SDA_LO()    gpio_set_dir(I2C_SDA, GPIO_OUT); DELAY(I2C_DELAY/4)  //output
  #define I2C_SCL_HI()    gpio_set_dir(I2C_SCL, GPIO_IN);  DELAY(I2C_DELAY)  //input
  #define I2C_SCL_LO()    gpio_set_dir(I2C_SCL, GPIO_OUT); DELAY(I2C_DELAY/2)  //output


  #define SendBit(data, mask)   \
  if(data & mask){   \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();






// I2C communication starts with a START condition, multiple single byte-transfers (MSB first) followed by an ACK/NACK and stops with a STOP condition;
// during data-transfer SDA may only change when SCL is LOW, during a START/STOP condition SCL is HIGH and SDA goes DOWN for a START and UP for a STOP.
// https://www.ti.com/lit/an/slva704/slva704.pdf
//***********************************************************************
//
//
//***********************************************************************
class I2C {
public:


//***********************************************************************
  I2C();
//***********************************************************************
  ~I2C();
//***********************************************************************
  void start();
//***********************************************************************
  void stop();
//***********************************************************************


//***********************************************************************
  void SendByte(uint8_t data);
//***********************************************************************
  uint8_t RecvBit(uint8_t mask);
//***********************************************************************
  uint8_t RecvByte(uint8_t last);
//***********************************************************************
  void resume();
//***********************************************************************
  void suspend();

  void begin();
  void beginTransmission(uint8_t addr);
  bool write(uint8_t byte);
  uint8_t endTransmission();
};
























/*
#ifdef __cplusplus
}
#endif
*/
#endif
