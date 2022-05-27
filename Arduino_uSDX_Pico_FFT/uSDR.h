#ifndef __uSDR_H__
#define __uSDR_H__

#ifdef __cplusplus
extern "C" {
#endif




//choose the serial to be used
//#define Serialx   Serial    //USB virtual serial  /dev/ttyACM0
#define Serialx   Serial1   //UART0  /dev/ttyUSB0


void uSDR_setup(void);
void uSDR_loop(void);




#ifdef __cplusplus
}
#endif
#endif
