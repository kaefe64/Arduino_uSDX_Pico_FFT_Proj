#ifndef __uSDR_H__
#define __uSDR_H__

#ifdef __cplusplus
extern "C" {
#endif


//#define PY2KLA_setup     1       //setup for PY2KLA hardware   (comment this line for default setup)
#define SW_VERSION    "Dec25"    //software version

//choose the serial to be used (names come from MBed library, look at "pins_arduino.h" and comments at .ino file)
//#define Serialx   SerialUSB    //USB virtual serial  /dev/ttyACM0
#define Serialx   Serial1   //UART0  /dev/ttyUSB0

#define LOOP_MS    100  //100 miliseconds


void uSDR_setup0(void);
void uSDR_setup(void);
void uSDR_loop(void);




#ifdef __cplusplus
}
#endif
#endif
