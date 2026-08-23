#ifndef __uSDR_H__
#define __uSDR_H__

#ifdef __cplusplus
extern "C" {
#endif


//#define PY2KLA_setup     1       //setup for PY2KLA hardware   (comment this line for default setup)
#define SW_VERSION       "Aug23 2026"    //software version

//choose the serial to be used (look at "pins_arduino.h" of the board core and comments at .ino file)
#define Serialx   Serial     //USB virtual serial CDC (on Pico, Earle Philhower core)  /dev/ttyACM0
//#define Serialx   Serial1  //UART0 (with external  USB/serial converter) /dev/ttyUSB0

#define LOOP_MS    100  //100 miliseconds


void uSDR_setup0(void);
void uSDR_setup(void);
void uSDR_loop(void);




#ifdef __cplusplus
}
#endif
#endif
