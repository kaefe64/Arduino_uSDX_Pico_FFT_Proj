#ifndef __RELAY_H__
#define __RELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * relay.h
 *
 * Created: Nov 2021
 * Author: Arjan te Marvelde
 *
 * See relay.c for more information 
 */


#ifdef PY2KLA_setup
#define I2C_Arduino_Pro_Mini  1    //=1 when I2C BPF and Atten is commanded with Arduino Pro Mini (allow SWR reading)
#else
#define I2C_Arduino_Pro_Mini  0    //=0 when I2C BPF and Atten is commanded with PCF8574
#endif

/* I2C address and pins */
#define I2C_BPF				0x20
#define I2C_RX 				0x21
#define I2C_SWR 	    0x22   // read 3 bytes = SWR, FOR and REF

//#define I2C_WAIT_us   (uint64_t)500000  absolute_time_t
#define I2C_TIMEOUT_us   10000


#define REL_LPF2	0x01
#define REL_BPF6	0x02
#define REL_BPF12	0x04
#define REL_BPF24	0x08
#define REL_BPF40	0x10

#define REL_ATT_30	0x03
#define REL_ATT_20	0x01
#define REL_ATT_10	0x02
#define REL_ATT_00	0x00
#define REL_PRE_10	0x04

extern void relay_setband(uint8_t val);
extern void relay_setattn(uint8_t val);
extern int relay_getband(void);
extern int relay_getattn(void);
extern void relay_init(void);

#ifdef __cplusplus
}
#endif
#endif
