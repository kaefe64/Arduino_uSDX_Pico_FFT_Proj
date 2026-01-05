/*
 
  Eeprom.cpp
 
  Created: Dec 2025
  Author: Klaus Fensterseifer
  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
  
 



 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above author information and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.

 




Saving data to the Eeprom on Arduino Pro Mini = 1K bytes   0x0000-0x03ff
There is a a recomended limit of about 100k eeprom write cycles  
In case of 1 write by day -> 100000/365 = 273 anos

Obs.: audio_buf[] uses 160k bytes for 10s of audio -> no space to save audio on eeprom

After power up
  read all memories from Eeprom
  set the memory 1 to run (Arjan5 will use the memory 1 to start running)

when changing band/config/freq
  change the band config/freq/from the actual memory in RAM (not saved on Eeprom yet)

when changing memory with button and dial
  use the data from RAM to set the next band

Menu Save Memory 1-16
  save the actual band/config/freq to Eeprom and RAM
  change the actual memory to the one saved


Functions:
  read memory N band control freq from Eeprom
  save memory N band control freq to Eeprom


 

  External calls to read/write the DFlash:
  ========================================



*/



#include "Arduino.h"
#include "uSDR.h"
extern "C" {
  //#include <hardware/sync.h>
  //#include <hardware/flash.h>
};
#include "Eeprom.h"
#include "hmi.h"
#include "dsp.h"
//#include "pico/multicore.h"
//#include "relay.h"


#define Eeprom_debug    10

#ifdef Eeprom_debug
//defines used to print the DFlash variable to help debug
#define PRT_LN(x)      Serialx.println(x)   //print the states
#define PRT(x)         Serialx.print(x)   //print the states
#else
#define PRT_LN(x)        //do not print the states
#define PRT(x)           //do not print the states
#endif


//#define I2C_SWR 	    0x22   // read 3 bytes = SWR, FOR and REF

/* I2C EEPROM   write (2 bytes = EEP address to read  + 1 byte numbytes to read) */
/* I2C EEPROM   write [ADDR_LOW] [ADDR_HIGH] [NUM_BYTES]       NUM_BYTES must be < 28 due to I2C buffer size */
#define I2C_EEP_ADD_NB 	  0x22  // write (receive) EEP address and numbytes

/* I2C EEPROM   read numbytes bytes (EEP address from EEP_ADD_RD) */
/* I2C EEPROM   read [VALUE1] [VALUE2] ...  ... [VALUEn] */
#define I2C_EEP_RD  	  0x23  // read data on eeprom address 

/* I2C EEPROM   write 2 bytes address + 1 byte numbytes + numbytes data...  */
/* I2C EEPROM   write [ADDR_LOW] [ADDR_HIGH] [NUM_BYTES] [VALUE1] [VALUE2] ... [VALUEn]    NUM_BYTES must be < I2C_MAX_NUMBYTES due to I2C buffer size */
#define I2C_EEP_WR  	  0x23  // write data to eeprom address 


#define EEPROM_MEMORY_BASE_ADDR   0x0100u
#define I2C_TIMEOUT_us   10000
#define MEMORY_BLOCK_SIZE  (FREQ_SIZE+HMI_NMENUS+1u+1u)  // +1=fft_gain +1=chksum

/**************************************************************************************
    eep_read_band_freq - reads the band data and freq
**************************************************************************************/
void eep_read_memory(uint8_t nmem)   //read the nmem memory data
{
	static uint8_t i2c_data[MEMORY_BLOCK_SIZE];
  int16_t ret;
  uint16_t addr = EEPROM_MEMORY_BASE_ADDR + (nmem * MEMORY_BLOCK_SIZE);
  st_memory_band  mem_band;
  uint16_t gain;
  uint8_t chksum = 0;

  //[ADDR_LOW] [ADDR_HIGH] [NUM_BYTES]
  i2c_data[0] = addr & 0xff;
  i2c_data[1] = (addr>>8);
  i2c_data[2] = MEMORY_BLOCK_SIZE;

  PRT(" write addr "); PRT(addr);
  //write the addres and number of bytes to read
  ret = i2c_write_timeout_us(i2c1, I2C_EEP_ADD_NB, i2c_data, 3, false, I2C_TIMEOUT_us);
	if (ret < 0)  // true to keep master control of bus
    {
    //Serialx.println("relay_setattn  i2c_write_blocking  if <0");  // false, we're done writing
		ret = i2c_write_timeout_us(i2c1, I2C_EEP_ADD_NB, i2c_data, 3, false, I2C_TIMEOUT_us);
    } 

  if (ret < 0)  //write address gone wrong
  {
    memory_band[nmem].vars[HMI_S_BPF] = 0xff;  //empty
  }
  else
  {
    //delay(1);
    delayMicroseconds(500);

    //[VALUE1] [VALUE2] ...  ... [VALUEn] 
    //read the band data and freq
    ret = i2c_read_blocking(i2c1, I2C_EEP_RD, i2c_data, MEMORY_BLOCK_SIZE, false); 
    PRT(" esperava "); PRT(MEMORY_BLOCK_SIZE); PRT(" e recebeu ");  PRT(ret);
    if (ret==MEMORY_BLOCK_SIZE)  //number os bytes received  (<0 if no answer from I2C slave)
      {
        for(uint8_t i=0; i<FREQ_SIZE; i++)
        {
          mem_band.mem_freq.u8[i] = i2c_data[i];
          chksum += i2c_data[i];
        }
        //tune/cursor mode agc pre vox bpf
        for(uint8_t i=0; i<HMI_NMENUS; i++)
        {
          mem_band.vars[i] = i2c_data[FREQ_SIZE+i];
          chksum += i2c_data[FREQ_SIZE+i];
        }
        gain = i2c_data[FREQ_SIZE+HMI_NMENUS];
        chksum += i2c_data[FREQ_SIZE+HMI_NMENUS];

        if(chksum == i2c_data[FREQ_SIZE+HMI_NMENUS+1])
        {
          PRT(" chksum OK ");
          memory_band[nmem] = mem_band;
          fft_gain[nmem] = (gain>16?16u:gain);  //validate (I should validate all vars)
        }
        else
        {
          PRT(" chksum NOK ");
          memory_band[nmem].vars[HMI_S_BPF] = 0xff;  //signal empty to load default values
          fft_gain[nmem] = 16u; //default
        }
      }
      else
      {
        memory_band[nmem].vars[HMI_S_BPF] = 0xff;  //empty
      }
  }
}


/**************************************************************************************
    eep_read_band_freq - reads the band data and freq
**************************************************************************************/
void eep_write_memory(uint8_t nmem)   //read the nmem memory data
{
	uint8_t i2c_data[3+MEMORY_BLOCK_SIZE];  //[ADDR_LOW] [ADDR_HIGH] [NUM_BYTES] 
  int16_t ret;
  uint16_t addr = EEPROM_MEMORY_BASE_ADDR + (nmem * MEMORY_BLOCK_SIZE);
  uint8_t chksum = 0;

  PRT(" write addr "); PRT(addr);

  //[ADDR_LOW] [ADDR_HIGH] [NUM_BYTES] [VALUE1] [VALUE2] ... [VALUEn]
  i2c_data[0] = addr & 0xff;
  i2c_data[1] = (addr>>8);
  i2c_data[2] = MEMORY_BLOCK_SIZE;
  for(uint8_t i=0; i<FREQ_SIZE; i++)
  {
    i2c_data[3+i] = memory_band[nmem].mem_freq.u8[i];
    chksum += memory_band[nmem].mem_freq.u8[i];
  }
  for(uint8_t i=0; i<HMI_NMENUS; i++)
  {
    i2c_data[3+FREQ_SIZE+i] = memory_band[nmem].vars[i];
    chksum += memory_band[nmem].vars[i];
  }  
  i2c_data[3+FREQ_SIZE+HMI_NMENUS] = fft_gain[nmem];
  chksum += fft_gain[nmem];
  i2c_data[3+FREQ_SIZE+HMI_NMENUS+1] = chksum;  //tam = 3+FREQ_SIZE+HMI_NMENUS + 1 = 3+MEMORY_BLOCK_SIZE

  PRT("  size "); PRT(3+FREQ_SIZE+HMI_NMENUS+1+1);
  PRT("  chksum "); PRT(chksum);

  //write the addres and number of bytes to read
  //i2c_write_blocking_(i2c0, I2C_VFO, data, 9, false);
  ret = i2c_write_timeout_us(i2c1, I2C_EEP_WR, i2c_data, (3+MEMORY_BLOCK_SIZE), false, I2C_TIMEOUT_us);
	if (ret < 0)  // true to keep master control of bus
  {
    PRT("  ret NOT OK "); PRT(ret);
    //Serialx.println("relay_setattn  i2c_write_blocking  if <0");  // false, we're done writing
		ret = i2c_write_timeout_us(i2c1, I2C_EEP_WR, i2c_data, (3+MEMORY_BLOCK_SIZE), false, I2C_TIMEOUT_us);
  } 
  else
  {
    PRT("  ret OK "); PRT(ret);
  }
  PRT_LN("   FIM");
}



/* save actual memory band data to the selected save memory*/
void Save_Band_Eeprom(uint8_t actual_mem, uint8_t save_mem)
{
  //copy data from actual band to the save mem
  memory_band[save_mem].mem_freq.u32 = memory_band[actual_mem].mem_freq.u32;
  //tune/cursor mode agc pre vox bpf
  for(uint8_t i=0; i<HMI_NMENUS; i++)
  {
    memory_band[save_mem].vars[i] = memory_band[actual_mem].vars[i];
  }  
  //write the new mem to eeprom
  eep_write_memory(save_mem);
}


//***********************************************************************
//
//
//***********************************************************************
void Eeprom_setup(void)
{
  PRT_LN("Eeprom_setup");
  //read memories band data from arduino pro mini eeprom I2C
  PRT("Eeprom read memory ");
  for(uint8_t m=0; m<HMI_NUM_OPT_MEMORY; m++)
  {
    PRT(" "); PRT(m);  
    eep_read_memory(m);
    if (memory_band[m].vars[HMI_S_BPF] > HMI_NUM_OPT_BPF)   //empty eeprom = 0xff
    {
      PRT_LN(" empty  ");
      //tune/cursor mode agc pre vox bpf
      memory_band[m].vars[0] = 4;
      memory_band[m].vars[1] = 1;
      memory_band[m].vars[2] = 2;
      memory_band[m].vars[3] = 3;
      memory_band[m].vars[4] = 0;
      memory_band[m].vars[5] = 2;
      memory_band[m].mem_freq.u32 = band2_hmi_freq_default;
    }
    else
    {
      PRT_LN(" ok  ");
    }
    //delay(1);
    delayMicroseconds(500);  //time for Arduino Pro Mini to process
  }
  PRT_LN("FIM");

}



//***********************************************************************
//
//
//***********************************************************************
void Eeprom_loop(void)
{



}






