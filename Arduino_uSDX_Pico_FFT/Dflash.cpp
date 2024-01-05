/*
 
  display.cpp
 
  Created: Ago 2022
  Author: Klaus Fensterseifer
  https://github.com/kaefe64/Arduino_uSDX_Pico_FFT_Proj
  
  
  Based on information from:
  https://www.makermatrix.com/blog/read-and-write-data-with-the-pi-pico-onboard-flash/
  

  https://forums.raspberrypi.com/viewtopic.php?t=311709
  https://forums.raspberrypi.com/viewtopic.php?t=311633
  https://github.com/raspberrypi/pico-examples/blob/master/flash/program/flash_program.c


  Data Flash Description
  ======================

  Two functions will be available to read/write a block of DATA_BLOCK_SIZE bytes of data (you can write less then DATA_BLOCK_SIZE bytes)
  This will be your area for storing data = DATA_BLOCK_SIZE bytes

  Using DATA_BLOCK_SIZE = 16
  To deal with the RPI Pico FLASH memory it will use some strategy (it will be transparent ot the user, the user only reads and writes the block of data):
  It will use the last sector of FLASH mem to write/read the non volatile data  (sector = 4096 bytes = minimum amount you can erase)
  It will write 4096/16 = 256 blocks in the sector,   256/16 = 16 blocks per page  (page = 256 bytes = minimum amount you can write)
  When writing a block, it will rewrite over the blocks already written with the same data (without erasing it first), 
                        and writing the new block on sequence on a blank area
  So it writes 128 blocks before the sector is full. After this the sector will need to be all erased, and the process restart.
  It is expected to have <20K number of flash sector erases before we can have errors (could take more), so  20K x 256 = 5120K writes
  With 1 write per day, it will last 5120K / 365 = 14 years  (only!!!, the option would be increase the FLASH area using more sectors - for now, save writings...)
  If after 14 years you are getting mem errors, you can try change the code "define" above to use another sector:
  #define FLASH_TARGET_OFFSET   (PICO_FLASH_SIZE_BYTES - (2 * FLASH_SECTOR_SIZE))  // to start address in the second last sector on FLASH mem


  There are two functions in the Pi Pico SDK used to write into the flash:

  flash_range_erase(uint32_t flash_offs, size_t count);
    erase (resets to 0xFF) count bytes of flash beginning at address flash_offs.
    count must be a multiple of the FLASH_SECTOR_SIZE (4096).

  flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
    program (sets bits to zero) one or more 256-byte pages (stored in *data) to the count bytes beginning at address flash_offs. 
    count must be a multiple of FLASH_PAGE_SIZE (256).



  External calls to read/write the DFlash:
  ========================================




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

 
*/



#include "Arduino.h"
#include "uSDR.h"
extern "C" {
  #include <hardware/sync.h>
  #include <hardware/flash.h>
};
#include "Dflash.h"
#include "hmi.h"
#include "pico/multicore.h"


//#define DFLASH_debug    10

#ifdef DFLASH_debug
//defines used to print the DFlash variable to help debug
#define PRT_LN(x)      Serialx.println(x)   //print the states
#define PRT(x)         Serialx.print(x)   //print the states
#else
#define PRT_LN(x)        //do not print the states
#define PRT(x)           //do not print the states
#endif


// 10 years in microseconds
//#define MULTICORE_LOCKOUT_TIMEOUT_us    (uint64_t)(10 * 365 * 24 * 60 * 60 * 1000 * 1000)
// 10s
#define MULTICORE_LOCKOUT_TIMEOUT_us    (uint64_t)(10 * 1000 * 1000)




/*
bool  Dflash_empty = false;
uint16_t npage;   //actual page
uint16_t nblock;  //last data flash block
*/


int16_t last_block;  //last block with data on the DFLASH - int to use value -1 as DFLASH empty
uint16_t DFLASH_in_use = 0;  //used to stop core1 for wirting on DFLASH memory



//***********************************************************************
//
// Read the block num from the data flash memory
//
//***********************************************************************
uint16_t Dflash_read_block(uint16_t block_num, uint8_t *data_bl, uint16_t data_siz)
{
  uint16_t ndata;
  uint32_t addr;
  uint8_t  data_block[DATA_BLOCK_SIZE];
  uint8_t chksum;
  uint8_t count_FFs = 0;  
  uint32_t freq;    

  if(data_siz > DATA_BLOCK_SIZE-1)  // -1 to leave 1 byte for chksum
    {
    PRT_LN("error read data_siz > DATA_BLOCK_SIZE");
    //return false;
    data_siz = DATA_BLOCK_SIZE-1;
    }


// The ARM cores have the entire address space of the flash memory-mapped.
// Read the flash value directly as if it were in RAM
// The RAM is included in the address space (RAM space = XIP_BASE)


  addr = DFLASH_ADDR_READ(block_num * DATA_BLOCK_SIZE);
  PRT_LN("Reading   nblock = " + String(block_num) + "     addr = " + String((block_num * DATA_BLOCK_SIZE)) + "   addr in page = " + String(addr));
  chksum = 0;
  for(ndata = 0; ndata < DATA_BLOCK_SIZE; ndata++)
    {   
      data_block[ndata] = *((const uint8_t *)(addr + ndata));  
      PRT(" " + String(data_block[ndata]));  
      if(ndata < data_siz)    
        {
        chksum += data_block[ndata]; 
        }      
      if(data_block[ndata] == 0xff)    
        {
          count_FFs++;
        }                
    }
  PRT_LN(" ");

  if(count_FFs == DATA_BLOCK_SIZE)  // (data_block[0] == 0xFF)    //found empty block
    {
    PRT_LN("--- found empty block");
    return 0;
    }
  else if(chksum != *((const uint8_t *)(addr + data_siz)))    //chksum on block next byte
    {
    PRT_LN("--- found block with wrong chksum");
    return 2;
    }
  else
    {
    for(ndata = 0; ndata < data_siz; ndata++)
      {   
      data_bl[ndata] = data_block[ndata];  
      }  
    data_bl[ndata] = chksum; 

#ifdef DFLASH_debug
      freq = data_bl[HMI_NMENUS_DFLASH+0];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+1];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+2];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+3];
    PRT_LN("--- found block ok    band = " + String(data_bl[HMI_S_BPF]) + "   freq = " + String(freq));
#endif
    
    return 1;   //block ok
    }
}





//***********************************************************************
//
// Init routine - Load HMI data from DFLASH
// 
// Load default values to band variables
// Read one block
//    if empty, use the default values or the ones already read, stop
//    if there is valid data, put in the correct band variable
//    repeat on next block
// Last block read will be the actual
// 
//***********************************************************************
void Init_HMI_data(uint8_t *actual_bnd)
{
  uint8_t   data_block[DATA_BLOCK_SIZE];
  uint16_t  i, j, data_index;
  uint8_t   last_band;
  uint16_t ret_read;
  uint16_t count_block = 0;

  //band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE];
  // read all block saving the last data for each band
  for(i=0; i < MAX_NBLOCK; i++)
    {
    ret_read = Dflash_read_block(i, data_block, BAND_VARS_SIZE_DFLASH);     
    if(ret_read == 1)  //block ok
      {
        last_block = i;      //keep the last_block index to use on next writing to the DFLASH     
        count_block++;
        //Serialx.print("\nRead block from DFLASH = OK   ");
        // the last band data read is the newest data and it will be in the vars for use in menu to switch bands
        last_band = data_block[HMI_S_BPF];    //last band read
        for(j = 0; j < HMI_NMENUS_DFLASH; j++)
          {
          band_vars[last_band][j] = data_block[j];  //put the data on the right band position to use in menus
          //Serialx.print(" " + String(band_vars[hmi_band][ndata]));
          }
        for(; j < HMI_NMENUS; j++)
          {
          band_vars[last_band][j] = 0;  //fill not saved menu with zeros
          //Serialx.print(" " + String(band_vars[hmi_band][ndata]));
          } 
        for(j = 0; j < 4; j++)
          {    
          band_vars[last_band][HMI_NMENUS+j] = data_block[HMI_NMENUS_DFLASH+j];  //copy freq saved
          //Serialx.print(" " + String(band_vars[hmi_band][ndata]));
          }
        //Serialx.println("\n");
      }
    else if(ret_read == 0) //empty block found = end of data on DFLASH found
      {
        if(count_block>0)
        {
          *actual_bnd = last_band;  //last band saved will be the actual band
          //Serialx.print(" " + String(band_vars[hmi_band][ndata]));
        }
        else
        {
          *actual_bnd = 2;  //no data in DFLASH, use default band vars
          last_block = i-1;   //  -1 means empty DFLASH
        }
        //Serialx.println("\nRead menu configuration from DFLASH = NOT OK    Using Default Values");
        break;        //stop reading fromDFLASH
      }
    else if(ret_read == 2) //wrong chksum
      {
        // ignores the block
        // does not stop to search for empty blcok        
      }
    }
  PRT_LN("INIT ok   last_block = " + String(last_block) + "   actual_bnd = " + String(*actual_bnd));

      //calculate the page for last_block+1
  uint16_t npage = (last_block+1) / MAX_NBLOCK_IN_PAGE;

      //calculte the block+1 position inside of the page
  uint16_t block_in_page = last_block - (npage * MAX_NBLOCK_IN_PAGE);
  
  PRT_LN("          npage = " + String(npage) + "   last block inpage = " + String(block_in_page));
}


//***********************************************************************
//
// If change the band, use the next band var as actual
//
//
//***********************************************************************



//***********************************************************************
//
//
//
//
//***********************************************************************
void Dflash_erase_sector(void)
{
      //erase all sector
      PRT_LN("DFLASH sector full -> start erase");

      DFLASH_in_use = 1;  //signal to core1 to stop interrupts while writing to the DFLASH
      sleep_us(500);  // 1/16kHz = 62.5us

      const bool locked = multicore_lockout_start_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us); //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to lockout = pause

      if (locked) 
      {
        PRT_LN("multicore_lock"); 

        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(DFLASH_ADDR_ERASE, FLASH_SECTOR_SIZE);  //size Must be a multiple of 4096 bytes (one sector).
        restore_interrupts (ints);
        bool unlocked;

        //do {
          unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
        //} while(!unlocked);
        
        if(!unlocked)      
        {
          PRT_LN("multicore_lock  NOT unlock");         
          unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
          if(!unlocked)      
          {
            PRT_LN("multicore_lock  NOT unlock");         
            unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
          }
        }
        else
        {
          PRT_LN("multicore_lock   unlock OK");
        }

      }
      else
      {
        PRT_LN("multicore NOT lock");     
      }

      PRT_LN("DFLASH sector full -> finish erase");

}




//***********************************************************************
//
//
//
//
//***********************************************************************
void Dflash_write_page(uint16_t npage_wr, uint8_t *pg_wr)
{
  uint32_t page_addr_write;

    page_addr_write = DFLASH_ADDR_WRITE(npage_wr);

    PRT_LN("Write  npage_wr = " + String(npage_wr) + "   last_block+1 = " + String(last_block+1) + "   page_addr_write = " + String(page_addr_write));

    
    PRT_LN("OverWrite writing start");


    DFLASH_in_use = 1;  //signal to core1 to stop interrupts while writing to the DFLASH
    sleep_us(500);  // 1/16kHz = 62.5us

    const bool locked = multicore_lockout_start_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us); //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to lockout = pause

    if (locked) 
    {
      PRT_LN("multicore_lock"); 

      uint32_t interrupts = save_and_disable_interrupts();
      flash_range_program(page_addr_write, pg_wr, FLASH_PAGE_SIZE);  
      restore_interrupts(interrupts);
      bool unlocked;

      //do {
        unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
      //} while(!unlocked);

      if(!unlocked)      
      {
        PRT_LN("multicore_lock  NOT unlock");         
        unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
        if(!unlocked)      
        {
          PRT_LN("multicore_lock  NOT unlock");         
          unlocked = multicore_lockout_end_timeout_us(MULTICORE_LOCKOUT_TIMEOUT_us);  //wait MULTICORE_LOCKOUT_TIMEOUT_us for Core1 to release from lockout
        }
      }
      else
      {
        PRT_LN("multicore_lock   unlock OK");
      }

    }
    else
    {
      PRT_LN("multicore NOT lock");     
    }

    DFLASH_in_use = 0;  //release core1 after writing to the DFLASH

    PRT_LN("OverWrite writing end"); 
}





//***********************************************************************
//
// Writes block to the data flash memory
//
// calculate the page for last_block+1
//
// Check if the last block has the same data, if it is the same, do not save
//
// if the sector is full
//         erase all sector
//         save all band variables with the actual as last one
//         (save all band block at once in the first page)
//         last_block = BAND_VARS_SIZE-1  (last position)
// else
//         read the page (256 bytes) of the last_block+1
//         fill with the block data on last_block+1 position
//         write to the DFLASH
//         last_block = last_block+1
//
//
//***********************************************************************
bool Dflash_write_block(uint8_t *data_bl)
{
  uint16_t ndata;
  uint16_t next_block_pos_in_page;
  uint32_t last_block_addr_read;
  uint32_t page_addr_read;
  uint8_t *ap_bl;
  uint8_t pg[FLASH_PAGE_SIZE];  
  uint16_t npage;   //next block's page
  uint16_t i,j,k;
  uint16_t block_in_page;
  uint16_t  data_index;
  uint32_t freq;  
  uint8_t chksum;

//check if it is the same as the last already saved
  last_block_addr_read = DFLASH_ADDR_READ(last_block * DATA_BLOCK_SIZE);
  for(data_index = 0; data_index < BAND_VARS_SIZE; data_index++)   //check band and freq
  {
    if(data_bl[data_index] != *((const uint8_t *)(last_block_addr_read + data_index)))               //band_vars[last_block][data_index])
    {
      break;
    }
  }
  if(data_index < BAND_VARS_SIZE)  //actual data is different from last band in DFLASH
  {
    PRT_LN("Last block different -> writing the new one");

    if(last_block+1 >= FLASH_SECTOR_SIZE)  //DFLASH sector full
    {
      Dflash_erase_sector();

      //fill the page with the band_vars[HMI_NUM_OPT_BPF][BAND_VARS_SIZE]
      k = 0;
      for(i=0; i<HMI_NUM_OPT_BPF; i++)
        {
          if(i!=hmi_band)   //band_vars[hmi_band][HMI_S_BPF])    //skip the actual band
          {
            for(j=0; j<BAND_VARS_SIZE; j++)
            {
              pg[k++] = band_vars[i][j];
            }
          }
        }
      for(j=0; j<BAND_VARS_SIZE; j++)  //put the actual band as the last on mem, to be the initial band after reset
      {
        pg[k++] = band_vars[hmi_band][j];
      }

      npage = 0;    // page is ready to write
      //the page data with the new block is ready to write
      Dflash_write_page(npage, pg);
      
      last_block = HMI_NUM_OPT_BPF-1;  //all 5 bands saved on erased memory, the last_block will be the last saved (=actual band)
    }
    else  // writing the block to the page
    {  
      //calculate the page for last_block+1
      npage = (last_block+1) / MAX_NBLOCK_IN_PAGE;

      //calculte the block+1 num inside of the page
      block_in_page = (last_block+1) - (npage * MAX_NBLOCK_IN_PAGE);

      //read the existing page
      page_addr_read = DFLASH_ADDR_READ(npage * FLASH_PAGE_SIZE);

      PRT_LN("Writing to the page " + String(npage) + "    and   block in page " + String(block_in_page));      
      PRT_LN("page_addr_read = " + String(page_addr_read));


      for(ndata = 0; ndata < FLASH_PAGE_SIZE; ndata++)  //full page read
        {   
        pg[ndata] = *((const uint8_t *)(page_addr_read + ndata));  //copy byte by byte
        }
      PRT_LN("Copy old page ok");

      //put the new data in the page
      next_block_pos_in_page = (block_in_page * DATA_BLOCK_SIZE);
  //    ap_bl = data_bl;
      for(ndata = 0; ndata < BAND_VARS_SIZE; ndata++)  //dflash address
        {   
        PRT(String(next_block_pos_in_page + ndata) + " ");
        }   
      PRT_LN(" ");
      chksum = 0;      
      for(ndata = 0; ndata < BAND_VARS_SIZE; ndata++)  //data to page
        {   
  //      pg[addr + ndata] = *ap_bl;  //copy byte by byte
        pg[next_block_pos_in_page + ndata] = data_bl[ndata];  //copy byte by byte
        chksum += data_bl[ndata];         
        PRT(String(data_bl[ndata]) + " ");
  //      PRT(String(*ap_bl) + " ");
  //      ap_bl++;
        }    
      pg[next_block_pos_in_page + BAND_VARS_SIZE] = chksum;

#ifdef DFLASH_debug      
      freq = data_bl[HMI_NMENUS_DFLASH+0];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+1];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+2];
      freq <<= 8;
      freq += data_bl[HMI_NMENUS_DFLASH+3];

      PRT_LN("  freq = " + String(freq));
#endif

      //npage already calculated
      //the page data with the new block is ready to write (overwrite the the old data with the same values and write the new ones)
      Dflash_write_page(npage, pg);
      
      last_block += 1;  //a new block on DFLASH
    }

  }  //if data not equal
  else
  {
      PRT_LN("The last block is the same, skip writing... "); 
  }  


  return true;
}
  






//***********************************************************************
//
//
//***********************************************************************
void Dflash_setup(void)
{



}





//***********************************************************************
//
//
//***********************************************************************
void Dflash_loop(void)
{



}






