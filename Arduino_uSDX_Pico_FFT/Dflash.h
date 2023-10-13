#ifndef __DFLASH_H__
#define __DFLASH_H__

#ifdef __cplusplus
extern "C" {
#endif




#define DATA_BLOCK_SIZE   16       // max number of number of bytes in the block

//PICO_FLASH_SIZE_BYTES # 2MB = 2097152 = 0x200000 The total size of the RP2040 flash, in bytes
//FLASH_SECTOR_SIZE     # 4KB  The size of one sector, in bytes (the minimum amount you can erase)
//FLASH_PAGE_SIZE       # 256B The size of one page, in bytes (the mimimum amount you can write)
//XIP_BASE              #     = 0x10000000 bytes for RAM
#define FLASH_SECTOR_SIZE   4096
#define MAX_NPAGE     (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)   //number of pages inside of a sector  4096 / 256 = 16 pages of 256 bytes on the sector
#define MAX_NBLOCK_IN_PAGE    (FLASH_PAGE_SIZE / DATA_BLOCK_SIZE)     //number of data blocks inside of a page
#define MAX_NBLOCK    (FLASH_SECTOR_SIZE / DATA_BLOCK_SIZE)     //4096 / 16 number of data blocks inside of a sector
#define FLASH_TARGET_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)  // start address for the last sector on FLASH mem

#define DFLASH_ADDR_READ(x)  (XIP_BASE +  FLASH_TARGET_OFFSET + (x))  //byte by byte
#define DFLASH_ADDR_WRITE(page)  (FLASH_TARGET_OFFSET + ((page)  * FLASH_PAGE_SIZE))  // FLASH_PAGE_SIZE
#define DFLASH_ADDR_ERASE  FLASH_TARGET_OFFSET  // FLASH_SECTOR_SIZE



extern uint16_t DFLASH_in_use;  //used to stop core1 for wirting on DFLASH memory


void Init_HMI_data(uint8_t *actual_bnd);
bool Dflash_write_block(uint8_t *data_bl);

void Dflash_setup(void);
void Dflash_loop(void);











#ifdef __cplusplus
}
#endif
#endif
