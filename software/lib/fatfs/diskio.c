/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs     (C)ChaN, 2025               */
/*-----------------------------------------------------------------------*/
/* Adapted for 6502/cc65 with SPI/SD card library                       */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <spi.h>
#include <sdcard.h>
#include "ff.h"			/* Basic definitions of FatFs */
#include "diskio.h"		/* Declarations FatFs API */

/* Device mapping */
#define DEV_SDCARD	0	/* Map SD card to physical drive 0 */

static bool initialized = false;
static bool protected = false;
static bool nodisk = false;

void spi_initialize() {
  spi_init(100, 0, 0);   /* divisor 100, CPOL=0, CPHA=0 */
  spi_cs_low();
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (BYTE pdrv) {       /* Physical drive number to identify the drive */
  int i, done;

  if (pdrv != DEV_SDCARD)
    return STA_NOINIT;

  spi_initialize();

  for (done = i = 0; i < 10; i++) {
    if (sd_init() == SD_SUCCESS) {
      done = 1;
      break;
    }
  }
  if (!done) {
    nodisk = true;
    return STA_NODISK;
  }
  
  if (sd_protected() != 0) {
    protected = true;
    return STA_PROTECT;
  }
  
  return 0;  // Card present and writable
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (BYTE pdrv) {	/* Physical drive number to identify the drive */
  int count = 0;
  
  if (pdrv != DEV_SDCARD)
    return STA_NOINIT;	                /* Supports only SD card */
  if (! nodisk)
    return STA_NODISK;		        /* No card in the socket */
  count = 0;
 again:
  spi_initialize();
  switch(sd_init()) {
  case SD_SUCCESS:
    spi_set_divisor(0x0);               /* Set to fastest speed */
    initialized = true;
    return 0;
  default:
    if (count++ < 10)
      goto again;
    return STA_NOINIT;
  }
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
  UINT i;
  
  if (pdrv != DEV_SDCARD)
    return RES_PARERR;	/* Invalid drive */
  
  if (!initialized) 
    return RES_NOTRDY;	/* Not initialized */
  
  if (!count)
    return RES_PARERR;	/* Invalid parameter */
  
  // Read multiple sectors
  for (i = 0; i < count; i++) {
    if (sd_read(sector + i, buff + (i * 512)) != SD_SUCCESS) {
      return RES_ERROR;
    }
  }
  
  return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {		
  UINT i;
  
  if (pdrv != DEV_SDCARD)
    return RES_PARERR;	/* Invalid drive */
  
  if (!initialized)
    return RES_NOTRDY;	/* Not initialized */
  
  if (!count)
    return RES_PARERR;	/* Invalid parameter */
  
  // Write multiple sectors
  for (i = 0; i < count; i++) {
    if (sd_write(sector + i, (BYTE*)(buff + (i * 512))) != SD_SUCCESS) {
      return RES_ERROR;
    }
  }
  
  return RES_OK;
}

#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
  DRESULT res;
  
  if (pdrv != DEV_SDCARD)
    return RES_PARERR;	  /* Invalid drive */
  
  if (!initialized)
    return RES_NOTRDY;	  /* Not initialized */
  
  res = RES_ERROR;
  
  switch (cmd) {
  case CTRL_SYNC:	  /* Complete pending write process */
                          /* For SD cards, write operations are typically synchronous */
    res = RES_OK;
    break;
    
  case GET_SECTOR_COUNT:  /* Get media size */
    // This would require implementing sd_get_sector_count() in your SD library
    // For now, return error - FatFS can work without this for basic operations
    res = RES_ERROR;
    break;
    
  case GET_SECTOR_SIZE:	/* Get sector size */
    *(WORD*)buff = 512;	/* SD cards use 512-byte sectors */
    res = RES_OK;
    break;
    
  case GET_BLOCK_SIZE:	/* Get erase block size */
    *(DWORD*)buff = 32;	/* SD cards typically have 32-sector erase blocks */
    res = RES_OK;
    break;
    
  default:
    res = RES_PARERR;
  }
  
  return res;
}
