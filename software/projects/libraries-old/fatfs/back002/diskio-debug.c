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
static bool protected   = false;
static bool nodisk      = true;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (BYTE pdrv) {       // Physical drive number to identify the drive 
  printf("disk_status called\n");
  if (pdrv != DEV_SDCARD) {
    printf("disk_status:  not sdcard %d\n", pdrv);
    return STA_NOINIT;
  }
  if (nodisk) {
    printf("disk_status: no disk true\n");
    return STA_NODISK;
  }
  if (protected) {
    printf("disk_status: sd_protected failed\n");
    return STA_PROTECT;
  }
  return 0;  // Card present and writable
}


/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (BYTE pdrv) {	/* Physical drive number to identify the drive */
  int count = 0;
  
  spi_cs_high();
  printf("disk_initialize called\n");
  if (pdrv != DEV_SDCARD) {
    printf("disk_initialize:  not sdcard %d\n", pdrv);
    return STA_NOINIT;	                /* Supports only SD card */
  }
  for (count = 0; count < 100; count++) {
    spi_init(100, 0, 0);   /* divisor 80, CPOL=0, CPHA=0 */
    spi_cs_low();
    if (sd_init() == SD_SUCCESS) {
      nodisk = false;
      protected = false;
      initialized = true;
      spi_set_divisor(0x0);  
      return 0;
    }
    spi_cs_high();
  }
  printf("disk_initialize: initialization failed\n");
  return STA_NOINIT;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
  UINT i;
  
  printf("disk_read called\n");
  if (pdrv != DEV_SDCARD) {
    printf("disk_read:  not sdcard %d\n", pdrv);
    return RES_PARERR;	/* Invalid drive */
  }
  
  if (!initialized)  {
    printf("disk_read: not initialized\n");
    return RES_NOTRDY;	/* Not initialized */
  }
  
  if (!count) {
    printf("disk_read: parameter error count = 0\n");
    return RES_PARERR;	/* Invalid parameter */
  }
  
  // Read multiple sectors
  for (i = 0; i < count; i++) {
    if (sd_read(sector + i, buff + (i * 512)) != SD_SUCCESS) {
      printf("disk_read: read error\n");
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
  
  printf("disk_write called\n");
  if (pdrv != DEV_SDCARD) {
    printf("disk_write:  not sdcard %d\n", pdrv);
    return RES_PARERR;	/* Invalid drive */
  }
  
  if (!initialized) {
    printf("disk_write:  not initialized\n");
    return RES_NOTRDY;	/* Not initialized */
  }
  
  if (!count) {
    printf("disk_write:  parameter error count = 0\n");
    return RES_PARERR;	/* Invalid parameter */
  }
  
  // Write multiple sectors
  for (i = 0; i < count; i++) {
    if (sd_write(sector + i, (BYTE*)(buff + (i * 512))) != SD_SUCCESS) {
      printf("disk_write:  write error\n");
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
  
  printf("disk_ioctl called\n");
  if (pdrv != DEV_SDCARD) {
    printf("disk_ioctl:  not sdcard %d\n", pdrv);
    return RES_PARERR;	  /* Invalid drive */
  }
  
  if (!initialized) {
    printf("disk_ioctl: not initialized\n");
    return RES_NOTRDY;	  /* Not initialized */
  }
  
  res = RES_ERROR;
  
  switch (cmd) {
  case CTRL_SYNC:	  /* Complete pending write process */
                          /* For SD cards, write operations are typically synchronous */
    res = RES_OK;
    break;
    
  case GET_SECTOR_COUNT:  /* Get media size */
    // This would require implementing sd_get_sector_count() in your SD library
    // For now, return error - FatFS can work without this for basic operations
    printf("disk_ioctl: get_sector_count RES_ERROR\n");
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
    printf("disk_ioctl: parameter error\n");
    res = RES_PARERR;
  }
  
  return res;
}
