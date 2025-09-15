/*-----------------------------------------------------------------------*/
/* FatFS f_lseek() Test Program                                          */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "ff.h"

FATFS fs;

#if !FF_FS_READONLY && !FF_FS_NORTC
DWORD get_fattime (void)
{
    // Return a fixed timestamp since we don't have RTC
    // Format: bit31:25=Year(0-127 +1980), bit24:21=Month(1-12), bit20:16=Day(1-31),
    //         bit15:11=Hour(0-23), bit10:5=Minute(0-59), bit4:0=Second(0-29 *2)
    
    // Fixed date: 2025-01-01 12:00:00
    return ((DWORD)(2025 - 1980) << 25)  // Year = 2025
           | ((DWORD)1 << 21)            // Month = January  
           | ((DWORD)1 << 16)            // Day = 1
           | ((DWORD)12 << 11)           // Hour = 12
           | ((DWORD)0 << 5)             // Minute = 0
           | ((DWORD)0 << 0);            // Second = 0
}
#endif



int main(void) {
    FRESULT fr;
    FIL fil;
    UINT bw, br;
    char write_data[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char read_buffer[16];
    
    printf("FatFS f_lseek Test\n");
    
    // Mount
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("Mount failed: %d\n", fr);
        return 1;
    }
    printf("Mounted OK\n");
    
    // Create test file with known data
    fr = f_open(&fil, "seektest.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("Create failed: %d\n", fr);
        return 1;
    }
    
    fr = f_write(&fil, write_data, strlen(write_data), &bw);
    f_close(&fil);
    if (fr != FR_OK) {
        printf("Write failed: %d\n", fr);
        return 1;
    }
    printf("Created file with %u bytes\n", bw);
    
    // Test random access reads
    fr = f_open(&fil, "seektest.txt", FA_READ);
    if (fr != FR_OK) {
        printf("Open for read failed: %d\n", fr);
        return 1;
    }
    
    // Test 1: Seek to position 10, read 5 bytes
    fr = f_lseek(&fil, 10);
    if (fr != FR_OK) {
        printf("Seek to 10 failed: %d\n", fr);
        f_close(&fil);
        return 1;
    }
    
    memset(read_buffer, 0, sizeof(read_buffer));
    fr = f_read(&fil, read_buffer, 5, &br);
    if (fr != FR_OK || br != 5) {
        printf("Read after seek failed: %d, bytes: %u\n", fr, br);
        f_close(&fil);
        return 1;
    }
    printf("Position 10: '%s' (expect 'ABCDE')\n", read_buffer);
    
    // Test 2: Seek to position 0, read 5 bytes
    fr = f_lseek(&fil, 0);
    if (fr != FR_OK) {
        printf("Seek to 0 failed: %d\n", fr);
        f_close(&fil);
        return 1;
    }
    
    memset(read_buffer, 0, sizeof(read_buffer));
    fr = f_read(&fil, read_buffer, 5, &br);
    if (fr != FR_OK || br != 5) {
        printf("Read from start failed: %d, bytes: %u\n", fr, br);
        f_close(&fil);
        return 1;
    }
    printf("Position 0: '%s' (expect '01234')\n", read_buffer);
    
    // Test 3: Seek to position 30, read remaining bytes
    fr = f_lseek(&fil, 30);
    if (fr != FR_OK) {
        printf("Seek to 30 failed: %d\n", fr);
        f_close(&fil);
        return 1;
    }
    
    memset(read_buffer, 0, sizeof(read_buffer));
    fr = f_read(&fil, read_buffer, 10, &br);
    if (fr != FR_OK) {
        printf("Read from 30 failed: %d, bytes: %u\n", fr, br);
        f_close(&fil);
        return 1;
    }
    printf("Position 30: '%s' (expect 'UVWXYZ')\n", read_buffer);
    
    f_close(&fil);
    
    printf("f_lseek test complete!\n");
    return 0;
}
