/*-----------------------------------------------------------------------*/
/* Minimal FatFS Test for Apple 1 (Low Memory)                          */
/*-----------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "ff.h"

// Single global FatFS object
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
    char data[32];  // Small buffer
    
    printf("FatFS Test\n");
    
    // Mount
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("Mount failed: %d\n", fr);
        return 1;
    }
    printf("Mounted OK\n");
    
    // Write test
    fr = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("Open failed: %d\n", fr);
        return 1;
    }
    
    strcpy(data, "Hello Apple 1!");
    fr = f_write(&fil, data, strlen(data), &bw);
    f_close(&fil);
    
    if (fr != FR_OK) {
        printf("Write failed: %d\n", fr);
        return 1;
    }
    printf("Wrote %u bytes\n", bw);
    
    // Read test
    fr = f_open(&fil, "test.txt", FA_READ);
    if (fr != FR_OK) {
        printf("Read open failed: %d\n", fr);
        return 1;
    }
    
    memset(data, 0, sizeof(data));
    fr = f_read(&fil, data, sizeof(data)-1, &br);
    f_close(&fil);
    
    if (fr != FR_OK) {
        printf("Read failed: %d\n", fr);
        return 1;
    }
    
    printf("Read %u bytes: %s\n", br, data);
    printf("Test complete\n");
    
    return 0;
}
