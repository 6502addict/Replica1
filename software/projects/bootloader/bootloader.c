#include <stdio.h>
#include <string.h>
#include "ff.h"

#define INPUT_BUFFER 0x0200
#define LOAD_ADDR    0x0300
#define MAX_SIZE     (0xC000 - 0x0300)  // Don't overwrite ROM

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

void main(void) {
    char filename[13];  // 8.3 + null
    char command[9];
    FATFS fs;
    FIL fil;
    FRESULT res;
    UINT bytes_read;
    char *buffer = (char *)INPUT_BUFFER;
    unsigned char *load_ptr = (unsigned char *)LOAD_ADDR;
    
    // Prompt for command
    printf("> ");
    
    // Read input line into buffer at $0200
    scanf("%127[^\n]", buffer);
    
    // Extract first word (command name)
    sscanf(buffer, "%8s", command);
    
    // Build filename: command.cmd
    strcpy(filename, command);
    strcat(filename, ".cmd");
    
    // Mount SD card
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("SD card error\n");
        return;
    }
    
    // Open file from root
    res = f_open(&fil, filename, FA_READ);
    if (res != FR_OK) {
        printf("File not found\n");
        return;
    }
    
    // Load entire file to $0300
    res = f_read(&fil, load_ptr, MAX_SIZE, &bytes_read);
    f_close(&fil);
    
    if (res != FR_OK) {
        printf("Read error\n");
        return;
    }
    
    printf("Loaded %u bytes\n", bytes_read);
    
    // Jump to $0300
    // Full command line still in buffer at $0200
    ((void (*)(void))LOAD_ADDR)();
}
