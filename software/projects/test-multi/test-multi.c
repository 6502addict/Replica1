/*
 * Multi-File Test Program for FatFS
 * Tests opening 4 disk images + 1 log file + 1 config file (6 total)
 * Simulates your final application usage pattern
 */

#include <stdio.h>
#include <string.h>
#include "ff.h"

// Test configuration
#define MAX_FILES 6
#define TEST_DATA_SIZE 512
#define LOG_ENTRIES 10
#define CONFIG_LINES 5

// File handles for simulation
static FIL disk_images[4];    // 4 floppy disk images  
static FIL log_file;          // 1 log file
static FIL config_file;       // 1 config file (temporary)

// Test data
static char test_buffer[TEST_DATA_SIZE];
static char log_buffer[128];
static char config_buffer[256];

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


/*
 * Initialize test data
 */
void init_test_data(void) {
    int i;
    
    // Fill test buffer with pattern
    for (i = 0; i < TEST_DATA_SIZE; i++) {
        test_buffer[i] = (char)('A' + (i % 26));
    }
    test_buffer[TEST_DATA_SIZE - 1] = '\0';
}

/*
 * Test 1: Open all files simultaneously
 */
int test_open_all_files(void) {
    FRESULT res;
    int i;
    char filename[32];
    
    printf("=== Test 1: Opening all files ===\n");
    
    // Open 4 disk image files
    for (i = 0; i < 4; i++) {
        sprintf(filename, "DISK%d.IMG", i);
        printf("Opening %s... ", filename);
        
        res = f_open(&disk_images[i], filename, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
        if (res != FR_OK) {
            printf("FAILED (error %d)\n", res);
            return -1;
        }
        printf("OK\n");
    }
    
    // Open log file
    printf("Opening LOG.TXT... ");
    res = f_open(&log_file, "LOG.TXT", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK\n");
    
    // Open config file
    printf("Opening CONFIG.TXT... ");
    res = f_open(&config_file, "CONFIG.TXT", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK\n");
    
    printf("SUCCESS: All 6 files opened simultaneously!\n\n");
    return 0;
}

/*
 * Test 2: Write data to all files
 */
int test_write_all_files(void) {
    FRESULT res;
    UINT bytes_written;
    int i, j;
    
    printf("=== Test 2: Writing to all files ===\n");
    
    // Write test data to each disk image
    for (i = 0; i < 4; i++) {
        printf("Writing to DISK%d.IMG... ", i);
        
        // Write different pattern to each disk
        sprintf(test_buffer, "DISK IMAGE %d - ", i);
        for (j = strlen(test_buffer); j < TEST_DATA_SIZE - 1; j++) {
            test_buffer[j] = 'A' + i;
        }
        test_buffer[TEST_DATA_SIZE - 1] = '\0';
        
        res = f_write(&disk_images[i], test_buffer, TEST_DATA_SIZE, &bytes_written);
        if (res != FR_OK || bytes_written != TEST_DATA_SIZE) {
            printf("FAILED (error %d, wrote %u bytes)\n", res, bytes_written);
            return -1;
        }
        printf("OK (%u bytes)\n", bytes_written);
    }
    
    // Write log entries
    printf("Writing to LOG.TXT... ");
    for (i = 0; i < LOG_ENTRIES; i++) {
        sprintf(log_buffer, "LOG ENTRY %d: System operation at time %d\n", i, i * 1000);
        res = f_write(&log_file, log_buffer, strlen(log_buffer), &bytes_written);
        if (res != FR_OK) {
            printf("FAILED at entry %d (error %d)\n", i, res);
            return -1;
        }
    }
    printf("OK (%d entries)\n", LOG_ENTRIES);
    
    // Write config data
    printf("Writing to CONFIG.TXT... ");
    for (i = 0; i < CONFIG_LINES; i++) {
        sprintf(config_buffer, "CONFIG_PARAM_%d=VALUE_%d\n", i, i * 10);
        res = f_write(&config_file, config_buffer, strlen(config_buffer), &bytes_written);
        if (res != FR_OK) {
            printf("FAILED at line %d (error %d)\n", i, res);
            return -1;
        }
    }
    printf("OK (%d lines)\n", CONFIG_LINES);
    
    printf("SUCCESS: All files written!\n\n");
    return 0;
}

/*
 * Test 3: Read back and verify data
 */
int test_read_all_files(void) {
    FRESULT res;
    UINT bytes_read;
    int i;
    static char read_buffer[TEST_DATA_SIZE];
    
    printf("=== Test 3: Reading from all files ===\n");
    
    // Seek back to beginning of all files
    for (i = 0; i < 4; i++) {
        f_lseek(&disk_images[i], 0);
    }
    f_lseek(&log_file, 0);
    f_lseek(&config_file, 0);
    
    // Read from each disk image
    for (i = 0; i < 4; i++) {
        static char expected[32];

        printf("Reading from DISK%d.IMG... ", i);
        
        res = f_read(&disk_images[i], read_buffer, TEST_DATA_SIZE, &bytes_read);
        if (res != FR_OK || bytes_read != TEST_DATA_SIZE) {
            printf("FAILED (error %d, read %u bytes)\n", res, bytes_read);
            return -1;
        }
        
        // Verify first part of data
        sprintf(expected, "DISK IMAGE %d - ", i);
        if (strncmp(read_buffer, expected, strlen(expected)) != 0) {
            printf("FAILED (data mismatch)\n");
            return -1;
        }
        printf("OK (%u bytes, data verified)\n", bytes_read);
    }
    
    // Read log file
    printf("Reading LOG.TXT... ");
    res = f_read(&log_file, read_buffer, sizeof(read_buffer), &bytes_read);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK (%u bytes)\n", bytes_read);
    
    // Read config file  
    printf("Reading CONFIG.TXT... ");
    res = f_read(&config_file, read_buffer, sizeof(read_buffer), &bytes_read);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK (%u bytes)\n", bytes_read);
    
    printf("SUCCESS: All files read and verified!\n\n");
    return 0;
}

/*
 * Test 4: Simulate real application usage
 * Keep 4 images + log open, close/reopen config file
 */
int test_real_usage_pattern(void) {
    FRESULT res;
    UINT bytes_written, bytes_read;
    static char temp_buffer[128];
    int i;
    
    printf("=== Test 4: Real application usage pattern ===\n");
    
    // Close config file (simulate reading config at startup then closing)
    printf("Closing CONFIG.TXT (simulate startup config read)... ");
    res = f_close(&config_file);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK\n");
    
    // Now we have 4 disk images + 1 log file open (5 total)
    printf("Current state: 4 disk images + 1 log file open (5 total)\n");
    
    // Simulate some disk operations
    for (i = 0; i < 4; i++) {
        printf("Writing to disk image %d... ", i);
        sprintf(temp_buffer, "OPERATION_%d_ON_DISK_%d\n", i * 10, i);
        f_lseek(&disk_images[i], 0);  // Go to start
        res = f_write(&disk_images[i], temp_buffer, strlen(temp_buffer), &bytes_written);
        if (res != FR_OK) {
            printf("FAILED (error %d)\n", res);
            return -1;
        }
        printf("OK\n");
    }
    
    // Add log entries
    printf("Adding log entries... ");
    for (i = 0; i < 3; i++) {
        sprintf(temp_buffer, "RUNTIME LOG %d: Disk operation completed\n", i);
        res = f_write(&log_file, temp_buffer, strlen(temp_buffer), &bytes_written);
        if (res != FR_OK) {
            printf("FAILED (error %d)\n", res);
            return -1;
        }
    }
    printf("OK\n");
    
    // Temporarily reopen config file (6th file)
    printf("Temporarily reopening CONFIG.TXT... ");
    res = f_open(&config_file, "CONFIG.TXT", FA_READ);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    
    // Read some config
    res = f_read(&config_file, temp_buffer, sizeof(temp_buffer), &bytes_read);
    if (res != FR_OK) {
        printf("FAILED reading (error %d)\n", res);
        return -1;
    }
    
    // Close config again
    f_close(&config_file);
    printf("OK (read %u bytes, closed again)\n", bytes_read);
    
    printf("SUCCESS: Real usage pattern works!\n\n");
    return 0;
}

/*
 * Close all remaining files
 */
void cleanup_files(void) {
    int i;
    
    printf("=== Cleanup: Closing all files ===\n");
    
    for (i = 0; i < 4; i++) {
        printf("Closing DISK%d.IMG... ", i);
        if (f_close(&disk_images[i]) == FR_OK) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
        }
    }
    
    printf("Closing LOG.TXT... ");
    if (f_close(&log_file) == FR_OK) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
    }
    
    printf("Cleanup complete\n");
}

/*
 * Main test program
 */
int multifile_test(void) {
    FATFS fs;
    FRESULT res;
    
    printf("=== FatFS Multi-File Test Program ===\n");
    printf("Testing: 4 disk images + 1 log + 1 config = 6 files\n\n");
    
    // Mount filesystem
    printf("Mounting filesystem... ");
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        return -1;
    }
    printf("OK\n\n");
    
    // Initialize test data
    init_test_data();
    
    // Run tests
    if (test_open_all_files() != 0) goto error;
    if (test_write_all_files() != 0) goto error;  
    if (test_read_all_files() != 0) goto error;
    if (test_real_usage_pattern() != 0) goto error;
    
    // Success!
    printf("=== ALL TESTS PASSED! ===\n");
    printf("Your system can handle:\n");
    printf("- 6 files open simultaneously\n");
    printf("- Read/write operations on all files\n");
    printf("- Real application usage pattern\n\n");
    
    cleanup_files();
    f_unmount("");
    return 0;
    
error:
    printf("=== TEST FAILED! ===\n");
    cleanup_files();
    f_unmount("");
    return -1;
}

/* 
 * Simple main function - adapt to your system
 */
int main(void) {
    return multifile_test();
}
