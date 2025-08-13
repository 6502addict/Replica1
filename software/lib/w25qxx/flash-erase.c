/*
 * flash-erase.c
 * Program to completely erase W25Qxx flash chips
 * Uses the generic W25Qxx library
 * For cc65 compiler compatibility
 * 
 * WARNING: This program will PERMANENTLY erase ALL data on the flash chip!
 */

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include "w25qxx.h"

/* Function prototypes */
static uint8_t get_user_confirmation(void);
static void print_chip_info(void);
static void print_warning_message(void);
static uint8_t wait_for_keypress(void);
static void print_progress_dots(uint8_t count);
static const char* get_chip_name(w25qxx_chip_t chip_type);

int main(void)
{
    uint8_t init_result;
    uint8_t confirmed;
    uint8_t erase_result;
    uint8_t progress_counter;
    
    printf("\nW25Qxx Flash Chip Erase Tool\n");
    printf("============================\n\n");
    
    /* Initialize the flash chip */
    printf("Initializing flash chip...\n");
    init_result = w25qxx_init();
    
    if (init_result != 0) {
        printf("ERROR: Cannot initialize flash chip!\n");
        printf("Check SPI connections and power supply.\n");
        printf("Press any key to exit...\n");
        wait_for_keypress();
        return 1;
    }
    
    printf("Flash chip initialized successfully!\n\n");
    
    /* Display chip information */
    print_chip_info();
    
    /* Display warning and get confirmation */
    print_warning_message();
    
    confirmed = get_user_confirmation();
    
    if (!confirmed) {
        printf("\nOperation cancelled by user.\n");
        printf("No data was erased.\n");
        return 0;
    }
    
    /* Final warning before erasure */
    printf("\nLAST CHANCE TO ABORT!\n");
    printf("Press 'Y' to proceed with COMPLETE CHIP ERASE, any other key to abort: ");
    
    if (toupper(getchar()) != 'Y') {
        printf("\nOperation aborted.\n");
        printf("No data was erased.\n");
        return 0;
    }
    
    /* Clear input buffer */
    while (getchar() != '\n') {
        /* consume remaining characters */
    }
    
    printf("\nStarting chip erase...\n");
    printf("This may take up to 100 seconds for large chips.\n");
    printf("DO NOT POWER OFF THE SYSTEM!\n\n");
    
    /* Perform the chip erase */
    printf("Erasing");
    erase_result = w25qxx_erase_chip();
    
    /* Show progress while waiting (erase_chip already waits, but show activity) */
    progress_counter = 0;
    while (w25qxx_is_busy() && progress_counter < 200) {
        print_progress_dots(1);
        progress_counter++;
        /* Small delay - implementation dependent */
        /* You might want to add a delay function here */
    }
    
    printf("\n\n");
    
    /* Check result */
    if (erase_result == 0) {
        printf("SUCCESS: Chip erase completed!\n");
        printf("All data has been erased from the flash chip.\n");
        printf("The chip is now ready for new data.\n");
    } else {
        printf("ERROR: Chip erase failed!\n");
        printf("The chip may be write-protected or damaged.\n");
        return 1;
    }
    
    /* Verify the erase by reading a few locations */
    printf("\nVerifying erase...\n");
    {
        uint8_t verify_buffer[16];
        uint32_t test_addresses[4];
        uint8_t i, j;
        uint8_t all_ff;
        
        /* Test beginning, middle, and end of chip */
        test_addresses[0] = 0x00000000;
        test_addresses[1] = w25qxx_get_chip_size() / 4;
        test_addresses[2] = w25qxx_get_chip_size() / 2;
        test_addresses[3] = w25qxx_get_chip_size() - 16;
        
        all_ff = 1;
        
        for (i = 0; i < 4; i++) {
            w25qxx_read(test_addresses[i], verify_buffer, 16);
            
            for (j = 0; j < 16; j++) {
                if (verify_buffer[j] != 0xFF) {
                    all_ff = 0;
                    break;
                }
            }
            
            if (!all_ff) {
                break;
            }
        }
        
        if (all_ff) {
            printf("SUCCESS: Verification passed - all tested locations contain 0xFF\n");
        } else {
            printf("WARNING: Verification failed - some locations not properly erased\n");
            printf("The chip may be damaged or write-protected\n");
        }
    }
    
    printf("\nFlash erase operation complete.\n");
    printf("Press any key to exit...\n");
    wait_for_keypress();
    
    return 0;
}

/*
 * Get user confirmation for the erase operation
 */
static uint8_t get_user_confirmation(void)
{
    char input[10];
    uint8_t i;
    
    printf("Do you want to COMPLETELY ERASE this flash chip? (yes/no): ");
    
    /* Read user input */
    i = 0;
    while (i < sizeof(input) - 1) {
        input[i] = getchar();
        if (input[i] == '\n' || input[i] == '\r') {
            break;
        }
        i++;
    }
    input[i] = '\0';
    
    /* Convert to lowercase and check */
    for (i = 0; input[i] != '\0'; i++) {
        input[i] = tolower(input[i]);
    }
    
    /* Check if user typed "yes" */
    if (input[0] == 'y' && input[1] == 'e' && input[2] == 's' && input[3] == '\0') {
        return 1;
    }
    
    return 0;
}

/*
 * Print information about the detected chip
 */
static void print_chip_info(void)
{
    w25qxx_chip_t chip_type;
    uint32_t chip_size;
    uint32_t size_mb;
    
    chip_type = w25qxx_get_chip_type();
    chip_size = w25qxx_get_chip_size();
    size_mb = chip_size / (1024UL * 1024UL);
    
    printf("DETECTED FLASH CHIP:\n");
    printf("--------------------\n");
    printf("Model: %s\n", get_chip_name(chip_type));
    printf("Size: %lu bytes (%lu MB)\n", chip_size, size_mb);
    printf("Sectors: %u (4KB each)\n", w25qxx_config.total_sectors);
    printf("Blocks: %u (64KB each)\n", w25qxx_config.total_blocks);
    printf("\n");
}

/*
 * Print warning message about data loss
 */
static void print_warning_message(void)
{
    printf("*********************************************\n");
    printf("*                 WARNING!                  *\n");
    printf("*********************************************\n");
    printf("*                                           *\n");
    printf("* This operation will PERMANENTLY ERASE     *\n");
    printf("* ALL DATA on the flash chip!               *\n");
    printf("*                                           *\n");
    printf("* - All files will be lost                  *\n");
    printf("* - All firmware will be erased             *\n");
    printf("* - This operation cannot be undone         *\n");
    printf("*                                           *\n");
    printf("* Make sure you have backups of any         *\n");
    printf("* important data before proceeding!         *\n");
    printf("*                                           *\n");
    printf("*********************************************\n\n");
}

/*
 * Wait for any keypress
 */
static uint8_t wait_for_keypress(void)
{
    return getchar();
}

/*
 * Print progress dots
 */
static void print_progress_dots(uint8_t count)
{
    uint8_t i;
    
    for (i = 0; i < count; i++) {
        putchar('.');
    }
}

/*
 * Get human-readable chip name
 */
static const char* get_chip_name(w25qxx_chip_t chip_type)
{
    switch (chip_type) {
        case W25Q16:
            return "W25Q16";
        case W25Q32:
            return "W25Q32";
        case W25Q64:
            return "W25Q64";
        case W25Q128:
            return "W25Q128";
        case W25Q256:
            return "W25Q256";
        default:
            return "Unknown";
    }
}
