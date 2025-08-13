/*
 * flash-dump.c
 * Program to dump and visualize W25Qxx flash chip content in hexadecimal
 * Displays blocks (64KB each) with hexadecimal and ASCII representation
 * Uses the generic W25Qxx library
 * For cc65 compiler compatibility
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "w25qxx.h"

/* Buffer size for reading data - must fit in memory */
#define READ_BUFFER_SIZE 1024

/* Function prototypes */
static void dump(uint8_t *buffer, uint32_t size, uint32_t base_address);
static void print_menu(void);
static uint8_t get_user_input(uint16_t* start_block, uint16_t* block_count);
static void dump_blocks(uint16_t start_block, uint16_t block_count);
static void print_chip_info(void);
static const char* get_chip_name(w25qxx_chip_t chip_type);
static uint8_t wait_for_keypress(void);
static void clear_input_buffer(void);

int main(void)
{
    uint8_t init_result;
    uint16_t start_block, block_count;
    uint8_t continue_program;
    
    printf("W25Qxx Flash Block Dump Tool\n");
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
    
    /* Main program loop */
    continue_program = 1;
    while (continue_program) {
        print_menu();
        
        if (get_user_input(&start_block, &block_count)) {
            dump_blocks(start_block, block_count);
            printf("\nPress any key to continue...\n");
            wait_for_keypress();
        } else {
            continue_program = 0;
        }
    }
    
    printf("Goodbye!\n");
    return 0;
}

/*
 * Enhanced dump function with base address display
 */
static void dump(uint8_t *buffer, uint32_t size, uint32_t base_address)
{
    uint32_t i, a, lsize;
    int c;
    
    for (a = 0; size > 0;) {
        printf("%08lX: ", base_address + a);
        lsize = (size >= 16) ? 16 : size;
        
        /* Print hex bytes */
        for (i = 0; i < lsize; i++) {
            printf("%02X ", buffer[a + i]);
        }
        
        /* Pad with spaces if less than 16 bytes */
        for (i = lsize; i < 16; i++) {
            printf("   ");
        }
        
        printf("  |");
        
        /* Print ASCII representation */
        for (i = 0; i < lsize; i++) {
            c = buffer[a + i];
            printf("%c", ((c < 0x20) || (c > 126)) ? '.' : c);
        }
        
        /* Pad ASCII section */
        for (i = lsize; i < 16; i++) {
            printf(" ");
        }
        
        a += lsize;
        size -= lsize;
        printf("|\n");
    }
}

/*
 * Print the main menu
 */
static void print_menu(void)
{
    printf("\n==========================================\n");
    printf("Flash Block Dump Menu\n");
    printf("==========================================\n");
    printf("Available blocks: 0 to %u (64KB each)\n", w25qxx_config.total_blocks - 1);
    printf("Total capacity: %lu bytes\n", w25qxx_get_chip_size());
    printf("\nEnter 'quit' to exit\n");
    printf("Or enter starting block and count:\n");
}

/*
 * Get user input for block range
 * Returns 1 if valid input, 0 if quit requested
 */
static uint8_t get_user_input(uint16_t* start_block, uint16_t* block_count)
{
    char input_line[50];
    char *token;
    long start_val, count_val;
    uint8_t i;
    
    printf("\nEnter command: ");
    
    /* Read entire line */
    i = 0;
    while (i < sizeof(input_line) - 1) {
        input_line[i] = getchar();
        if (input_line[i] == '\n' || input_line[i] == '\r') {
            break;
        }
        i++;
    }
    input_line[i] = '\0';
    
    /* Convert to lowercase and check for quit */
    for (i = 0; input_line[i] != '\0'; i++) {
        input_line[i] = tolower(input_line[i]);
    }
    
    if (strstr(input_line, "quit") != NULL || strstr(input_line, "exit") != NULL) {
        return 0;
    }
    
    /* Parse numbers */
    token = strtok(input_line, " \t");
    if (token == NULL) {
        printf("ERROR: Please enter starting block number\n");
        return 1; /* Continue, but don't dump */
    }
    
    start_val = atol(token);
    
    token = strtok(NULL, " \t");
    if (token == NULL) {
        count_val = 1; /* Default to 1 block */
    } else {
        count_val = atol(token);
    }
    
    /* Validate input */
    if (start_val < 0 || start_val >= w25qxx_config.total_blocks) {
        printf("ERROR: Starting block must be 0 to %u\n", w25qxx_config.total_blocks - 1);
        return 1;
    }
    
    if (count_val <= 0) {
        printf("ERROR: Block count must be greater than 0\n");
        return 1;
    }
    
    if (start_val + count_val > w25qxx_config.total_blocks) {
        printf("ERROR: Block range exceeds chip capacity\n");
        printf("Maximum blocks from block %ld: %u\n", 
               start_val, w25qxx_config.total_blocks - (uint16_t)start_val);
        return 1;
    }
    
    *start_block = (uint16_t)start_val;
    *block_count = (uint16_t)count_val;
    
    return 1; /* Valid input, proceed with dump */
}

/*
 * Dump the specified range of blocks
 */
static void dump_blocks(uint16_t start_block, uint16_t block_count)
{
    static uint8_t read_buffer[READ_BUFFER_SIZE];
    uint32_t block_address, current_address;
    uint32_t bytes_remaining, bytes_to_read;
    uint16_t current_block;
    
    printf("\n==========================================\n");
    printf("Dumping %u block(s) starting from block %u\n", block_count, start_block);
    printf("==========================================\n");
    
    for (current_block = start_block; current_block < start_block + block_count; current_block++) {
        printf("\n--- Block %u (Address 0x%08lX to 0x%08lX) ---\n",
               current_block,
               (uint32_t)current_block * W25QXX_BLOCK_SIZE,
               (uint32_t)(current_block + 1) * W25QXX_BLOCK_SIZE - 1);
        
        block_address = (uint32_t)current_block * W25QXX_BLOCK_SIZE;
        bytes_remaining = W25QXX_BLOCK_SIZE;
        current_address = block_address;
        
        /* Read and dump the block in chunks */
        while (bytes_remaining > 0) {
            bytes_to_read = (bytes_remaining > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : bytes_remaining;
            
            /* Read chunk from flash */
            w25qxx_read(current_address, read_buffer, (uint16_t)bytes_to_read);
            
            /* Dump the chunk */
            dump(read_buffer, bytes_to_read, current_address);
            
            current_address += bytes_to_read;
            bytes_remaining -= bytes_to_read;
        }
        
        /* Pause between blocks if dumping multiple blocks */
        if (current_block < start_block + block_count - 1) {
            printf("\n--- End of Block %u ---\n", current_block);
            printf("Press any key for next block, 'q' to return to menu: ");
            if (tolower(getchar()) == 'q') {
                clear_input_buffer();
                break;
            }
            clear_input_buffer();
        }
    }
    
    printf("\n--- Dump Complete ---\n");
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
    size_mb = chip_size / (1024 * 1024);
    
    printf("DETECTED FLASH CHIP:\n");
    printf("--------------------\n");
    printf("Model: %s\n", get_chip_name(chip_type));
    printf("Size: %lu bytes (%lu MB)\n", chip_size, size_mb);
    printf("Total Blocks: %u (64KB each)\n", w25qxx_config.total_blocks);
    printf("Total Sectors: %u (4KB each)\n", w25qxx_config.total_sectors);
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

/*
 * Wait for any keypress
 */
static uint8_t wait_for_keypress(void)
{
    return getchar();
}

/*
 * Clear input buffer
 */
static void clear_input_buffer(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {
        /* consume remaining characters */
    }
}
