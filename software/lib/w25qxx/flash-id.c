/*
 * flash-id.c
 * Simple program to identify W25Qxx flash chips and display their capacity
 * Uses ONLY the generic W25Qxx library functions
 * For cc65 compiler compatibility
 */

#include <stdio.h>
#include <stdint.h>
#include "w25qxx.h"

/* Function prototypes */
static void print_chip_info(void);
static const char* get_chip_name(w25qxx_chip_t chip_type);
static void print_capacity_info(uint32_t size_bytes);
static void print_hex_byte(uint8_t value);

int main(void)
{
    uint8_t init_result;
    
    printf("W25Qxx Flash Identification Tool\n");
    printf("================================\n\n");
    
    /* Initialize the library - this handles all chip detection */
    printf("Initializing flash library...\n");
    init_result = w25qxx_init();
    
    if (init_result == 0) {
        printf("SUCCESS: Flash chip detected and initialized!\n\n");
        print_chip_info();
    } else {
        printf("ERROR: Flash chip not detected or not supported!\n\n");
        printf("Possible causes:\n");
        printf("- No flash chip connected\n");
        printf("- SPI connection problems\n");
        printf("- Power supply issues\n");
        printf("- Unsupported chip type\n");
        printf("- Wrong SPI mode or timing\n\n");
        printf("Check your hardware connections and try again.\n");
        return 1;
    }
    
    printf("Flash identification complete.\n");
    return 0;
}

/*
 * Print detailed information about the detected chip
 */
static void print_chip_info(void)
{
    w25qxx_chip_t chip_type;
    uint32_t chip_size;
    uint8_t status1, status2, status3;
    
    chip_type = w25qxx_get_chip_type();
    chip_size = w25qxx_get_chip_size();
    
    printf("CHIP INFORMATION:\n");
    printf("-----------------\n");
    printf("Chip Model: %s\n", get_chip_name(chip_type));
    
    print_capacity_info(chip_size);
    
    /* Display addressing mode */
    if (chip_type == W25Q256) {
        printf("Addressing: 4-byte mode (for 32MB capacity)\n");
    } else {
        printf("Addressing: 3-byte mode\n");
    }
    
    /* Display organization information */
    printf("\nMEMORY ORGANIZATION:\n");
    printf("--------------------\n");
    printf("Total Size: %lu bytes\n", chip_size);
    printf("Pages: %u (%u bytes each)\n", 
           w25qxx_config.total_pages, W25QXX_PAGE_SIZE);
    printf("Sectors: %u (%u bytes each)\n", 
           w25qxx_config.total_sectors, W25QXX_SECTOR_SIZE);
    printf("64KB Blocks: %u (%u bytes each)\n", 
           w25qxx_config.total_blocks, W25QXX_BLOCK_SIZE);
    
    /* Read and display status registers using library functions */
    printf("\nSTATUS REGISTERS:\n");
    printf("-----------------\n");
    
    status1 = w25qxx_get_status();
    printf("Status 1: 0x");
    print_hex_byte(status1);
    printf(" (Busy: %s, WEL: %s)\n",
           (status1 & W25Q_STATUS_BUSY) ? "YES" : "NO",
           (status1 & W25Q_STATUS_WEL) ? "YES" : "NO");
    
    status2 = w25qxx_get_status2();
    printf("Status 2: 0x");
    print_hex_byte(status2);
    printf("\n");
    
    status3 = w25qxx_get_status3();
    printf("Status 3: 0x");
    print_hex_byte(status3);
    printf("\n");
    
    /* Display valid address range */
    printf("\nADDRESS RANGE:\n");
    printf("--------------\n");
    printf("Valid addresses: 0x00000000 to 0x%08lX\n", w25qxx_get_max_address());
    
    /* Display chip state */
    printf("\nCHIP STATE:\n");
    printf("-----------\n");
    printf("Busy: %s\n", w25qxx_is_busy() ? "YES" : "NO");
    printf("Ready for operations: %s\n", w25qxx_is_busy() ? "NO" : "YES");
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
 * Print capacity information in different units
 */
static void print_capacity_info(uint32_t size_bytes)
{
    uint32_t size_kb, size_mb;
    
    size_kb = size_bytes / 1024;
    size_mb = size_kb / 1024;
    
    printf("Capacity: %lu bytes", size_bytes);
    
    if (size_kb > 0) {
        printf(" (%lu KB", size_kb);
        if (size_mb > 0) {
            printf(" / %lu MB", size_mb);
        }
        printf(")");
    }
    printf("\n");
}

/*
 * Print a byte in hexadecimal format (cc65 compatible)
 */
static void print_hex_byte(uint8_t value)
{
    char hex_chars[16] = {'0','1','2','3','4','5','6','7',
                          '8','9','A','B','C','D','E','F'};
    
    putchar(hex_chars[(value >> 4) & 0x0F]);
    putchar(hex_chars[value & 0x0F]);
}
