#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <conio.h>
#include <spi.h>
#include <sdcard.h>

void dump(uint8_t *buffer, size_t size) {
    uint32_t i, a, lsize;
    int c;

    for (a = 0;;) {
        printf("%08X: ", a);
        lsize = (size >= 16) ? 16 : size;
        for (i = 0; i < lsize; i++)
            printf("%02X ", buffer[a + i]);
        for (i = lsize; i < 16; i++)
            printf("   ");
        printf("  |");
        for (i = 0; i < lsize; i++) {
            c = buffer[a + i];
            printf("%c", ((c < 0x20) || (c > 126)) ? '.' : c);
        }
        for (i = size; i < 32; i++)
            printf(" ");
        a += lsize;
        size -= lsize;
        printf("|\n");
        if (size <= 0)
            return;
    }
}


/*
 * Display first 16 bytes of a block in hex
 */
void display_block_header(uint8_t *buffer)
{
    uint8_t i;
    
    for (i = 0; i < 16; i++) {
        printf("%02X ", buffer[i]);
        if ((i % 8) == 7) {
            printf("\n");
        }
    }
    if ((i % 8) != 0) {
        printf("\n");
    }
}

/*
 * Test read/write functions with error code handling
 */
void test_sd_read_write(void)
{
    static uint8_t buffer[SD_BLOCK_SIZE];
    static uint8_t test_buffer[SD_BLOCK_SIZE];
    unsigned int i;
    uint8_t result;
    
    printf("Testing SD card read/write...\n");
    
    /* Fill test buffer with pattern */
    for (i = 0; i < SD_BLOCK_SIZE; i++) {
        test_buffer[i] = (uint8_t)(i & 0xFF);
    }
    
    /* Read block 0 (MBR) first to see what's there */
    printf("\nReading MBR (block 0)...\n");
    result = sd_read(0, buffer);
    
    if (result == SD_SUCCESS) {
        printf("MBR read successful\n");
        printf("MBR signature: %02X %02X\n", buffer[510], buffer[511]);
        if (buffer[510] == 0x55 && buffer[511] == 0xAA) {
            printf("Valid MBR found!\n");
        } else {
            printf("Invalid MBR signature\n");
        }
        
        /* Display first 16 bytes of MBR */
        printf("MBR header:\n");
        display_block_header(buffer);
    } else {
        printf("MBR read failed: %s (code 0x%02X)\n", 
               sd_error_string(result), result);
        return;  /* Don't continue if we can't read */
    }
    
    /* Test with a safe block number (don't overwrite MBR!) */
    /* Use block 1000 as test block */
    printf("\nTesting write to block 1000...\n");
    result = sd_write(1000, test_buffer);
    
    if (result != SD_SUCCESS) {
        printf("Write test failed: %s (code 0x%02X)\n", 
               sd_error_string(result), result);
        return;
    }
    printf("Write successful\n");
    
    /* Clear buffer and read back */
    for (i = 0; i < SD_BLOCK_SIZE; i++) {
        buffer[i] = 0x00;
    }
    
    printf("Reading back block 1000...\n");
    result = sd_read(1000, buffer);
    
    if (result != SD_SUCCESS) {
        printf("Read test failed: %s (code 0x%02X)\n", 
               sd_error_string(result), result);
        return;
    }
    printf("Read successful\n");
    
    /* Verify data */
    for (i = 0; i < SD_BLOCK_SIZE; i++) {
        if (buffer[i] != test_buffer[i]) {
            printf("Data mismatch at byte %u: wrote %02X, read %02X\n", 
                   i, test_buffer[i], buffer[i]);
            return;
        }
    }
    printf("Read/write test PASSED!\n");
    
    /* Show first 16 bytes of the test pattern we wrote */
    printf("Test data pattern (first 16 bytes):\n");
    display_block_header(buffer);
}


/*
 * Initialize SD card with detailed error reporting
 */
uint8_t init_sd_card(void)
{
    uint8_t result;
    
    printf("Starting SD card initialization...\n");
    
    /* Send initial clocks */
    printf("Sending initial clock cycles...\n");
    
    /* Initialize SD card */
    result = sd_init();
    
    if (result == SD_SUCCESS) {
        printf("SD card initialization successful!\n");
        return SD_SUCCESS;
    } else {
        printf("SD card initialization failed: %s (code 0x%02X)\n", 
               sd_error_string(result), result);
        return result;
    }
}

/*
 * Main program with comprehensive error handling
 */
int main(void)
{
    uint8_t init_result;
    
    printf("SD Card Test Program\n");
    printf("===================\n");
    
    /* Initialize SPI */
    printf("Initializing SPI interface...\n");
    spi_init(0x08, 0, 0);   /* divisor 8, CPOL=0, CPHA=0 */
    spi_cs_low();
    
    /* Initialize SD card */
    init_result = init_sd_card();
    
    if (init_result == SD_SUCCESS) {
        /* Switch to faster SPI speed for data operations */
        printf("Switching to faster SPI speed...\n");
        spi_set_divisor(0x00);  /* Fastest speed */
        
        /* Run read/write tests */
        test_sd_read_write();
    } else {
        printf("Cannot proceed with tests due to initialization failure\n");
        
        /* Provide some diagnostic information */
        printf("\nDiagnostic suggestions:\n");
        switch (init_result) {
            case SD_ERROR_CMD0:
                printf("- Check SPI connections\n");
                printf("- Verify SD card is properly inserted\n");
                printf("- Check power supply to SD card\n");
                break;
            case SD_ERROR_CMD8:
            case SD_ERROR_UNKNOWN_CMD8:
                printf("- SD card may not support SDHC\n");
                printf("- Check voltage levels\n");
                break;
            case SD_ERROR_V1_CARD:
                printf("- SD v1.x cards are not supported by this code\n");
                printf("- Use an SDHC card instead\n");
                break;
            case SD_ERROR_ACMD41_TIMEOUT:
            case SD_ERROR_CMD55:
                printf("- Card may be defective\n");
                printf("- Try a different SD card\n");
                printf("- Check SPI timing\n");
                break;
            default:
                printf("- Check all connections\n");
                printf("- Verify SPI interface is working\n");
                break;
        }
    }
    
    /* Clean up */
    spi_cs_high();
    fflush(stdout);
    
    printf("\nProgram complete. Final result: %s\n", 
           sd_error_string(init_result));
    
    return (init_result == SD_SUCCESS) ? 0 : 1;
}
