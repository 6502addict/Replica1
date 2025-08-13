/*
 * W25Qxx Flash Library
 * Generic read/write/erase functions for W25Q16/32/64/128/256 SPI Flash
 * For Apple 1 with cc65 compiler - cc65 compatible (no variable init in blocks)
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "spi.h"
#include "w25qxx.h"

/* Global configuration structure */
w25qxx_config_t w25qxx_config;

/* Internal function prototypes */
static void w25qxx_wait_ready(void);
static uint8_t w25qxx_read_status_reg(uint8_t cmd);
static void w25qxx_write_enable(void);
static void w25qxx_send_address(uint32_t address);
static uint8_t w25qxx_detect_chip(uint8_t capacity_id);

/* Chip configuration lookup table */
static const w25qxx_config_t chip_configs[6] = {
    /* W25Q_UNKNOWN */
    {W25Q_UNKNOWN, 0, 0, 0, 0, 3},
    /* W25Q16 - 2MB */
    {W25Q16, 2097152UL, 8192, 512, 32, 3},
    /* W25Q32 - 4MB */
    {W25Q32, 4194304UL, 16384, 1024, 64, 3},
    /* W25Q64 - 8MB */
    {W25Q64, 8388608UL, 32768, 2048, 128, 3},
    /* W25Q128 - 16MB */
    {W25Q128, 16777216UL, 65536, 4096, 256, 3},
    /* W25Q256 - 32MB */
    {W25Q256, 33554432UL, 131072, 8192, 512, 4}
};

/*
 * Initialize the W25Qxx flash chip
 * Returns 0 on success, 1 on failure
 */
uint8_t w25qxx_init(void)
{
    uint8_t mfg_id, mem_type, capacity;
    w25qxx_chip_t detected_chip;
    
    /* Initialize SPI - Mode 0, ~1MHz */
    spi_init(2, 0, 0);
    
    /* Read JEDEC ID to identify chip */
    if (w25qxx_read_jedec_id(&mfg_id, &mem_type, &capacity) != 0) {
        w25qxx_config = chip_configs[0]; /* Unknown chip */
        return 1;
    }
    
    /* Verify manufacturer and memory type */
    if (mfg_id != W25QXX_MFG_ID || mem_type != W25QXX_MEM_TYPE) {
        w25qxx_config = chip_configs[0]; /* Unknown chip */
        return 1;
    }
    
    /* Detect chip type from capacity ID */
    detected_chip = w25qxx_detect_chip(capacity);
    if (detected_chip == W25Q_UNKNOWN) {
        w25qxx_config = chip_configs[0]; /* Unknown chip */
        return 1;
    }
    
    /* Copy configuration for detected chip */
    w25qxx_config = chip_configs[detected_chip];
    
    /* For W25Q256, enter 4-byte addressing mode */
    if (detected_chip == W25Q256) {
        w25qxx_enter_4byte_mode();
    }
    
    return 0; /* Success */
}

/*
 * Read JEDEC ID from chip
 * Returns 0 on success, 1 on failure
 */
uint8_t w25qxx_read_jedec_id(uint8_t* mfg_id, uint8_t* mem_type, uint8_t* capacity)
{
    spi_cs_low();
    spi_transfer(W25Q_CMD_JEDEC_ID);
    *mfg_id = spi_transfer(0x00);
    *mem_type = spi_transfer(0x00);
    *capacity = spi_transfer(0x00);
    spi_cs_high();
    
    /* Basic sanity check */
    if (*mfg_id == 0x00 || *mfg_id == 0xFF) {
        return 1; /* Communication error */
    }
    
    return 0;
}

/*
 * Get current chip type
 */
w25qxx_chip_t w25qxx_get_chip_type(void)
{
    return w25qxx_config.chip_type;
}

/*
 * Get chip size in bytes
 */
uint32_t w25qxx_get_chip_size(void)
{
    return w25qxx_config.total_size;
}

/*
 * Read data from flash
 */
void w25qxx_read(uint32_t address, uint8_t* buffer, uint16_t length)
{
    uint16_t i;
    
    spi_cs_low();
    
    /* Send appropriate read command based on chip type */
    if (w25qxx_config.chip_type == W25Q256) {
        spi_transfer(W25Q_CMD_READ_DATA_4B);
    } else {
        spi_transfer(W25Q_CMD_READ_DATA);
    }
    
    /* Send address */
    w25qxx_send_address(address);
    
    /* Read data */
    for (i = 0; i < length; i++) {
        buffer[i] = spi_transfer(0x00);
    }
    
    spi_cs_high();
}

/*
 * Fast read data from flash (with dummy byte)
 */
void w25qxx_fast_read(uint32_t address, uint8_t* buffer, uint16_t length)
{
    uint16_t i;
    
    spi_cs_low();
    
    /* Send appropriate fast read command based on chip type */
    if (w25qxx_config.chip_type == W25Q256) {
        spi_transfer(W25Q_CMD_FAST_READ_4B);
    } else {
        spi_transfer(W25Q_CMD_FAST_READ);
    }
    
    /* Send address */
    w25qxx_send_address(address);
    
    /* Send dummy byte */
    spi_transfer(0x00);
    
    /* Read data */
    for (i = 0; i < length; i++) {
        buffer[i] = spi_transfer(0x00);
    }
    
    spi_cs_high();
}

/*
 * Write a page of data (up to 256 bytes)
 * address: must be page-aligned for optimal performance
 * Returns 0 on success, 1 on failure
 */
uint8_t w25qxx_write_page(uint32_t address, const uint8_t* buffer, uint16_t length)
{
    uint16_t i;
    
    if (length > W25QXX_PAGE_SIZE) {
        return 1; /* Too many bytes */
    }
    
    if (!w25qxx_is_valid_address(address)) {
        return 1; /* Invalid address */
    }
    
    /* Enable writing */
    w25qxx_write_enable();
    
    spi_cs_low();
    
    /* Send appropriate page program command */
    if (w25qxx_config.chip_type == W25Q256) {
        spi_transfer(W25Q_CMD_PAGE_PROGRAM_4B);
    } else {
        spi_transfer(W25Q_CMD_PAGE_PROGRAM);
    }
    
    /* Send address */
    w25qxx_send_address(address);
    
    /* Write data */
    for (i = 0; i < length; i++) {
        spi_transfer(buffer[i]);
    }
    
    spi_cs_high();
    
    /* Wait for write to complete */
    w25qxx_wait_ready();
    
    return 0; /* Success */
}

/*
 * Write data of any length (handles page boundaries)
 */
uint8_t w25qxx_write(uint32_t address, const uint8_t* buffer, uint16_t length)
{
    uint16_t page_offset, page_remaining, write_length;
    uint16_t bytes_written;
    
    bytes_written = 0;
    
    while (bytes_written < length) {
        /* Calculate how many bytes we can write in current page */
        page_offset = address & 0xFF; /* Offset within current page */
        page_remaining = W25QXX_PAGE_SIZE - page_offset;
        
        if ((length - bytes_written) < page_remaining) {
            write_length = length - bytes_written;
        } else {
            write_length = page_remaining;
        }
        
        /* Write this chunk */
        if (w25qxx_write_page(address, buffer + bytes_written, write_length) != 0) {
            return 1; /* Write failed */
        }
        
        /* Update for next iteration */
        address += write_length;
        bytes_written += write_length;
    }
    
    return 0; /* Success */
}

/*
 * Erase a 4KB sector
 */
uint8_t w25qxx_erase_sector(uint32_t address)
{
    if (!w25qxx_is_valid_address(address)) {
        return 1; /* Invalid address */
    }
    
    /* Enable writing */
    w25qxx_write_enable();
    
    spi_cs_low();
    
    /* Send appropriate sector erase command */
    if (w25qxx_config.chip_type == W25Q256) {
        spi_transfer(W25Q_CMD_SECTOR_ERASE_4B);
    } else {
        spi_transfer(W25Q_CMD_SECTOR_ERASE);
    }
    
    /* Send address */
    w25qxx_send_address(address);
    
    spi_cs_high();
    
    /* Wait for erase to complete (can take up to 400ms) */
    w25qxx_wait_ready();
    
    return 0; /* Success */
}

/*
 * Erase a 32KB block
 */
uint8_t w25qxx_erase_block_32k(uint32_t address)
{
    if (!w25qxx_is_valid_address(address)) {
        return 1; /* Invalid address */
    }
    
    /* Enable writing */
    w25qxx_write_enable();
    
    spi_cs_low();
    spi_transfer(W25Q_CMD_BLOCK_ERASE_32K);
    
    /* Send address (3-byte only, 32K erase not available in 4-byte mode) */
    spi_transfer((address >> 16) & 0xFF);  /* A23-A16 */
    spi_transfer((address >> 8) & 0xFF);   /* A15-A8 */
    spi_transfer(address & 0xFF);          /* A7-A0 */
    
    spi_cs_high();
    
    /* Wait for erase to complete (can take up to 1 second) */
    w25qxx_wait_ready();
    
    return 0; /* Success */
}

/*
 * Erase a 64KB block
 */
uint8_t w25qxx_erase_block_64k(uint32_t address)
{
    if (!w25qxx_is_valid_address(address)) {
        return 1; /* Invalid address */
    }
    
    /* Enable writing */
    w25qxx_write_enable();
    
    spi_cs_low();
    
    /* Send appropriate block erase command */
    if (w25qxx_config.chip_type == W25Q256) {
        spi_transfer(W25Q_CMD_BLOCK_ERASE_64K_4B);
    } else {
        spi_transfer(W25Q_CMD_BLOCK_ERASE_64K);
    }
    
    /* Send address */
    w25qxx_send_address(address);
    
    spi_cs_high();
    
    /* Wait for erase to complete (can take up to 2 seconds) */
    w25qxx_wait_ready();
    
    return 0; /* Success */
}

/*
 * Erase entire chip (WARNING: This erases everything!)
 */
uint8_t w25qxx_erase_chip(void)
{
    /* Enable writing */
    w25qxx_write_enable();
    
    spi_cs_low();
    spi_transfer(W25Q_CMD_CHIP_ERASE);
    spi_cs_high();
    
    /* Wait for erase to complete (can take up to 100 seconds for W25Q256) */
    w25qxx_wait_ready();
    
    return 0; /* Success */
}

/*
 * Check if flash is busy
 */
uint8_t w25qxx_is_busy(void)
{
    return (w25qxx_read_status_reg(W25Q_CMD_READ_STATUS1) & W25Q_STATUS_BUSY) ? 1 : 0;
}

/*
 * Get flash status register 1
 */
uint8_t w25qxx_get_status(void)
{
    return w25qxx_read_status_reg(W25Q_CMD_READ_STATUS1);
}

/*
 * Get flash status register 2
 */
uint8_t w25qxx_get_status2(void)
{
    return w25qxx_read_status_reg(W25Q_CMD_READ_STATUS2);
}

/*
 * Get flash status register 3
 */
uint8_t w25qxx_get_status3(void)
{
    return w25qxx_read_status_reg(W25Q_CMD_READ_STATUS3);
}

/*
 * Power down the flash chip
 */
void w25qxx_power_down(void)
{
    spi_cs_low();
    spi_transfer(W25Q_CMD_POWER_DOWN);
    spi_cs_high();
}

/*
 * Wake up the flash chip from power down
 */
void w25qxx_wake_up(void)
{
    spi_cs_low();
    spi_transfer(W25Q_CMD_RELEASE_POWERDOWN);
    spi_cs_high();
}

/*
 * Enter 4-byte addressing mode (for W25Q256)
 */
uint8_t w25qxx_enter_4byte_mode(void)
{
    if (w25qxx_config.chip_type != W25Q256) {
        return 1; /* Not supported on this chip */
    }
    
    spi_cs_low();
    spi_transfer(W25Q_CMD_ENTER_4B_MODE);
    spi_cs_high();
    
    return 0;
}

/*
 * Exit 4-byte addressing mode (for W25Q256)
 */
uint8_t w25qxx_exit_4byte_mode(void)
{
    if (w25qxx_config.chip_type != W25Q256) {
        return 1; /* Not supported on this chip */
    }
    
    spi_cs_low();
    spi_transfer(W25Q_CMD_EXIT_4B_MODE);
    spi_cs_high();
    
    return 0;
}

/*
 * Check if address is valid for current chip
 */
uint8_t w25qxx_is_valid_address(uint32_t address)
{
    return (address < w25qxx_config.total_size) ? 1 : 0;
}

/*
 * Get maximum valid address for current chip
 */
uint32_t w25qxx_get_max_address(void)
{
    return w25qxx_config.total_size - 1;
}

/* Internal helper functions */

static uint8_t w25qxx_read_status_reg(uint8_t cmd)
{
    uint8_t status;
    
    spi_cs_low();
    spi_transfer(cmd);
    status = spi_transfer(0x00);
    spi_cs_high();
    
    return status;
}

static void w25qxx_write_enable(void)
{
    spi_cs_low();
    spi_transfer(W25Q_CMD_WRITE_ENABLE);
    spi_cs_high();
}

static void w25qxx_wait_ready(void)
{
    while (w25qxx_read_status_reg(W25Q_CMD_READ_STATUS1) & W25Q_STATUS_BUSY) {
        /* Wait for operation to complete */
        /* Could add timeout here for safety */
    }
}

static void w25qxx_send_address(uint32_t address)
{
    if (w25qxx_config.addr_bytes == 4) {
        /* 4-byte addressing for W25Q256 */
        spi_transfer((address >> 24) & 0xFF);  /* A31-A24 */
        spi_transfer((address >> 16) & 0xFF);  /* A23-A16 */
        spi_transfer((address >> 8) & 0xFF);   /* A15-A8 */
        spi_transfer(address & 0xFF);          /* A7-A0 */
    } else {
        /* 3-byte addressing for other chips */
        spi_transfer((address >> 16) & 0xFF);  /* A23-A16 */
        spi_transfer((address >> 8) & 0xFF);   /* A15-A8 */
        spi_transfer(address & 0xFF);          /* A7-A0 */
    }
}

static uint8_t w25qxx_detect_chip(uint8_t capacity_id)
{
    switch (capacity_id) {
        case W25Q16_CAPACITY_ID:
            return W25Q16;
        case W25Q32_CAPACITY_ID:
            return W25Q32;
        case W25Q64_CAPACITY_ID:
            return W25Q64;
        case W25Q128_CAPACITY_ID:
            return W25Q128;
        case W25Q256_CAPACITY_ID:
            return W25Q256;
        default:
            return W25Q_UNKNOWN;
    }
}

