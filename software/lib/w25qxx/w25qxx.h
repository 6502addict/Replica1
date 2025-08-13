/*
 * File: w25qxx.h
 * Generic W25Qxx SPI Flash Library Header
 * Supports: W25Q16, W25Q32, W25Q64, W25Q128, W25Q256
 * For cc65 compiler compatibility
 */

#ifndef W25QXX_H
#define W25QXX_H

#include <stdint.h>

/* Chip identification values */
#define W25QXX_MFG_ID           0xEF    /* Winbond manufacturer ID */
#define W25QXX_MEM_TYPE         0x40    /* Memory type for all W25Q series */

/* Capacity IDs for different chips */
#define W25Q16_CAPACITY_ID      0x15    /* 16Mbit / 2MB */
#define W25Q32_CAPACITY_ID      0x16    /* 32Mbit / 4MB */
#define W25Q64_CAPACITY_ID      0x17    /* 64Mbit / 8MB */
#define W25Q128_CAPACITY_ID     0x18    /* 128Mbit / 16MB */
#define W25Q256_CAPACITY_ID     0x19    /* 256Mbit / 32MB */

/* Chip type enumeration */
typedef enum {
    W25Q_UNKNOWN = 0,
    W25Q16 = 1,
    W25Q32 = 2,
    W25Q64 = 3,
    W25Q128 = 4,
    W25Q256 = 5
} w25qxx_chip_t;

/* Chip configuration structure */
typedef struct {
    w25qxx_chip_t chip_type;
    uint32_t total_size;        /* Total size in bytes */
    uint16_t total_pages;       /* Total number of pages */
    uint16_t total_sectors;     /* Total number of 4KB sectors */
    uint8_t  total_blocks;      /* Total number of 64KB blocks */
    uint8_t  addr_bytes;        /* Number of address bytes (3 or 4) */
} w25qxx_config_t;

/* Common Flash Parameters */
#define W25QXX_PAGE_SIZE        256         /* 256 bytes per page */
#define W25QXX_SECTOR_SIZE      4096        /* 4KB per sector */
#define W25QXX_BLOCK_SIZE       65536       /* 64KB per block */

/* Flash Commands */
#define W25Q_CMD_WRITE_ENABLE       0x06    /* Write Enable */
#define W25Q_CMD_WRITE_DISABLE      0x04    /* Write Disable */
#define W25Q_CMD_READ_STATUS1       0x05    /* Read Status Register 1 */
#define W25Q_CMD_READ_STATUS2       0x35    /* Read Status Register 2 */
#define W25Q_CMD_READ_STATUS3       0x15    /* Read Status Register 3 */
#define W25Q_CMD_WRITE_STATUS       0x01    /* Write Status Register */
#define W25Q_CMD_PAGE_PROGRAM       0x02    /* Page Program */
#define W25Q_CMD_SECTOR_ERASE       0x20    /* Sector Erase 4KB */
#define W25Q_CMD_BLOCK_ERASE_32K    0x52    /* Block Erase 32KB */
#define W25Q_CMD_BLOCK_ERASE_64K    0xD8    /* Block Erase 64KB */
#define W25Q_CMD_CHIP_ERASE         0xC7    /* Chip Erase */
#define W25Q_CMD_READ_DATA          0x03    /* Read Data */
#define W25Q_CMD_FAST_READ          0x0B    /* Fast Read */
#define W25Q_CMD_JEDEC_ID           0x9F    /* Read JEDEC ID */
#define W25Q_CMD_POWER_DOWN         0xB9    /* Power Down */
#define W25Q_CMD_RELEASE_POWERDOWN  0xAB    /* Release Power Down */

/* Commands for W25Q256 (4-byte addressing) */
#define W25Q_CMD_READ_DATA_4B       0x13    /* Read Data 4-byte address */
#define W25Q_CMD_FAST_READ_4B       0x0C    /* Fast Read 4-byte address */
#define W25Q_CMD_PAGE_PROGRAM_4B    0x12    /* Page Program 4-byte address */
#define W25Q_CMD_SECTOR_ERASE_4B    0x21    /* Sector Erase 4-byte address */
#define W25Q_CMD_BLOCK_ERASE_64K_4B 0xDC    /* Block Erase 64KB 4-byte address */
#define W25Q_CMD_ENTER_4B_MODE      0xB7    /* Enter 4-byte address mode */
#define W25Q_CMD_EXIT_4B_MODE       0xE9    /* Exit 4-byte address mode */

/* Status Register Bits */
#define W25Q_STATUS_BUSY            0x01    /* Busy bit */
#define W25Q_STATUS_WEL             0x02    /* Write Enable Latch */
#define W25Q_STATUS_BP0             0x04    /* Block Protect bit 0 */
#define W25Q_STATUS_BP1             0x08    /* Block Protect bit 1 */
#define W25Q_STATUS_BP2             0x10    /* Block Protect bit 2 */
#define W25Q_STATUS_TB              0x20    /* Top/Bottom Protect */
#define W25Q_STATUS_SEC             0x40    /* Sector Protect */
#define W25Q_STATUS_SRP0            0x80    /* Status Register Protect 0 */

/* Address calculation macros */
#define W25QXX_SECTOR_ADDR(n)   ((uint32_t)(n) * W25QXX_SECTOR_SIZE)
#define W25QXX_BLOCK_ADDR(n)    ((uint32_t)(n) * W25QXX_BLOCK_SIZE)
#define W25QXX_PAGE_ADDR(n)     ((uint32_t)(n) * W25QXX_PAGE_SIZE)

/* Validation and conversion macros */
#define W25QXX_ADDR_TO_SECTOR(addr) ((addr) / W25QXX_SECTOR_SIZE)
#define W25QXX_ADDR_TO_BLOCK(addr)  ((addr) / W25QXX_BLOCK_SIZE)
#define W25QXX_ADDR_TO_PAGE(addr)   ((addr) / W25QXX_PAGE_SIZE)

/* Global configuration - set by init function */
extern w25qxx_config_t w25qxx_config;

/* Function prototypes */

/* Initialization */
uint8_t w25qxx_init(void);
w25qxx_chip_t w25qxx_get_chip_type(void);
uint32_t w25qxx_get_chip_size(void);

/* Low-level chip identification */
uint8_t w25qxx_read_jedec_id(uint8_t* mfg_id, uint8_t* mem_type, uint8_t* capacity);

/* Read functions */
void w25qxx_read(uint32_t address, uint8_t* buffer, uint16_t length);
void w25qxx_fast_read(uint32_t address, uint8_t* buffer, uint16_t length);

/* Write functions */
uint8_t w25qxx_write_page(uint32_t address, const uint8_t* buffer, uint16_t length);
uint8_t w25qxx_write(uint32_t address, const uint8_t* buffer, uint16_t length);

/* Erase functions */
uint8_t w25qxx_erase_sector(uint32_t address);  /* Erase 4KB */
uint8_t w25qxx_erase_block_32k(uint32_t address); /* Erase 32KB */
uint8_t w25qxx_erase_block_64k(uint32_t address); /* Erase 64KB */
uint8_t w25qxx_erase_chip(void);                /* Erase entire chip */

/* Status and control functions */
uint8_t w25qxx_is_busy(void);
uint8_t w25qxx_get_status(void);
uint8_t w25qxx_get_status2(void);
uint8_t w25qxx_get_status3(void);

/* Power management */
void w25qxx_power_down(void);
void w25qxx_wake_up(void);

/* 4-byte addressing mode (for W25Q256) */
uint8_t w25qxx_enter_4byte_mode(void);
uint8_t w25qxx_exit_4byte_mode(void);

/* Utility functions */
uint8_t w25qxx_is_valid_address(uint32_t address);
uint32_t w25qxx_get_max_address(void);

#endif /* W25QXX_H */
