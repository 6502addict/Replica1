/*
 * WD1771 Floppy Disk Controller Emulation
 * Header file with register definitions and function prototypes
 */

#ifndef WD1771_H
#define WD1771_H

#include <stdint.h>
#include "ff.h"

// WD1771 Register addresses
#define WD1771_STATUS_REG    0
#define WD1771_COMMAND_REG   0
#define WD1771_TRACK_REG     1
#define WD1771_SECTOR_REG    2
#define WD1771_DATA_REG      3

// Status register bits
#define WD1771_STAT_BUSY        0x01
#define WD1771_STAT_DRQ         0x02
#define WD1771_STAT_LOST_DATA   0x04
#define WD1771_STAT_CRC_ERROR   0x08
#define WD1771_STAT_SEEK_ERROR  0x10
#define WD1771_STAT_RNF         0x10    // Record Not Found (for read/write)
#define WD1771_STAT_WRITE_PROT  0x40
#define WD1771_STAT_NOT_READY   0x80

// Command codes
#define WD1771_CMD_RESTORE      0x00
#define WD1771_CMD_SEEK         0x10
#define WD1771_CMD_STEP         0x20
#define WD1771_CMD_STEP_IN      0x40
#define WD1771_CMD_STEP_OUT     0x60
#define WD1771_CMD_READ_SECTOR  0x80
#define WD1771_CMD_READ_MULTI   0x90
#define WD1771_CMD_WRITE_SECTOR 0xA0
#define WD1771_CMD_WRITE_MULTI  0xB0
#define WD1771_CMD_READ_ADDRESS 0xC0
#define WD1771_CMD_READ_TRACK   0xE0
#define WD1771_CMD_WRITE_TRACK  0xF0

// Command flag bits
#define WD1771_FLAG_UPDATE_TRACK 0x10
#define WD1771_FLAG_MOTOR_ON     0x08
#define WD1771_FLAG_VERIFY       0x04
#define WD1771_FLAG_STEP_RATE    0x03

// WD1771 controller state structure
typedef struct {
    uint8_t status_reg;
    uint8_t track_reg;
    uint8_t sector_reg;
    uint8_t data_reg;
    uint8_t current_track;
    uint8_t max_track;
    uint8_t sectors_per_track;
    uint8_t write_protected;
    FIL *disk_file;
    char *disk_filename;
} WD1771_State;

// Function prototypes
int wd1771_init(const char *ima_filename);
void wd1771_cleanup(void);
uint8_t wd1771_read_register(uint8_t reg);
void wd1771_write_register(uint8_t reg, uint8_t value);
void wd1771_execute_command(uint8_t command);

// Command implementations
void wd1771_cmd_restore(uint8_t flags);
void wd1771_cmd_seek(uint8_t flags);
void wd1771_cmd_read_sector(uint8_t flags);
void wd1771_cmd_write_sector(uint8_t flags);

// Helper functions
uint32_t wd1771_calculate_sector_position(uint8_t track, uint8_t sector);
int wd1771_read_sir_info(void);
void wd1771_set_status_bit(uint8_t bit);
void wd1771_clear_status_bit(uint8_t bit);

#endif // WD1771_H
