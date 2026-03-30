/*
 * WD1771 Floppy Disk Controller Emulation
 * Implementation for .IMA disk images with FLEX filesystem
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "wd1771.h"

// Global controller state
static WD1771_State wd1771_state;
static FIL disk_file;
static char disk_filename[32];

char sector_buffer[256];

/*
 * Initialize WD1771 controller with IMA disk image
 */
int wd1771_init(const char *ima_filename) {
    FRESULT res;
    
    memset(&wd1771_state, 0, sizeof(WD1771_State));
    
    // Store filename
    strncpy(disk_filename, ima_filename, sizeof(disk_filename) - 1);
    disk_filename[sizeof(disk_filename) - 1] = '\0';
    wd1771_state.disk_filename = disk_filename;
    wd1771_state.disk_file = &disk_file;
    
    // Open disk image
    res = f_open(&disk_file, ima_filename, FA_READ | FA_WRITE);
    if (res != FR_OK) {
        printf("WD1771: Error opening %s: %d\n", ima_filename, res);
        return -1;
    }
    
    // Read SIR to get disk geometry
    if (wd1771_read_sir_info() < 0) {
        printf("WD1771: Warning - could not read SIR, using defaults\n");
        wd1771_state.max_track = 79;
        wd1771_state.sectors_per_track = 18;
    }
    
    // Initialize registers
    wd1771_state.status_reg = WD1771_STAT_NOT_READY;
    wd1771_state.track_reg = 0xFF;  // Unknown track
    wd1771_state.sector_reg = 1;
    wd1771_state.data_reg = 0;
    wd1771_state.current_track = 0xFF;  // Unknown position
    wd1771_state.write_protected = 0;   // Assume writable
    
    printf("WD1771: Initialized with %s (tracks: %d, sectors: %d)\n", 
           ima_filename, wd1771_state.max_track, wd1771_state.sectors_per_track);
    
    return 0;
}

/*
 * Cleanup WD1771 controller
 */
void wd1771_cleanup(void) {
    if (wd1771_state.disk_file) {
        f_close(wd1771_state.disk_file);
        wd1771_state.disk_file = NULL;
    }
    memset(&wd1771_state, 0, sizeof(WD1771_State));
}

/*
 * Read SIR information to get disk geometry
 */
int wd1771_read_sir_info(void) {
    static uint8_t buffer[256];
    FRESULT res;
    UINT bytes_read;
    uint32_t sir_position;
    
    // SIR is in block 2 (track 0, sector 3)
    sir_position = 2 * 256;
    
    res = f_lseek(wd1771_state.disk_file, sir_position);
    if (res != FR_OK) return -1;
    
    res = f_read(wd1771_state.disk_file, buffer, 256, &bytes_read);
    if (res != FR_OK || bytes_read != 256) return -1;
    
    // Extract geometry from SIR (offset 16 + 21 for max track, +22 for max sector)
    wd1771_state.max_track = buffer[16 + 21];
    wd1771_state.sectors_per_track = buffer[16 + 22];
    
    return 0;
}

/*
 * Calculate absolute position for track/sector in IMA file
 */
uint32_t wd1771_calculate_sector_position(uint8_t track, uint8_t sector) {
    uint32_t position;
    
    if (track == 0) {
        // Track 0 has only 10 sectors (sectors 1-10)
        if (sector < 1 || sector > 10) {
            return 0xFFFFFFFF;  // Invalid sector
        }
        position = (sector - 1) * 256;
    } else {
        // Other tracks are padded and have full sectors_per_track
        if (sector < 1 || sector > wd1771_state.sectors_per_track) {
            return 0xFFFFFFFF;  // Invalid sector
        }
        // Track 0 takes 10 * 256 bytes, then each track takes sectors_per_track * 256
	//        position = (10 * 256) + ((track - 1) * wd1771_state.sectors_per_track * 256) + ((sector - 1) * 256);
	position = track * (wd1771_state.sectors_per_track * 256) + ((sector - 1) * 256);
    }
    
    return position;
}

/*
 * Set status register bit
 */
void wd1771_set_status_bit(uint8_t bit) {
    wd1771_state.status_reg |= bit;
}

/*
 * Clear status register bit
 */
void wd1771_clear_status_bit(uint8_t bit) {
    wd1771_state.status_reg &= ~bit;
}

/*
 * Read from WD1771 register
 */
uint8_t wd1771_read_register(uint8_t reg) {
    switch (reg) {
        case WD1771_STATUS_REG:
            return wd1771_state.status_reg;
        case WD1771_TRACK_REG:
            return wd1771_state.track_reg;
        case WD1771_SECTOR_REG:
            return wd1771_state.sector_reg;
        case WD1771_DATA_REG:
            wd1771_clear_status_bit(WD1771_STAT_DRQ);
            return wd1771_state.data_reg;
        default:
            return 0xFF;
    }
}

/*
 * Write to WD1771 register
 */
void wd1771_write_register(uint8_t reg, uint8_t value) {
    switch (reg) {
        case WD1771_COMMAND_REG:
            wd1771_execute_command(value);
            break;
        case WD1771_TRACK_REG:
            wd1771_state.track_reg = value;
            break;
        case WD1771_SECTOR_REG:
            wd1771_state.sector_reg = value;
            break;
        case WD1771_DATA_REG:
            wd1771_state.data_reg = value;
            wd1771_clear_status_bit(WD1771_STAT_DRQ);
            break;
    }
}

/*
 * Execute WD1771 command
 */
void wd1771_execute_command(uint8_t command) {
    uint8_t cmd_type = command & 0xF0;
    uint8_t flags = command & 0x0F;
    
    // Set BUSY flag
    wd1771_set_status_bit(WD1771_STAT_BUSY);
    wd1771_clear_status_bit(WD1771_STAT_DRQ | WD1771_STAT_LOST_DATA | 
                            WD1771_STAT_CRC_ERROR | WD1771_STAT_SEEK_ERROR | 
                            WD1771_STAT_RNF | WD1771_STAT_WRITE_PROT);
    
    switch (cmd_type) {
        case WD1771_CMD_RESTORE:
            wd1771_cmd_restore(flags);
            break;
        case WD1771_CMD_SEEK:
            wd1771_cmd_seek(flags);
            break;
        case WD1771_CMD_READ_SECTOR:
            wd1771_cmd_read_sector(flags);
            break;
        case WD1771_CMD_WRITE_SECTOR:
            wd1771_cmd_write_sector(flags);
            break;
        default:
            // Unsupported command
            wd1771_clear_status_bit(WD1771_STAT_BUSY);
            break;
    }
}

/*
 * RESTORE command - seek to track 0
 */
void wd1771_cmd_restore(uint8_t flags) {
    // Simulate seeking to track 0
    wd1771_state.current_track = 0;
    wd1771_state.track_reg = 0;
    
    // Clear BUSY flag
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    
    printf("WD1771: RESTORE completed\n");
}

/*
 * SEEK command - seek to track specified in data register
 */
void wd1771_cmd_seek(uint8_t flags) {
    uint8_t target_track = wd1771_state.data_reg;
    
    // Check if target track is valid
    if (target_track > wd1771_state.max_track) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: SEEK error - track %d > max %d\n", target_track, wd1771_state.max_track);
        return;
    }
    
    // Simulate seeking
    wd1771_state.current_track = target_track;
    
    // Update track register if UPDATE_TRACK flag is set
    if (flags & WD1771_FLAG_UPDATE_TRACK) {
        wd1771_state.track_reg = target_track;
    }
    
    // Clear BUSY flag
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    
    printf("WD1771: SEEK to track %d completed\n", target_track);
}

/*
 * STEP command - step one track in current direction
 */
void wd1771_cmd_step(uint8_t flags) {
    // Step rate from flags (bits 0-1)
    // Motor on flag (bit 3) 
    // Update track register flag (bit 4)
    
    // For simplicity, we'll step in the last direction
    // Real hardware remembers the last step direction
    
    // Update track register if flag is set
    if (flags & WD1771_FLAG_UPDATE_TRACK) {
        // We don't know direction, so don't change track register
    }
    
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    printf("WD1771: STEP completed\n");
}

/*
 * STEP IN command - step toward higher track numbers
 */
void wd1771_cmd_step_in(uint8_t flags) {
    // Check if we can step in
    if (wd1771_state.current_track >= wd1771_state.max_track) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: STEP IN error - already at max track %d\n", wd1771_state.max_track);
        return;
    }
    
    // Step in (increment track)
    wd1771_state.current_track++;
    
    // Update track register if flag is set
    if (flags & WD1771_FLAG_UPDATE_TRACK) {
        wd1771_state.track_reg = wd1771_state.current_track;
    }
    
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    printf("WD1771: STEP IN to track %d completed\n", wd1771_state.current_track);
}

/*
 * STEP OUT command - step toward track 0
 */
void wd1771_cmd_step_out(uint8_t flags) {
    // Check if we can step out  
    if (wd1771_state.current_track == 0) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: STEP OUT error - already at track 0\n");
        return;
    }
    
    // Step out (decrement track)
    wd1771_state.current_track--;
    
    // Update track register if flag is set
    if (flags & WD1771_FLAG_UPDATE_TRACK) {
        wd1771_state.track_reg = wd1771_state.current_track;
    }
    
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    printf("WD1771: STEP OUT to track %d completed\n", wd1771_state.current_track);
}

void wd1771_cmd_read_sector(uint8_t flags) {
    uint8_t track = wd1771_state.current_track;
    uint8_t sector = wd1771_state.sector_reg;
    uint32_t position;
    FRESULT res;
    UINT bytes_read;
    //    static uint8_t buffer[256];
    
    // Validate track/sector
    if (track > wd1771_state.max_track) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        return;
    }
    
    // Calculate sector position
    position = wd1771_calculate_sector_position(track, sector);
    if (position == 0xFFFFFFFF) {
        wd1771_set_status_bit(WD1771_STAT_RNF);  // Record Not Found
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: READ SECTOR error - invalid sector %d on track %d\n", sector, track);
        return;
    }
    
    // Read sector from disk image
    res = f_lseek(wd1771_state.disk_file, position);
    if (res != FR_OK) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: READ SECTOR seek error: %d\n", res);
        return;
    }
    
    res = f_read(wd1771_state.disk_file, sector_buffer, 256, &bytes_read);
    if (res != FR_OK || bytes_read != 256) {
        wd1771_set_status_bit(WD1771_STAT_CRC_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: READ SECTOR read error: %d (bytes: %d)\n", res, bytes_read);
        return;
    }
    
    // Store first byte in data register and set DRQ
    wd1771_state.data_reg = sector_buffer[0];
    wd1771_set_status_bit(WD1771_STAT_DRQ);
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    
    printf("WD1771: READ SECTOR T%d:S%d completed\n", track, sector);
}

/*
 * WRITE SECTOR command
 */
void wd1771_cmd_write_sector(uint8_t flags) {
    uint8_t track = wd1771_state.current_track;
    uint8_t sector = wd1771_state.sector_reg;
    uint32_t position;
    FRESULT res;
    UINT bytes_written;
    //    static uint8_t buffer[256];
    
    // Check write protection
    if (wd1771_state.write_protected) {
        wd1771_set_status_bit(WD1771_STAT_WRITE_PROT);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        return;
    }
    
    // Validate track/sector
    if (track > wd1771_state.max_track) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        return;
    }
    
    // Calculate sector position
    position = wd1771_calculate_sector_position(track, sector);
    if (position == 0xFFFFFFFF) {
        wd1771_set_status_bit(WD1771_STAT_RNF);  // Record Not Found
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: WRITE SECTOR error - invalid sector %d on track %d\n", sector, track);
        return;
    }
    
    // For simplicity, fill buffer with data register value
    // In real implementation, this would come from DMA or CPU data transfers
    memset(sector_buffer, wd1771_state.data_reg, 256);
    
    // Write sector to disk image
    res = f_lseek(wd1771_state.disk_file, position);
    if (res != FR_OK) {
        wd1771_set_status_bit(WD1771_STAT_SEEK_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: WRITE SECTOR seek error: %d\n", res);
        return;
    }
    
    res = f_write(wd1771_state.disk_file, sector_buffer, 256, &bytes_written);
    if (res != FR_OK || bytes_written != 256) {
        wd1771_set_status_bit(WD1771_STAT_CRC_ERROR);
        wd1771_clear_status_bit(WD1771_STAT_BUSY);
        printf("WD1771: WRITE SECTOR write error: %d (bytes: %d)\n", res, bytes_written);
        return;
    }
    
    // Sync to ensure data is written
    f_sync(wd1771_state.disk_file);
    
    wd1771_clear_status_bit(WD1771_STAT_BUSY);
    
    printf("WD1771: WRITE SECTOR T%d:S%d completed\n", track, sector);
}
