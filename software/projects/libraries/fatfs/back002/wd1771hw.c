/*
 * WD1771 Hardware Interface
 * Memory-mapped interface for WD1771 at $C230
 * Apple 1/Replica 1 compatible
 */

#include <stdio.h>
#include <stdint.h>

/* WD1771 base address */
#define WD1771_BASE     0xC230

/* WD1771 register offsets */
#define WD1771_CMD_STATUS   0   /* Write=Command, Read=Status */
#define WD1771_TRACK        1   /* Track register */
#define WD1771_SECTOR       2   /* Sector register */
#define WD1771_DATA         3   /* Data register */

/* Status register bits */
#define STAT_BUSY       0x01
#define STAT_DRQ        0x02
#define STAT_LOST_DATA  0x04
#define STAT_CRC_ERROR  0x08
#define STAT_SEEK_ERROR 0x10
#define STAT_RNF        0x10    /* Record Not Found (for read/write) */
#define STAT_WRITE_PROT 0x40
#define STAT_NOT_READY  0x80

/* Command codes */
#define CMD_RESTORE     0x00
#define CMD_SEEK        0x10
#define CMD_STEP        0x20
#define CMD_STEP_IN     0x40
#define CMD_STEP_OUT    0x60
#define CMD_READ_SECTOR 0x80
#define CMD_WRITE_SECTOR 0xA0

/* Command flags */
#define FLAG_UPDATE_TRACK 0x10
#define FLAG_MOTOR_ON     0x08
#define FLAG_VERIFY       0x04
#define FLAG_STEP_RATE    0x03

/*
 * Read from WD1771 register
 */
uint8_t wd1771_read(uint8_t reg) {
    volatile uint8_t *addr;
    
    addr = (volatile uint8_t *)(WD1771_BASE + reg);
    return *addr;
}

/*
 * Write to WD1771 register
 */
void wd1771_write(uint8_t reg, uint8_t value) {
    volatile uint8_t *addr;
    
    addr = (volatile uint8_t *)(WD1771_BASE + reg);
    *addr = value;
}

/*
 * Read status register
 */
uint8_t wd1771_get_status(void) {
    return wd1771_read(WD1771_CMD_STATUS);
}

/*
 * Execute command
 */
void wd1771_command(uint8_t cmd) {
    wd1771_write(WD1771_CMD_STATUS, cmd);
}

/*
 * Set track register
 */
void wd1771_set_track(uint8_t track) {
    wd1771_write(WD1771_TRACK, track);
}

/*
 * Get track register
 */
uint8_t wd1771_get_track(void) {
    return wd1771_read(WD1771_TRACK);
}

/*
 * Set sector register
 */
void wd1771_set_sector(uint8_t sector) {
    wd1771_write(WD1771_SECTOR, sector);
}

/*
 * Get sector register
 */
uint8_t wd1771_get_sector(void) {
    return wd1771_read(WD1771_SECTOR);
}

/*
 * Write data register
 */
void wd1771_write_data(uint8_t data) {
    wd1771_write(WD1771_DATA, data);
}

/*
 * Read data register
 */
uint8_t wd1771_read_data(void) {
    return wd1771_read(WD1771_DATA);
}

/*
 * Wait for controller not busy
 */
void wd1771_wait_not_busy(void) {
    while (wd1771_get_status() & STAT_BUSY) {
        /* Wait for command completion */
    }
}

/*
 * Wait for data request
 */
int wd1771_wait_drq(void) {
    uint8_t status;
    int timeout;
    
    timeout = 10000;
    while (timeout-- > 0) {
        status = wd1771_get_status();
        if (status & STAT_DRQ) {
            return 1;  /* DRQ set */
        }
        if (!(status & STAT_BUSY)) {
            return 0;  /* Command completed without DRQ */
        }
    }
    return -1;  /* Timeout */
}

/*
 * RESTORE command - seek to track 0
 */
int wd1771_restore(void) {
    wd1771_command(CMD_RESTORE);
    wd1771_wait_not_busy();
    
    return (wd1771_get_status() & STAT_SEEK_ERROR) ? -1 : 0;
}

/*
 * SEEK command - seek to specified track
 */
int wd1771_seek(uint8_t track) {
    wd1771_write_data(track);
    wd1771_command(CMD_SEEK | FLAG_UPDATE_TRACK);
    wd1771_wait_not_busy();
    
    return (wd1771_get_status() & STAT_SEEK_ERROR) ? -1 : 0;
}

/*
 * STEP IN command
 */
int wd1771_step_in(void) {
    wd1771_command(CMD_STEP_IN | FLAG_UPDATE_TRACK);
    wd1771_wait_not_busy();
    
    return (wd1771_get_status() & STAT_SEEK_ERROR) ? -1 : 0;
}

/*
 * STEP OUT command
 */
int wd1771_step_out(void) {
    wd1771_command(CMD_STEP_OUT | FLAG_UPDATE_TRACK);
    wd1771_wait_not_busy();
    
    return (wd1771_get_status() & STAT_SEEK_ERROR) ? -1 : 0;
}

/*
 * Read sector into buffer
 */
int wd1771_read_sector_buffer(uint8_t track, uint8_t sector, uint8_t *buffer) {
    uint8_t status;
    int i;
    
    /* Set up registers */
    wd1771_set_sector(sector);
    
    /* Seek to track if needed */
    if (wd1771_get_track() != track) {
        if (wd1771_seek(track) < 0) {
            return -1;
        }
    }
    
    /* Start read command */
    wd1771_command(CMD_READ_SECTOR);
    
    /* Read 256 bytes */
    for (i = 0; i < 256; i++) {
        if (wd1771_wait_drq() != 1) {
            break;  /* No more data or error */
        }
        buffer[i] = wd1771_read_data();
    }
    
    /* Wait for completion */
    wd1771_wait_not_busy();
    
    /* Check for errors */
    status = wd1771_get_status();
    if (status & (STAT_CRC_ERROR | STAT_RNF)) {
        return -1;
    }
    
    return i;  /* Return bytes read */
}

/*
 * Write sector from buffer
 */
int wd1771_write_sector_buffer(uint8_t track, uint8_t sector, uint8_t *buffer) {
    uint8_t status;
    int i;
    
    /* Set up registers */
    wd1771_set_sector(sector);
    
    /* Seek to track if needed */
    if (wd1771_get_track() != track) {
        if (wd1771_seek(track) < 0) {
            return -1;
        }
    }
    
    /* Start write command */
    wd1771_command(CMD_WRITE_SECTOR);
    
    /* Write 256 bytes */
    for (i = 0; i < 256; i++) {
        if (wd1771_wait_drq() != 1) {
            break;  /* Controller not ready or error */
        }
        wd1771_write_data(buffer[i]);
    }
    
    /* Wait for completion */
    wd1771_wait_not_busy();
    
    /* Check for errors */
    status = wd1771_get_status();
    if (status & (STAT_CRC_ERROR | STAT_WRITE_PROT)) {
        return -1;
    }
    
    return i;  /* Return bytes written */
}

/*
 * Display status register in human readable form
 */
void wd1771_print_status(void) {
    uint8_t status;
    
    status = wd1771_get_status();
    
    printf("STATUS: 0x%02X [", status);
    if (status & STAT_BUSY) printf("BUSY ");
    if (status & STAT_DRQ) printf("DRQ ");
    if (status & STAT_LOST_DATA) printf("LOST ");
    if (status & STAT_CRC_ERROR) printf("CRC ");
    if (status & STAT_SEEK_ERROR) printf("SEEK ");
    if (status & STAT_RNF) printf("RNF ");
    if (status & STAT_WRITE_PROT) printf("WP ");
    if (status & STAT_NOT_READY) printf("NRDY ");
    printf("]\n");
}

/*
 * Display all registers
 */
void wd1771_print_registers(void) {
    printf("WD1771 REGISTERS:\n");
    wd1771_print_status();
    printf("TRACK:  0x%02X (%d)\n", wd1771_get_track(), wd1771_get_track());
    printf("SECTOR: 0x%02X (%d)\n", wd1771_get_sector(), wd1771_get_sector());
    printf("DATA:   0x%02X\n", wd1771_read_data());
}

/*
 * Simple test program
 */
int main(void) {
    uint8_t buffer[256];
    int i;
    
    printf("WD1771 HARDWARE TEST\n");
    printf("BASE ADDRESS: $%04X\n", WD1771_BASE);
    printf("===================\n\n");
    
    /* Display initial status */
    printf("INITIAL STATE:\n");
    wd1771_print_registers();
    printf("\n");
    
    /* Test RESTORE */
    printf("EXECUTING RESTORE...\n");
    if (wd1771_restore() == 0) {
        printf("RESTORE OK\n");
    } else {
        printf("RESTORE ERROR\n");
    }
    wd1771_print_status();
    printf("\n");
    
    /* Test SEEK */
    printf("SEEKING TO TRACK 5...\n");
    if (wd1771_seek(5) == 0) {
        printf("SEEK OK\n");
    } else {
        printf("SEEK ERROR\n");
    }
    wd1771_print_registers();
    printf("\n");
    
    /* Test READ SECTOR */
    printf("READING TRACK 0, SECTOR 3...\n");
    if (wd1771_read_sector_buffer(0, 3, buffer) > 0) {
        printf("READ OK - FIRST 16 BYTES:\n");
        for (i = 0; i < 16; i++) {
            printf("%02X ", buffer[i]);
            if ((i & 7) == 7) printf(" ");
        }
        printf("\n");
    } else {
        printf("READ ERROR\n");
        wd1771_print_status();
    }
    printf("\n");
    
    /* Test STEP IN */
    printf("STEPPING IN...\n");
    if (wd1771_step_in() == 0) {
        printf("STEP IN OK\n");
    } else {
        printf("STEP IN ERROR\n");
    }
    wd1771_print_registers();
    printf("\n");
    
    printf("WD1771 TEST COMPLETE\n");
    return 0;
}
