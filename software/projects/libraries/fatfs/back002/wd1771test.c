/*
 * WD1771 Test Program
 * Interactive test program for WD1771 controller emulation
 * Based on dskbrowser.c structure
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include "ff.h"
#include "wd1771.h"

#define SECTOR_SIZE 256
#define MAX_COMMAND_LEN 64

/* Global variables */
extern uint8_t sector_buffer[SECTOR_SIZE];
static int buffer_valid = 0;
static FATFS fs;

#if !FF_FS_READONLY && !FF_FS_NORTC
DWORD get_fattime (void)
{
    /* Return a fixed timestamp since we don't have RTC */
    return ((DWORD)(2025 - 1980) << 25)  /* Year = 2025 */
           | ((DWORD)1 << 21)            /* Month = January */
           | ((DWORD)1 << 16)            /* Day = 1 */
           | ((DWORD)12 << 11)           /* Hour = 12 */
           | ((DWORD)0 << 5)             /* Minute = 0 */
           | ((DWORD)0 << 0);            /* Second = 0 */
}
#endif

/*
 * Dump function from dskbrowser.c
 */
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
        for (i = lsize; i < 16; i++)
            printf(" ");
        a += lsize;
        size -= lsize;
        printf("|\n");
        if (size <= 0)
            return;
    }
}

/*
 * Parse hex value from string
 */
int parse_hex(const char *str, uint8_t *value) {
    char *endptr;
    unsigned long val;
    
    val = strtoul(str, &endptr, 16);
    
    if (*endptr != '\0' || val > 0xFF) {
        return -1;
    }
    
    *value = (uint8_t)val;
    return 0;
}

/*
 * Parse decimal value from string
 */
int parse_decimal(const char *str, uint8_t *value) {
    char *endptr;
    unsigned long val;
    
    val = strtoul(str, &endptr, 10);
    
    if (*endptr != '\0' || val > 255) {
        return -1;
    }
    
    *value = (uint8_t)val;
    return 0;
}

/*
 * Read complete sector after READ SECTOR command
 */
void read_complete_sector(void) {
    int i;
    uint8_t status;
    
    /* Check if DRQ is set */
    status = wd1771_read_register(WD1771_STATUS_REG);
    if (!(status & WD1771_STAT_DRQ)) {
        printf("No data available (DRQ not set)\n");
        return;
    }
    
    /* Read first byte (already in data register) */
    //sector_buffer[0] = wd1771_read_register(WD1771_DATA_REG);
    
    /* Simulate reading remaining 255 bytes */
    //for (i = 1; i < SECTOR_SIZE; i++) {
    //        sector_buffer[i] = 0x00; /* Placeholder */
    //}
    
    buffer_valid = 1;
    
    printf("Sector read into buffer:\n");
    dump(sector_buffer, SECTOR_SIZE);
}

/*
 * Write sector from buffer
 */
void write_sector_from_buffer(void) {
    int i;
    
    if (!buffer_valid) {
        printf("No valid data in buffer. Use 'read' command first.\n");
        return;
    }
    
    /* Write first byte to data register */
    wd1771_write_register(WD1771_DATA_REG, sector_buffer[0]);
    
    /* Execute write command */
    wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_WRITE_SECTOR);
    
    /* Simulate writing remaining bytes */
    for (i = 1; i < SECTOR_SIZE; i++) {
        wd1771_write_register(WD1771_DATA_REG, sector_buffer[i]);
    }
    
    printf("Buffer written to sector\n");
}

/*
 * Display help
 */
void show_help(void) {
    printf("\nWD1771 Test Commands:\n");
    printf("=====================\n");
    printf("status          - Display status register\n");
    printf("track <n>       - Set track register (0-255)\n");
    printf("sector <n>      - Set sector register (1-255)\n");
    printf("data <hex>      - Set data register (00-FF hex)\n");
    printf("cmd <hex>       - Execute command (hex value)\n");
    printf("restore         - Execute RESTORE command\n");
    printf("seek <track>    - Execute SEEK to track\n");
    printf("step            - Execute STEP command\n");
    printf("stepin          - Execute STEP IN command\n");
    printf("stepout         - Execute STEP OUT command\n");
    printf("read            - Execute READ SECTOR and display\n");
    printf("write           - Execute WRITE SECTOR from buffer\n");
    printf("dump            - Display current sector buffer\n");
    printf("regs            - Display all registers\n");
    printf("help            - Show this help\n");
    printf("quit            - Exit program\n");
    printf("\nCommand codes:\n");
    printf("00-0F: RESTORE    10-1F: SEEK      20-2F: STEP\n");
    printf("40-4F: STEP IN    60-6F: STEP OUT\n");
    printf("80-8F: READ SEC   A0-AF: WRITE SEC\n");
}

/*
 * Display register status
 */
void show_registers(void) {
    uint8_t status, track, sector, data;
    
    status = wd1771_read_register(WD1771_STATUS_REG);
    track = wd1771_read_register(WD1771_TRACK_REG);
    sector = wd1771_read_register(WD1771_SECTOR_REG);
    data = wd1771_read_register(WD1771_DATA_REG);
    
    printf("Registers:\n");
    printf("  Status: 0x%02X ", status);
    printf("[%s%s%s%s%s%s%s%s]\n",
           (status & WD1771_STAT_BUSY) ? "BUSY " : "",
           (status & WD1771_STAT_DRQ) ? "DRQ " : "",
           (status & WD1771_STAT_LOST_DATA) ? "LOST " : "",
           (status & WD1771_STAT_CRC_ERROR) ? "CRC " : "",
           (status & WD1771_STAT_SEEK_ERROR) ? "SEEK_ERR " : "",
           (status & WD1771_STAT_RNF) ? "RNF " : "",
           (status & WD1771_STAT_WRITE_PROT) ? "WP " : "",
           (status & WD1771_STAT_NOT_READY) ? "NRDY " : "");
    printf("  Track:  0x%02X (%d)\n", track, track);
    printf("  Sector: 0x%02X (%d)\n", sector, sector);
    printf("  Data:   0x%02X\n", data);
}

/*
 * Process command line
 */
void process_command(char *command) {
    char *cmd, *arg;
    uint8_t value;
    char *p;
    
    /* Split command and argument */
    cmd = strtok(command, " \t\n");
    if (!cmd) return;
    
    arg = strtok(NULL, " \t\n");
    
    /* Convert command to lowercase */
    for (p = cmd; *p; p++) {
        *p = tolower(*p);
    }

    printf("cmd := '%s'\n", cmd);
    
    if (strcasecmp(cmd, "STATUS") == 0) {
        printf("Status: 0x%02X\n", wd1771_read_register(WD1771_STATUS_REG));
        
    } else if (strcasecmp(cmd, "TRACK") == 0) {
        if (!arg || parse_decimal(arg, &value) < 0) {
            printf("Usage: track <0-255>\n");
            return;
        }
        wd1771_write_register(WD1771_TRACK_REG, value);
        printf("Track register set to %d\n", value);
        
    } else if (strcasecmp(cmd, "SECTOR") == 0) {
        if (!arg || parse_decimal(arg, &value) < 0) {
            printf("Usage: sector <1-255>\n");
            return;
        }
        wd1771_write_register(WD1771_SECTOR_REG, value);
        printf("Sector register set to %d\n", value);
        
    } else if (strcasecmp(cmd, "DATA") == 0) {
        if (!arg || parse_hex(arg, &value) < 0) {
            printf("Usage: data <hex value 00-FF>\n");
            return;
        }
        wd1771_write_register(WD1771_DATA_REG, value);
        printf("Data register set to 0x%02X\n", value);
        
    } else if (strcasecmp(cmd, "CMD") == 0) {
        if (!arg || parse_hex(arg, &value) < 0) {
            printf("Usage: cmd <hex command 00-FF>\n");
            return;
        }
        printf("Executing command 0x%02X\n", value);
        wd1771_write_register(WD1771_COMMAND_REG, value);
        
    } else if (strcasecmp(cmd, "RESTORE") == 0) {
        printf("Executing RESTORE command\n");
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_RESTORE);
        
    } else if (strcasecmp(cmd, "SEEK") == 0) {
        if (!arg || parse_decimal(arg, &value) < 0) {
            printf("Usage: seek <track 0-255>\n");
            return;
        }
        printf("Seeking to track %d\n", value);
        wd1771_write_register(WD1771_DATA_REG, value);
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_SEEK | WD1771_FLAG_UPDATE_TRACK);
        
    } else if (strcasecmp(cmd, "STEP") == 0) {
        printf("Executing STEP command\n");
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_STEP | WD1771_FLAG_UPDATE_TRACK);
        
    } else if (strcasecmp(cmd, "STEPIN") == 0) {
        printf("Executing STEP IN command\n");
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_STEP_IN | WD1771_FLAG_UPDATE_TRACK);
        
    } else if (strcasecmp(cmd, "STEPOUT") == 0) {
        printf("Executing STEP OUT command\n");
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_STEP_OUT | WD1771_FLAG_UPDATE_TRACK);
        
    } else if (strcasecmp(cmd, "READ") == 0) {
        printf("Executing READ SECTOR command\n");
        wd1771_write_register(WD1771_COMMAND_REG, WD1771_CMD_READ_SECTOR);
        read_complete_sector();
        
    } else if (strcasecmp(cmd, "WRITE") == 0) {
        printf("Executing WRITE SECTOR command\n");
        write_sector_from_buffer();
        
    } else if (strcasecmp(cmd, "DUMP") == 0) {
        if (buffer_valid) {
            printf("Current sector buffer:\n");
            dump(sector_buffer, SECTOR_SIZE);
        } else {
            printf("No valid data in buffer\n");
        }
        
    } else if (strcasecmp(cmd, "REGS") == 0) {
        show_registers();
        
    } else if (strcasecmp(cmd, "HELP") == 0) {
        show_help();
        
    } else if (strcasecmp(cmd, "QUIT") == 0 || strcasecmp(cmd, "EXIT") == 0) {
        /* Will be handled by main loop */
        
    } else {
        printf("Unknown command: %s\n", cmd);
        printf("Type 'help' for available commands\n");
    }
}

/*
 * Main program
 */
int main(void) {
    FRESULT res;
    char command[MAX_COMMAND_LEN];
    
    printf("WD1771 Controller Test Program\n");
    printf("==============================\n");
    
    /* Mount filesystem */
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("Error mounting filesystem: %d\n", res);
        return -1;
    }
    
    /* Initialize WD1771 with FLEX9.IMA */
    if (wd1771_init("FLEX9.IMA") < 0) {
        printf("Failed to initialize WD1771 controller\n");
        f_unmount("");
        return -1;
    }
    
    printf("WD1771 controller initialized with FLEX9.IMA\n");
    printf("Type 'help' for available commands\n\n");
    
    /* Main command loop */
    while (1) {
        printf("WD1771> ");
        fflush(stdout);
        
        if (!fgets(command, sizeof(command), stdin)) {
            break;
        }
        
        /* Remove newline */
        command[strcspn(command, "\n")] = '\0';
        
        /* Skip empty commands */
        if (strlen(command) == 0) {
            continue;
        }
        
        /* Check for quit */
        if (strncmp(command, "quit", 4) == 0 || strncmp(command, "exit", 4) == 0) {
            break;
        }
        
        /* Process command */
        process_command(command);
        printf("\n");
    }
    
    /* Cleanup */
    wd1771_cleanup();
    f_unmount("");
    printf("Goodbye!\n");
    return 0;
}
