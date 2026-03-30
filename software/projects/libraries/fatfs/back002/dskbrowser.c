/*
 * FLEX Disk Image Explorer
 * C89/cc65 compatible disk image browser
 * Supports .DSK and .IMA files with FLEX filesystem
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "ff.h"

/* FLEX constants from your flexdisk.h */
#define FLEX_SIR_TRACK    0
#define FLEX_SIR_SECTOR   3  
#define FLEX_SIR_OFFSET   16
#define FLEX_DIR_TRACK    0
#define FLEX_DIR_SECTOR   5
#define FLEX_DIR_OFFSET   16
#define FLEX_DIR_LENGTH   24

#define MAX_FILES 100
#define FILENAME_LEN 32
#define BUFFER_SIZE 256

/* File list structure */
typedef struct {
    char filename[FILENAME_LEN];
    FSIZE_t size;
    int sector_count;
} DiskFile;

/* FLEX structures (simplified from your code) */
typedef struct {
    uint8_t track;
    uint8_t sector;
} FlexTS;

typedef struct {
    uint8_t labelName[12];
    uint16_t volNumber;
    FlexTS firstFree;
    FlexTS lastFree;
    uint16_t freeSectors;
    uint8_t maxTrack;
    uint8_t maxSector;
} FlexSIR;

typedef struct {
    char filename[12];
    FlexTS start;
    FlexTS end;
    uint16_t length;
    uint8_t randomFlag;
    uint8_t month;
    uint8_t day;
    uint8_t year;
} FlexDirEntry;

/* Global variables */
static DiskFile disk_files[MAX_FILES];
static int file_count = 0;
static FIL current_disk;
static FlexSIR sir_info;
static uint8_t sector_buffer[BUFFER_SIZE];
static int current_block = 0;
static int max_blocks = 0;

#if !FF_FS_READONLY && !FF_FS_NORTC
DWORD get_fattime (void)
{
    // Return a fixed timestamp since we don't have RTC
    // Format: bit31:25=Year(0-127 +1980), bit24:21=Month(1-12), bit20:16=Day(1-31),
    //         bit15:11=Hour(0-23), bit10:5=Minute(0-59), bit4:0=Second(0-29 *2)
    
    // Fixed date: 2025-01-01 12:00:00
    return ((DWORD)(2025 - 1980) << 25)  // Year = 2025
           | ((DWORD)1 << 21)            // Month = January  
           | ((DWORD)1 << 16)            // Day = 1
           | ((DWORD)12 << 11)           // Hour = 12
           | ((DWORD)0 << 5)             // Minute = 0
           | ((DWORD)0 << 0);            // Second = 0
}
#endif


/*
 * dump function
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
 * Calculate block position for .DSK vs .IMA files
 */
uint32_t calculate_block_position(const char* filename, int block_num) {
    int is_ima;
    uint32_t position;
    
    /* Check if it's an IMA file */
    is_ima = (strlen(filename) > 4 && 
              (strcmp(&filename[strlen(filename)-4], ".IMA") == 0 ||
               strcmp(&filename[strlen(filename)-4], ".ima") == 0));
    
    if (block_num < 10) {
        /* Track 0 - special handling */
        if (is_ima) {
            /* IMA: Track 0 padded to same size as other tracks */
            position = block_num * 256;
        } else {
            /* DSK: Track 0 not padded */
            position = block_num * 256;
        }
    } else {
        /* Other tracks */
        if (is_ima) {
            /* IMA: All tracks same size */
            position = block_num * 256;
        } else {
            /* DSK: Track 0 smaller, adjust for other tracks */
            position = block_num * 256;
        }
    }
    
    return position;
}

/*
 * Read a 256-byte block from disk image
 */
int read_disk_block(FIL *fp, const char* filename, int block_num, uint8_t *buffer) {
    FRESULT res;
    UINT bytes_read;
    uint32_t position;
    
    position = calculate_block_position(filename, block_num);
    
    res = f_lseek(fp, position);
    if (res != FR_OK) {
        printf("Seek error: %d\n", res);
        return -1;
    }
    
    res = f_read(fp, buffer, BUFFER_SIZE, &bytes_read);
    if (res != FR_OK) {
        printf("Read error: %d\n", res);
        return -1;
    }
    
    if (bytes_read != BUFFER_SIZE) {
        printf("Warning: Read %u bytes instead of %d\n", bytes_read, BUFFER_SIZE);
    }
    
    return bytes_read;
}

/*
 * Extract FLEX filename from directory entry
 */
void extract_flex_filename(uint8_t *data, int offset, char *filename) {
    int i, j;
    
    /* Copy name part (8 bytes) */
    j = 0;
    for (i = 0; i < 8; i++) {
        if (data[offset + i] > 0x20) {
            filename[j++] = data[offset + i];
        }
    }
    
    /* Add extension if present */
    if (data[offset + 8] > 0x20) {
        filename[j++] = '.';
        for (i = 8; i < 11; i++) {
            if (data[offset + i] > 0x20) {
                filename[j++] = data[offset + i];
            }
        }
    }
    
    filename[j] = '\0';
}

/*
 * Read FLEX SIR (System Information Record)
 */
int read_flex_sir(FIL *fp, const char* filename, FlexSIR *sir) {
    static uint8_t buffer[BUFFER_SIZE];
    int offset;
    
    if (read_disk_block(fp, filename, 2, buffer) < 0) {
        return -1;
    }
    
    offset = FLEX_SIR_OFFSET;
    
    /* Extract label name */
    memcpy(sir->labelName, &buffer[offset], 11);
    sir->labelName[11] = '\0';
    offset += 11;
    
    /* Volume number */
    sir->volNumber = (buffer[offset] << 8) | buffer[offset + 1];
    offset += 2;
    
    /* First free sector */
    sir->firstFree.track = buffer[offset++];
    sir->firstFree.sector = buffer[offset++];
    
    /* Last free sector */
    sir->lastFree.track = buffer[offset++];
    sir->lastFree.sector = buffer[offset++];
    
    /* Free sectors count */
    sir->freeSectors = (buffer[offset] << 8) | buffer[offset + 1];
    offset += 2;
    
    /* Skip date (3 bytes) */
    offset += 3;
    
    /* Max track and sector */
    sir->maxTrack = buffer[offset++];
    sir->maxSector = buffer[offset++];
    
    return 0;
}

/*
 * Read FLEX directory
 */
int read_flex_directory(FIL *fp, const char* filename, FlexSIR *sir) {
    static uint8_t buffer[BUFFER_SIZE];
    FlexDirEntry entry;
    int block_num;
    int i, offset;
    int entry_count;
    int next_track, next_sector;
    
    printf("\nFLEX Directory Listing\n");
    printf("======================\n");
    printf("Disk: %s\n", sir->labelName);
    printf("Volume: %d, Tracks: %d, Sectors: %d\n", 
           sir->volNumber, sir->maxTrack, sir->maxSector);
    printf("Free sectors: %d\n", sir->freeSectors);
    printf("\nFilename     Start   End   Length Date\n");
    printf("------------ ------- ------- ------ ----------\n");
    
    /* Start with first directory block */
    block_num = 4; /* Block 4 = Track 0, Sector 5 */
    entry_count = 0;
    
    do {
        if (read_disk_block(fp, filename, block_num, buffer) < 0) {
            return -1;
        }
        
        /* Get next directory block pointer */
        next_track = buffer[0];
        next_sector = buffer[1];
        
        /* Read directory entries in this block */
        for (i = 0; i < 10; i++) {
            offset = FLEX_DIR_OFFSET + i * FLEX_DIR_LENGTH;
            
            /* Check if entry exists */
            if (buffer[offset] == 0) {
                continue; /* Empty entry */
            }
            if (buffer[offset] == 0xFF) {
                continue; /* Deleted entry */
            }
            
            /* Extract filename */
            extract_flex_filename(buffer, offset, entry.filename);
            offset += 11;
            
            /* Skip extension field */
            offset += 1;
            
            /* Start track/sector */
            entry.start.track = buffer[offset++];
            entry.start.sector = buffer[offset++];
            
            /* End track/sector */
            entry.end.track = buffer[offset++];
            entry.end.sector = buffer[offset++];
            
            /* File length in sectors */
            entry.length = (buffer[offset] << 8) | buffer[offset + 1];
            offset += 2;
            
            /* Random file flag */
            entry.randomFlag = buffer[offset++];
            
            /* Skip reserved byte */
            offset++;
            
            /* Date */
            entry.month = buffer[offset++];
            entry.day = buffer[offset++]; 
            entry.year = buffer[offset++];
            
            /* Display entry */
            printf("%-12s %3d:%-3d %3d:%-3d %6d       %02d/%02d/%02d\n",
                   entry.filename,
                   entry.start.track, entry.start.sector,
                   entry.end.track, entry.end.sector,
                   entry.length,
                   entry.month, entry.day, 
                   (entry.year < 50) ? entry.year + 2000 : entry.year + 1900);
            
            entry_count++;
        }
        
        /* Calculate next directory block */
        if (next_track == 0 && next_sector == 0) {
            break; /* End of directory */
        }
        
        /* Convert track/sector to block number */
        if (next_track == 0) {
            block_num = next_sector - 1;
        } else {
            block_num = (next_track * sir->maxSector) + (next_sector - 1);
        }
        
    } while (1);
    
    printf("\nTotal files: %d\n", entry_count);
    return entry_count;
}

/*
 * Scan for .DSK and .IMA files
 */
int scan_disk_files(void) {
    DIR dir;
    FILINFO fno;
    FRESULT res;
    int len;
    
    file_count = 0;
    
    res = f_opendir(&dir, "/");
    if (res != FR_OK) {
        printf("Error opening directory: %d\n", res);
        return -1;
    }
    
    while (file_count < MAX_FILES) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) 
            break;
        
        if (fno.fattrib & AM_DIR) 
            continue; /* Skip directories */
        
        len = strlen(fno.fname);
        if (len > 4) {
            if ((strcmp(&fno.fname[len-4], ".DSK") == 0) ||
                (strcmp(&fno.fname[len-4], ".dsk") == 0) ||
                (strcmp(&fno.fname[len-4], ".IMA") == 0) ||
                (strcmp(&fno.fname[len-4], ".ima") == 0)) {
                
                strcpy(disk_files[file_count].filename, fno.fname);
                disk_files[file_count].size = fno.fsize;
                disk_files[file_count].sector_count = (int)(fno.fsize / 256);
                file_count++;
            }
        }
    }
    f_closedir(&dir);
    return file_count;
}

/*
 * Display file selection menu
 */
int show_file_menu(void) {
    int i, choice;
    
    printf("\nDisk Image Files Found:\n");
    printf("=======================\n");
    
    for (i = 0; i < file_count; i++) {
        printf("%d. %-20s (%lu bytes, %d sectors)\n",
               i + 1, disk_files[i].filename, 
               (unsigned long)disk_files[i].size,
               disk_files[i].sector_count);
    }
    
    printf("\n0. Exit\n");
    printf("\nSelect file (0-%d): ", file_count);
    
    scanf("%d", &choice);
    while (getchar() != '\n'); /* consume rest of line */
    
    if (choice < 0 || choice > file_count) {
        return -1;
    }
    
    return choice - 1; /* Convert to 0-based index */
}

/*
 * Block browser menu
 */
void block_browser(const char* filename) {
    char command;
    int block_num;
    int bytes_read;
    
    printf("\nBlock Browser Commands:\n");
    printf("n - Next block\n");
    printf("p - Previous block  \n");
    printf("g - Go to specific block\n");
    printf("q - Quit block browser\n");
    
    while (1) {
        printf("\n[Block %d/%d] Command (n/p/g/q): ", current_block, max_blocks - 1);
        command = getchar();
        while (getchar() != '\n'); /* consume rest of line */
        
        switch (command) {
            case 'n':
            case 'N':
                if (current_block < max_blocks - 1) {
                    current_block++;
                } else {
                    printf("Already at last block\n");
                    continue;
                }
                break;
                
            case 'p':
            case 'P':
                if (current_block > 0) {
                    current_block--;
                } else {
                    printf("Already at first block\n");
                    continue;
                }
                break;
                
            case 'g':
            case 'G':
                printf("Go to block (0-%d): ", max_blocks - 1);
                scanf("%d", &block_num);
                while (getchar() != '\n'); /* consume rest of line */
                
                if (block_num >= 0 && block_num < max_blocks) {
                    current_block = block_num;
                } else {
                    printf("Invalid block number\n");
                    continue;
                }
                break;
                
            case 'q':
            case 'Q':
                return;
                
            default:
                printf("Invalid command\n");
                continue;
        }
        
        /* Read and display the block */
        bytes_read = read_disk_block(&current_disk, filename, current_block, sector_buffer);
        if (bytes_read > 0) {
            printf("\nBlock %d (0x%08X):\n", current_block, current_block * 256);
            dump(sector_buffer, bytes_read);
        } else {
            printf("Error reading block %d\n", current_block);
        }
    }
}

/*
 * Main disk operations menu
 */
void disk_operations_menu(int file_index) {
    char command;
    const char* filename;
    FRESULT res;
    
    filename = disk_files[file_index].filename;
    max_blocks = disk_files[file_index].sector_count;
    current_block = 0;
    
    printf("\nOpening disk image: %s\n", filename);
    
    /* Open the disk file */
    res = f_open(&current_disk, filename, FA_READ);
    if (res != FR_OK) {
        printf("Error opening file: %d\n", res);
        return;
    }
    
    /* Try to read FLEX SIR */
    if (read_flex_sir(&current_disk, filename, &sir_info) < 0) {
        printf("Warning: Could not read FLEX SIR - may not be a FLEX disk\n");
        strcpy((char*)sir_info.labelName, "UNKNOWN");
        sir_info.maxTrack = 79;
        sir_info.maxSector = 18;
    }
    
    while (1) {
        printf("\nDisk Operations Menu:\n");
        printf("=====================\n");
        printf("1. Show FLEX directory\n");
        printf("2. Browse blocks (hex dump)\n");
        printf("3. Return to file selection\n");
        printf("\nChoice (1-3): ");
        
        command = getchar();
        while (getchar() != '\n'); /* consume rest of line */
        
        switch (command) {
            case '1':
                if (read_flex_directory(&current_disk, filename, &sir_info) < 0) {
                    printf("Error reading directory\n");
                }
                break;
                
            case '2':
                printf("Starting block browser...\n");
                current_block = 0;
                if (read_disk_block(&current_disk, filename, current_block, sector_buffer) > 0) {
                    printf("\nBlock %d (0x%08X):\n", current_block, current_block * 256);
                    dump(sector_buffer, BUFFER_SIZE);
                    block_browser(filename);
                }
                break;
                
            case '3':
                f_close(&current_disk);
                return;
                
            default:
                printf("Invalid choice\n");
                break;
        }
    }
}

/*
 * Main program
 */
int main(void) {
    FATFS fs;
    FRESULT res;
    int file_index;
    
    printf("FLEX Disk Image Explorer\n");
    printf("========================\n");
    
    /* Mount filesystem */
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("Error mounting filesystem: %d\n", res);
        return -1;
    }
    
    while (1) {
        /* Scan for disk files */
        if (scan_disk_files() <= 0) {
            printf("No .DSK or .IMA files found\n");
            break;
        }
        
        /* Show file selection menu */
        file_index = show_file_menu();
        if (file_index < 0) {
            break; /* Exit selected */
        }
        
        /* Open selected file and show operations menu */
        disk_operations_menu(file_index);
    }
    
    f_unmount("");
    printf("Goodbye!\n");
    return 0;
}
