/*
 * SD Card File Lister
 * C89/cc65 compatible directory listing utility
 * Shows filename, size, date, and attributes
 */

#include <stdio.h>
#include <string.h>
#include "ff.h"

/* File type indicators */
#define TYPE_FILE       '-'
#define TYPE_DIR        'd'
#define TYPE_READONLY   'r'
#define TYPE_HIDDEN     'h'
#define TYPE_SYSTEM     's'
#define TYPE_ARCHIVE    'a'

/* Month names for date display */
static const char* months[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

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
 * Convert FatFS date/time to readable format
 */
void format_date_time(WORD fdate, WORD ftime, char* buffer) {
    int year, month, day, hour, minute;
    
    /* Extract date components */
    year = 1980 + ((fdate >> 9) & 0x7F);
    month = (fdate >> 5) & 0x0F;
    day = fdate & 0x1F;
    
    /* Extract time components */
    hour = (ftime >> 11) & 0x1F;
    minute = (ftime >> 5) & 0x3F;
    
    /* Validate month */
    if (month < 1 || month > 12) {
        month = 1;
    }
    
    /* Format: "01-Jan-2025 14:30" */
    sprintf(buffer, "%02d-%s-%04d %02d:%02d", 
            day, months[month-1], year, hour, minute);
}

/*
 * Format file size with appropriate units
 */
void format_file_size(FSIZE_t size, char* buffer) {
    if (size < 1024) {
        sprintf(buffer, "%8lu B", (unsigned long)size);
    } else if (size < 1024L * 1024L) {
        sprintf(buffer, "%7lu KB", (unsigned long)(size / 1024));
    } else {
        sprintf(buffer, "%7lu MB", (unsigned long)(size / (1024L * 1024L)));
    }
}

/*
 * Get file type and attributes string
 */
void get_file_attributes(BYTE attrib, char* buffer) {
    buffer[0] = (attrib & AM_DIR) ? TYPE_DIR : TYPE_FILE;
    buffer[1] = (attrib & AM_RDO) ? TYPE_READONLY : '-';
    buffer[2] = (attrib & AM_HID) ? TYPE_HIDDEN : '-';
    buffer[3] = (attrib & AM_SYS) ? TYPE_SYSTEM : '-';
    buffer[4] = (attrib & AM_ARC) ? TYPE_ARCHIVE : '-';
    buffer[5] = '\0';
}

/*
 * List files in current directory
 */
int list_files(const char* path) {
    DIR dir;
    FILINFO fno;
    FRESULT res;
    char date_str[20];
    char size_str[12];
    char attr_str[6];
    unsigned long total_files = 0;
    unsigned long total_dirs = 0;
    unsigned long total_size = 0;
    
    printf("Directory listing for: %s\n", path);
    printf("=====================================\n");
    printf("Attr     Size      Date/Time        Name\n");
    printf("-----  --------  ----------------  ------------\n");
    
    /* Open directory */
    res = f_opendir(&dir, path);
    if (res != FR_OK) {
        printf("Error opening directory: %d\n", res);
        return -1;
    }
    
    /* Read directory entries */
    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK) {
            printf("Error reading directory: %d\n", res);
            break;
        }
        
        /* End of directory? */
        if (fno.fname[0] == 0) {
            break;
        }
        
        /* Skip hidden files starting with . (optional) */
        if (fno.fname[0] == '.') {
            continue;
        }
        
        /* Get file attributes */
        get_file_attributes(fno.fattrib, attr_str);
        
        /* Format date/time */
        format_date_time(fno.fdate, fno.ftime, date_str);
        
        /* Format file size */
        if (fno.fattrib & AM_DIR) {
            strcpy(size_str, "   <DIR>");
            total_dirs++;
        } else {
            format_file_size(fno.fsize, size_str);
            total_files++;
            total_size += fno.fsize;
        }
        
        /* Print file information */
        printf("%-5s %s  %s  %s\n", 
               attr_str, size_str, date_str, fno.fname);
    }
    
    f_closedir(&dir);
    
    /* Print summary */
    printf("-----  --------  ----------------  ------------\n");
    printf("Summary: %lu file(s), %lu dir(s)\n", total_files, total_dirs);
    
    if (total_size > 0) {
        char total_str[12];
        format_file_size(total_size, total_str);
        printf("Total size: %s\n", total_str);
    }
    
    return 0;
}

/*
 * List files with detailed information
 */
int list_files_detailed(const char* path) {
    DIR dir;
    FILINFO fno;
    FRESULT res;
    char date_str[20];
    char size_str[12];
    unsigned long count = 0;
    
    printf("Detailed listing for: %s\n", path);
    printf("=====================================\n");
    
    res = f_opendir(&dir, path);
    if (res != FR_OK) {
        printf("Error opening directory: %d\n", res);
        return -1;
    }
    
    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) {
            break;
        }
        
        if (fno.fname[0] == '.') {
            continue;
        }
        
        count++;
        printf("\n--- File %lu ---\n", count);
        printf("Name:       %s\n", fno.fname);
        
        if (fno.fattrib & AM_DIR) {
            printf("Type:       Directory\n");
            printf("Size:       <DIR>\n");
        } else {
            printf("Type:       File\n");
            format_file_size(fno.fsize, size_str);
            printf("Size:       %s (%lu bytes)\n", size_str, (unsigned long)fno.fsize);
        }
        
        format_date_time(fno.fdate, fno.ftime, date_str);
        printf("Modified:   %s\n", date_str);
        
        printf("Attributes: ");
        if (fno.fattrib & AM_DIR) printf("Directory ");
        if (fno.fattrib & AM_RDO) printf("Read-only ");
        if (fno.fattrib & AM_HID) printf("Hidden ");
        if (fno.fattrib & AM_SYS) printf("System ");
        if (fno.fattrib & AM_ARC) printf("Archive ");
        if (fno.fattrib == 0) printf("Normal");
        printf("\n");
    }
    
    f_closedir(&dir);
    printf("\nTotal: %lu items\n", count);
    return 0;
}

/*
 * Show disk space information
 */
int show_disk_info(void) {
    FATFS *fs;
    DWORD free_clusters;
    DWORD total_sectors, free_sectors;
    FRESULT res;
    
    printf("Disk Information\n");
    printf("================\n");
    
    /* Get volume information */
    res = f_getfree("", &free_clusters, &fs);
    if (res != FR_OK) {
        printf("Error getting disk info: %d\n", res);
        return -1;
    }
    
    /* Calculate total and free space */
    total_sectors = (fs->n_fatent - 2) * fs->csize;
    free_sectors = free_clusters * fs->csize;
    
    //    printf("Sector size:    %u bytes\n", (unsigned int)fs->ssize);
    printf("Cluster size:   %u sectors\n", (unsigned int)fs->csize);
    printf("Total clusters: %lu\n", (unsigned long)(fs->n_fatent - 2));
    printf("Free clusters:  %lu\n", (unsigned long)free_clusters);
    
    /* Convert to KB/MB for readability */
    printf("Total space:    %lu KB\n", (unsigned long)(total_sectors / 2));
    printf("Free space:     %lu KB\n", (unsigned long)(free_sectors / 2));
    printf("Used space:     %lu KB\n", (unsigned long)((total_sectors - free_sectors) / 2));
    
    return 0;
}

/*
 * Main program
 */
int main(void) {
    FATFS fs;
    FRESULT res;
    char command;
    
    printf("SD Card File Lister v1.0\n");
    printf("========================\n\n");
    
    /* Mount filesystem */
    printf("Mounting SD card... ");
    res = f_mount(&fs, "", 1);
    if (res != FR_OK) {
        printf("FAILED (error %d)\n", res);
        printf("Make sure SD card is inserted and formatted.\n");
        return -1;
    }
    printf("OK\n\n");
    
    /* Simple menu */
    while (1) {
        printf("\nSD Card File Lister\n");
        printf("===================\n");
        printf("1 - List files (compact)\n");
        printf("2 - List files (detailed)\n");
        printf("3 - Show disk information\n");
        printf("4 - Exit\n");
        printf("\nChoice (1-4): ");
        
        /* Get user input */
        command = getchar();
        while (getchar() != '\n'); /* consume rest of line */
        
        printf("\n");
        
        switch (command) {
            case '1':
                list_files("/");
                break;
                
            case '2':
                list_files_detailed("/");
                break;
                
            case '3':
                show_disk_info();
                break;
                
            case '4':
                printf("Unmounting SD card...\n");
                f_unmount("");
                printf("Goodbye!\n");
                return 0;
                
            default:
                printf("Invalid choice. Please enter 1, 2, 3, or 4.\n");
                break;
        }
        
        printf("\nPress ENTER to continue...");
        while (getchar() != '\n');
    }
    
    return 0;
}
