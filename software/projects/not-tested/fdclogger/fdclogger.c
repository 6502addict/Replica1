#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "ff.h"      // FatFS library

// FDC Register addresses (6502 side)
#define FDC_BASE        0xE040
#define FDC_DRIVE_SEL   (*(volatile unsigned char*)(FDC_BASE + 0))
#define FDC_CMD         (*(volatile unsigned char*)(FDC_BASE + 1))
#define FDC_STATUS      (*(volatile unsigned char*)(FDC_BASE + 2))
#define FDC_TRACK       (*(volatile unsigned char*)(FDC_BASE + 3))
#define FDC_SECTOR      (*(volatile unsigned char*)(FDC_BASE + 4))
#define FDC_DATA        (*(volatile unsigned char*)(FDC_BASE + 5))
#define FDC_CMD_PENDING (*(volatile unsigned char*)(FDC_BASE + 6))

// ACIA Register addresses (6502 side)
#define ACIA_BASE       0xE050
#define ACIA_STATUS     (*(volatile unsigned char*)(ACIA_BASE + 0))
#define ACIA_DATA       (*(volatile unsigned char*)(ACIA_BASE + 1))

// FDC Command definitions (FD1771 compatible - base commands only)
#define CMD_TYPE_RESTORE     0x00  // Type I: Restore (bits 7-4 = 0000)
#define CMD_TYPE_SEEK        0x10  // Type I: Seek (bits 7-4 = 0001) 
#define CMD_TYPE_STEP        0x20  // Type I: Step (bits 7-4 = 0010)
#define CMD_TYPE_STEP_IN     0x40  // Type I: Step In (bits 7-4 = 0100)
#define CMD_TYPE_STEP_OUT    0x60  // Type I: Step Out (bits 7-4 = 0110)
#define CMD_TYPE_READ_SECTOR 0x80  // Type II: Read Sector (bits 7-5 = 100)
#define CMD_TYPE_WRITE_SECTOR 0xA0 // Type II: Write Sector (bits 7-5 = 101)
#define CMD_TYPE_READ_ADDR   0xC0  // Type III: Read Address (bits 7-4 = 1100)
#define CMD_TYPE_READ_TRACK  0xE0  // Type III: Read Track (bits 7-4 = 1110)
#define CMD_TYPE_WRITE_TRACK 0xF0  // Type III: Write Track (bits 7-4 = 1111)
#define CMD_TYPE_FORCE_INT   0xD0  // Type IV: Force Interrupt (bits 7-4 = 1101)

// ACIA Status bits
#define ACIA_RDRF       0x01  // Receive Data Register Full
#define ACIA_TDRE       0x02  // Transmit Data Register Empty

// Global variables
static FATFS fs;            // FatFS filesystem object
static FIL logfile;         // Log file object
static unsigned char last_fdc_cmd = 0xFF;
static unsigned char last_acia_status = 0xFF;
static unsigned long log_counter = 0;

// Function prototypes
void init_system(void);
void check_fdc_commands(void);
void check_acia_activity(void);
void log_fdc_command(unsigned char cmd, unsigned char drive, unsigned char track, unsigned char sector);
void log_acia_activity(unsigned char status, unsigned char data, int is_receive);
void log_message(const char* message);
const char* get_fdc_command_name(unsigned char cmd);
void decode_fdc_command(unsigned char cmd, char* details);
void cleanup_and_exit(void);

int main(void)
{
    printf("FDC/ACIA Monitor and Logger v1.0\n");
    printf("================================\n\n");
    
    // Initialize system
    init_system();
    
    printf("Monitoring FDC and ACIA activity...\n");
    printf("Monitor running - reset to exit\n\n");
    
    // Main monitoring loop - runs forever
    while (1) {
        check_fdc_commands();
        check_acia_activity();
        
        // Small delay to prevent overwhelming the system
        // Note: cc65 doesn't have usleep, so we use a simple loop
        {
            volatile int i;
            for (i = 0; i < 1000; i++) {
                // Small delay
            }
        }
    }
    
    // This line never reached on Replica1
    return 0;
}

void init_system(void)
{
    FRESULT fr;
    
    printf("Initializing SD card and FatFS...\n");
    
    // Mount the filesystem
    fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        printf("ERROR: Failed to mount SD card (error %d)\n", fr);
        exit(1);
    }
    
    // Open/create log file
    fr = f_open(&logfile, "FDC_ACIA.LOG", FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("ERROR: Failed to create log file (error %d)\n", fr);
        exit(1);
    }
    
    // Write log header
    log_message("FDC/ACIA Monitor Log Started");
    log_message("============================");
    
    printf("System initialized successfully\n\n");
}

void check_fdc_commands(void)
{
    unsigned char cmd_pending;
    unsigned char cmd, drive, track, sector;
    
    // Check if new command is pending
    cmd_pending = FDC_CMD_PENDING;
    
    if (cmd_pending == 0x01) {
        // Read command and parameters
        cmd = FDC_CMD;
        drive = FDC_DRIVE_SEL;
        track = FDC_TRACK;
        sector = FDC_SECTOR;
        
        // Log the command
        log_fdc_command(cmd, drive, track, sector);
        
        // Simulate command processing and set status
        {
            volatile int i;
            for (i = 0; i < 5000; i++) {
                // Simulate processing delay
            }
        }
        
        // Set status to indicate command completed
        FDC_STATUS = 0x00;  // No errors, command completed
        
        last_fdc_cmd = cmd;
    }
}

void check_acia_activity(void)
{
    unsigned char status, data;
    
    // Read ACIA status
    status = ACIA_STATUS;
    
    // Check for received data
    if ((status & ACIA_RDRF) && !(last_acia_status & ACIA_RDRF)) {
        data = ACIA_DATA;
        log_acia_activity(status, data, 1);  // 1 = receive
    }
    
    // Check for transmit ready change
    if ((status & ACIA_TDRE) != (last_acia_status & ACIA_TDRE)) {
        if (status & ACIA_TDRE) {
            // Transmit register became empty - data was sent
            log_acia_activity(status, 0, 0);  // 0 = transmit
        }
    }
    
    last_acia_status = status;
}

void log_fdc_command(unsigned char cmd, unsigned char drive, unsigned char track, unsigned char sector)
{
    char buffer[256];
    char cmd_details[128];
    const char* cmd_name = get_fdc_command_name(cmd);
    
    // Decode command parameters
    decode_fdc_command(cmd, cmd_details);
    
    // Create detailed log message
    sprintf(buffer, "FDC[%04lu]: %s (0x%02X) %s Drive=%d Track=%d Sector=%d", 
            log_counter++, cmd_name, cmd, cmd_details, drive, track, sector);
    
    // Display on screen
    printf("%s\n", buffer);
    
    // Write to log file
    log_message(buffer);
}

void log_acia_activity(unsigned char status, unsigned char data, int is_receive)
{
    char buffer[128];
    
    if (is_receive) {
        // Data received
        if (data >= 32 && data < 127) {
            sprintf(buffer, "ACIA[%04lu]: RX='%c' (0x%02X) Status=0x%02X", 
                    log_counter++, data, data, status);
        } else {
            sprintf(buffer, "ACIA[%04lu]: RX=0x%02X Status=0x%02X", 
                    log_counter++, data, status);
        }
    } else {
        // Transmit ready
        sprintf(buffer, "ACIA[%04lu]: TX Ready Status=0x%02X", 
                log_counter++, status);
    }
    
    // Display on screen
    printf("%s\n", buffer);
    
    // Write to log file
    log_message(buffer);
}

void log_message(const char* message)
{
    UINT bytes_written;
    
    // Write message to log file
    f_write(&logfile, message, strlen(message), &bytes_written);
    f_write(&logfile, "\r\n", 2, &bytes_written);
    
    // Sync to ensure data is written to SD card
    f_sync(&logfile);
}

const char* get_fdc_command_name(unsigned char cmd)
{
    // Decode based on command type (upper bits)
    if ((cmd & 0xF0) == CMD_TYPE_RESTORE) return "RESTORE";
    if ((cmd & 0xF0) == CMD_TYPE_SEEK) return "SEEK";
    if ((cmd & 0xF0) == CMD_TYPE_STEP) return "STEP";
    if ((cmd & 0xF0) == CMD_TYPE_STEP_IN) return "STEP_IN";
    if ((cmd & 0xF0) == CMD_TYPE_STEP_OUT) return "STEP_OUT";
    if ((cmd & 0xE0) == CMD_TYPE_READ_SECTOR) return "READ_SECTOR";
    if ((cmd & 0xE0) == CMD_TYPE_WRITE_SECTOR) return "WRITE_SECTOR";
    if ((cmd & 0xF0) == CMD_TYPE_READ_ADDR) return "READ_ADDRESS";
    if ((cmd & 0xF0) == CMD_TYPE_READ_TRACK) return "READ_TRACK";
    if ((cmd & 0xF0) == CMD_TYPE_WRITE_TRACK) return "WRITE_TRACK";
    if ((cmd & 0xF0) == CMD_TYPE_FORCE_INT) return "FORCE_INTERRUPT";
    
    return "UNKNOWN";
}

void decode_fdc_command(unsigned char cmd, char* details)
{
    details[0] = '\0';  // Start with empty string
    
    // Type I Commands (Restore, Seek, Step variants)
    if ((cmd & 0x80) == 0) {
        // Type I command
        if (cmd & 0x08) strcat(details, "h=1 ");  // Head load
        if (cmd & 0x04) strcat(details, "V=1 ");  // Verify
        
        // Step rate
        switch (cmd & 0x03) {
            case 0: strcat(details, "r=6ms"); break;
            case 1: strcat(details, "r=12ms"); break;
            case 2: strcat(details, "r=2ms"); break;
            case 3: strcat(details, "r=3ms"); break;
        }
    }
    // Type II Commands (Read/Write Sector)
    else if ((cmd & 0xE0) == 0x80 || (cmd & 0xE0) == 0xA0) {
        // Type II command
        if (cmd & 0x10) strcat(details, "m=1 ");  // Multiple sectors
        if (cmd & 0x08) strcat(details, "S=1 ");  // Side select
        if (cmd & 0x04) strcat(details, "E=1 ");  // 15ms delay
        if (cmd & 0x02) strcat(details, "P=1 ");  // Side compare
        if (cmd & 0x01) strcat(details, "a0=1 "); // Data address mark
    }
    // Type III Commands (Read Address/Track, Write Track)
    else if ((cmd & 0xF0) == 0xC0 || (cmd & 0xF0) == 0xE0 || (cmd & 0xF0) == 0xF0) {
        // Type III command  
        if (cmd & 0x08) strcat(details, "S=1 ");  // Side select
        if (cmd & 0x04) strcat(details, "E=1 ");  // 15ms delay
        if (cmd & 0x02) strcat(details, "P=1 ");  // Side compare
    }
    // Type IV Commands (Force Interrupt)
    else if ((cmd & 0xF0) == 0xD0) {
        // Type IV command
        if (cmd & 0x08) strcat(details, "I3=1 ");  // Interrupt on any condition
        if (cmd & 0x04) strcat(details, "I2=1 ");  // Interrupt on ready-to-not ready
        if (cmd & 0x02) strcat(details, "I1=1 ");  // Interrupt on not ready-to-ready  
        if (cmd & 0x01) strcat(details, "I0=1 ");  // Immediate interrupt
    }
}

void cleanup_and_exit(void)
{
    log_message("FDC/ACIA Monitor Stopped");
    f_close(&logfile);
    f_unmount("");
    printf("\nLog file closed. Exiting...\n");
}

// Note: Replica1 has no interrupts, so no CTRL+C handling
// Program runs until reset or power off
