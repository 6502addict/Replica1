#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

uint8_t sd_protected(void) 
{
    unsigned long card_status;
    
    // Send CMD13 to get card status
    if (sd_cmd13(&card_status) != 0) {
        return SD_ERROR_CMD13;  // Error - couldn't get status
    }
    
    // Check bit 13 of card status register
    // Bit 13 = WP_VIOLATION (write protect violation)
    // But more commonly, check if card is locked (bit 25)
    
    // Check write protect switch (bit 13)
    if (card_status & (1UL << 13)) {
        return SD_ERROR_PROTECTED;  // Write protected
    }
    
    // Check card locked status (bit 25) 
    if (card_status & (1UL << 25)) {
        return SD_ERROR_LOCKED;  // Card is locked
    }
    
    return 0;  // Not write protected
}
