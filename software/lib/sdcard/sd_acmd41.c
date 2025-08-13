#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * ACMD41 
 */
uint8_t sd_acmd41(void) {
  return sd_cmd_response(0x69, 0x40, 0x00, 0x00, 0x00);
}

