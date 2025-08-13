#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Send CMD55 (APP_CMD)
 */
uint8_t sd_cmd55() {
  return sd_cmd_response(0x77, 0x00, 0x00, 0x00, 0x00);
}


