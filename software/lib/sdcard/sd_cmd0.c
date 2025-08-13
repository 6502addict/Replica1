#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

/*
 * send CMD0 command
 * Returns: 01 = success, error code otherwise
 */
uint8_t sd_cmd0() {
  return sd_cmd_response(0x40, 0x00, 0x00, 0x00, 0x00);
}

