#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Complete SD command with response
 * Sends command and waits for R1 response
 */
uint8_t sd_cmd_response(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3) {
  sd_cmd(cmd, arg0, arg1, arg2, arg3);
  return sd_r1_response();
}


