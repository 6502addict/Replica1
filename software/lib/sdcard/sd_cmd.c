#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

/*
 * Send SD command with automatic CRC7 calculation
 */
void sd_cmd(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3) {
  spi_transfer(cmd);
  spi_transfer(arg0);
  spi_transfer(arg1);
  spi_transfer(arg2);
  spi_transfer(arg3);
  spi_transfer(sd_common_crc(cmd, arg0, arg1, arg2, arg3));
}

