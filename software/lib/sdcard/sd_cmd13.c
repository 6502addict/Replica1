#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

#define CMD13 13

/*
 * Send CMD13 and get 32-bit status response
 */
uint8_t sd_cmd13(uint32_t *status) 
{
  uint8_t i;
  uint8_t response;
  uint8_t status_bytes[4];
  
  // Send CMD13 with argument 0 (for SPI mode)
  sd_cmd(CMD13, 0x00, 0x00, 0x00, 0x00);
  /* Send up to 8 FF bytes to get response */
  response = spi_transfer(0xFF);
  if ((response & 0x80) == 0x80) 
    return SD_ERROR_CMD13;
  for (i = 0; i < 4; i ++)
    status_bytes[i] = spi_transfer(0xFF);
  *status = ((uint32_t)status_bytes[0] << 24) |
            ((uint32_t)status_bytes[1] << 16) |
            ((uint32_t)status_bytes[2] << 8) |
            ((uint32_t)status_bytes[3]);
  return SD_SUCCESS;
}
