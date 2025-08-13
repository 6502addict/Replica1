#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Wait for and receive R1 response (most common)
 * Returns: R1 response byte, or 0xFF if timeout
 */
uint8_t sd_r1_response(void) {
  uint8_t response;
  uint8_t attempts;
  
  attempts = 0;
  /* Send up to 8 FF bytes to get response */
  while (attempts < 8) {
    response = spi_transfer(0xFF);
    /* Valid R1 response has bit 7 = 0 */
    if ((response & 0x80) == 0) 
      return response;
    attempts++;
  }
  return 0xFF;
}

