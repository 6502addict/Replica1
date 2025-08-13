#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Get R1 response and additional data bytes (for CMD8, CMD58, etc.)
 * response_buf: buffer to store the response data
 * data_bytes: number of additional bytes to read after R1
 * Returns: R1 response byte, additional data in buffer
 */
uint8_t sd_r1_data(uint8_t *response_buf, uint8_t data_bytes)
{
  uint8_t r1_response;
  uint8_t i;
  
  /* Get R1 response first */
  r1_response = sd_r1_response();
  
  if (r1_response == 0xFF) {
    return 0xFF;  /* Timeout */
  }
  
  /* Read additional data bytes */
  for (i = 0; i < data_bytes; i++) {
    response_buf[i] = spi_transfer(0xFF);
  }
  
  return r1_response;
}
