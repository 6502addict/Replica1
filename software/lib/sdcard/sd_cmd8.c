#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Send command and get R7 response (CMD8)
 * r7_data: buffer for 4-byte R7 response data
 * Returns: R1 response byte
 */
uint8_t sd_cmd8(uint8_t *r7_data) {
  sd_cmd(0x48, 0x00, 0x00, 0x01, 0xAA);
  /* Get R1 + 4 bytes of R7 data */
  return sd_r1_data(r7_data, 4);
}
