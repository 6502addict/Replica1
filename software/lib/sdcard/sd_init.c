#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>

/*
 * Complete SD card initialization sequence
 * Returns: 0 = success, error code otherwise
 */
uint8_t sd_init(void) {
  uint8_t r1_response;
  uint8_t r7_data[4];
  uint8_t attempts;
  uint8_t r1_acmd41;
  uint8_t cmd55_failures = 0;
  uint8_t i;
  
  /* Send initial clock cycles to wake up card */
  for (i = 0; i < 10; i++) 
    spi_transfer(0xFF);
  
  /* Step 1: CMD0 (GO_IDLE_STATE) */
  if (sd_cmd0() != 0x01) 
    return SD_ERROR_CMD0;
  
  /* Step 2: CMD8 (SEND_IF_COND) */
  r1_response = sd_cmd8(r7_data);
  
  if (r1_response == 0x01) {
    /* CMD8 OK - SD v2.0+ card */
    /* Verify voltage range and check pattern */
    if (r7_data[2] != 0x01 || r7_data[3] != 0xAA) 
      return SD_ERROR_CMD8;
    /* If we get here, CMD8 succeeded with correct response */
  } else if (r1_response == 0x05) {
    /* CMD8 illegal command - SD v1.x card */
    return SD_ERROR_V1_CARD;
  } else {
    /* CMD8 failed with unexpected error */
    return SD_ERROR_UNKNOWN_CMD8;
  }
  
  /* Step 3: ACMD41 loop */
  attempts = 0;
  cmd55_failures = 0;
  
  while (attempts < 100) {  
    /* Send CMD55 (APP_CMD) */
    if (sd_cmd55() > 1) {
      /* CMD55 failed */
      cmd55_failures++;
      if (cmd55_failures >= 10)
	return SD_ERROR_CMD55;
      attempts++;
      sd_delay();
      continue;
    }
    
    /* CMD55 succeeded, reset failure counter */
    cmd55_failures = 0;
    
    /* Send ACMD41 (SD_SEND_OP_COND) */
    r1_acmd41 = sd_acmd41();
    
    if (r1_acmd41 == 0x00) {
      /* Card is ready! */
      return SD_SUCCESS;
    }
    
    if (r1_acmd41 != 0x01) {
      /* Unexpected error from ACMD41 - continue trying */
      attempts++;
      sd_delay();
      continue;
    }
    
    /* r1_acmd41 == 0x01 means card is still initializing */
    attempts++;
    sd_delay();
  }
  
  /* Timeout after 100 attempts */
  return SD_ERROR_ACMD41_TIMEOUT;
}
