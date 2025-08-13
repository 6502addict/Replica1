#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Read single block from SD card
 * block_num: block number to read (for SDHC cards, this is the block number)
 * buffer: 512-byte buffer to store the data
 * Returns: 0 = success, non-zero = error
 */
uint8_t sd_read(unsigned long block_num, uint8_t *buffer)
{
  uint8_t token;
  unsigned int i;
  uint8_t crc1, crc2;
  uint8_t attempts;
  
  /* Send CMD17 (READ_SINGLE_BLOCK) */
  /* For SDHC cards, block_num is the block address */
  sd_cmd(0x51, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), 
	 (uint8_t)(block_num >> 8), (uint8_t)(block_num));
  /* Get R1 response */
  if (sd_r1_response() != 0x00) 
    return SD_ERROR_CMD17;
  
  /* Wait for data start token (0xFE) */
  attempts = 0;
  while (attempts < 100) {
    if ((token = spi_transfer(0xFF))  == DATA_START_TOKEN)
      break;
    if (token != 0xFF)
      return SD_ERROR_READ_TOKEN;
    attempts++;
  }
  if (attempts >= 100) 
    return SD_ERROR_READ_TIMEOUT;
  
  /* Read 512 bytes of data */
  for (i = 0; i < SD_BLOCK_SIZE; i++) {
    buffer[i] = spi_transfer(0xFF);
  }
  
  /* Read CRC (2 bytes) - we ignore it in SPI mode */
  crc1 = spi_transfer(0xFF);
  crc2 = spi_transfer(0xFF);
    
  return SD_SUCCESS;
}
