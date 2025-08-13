#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Write single block to SD card
 * block_num: block number to write
 * buffer: 512-byte buffer containing data to write
 * Returns: 0 = success, non-zero = error
 */
uint8_t sd_write(unsigned long block_num, uint8_t *buffer)
{
  uint8_t data_response;
  unsigned int i;
  uint8_t status;
  unsigned int timeout;
  
  /* Send CMD24 (WRITE_BLOCK) */
  sd_cmd(0x58, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), (uint8_t)(block_num >> 8), (uint8_t)(block_num));
  /* Get R1 response */
  if (sd_r1_response() != 0x00) 
    return SD_ERROR_CMD24;
  /* Send data start token */
  spi_transfer(DATA_START_TOKEN);
  /* Send 512 bytes of data */
  for (i = 0; i < SD_BLOCK_SIZE; i++) 
    spi_transfer(buffer[i]);
  /* Send dummy CRC (2 bytes) - ignored in SPI mode */
  spi_transfer(0xFF);
  spi_transfer(0xFF);
  /* Get data response token */
  data_response = spi_transfer(0xFF);
  /* Check data response */
  if ((data_response & 0x1F) != DATA_ACCEPT_TOKEN) 
    return SD_ERROR_WRITE_REJECT;
  /* Wait for card to finish writing (card will hold MISO low while busy) */
  timeout = 0;
  while (timeout < 65000U) {
    status = spi_transfer(0xFF);
    if (status != 0x00){
      break;  /* Card finished writing */
      timeout++;
    }
  }
  if (timeout >= 65000U)
    return SD_ERROR_WRITE_TIMEOUT;
  return SD_SUCCESS;
}

