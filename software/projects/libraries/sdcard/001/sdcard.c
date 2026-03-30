#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>
#include <timer.h>

#define SD_DUMMY_CLOCKS      80
#define SD_INIT_SPEED       400
#define SD_FAST_SPEED      2500
#define SD_POWER_UP_DELAY   250
#define SD_CMD0_RETRY        10
#define SD_CMD0_DELAY       100
#define SD_CMD8_RETRY       100
#define SD_CMD8_DELAY        50
#define SD_CMD5_RETRY        10
#define SD_CMD55_DELAY        1
#define SD_CMD55_RETRY       10
#define SD_CMD58_RETRY       10
#define SD_ACMD41_RETRY    1000 

#define SDCARD_V1            0
#define SDCARD_V2            1

#define GO_IDLE_STATE            0   // CMD0
#define SEND_OP_COND             1   // CMD1
#define SEND_IF_COND             8   // CMD8
#define SEND_CSD                 9   // CMD9
#define STOP_TRANSMISSION       12   // CMD12
#define SEND_STATUS             13   // CMD13
#define SET_BLOCKLEN            16   // CMD16
#define READ_SINGLE_BLOCK       17   // CMD17
#define READ_MULTIPLE_BLOCK     18   // CMD17
#define WRITE_SINGLE_BLOCK      24   // CMD24
#define WRITE_MULTIPLE_BLOCK    25   // CMD25
#define PROGRAM_CSD             27   // CMD27
#define SET_WRITE_PROT          28   // CMD28
#define CLR_WRITE_PROT          29   // CMD29
#define ERASE_WR_BLK_START      32   // CMD32
#define ERASE_WR_BLK_END        33   // CMD33
#define ERASE                   38   // CMD38
#define APP_CMD                 55   // CMD55
#define READ_OCR                58   // CMD58
#define CRC_ON_OFF              59   // CMD59


#define SD_STATUS               13   // ACMD13
#define SEND_NUM_WR_BLOCKS      22   // ACMD22
#define SET_WR_BLK_ERASE_COUNT  23   // ACMD23
#define SD_SEND_OP_COND         41   // ACMD41
#define SET_CLR_CARD_DETECT     42   // ACMD42
#define SEND_SCR                51   // ACMD51


typedef enum {
  SD_INACTIVE,
  SD_POWER_UP,
  SD_SEND_CMD0,
  SD_SEND_CMD8,
  SD_V1,
  SD_V2,
  SD_SEND_CMD58,
  SD_SEND_CMD55,
  SD_SEND_ACMD41,
  SD_INITIALIZED
} sd_init_state_t;

uint8_t sd_crc7_byte(uint8_t crc, uint8_t data) {
  uint8_t i;
  
  for (i = 0; i < 8; i++) {
    crc <<= 1;
    if ((crc ^ data) & 0x80) 
      crc ^= 0x09;
    data <<= 1;
  }
  return crc;
}

uint8_t sd_cmd(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3) {
  uint8_t crc, a, r;
  
  crc = sd_crc7_byte(0, cmd | 0x40);
  spi_transfer(cmd | 0x40);  
  crc = sd_crc7_byte(crc, arg0);
  spi_transfer(arg0);  
  crc = sd_crc7_byte(crc, arg1);
  spi_transfer(arg1);  
  crc = sd_crc7_byte(crc, arg2);
  spi_transfer(arg2);  
  crc = sd_crc7_byte(crc, arg3);
  spi_transfer(arg3);  
  crc = (crc << 1) | 0x01;  
  spi_transfer(crc);  
  for (a = 10; a > 0; a++) 
    if (((r = spi_transfer(0xFF)) & 0x80) == 0) 
      return r;
  return 0xFF;
}

/*
 * send CMD1 command
 * Returns: 01 = success, error code otherwise
 */
/*
uint8_t sd_cmd1() {
  return sd_cmd_response(0x41, 0x00, 0x00, 0x00, 0x00);
}
*/


/*
 * Send CMD13 and get 32-bit status response
 */
/*
uint8_t sd_cmd13(uint8_t *r1, uint8_t *r2) {
  uint8_t i;
  
  // Send CMD13 with argument 0 (for SPI mode)
  sd_cmd(CMD13, 0x00, 0x00, 0x00, 0x00);
  for (i = 0; i < 20; i++) {
    *r1= spi_transfer(0xFF);
    if (*r1 != 0xFF)
      break;
  }
  *r2= spi_transfer(0xFF);
  return SD_SUCCESS;
}
*/


/*
 * Error code to string function (optional, for debugging)
 * You can remove this if you want to keep code size minimal
 */
const char* sd_error_string(uint8_t error_code) {
  switch (error_code) {
  case SD_SUCCESS:               return "Success";
  case SD_ERROR:                 return "Error";
  case SD_ERROR_CMD0:            return "CMD0 Failed";
  case SD_ERROR_CMD8:            return "CMD8 Failed";
  case SD_ERROR_CMD55:           return "CMD55 Failed";
  case SD_ERROR_CMD58:           return "CMD58 Failed";
  case SD_ERROR_ACMD41:          return "ACMD41 Failed";
  case SD_ERROR_CMD17:           return "CMD17 Failed";
  case SD_ERROR_CMD24:           return "CMD24 Failed";
  case SD_ERROR_V1_CARD:         return "v1.0 sdcard not supported";
  case SD_ERROR_ACMD41_TIMEOUT:  return "ACMD41 timeout";
  case SD_ERROR_UNKNOWN_CMD8:    return "CMD8 returned unexpected response";
  case SD_ERROR_READ_TOKEN:      return "Read data token timeout/error";
  case SD_ERROR_READ_TIMEOUT:    return "Read operation timeout";
  case SD_ERROR_WRITE_REJECT:    return "Write data rejected";
  case SD_ERROR_WRITE_TIMEOUT:   return "Write operation timeout";
  case SD_ERROR_CMD13:           return "CMD13 (READ_STATUS) failed";
  case SD_ERROR_PROTECTED:       return "sdcard is write protected";
  case SD_ERROR_LOCKED:          return "sdcard is locked";
  default:                       
    printf("unknown error code %2x\n", error_code);
    return NULL;
  };
}

uint8_t sd_init(void) {
  sd_init_state_t state;
  int16_t retry;
  int16_t acmd41_retry;
  int16_t i;
  uint8_t r1;
  uint8_t r7_data[4];
  uint8_t ocr_data[4];
  uint8_t sdcard_type;
  uint8_t inactive_count;  

  printf("starting sdcard initialization\n");
  state = SD_INACTIVE;
  inactive_count = 0;
  for (;;) {
    switch(state) {
    case SD_INACTIVE:
      if (inactive_count++ > 3)
	return SD_ERROR;
      printf("SD_INACTIVE\n");
      retry = 0;
      state = SD_POWER_UP;
      break;
      
    case SD_POWER_UP:
      printf("SD_POWER_UP\n");
      // initialize cpol/cpha
      spi_init(100, 0, 0);
      spi_set_frequency_khz(SD_INIT_SPEED);
      // set spi cs high
      sd_deselect(); //spi_cs_high();
      /* Send initial clock cycles to wake up card */
      for (i = 0; i < SD_DUMMY_CLOCKS/8; i++) 
	spi_transfer(0xFF);
      sd_delay(SD_POWER_UP_DELAY);
      sd_select(); //spi_cs_low();
      state = SD_SEND_CMD0;
      break;
      
    case SD_SEND_CMD0:
      printf("SD_SEND_CMD0\n");
      /* Step 1: CMD0 (GO_IDLE_STATE) */
      sd_delay(SD_CMD0_DELAY);
      switch((r1 = sd_cmd(GO_IDLE_STATE, 0x00, 0x00, 0x00, 0x00))) {
      case 0x01:
	retry = 0;
	state = SD_SEND_CMD8;
	break;

      default: // failed
	printf("CMD0 returned: %x\n", r1);
	if (retry++ < SD_CMD0_RETRY) {
	  state = SD_SEND_CMD0;
	} else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD0;
	}
	break;
      }
      break;

    case SD_SEND_CMD8:
      printf("SD_SEND_CMD8\n");
      sd_delay(SD_CMD8_DELAY);
      switch(sd_cmd(SEND_IF_COND, 0x00, 0x00, 0x01, 0xAA)) {
      case 0x01:  // sdcard v2
	// fetch cmd8 data
	for (i = 0; i < 4; i++) 
	  r7_data[i] = spi_transfer(0xFF);
	if (r7_data[3] != 0xAA) {
	  printf("CMD8 check pattern mismatch %2x\n", r7_data[3]);
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
	}
        if ((r7_data[2] & 0x01) == 0) {
	  printf("Voltage range not supported %2x\n", r7_data[2]);
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
        }
	state = SD_V2;
	break;

      case 0x05:  // sdcard v1
	state = SD_V1;
	break;

      default:    /* failed retry */
	printf("CMD8 returned: %x\n", r1);
	if (retry++ < SD_CMD8_RETRY) 
	  state = SD_SEND_CMD8;
	else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
	}
      }
      break;
      
    case SD_V1:  
      printf("SD_V1\n");
      acmd41_retry = retry = 0;
      sdcard_type = SDCARD_V1;
      state = SD_SEND_CMD58;
      break;

    case SD_SEND_CMD58:
      printf("SD_SEND_CMD58\n");
      switch(sd_cmd(READ_OCR, 0x00, 0x00, 0x00, 0x00)) {
      case 0x00: // successfull checking for voltage
	for (i = 0; i < 4; i++) 
	  ocr_data[i] = spi_transfer(0xFF);
	if (ocr_data[1] & 0x30) {
	  retry = 0;
	  state = SD_SEND_ACMD41;
	}	  
	break;

      case 0x05: // illegal command
	printf("CMD58 not supported not an SDCARD\n");
	state = SD_INACTIVE;
	return SD_ERROR_CMD58;
	
      case 0x01: // card still busy retry
	if (retry++ < SD_CMD58_RETRY) 
	  state = SD_SEND_CMD58;
	else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD58;
	}
	break;

      default:
	printf("CMD58 returned: %x\n", r1);
	if (retry++ < SD_CMD58_RETRY) 
	  state = SD_SEND_CMD58;
	else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD58;
	}
	break;
      }
      break;
      
    case SD_V2:
      printf("SD_V2\n");
      acmd41_retry = retry = 0;
      sdcard_type = SDCARD_V2;
      state = SD_SEND_CMD55;
      break; 

    case SD_SEND_CMD55:
      /* Send CMD55 (APP_CMD) */
      printf("SD_SEND_CMD55\n");
      switch((r1 = sd_cmd(APP_CMD, 0x00, 0x00, 0x00, 0x00))) {
      case 0x00:
      case 0x01:
	state = SD_SEND_ACMD41;
	break;
      default:
	printf("CMD55 returned: %x\n", r1);
	if (retry++ < SD_CMD55_RETRY) {
	  sd_delay(SD_CMD55_DELAY);
	  state = SD_SEND_CMD55;
	} else { 
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD55;
	}	  
	break;
      }
      break;
      
    case SD_SEND_ACMD41:
      printf("SD_SEND_ACMD41\n");
      switch(sd_cmd(SD_SEND_OP_COND, (sdcard_type == SDCARD_V2) ? 0x40 : 0x00, 0x00, 0x00, 0x00)) {
      case 0x00:
	printf("INITIALIZED\n");
	state = SD_INITIALIZED;
	break;
      default:	
	if (acmd41_retry++ < SD_ACMD41_RETRY) {
	  retry = 0; // for CMD55
	  state = SD_SEND_CMD55;  
	} else {
	  state = SD_INACTIVE;
	  return SD_ERROR_ACMD41;
	}
	break;
      }
      break;
      
    case SD_INITIALIZED:
      spi_set_frequency_khz(SD_FAST_SPEED);    
      sd_deselect(); 
      return SD_SUCCESS;
    }
  }
}

/*
 * Read single block from SD card
 * block_num: block number to read (for SDHC cards, this is the block number)
 * buffer: 512-byte buffer to store the data
 * Returns: 0 = success, non-zero = error
 */
uint8_t sd_read(unsigned long block_num, uint8_t *buffer)
{
  uint8_t token;
  unsigned int i, a;
  uint8_t crc1, crc2;

  
  sd_select();
  // Send CMD17 (READ_SINGLE_BLOCK) 
  // For SDHC cards, block_num is the block address 
  if (sd_cmd(READ_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), 
	     (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return SD_ERROR_CMD17;
  }
  // Wait for data start token (0xFE) 
  for(a = 5000; a > 0; a--) {
    // Read 512 bytes of data 
    if ((token = spi_transfer(0xFF))  == DATA_START_TOKEN) {
      for (i = 0; i < SD_BLOCK_SIZE; i++) 
	buffer[i] = spi_transfer(0xFF);
      // Read CRC (2 bytes) - we ignore it in SPI mode 
      crc1 = spi_transfer(0xFF);
      crc2 = spi_transfer(0xFF);
      sd_deselect();
      return SD_SUCCESS;
    } else if (token != 0xFF) {
      sd_deselect();
      return SD_ERROR_READ_TOKEN;
    }
  }
  sd_deselect();
  return SD_ERROR_READ_TIMEOUT;
}

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
  
  sd_select();
  /* Send CMD24 (WRITE_BLOCK) */
  if (sd_cmd(WRITE_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return SD_ERROR_CMD24;
  }
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
  if ((data_response & 0x1F) != DATA_ACCEPT_TOKEN) {
    sd_deselect();
    return SD_ERROR_WRITE_REJECT;
  }
  /* Wait for card to finish writing (card will hold MISO low while busy) */
  timeout = 0;
  while (timeout < 65000U) {
    status = spi_transfer(0xFF);
    if (status != 0x00){
      break;  /* Card finished writing */
    }
    timeout++;
  }
  if (timeout >= 65000U) {
    sd_deselect();
    return SD_ERROR_WRITE_TIMEOUT;
  }
  sd_deselect();
  return SD_SUCCESS;
}
