#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>
#include <timer.h>

#define CPOL                  0
#define CPHA                  0
#define SD_INIT_SPEED       100
#define SD_FAST_SPEED      1000 // 2500 works with gigastone

#define SD_DUMMY_CLOCKS      80
#define SD_POWER_UP_DELAY   250
#define SD_CMD0_RETRY        10
#define SD_CMD0_DELAY       100
#define SD_CMD1_DELAY         5
#define SD_CMD1_RETRY        25
#define SD_CMD8_RETRY       100
#define SD_CMD8_DELAY        50
#define SD_CMD5_RETRY        10
#define SD_CMD55_DELAY      250
#define SD_CMD55_RETRY       10
#define SD_CMD58_RETRY       10
#define SD_ACMD41_RETRY    1000 

typedef enum {
  SD_INACTIVE,
  SD_POWER_UP,
  SD_SEND_CMD0,
  SD_SEND_CMD1,
  SD_SEND_CMD8,
  SD_V1,
  SD_V2,
  SD_SEND_CMD58,
  SD_SEND_CMD55,
  SD_SEND_ACMD41,
  SD_SEND_CMD16,
  SD_INITIALIZED
} sd_init_state_t;

uint8_t sdcard_type;

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
  case SD_ERROR_CMD1:            return "CMD1 Failed";
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
  uint8_t inactive_count;  

  state = SD_INACTIVE;
  inactive_count = 0;
  for (;;) {
    switch(state) {
    case SD_INACTIVE:
      if (inactive_count++ > 3)
	return SD_ERROR;
      retry = 0;
      state = SD_POWER_UP;
      break;
      
    case SD_POWER_UP:
      spi_init(100, CPOL, CPHA);
      spi_set_frequency_khz(SD_INIT_SPEED);
      sd_deselect();
      for (i = 0; i < SD_DUMMY_CLOCKS/8; i++) 
	spi_transfer(0xFF);
      sd_delay(SD_POWER_UP_DELAY);
      state = SD_SEND_CMD0;
      break;
      
    case SD_SEND_CMD0:
      sd_select(); 
      sd_delay(SD_CMD0_DELAY);
      switch((r1 = sd_cmd(GO_IDLE_STATE, 0x00, 0x00, 0x00, 0x00))) {
      case R1_IDLE_STATE:
	retry = 0;
	state = SD_SEND_CMD8;
	break;

      default:
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
      sd_delay(SD_CMD8_DELAY);
      switch(sd_cmd(SEND_IF_COND, 0x00, 0x00, 0x01, 0xAA)) {
      case R1_IDLE_STATE:
	for (i = 0; i < 4; i++) 
	  r7_data[i] = spi_transfer(0xFF);
	if (r7_data[3] != 0xAA) {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
	}
        if ((r7_data[2] & 0x01) == 0) {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
        }
	state = SD_V2;
	break;

      case 0x05: 
	state = SD_V1;
	break;

      default: 
	if (retry++ < SD_CMD8_RETRY) 
	  state = SD_SEND_CMD8;
	else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD8;
	}
      }
      break;
      
    case SD_V1:  
      acmd41_retry = retry = 0;
      sdcard_type = SDCARD_V1;
      state = SD_SEND_CMD55;
      break;

    case SD_SEND_CMD1:
      sd_delay(SD_CMD1_DELAY);
      switch((r1 = sd_cmd(SEND_OP_COND, 0x00, 0x00, 0x00, 0x00))) {
      case R1_IDLE_STATE:
	state = SD_SEND_CMD16;
	break;

      default:
	if (retry++ < SD_CMD1_RETRY) {
	  state = SD_SEND_CMD1;
	} else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD1;
	}
	break;
      }
      break;
      

    case SD_SEND_CMD58:
      switch(sd_cmd(READ_OCR, 0x00, 0x00, 0x00, 0x00)) {
      case R1_READY: 
	for (i = 0; i < 4; i++) 
	  ocr_data[i] = spi_transfer(0xFF);
	if (sdcard_type == SDCARD_V1) {
	  if (ocr_data[1] & 0x30) {
	    retry = 0;
	    state = SD_SEND_ACMD41;
	  }	  
	} else {
	  if (ocr_data[0] & 0x40) 
	    sdcard_type = SDCARD_SDHC;
	  else 
	    sdcard_type = SDCARD_SDSC;

	}
	state = SD_SEND_CMD16;
	break;

      case R1_ILLEGAL_CMD_IDLE: 
	state = SD_INACTIVE;
	return SD_ERROR_CMD58;
	
      case R1_IDLE_STATE:
	if (retry++ < SD_CMD58_RETRY) 
	  state = SD_SEND_CMD58;
	else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD58;
	}
	break;

      default:
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
      acmd41_retry = retry = 0;
      sdcard_type = SDCARD_V2;
      state = SD_SEND_CMD55;
      break; 

    case SD_SEND_CMD55:
      switch((r1 = sd_cmd(APP_CMD, 0x00, 0x00, 0x00, 0x00))) {
      case R1_READY:
      case R1_IDLE_STATE:
      	state = SD_SEND_ACMD41;
      	break;
      default:
	if (acmd41_retry++ < SD_ACMD41_RETRY) {
	  state = SD_SEND_CMD55;
	} else {
	  state = SD_INACTIVE;
	  return SD_ERROR_CMD55;
	}
      }
      break;

    case SD_SEND_ACMD41:
      sd_delay(50);
      switch(sd_cmd(SD_SEND_OP_COND, (sdcard_type != SDCARD_V1) ? 0x40 : 0x00, 0x00, 0x00, 0x00)) {
      case R1_READY:
	state = SD_SEND_CMD58;
	break;

      default:	
	if (acmd41_retry++ < SD_ACMD41_RETRY) {
	  retry = 0; 
	  state = SD_SEND_CMD55;  
	} else {
	  state = SD_INACTIVE;
	  return SD_ERROR_ACMD41;
	}
	break;
      }
      break;
      
    case SD_SEND_CMD16:
      if (sd_cmd(SET_BLOCKLEN, 0x00, 0x00, 0x02, 0x00) == R1_READY) {
	state = SD_INITIALIZED;
      } else {
	state = SD_INACTIVE;
	return SD_ERROR_SET_BLOCKLEN;
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
  if (sdcard_type = SDCARD_SDHC)
    block_num *= 512;
  if (sd_cmd(READ_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), 
	     (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return SD_ERROR_CMD17;
  }
  for(a = 5000; a > 0; a--) {
    if ((token = spi_transfer(0xFF))  == DATA_START_TOKEN) {
      for (i = 0; i < SD_BLOCK_SIZE; i++) 
	buffer[i] = spi_transfer(0xFF);
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
  if (sd_cmd(SEND_STATUS, 0x00, 0x00, 0x00, 0x00) == 0x00) {
    switch(spi_transfer(0xFF)) {
    case 0x00:
      break;
    default: 
      sd_deselect();
      return SD_ERROR_CMD13;
    }
  }
  if (sdcard_type = SDCARD_SDHC)
    block_num *= 512;
  if (sd_cmd(WRITE_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return SD_ERROR_CMD24;
  }
  spi_transfer(DATA_START_TOKEN);
  for (i = 0; i < SD_BLOCK_SIZE; i++) 
    spi_transfer(buffer[i]);
  spi_transfer(0xFF);
  spi_transfer(0xFF);
  data_response = spi_transfer(0xFF);
  if ((data_response & 0x1F) != DATA_ACCEPT_TOKEN) {
    sd_deselect();
    return SD_ERROR_WRITE_REJECT;
  }
  timeout = 0;
  while (timeout < 65000U) {
    status = spi_transfer(0xFF);
    if (status != 0x00)
      break; 
    timeout++;
  }
  if (timeout >= 65000U) {
    sd_deselect();
    return SD_ERROR_WRITE_TIMEOUT;
  }
  sd_deselect();
  return SD_SUCCESS;
}
