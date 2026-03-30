#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <spi.h>
#include <sdcard.h>
#include <timer.h>

#define CPOL                  0
#define CPHA                  0
#define SD_INIT_SPEED       100
#define SD_FAST_SPEED      1000 // 2500 works with gigastone

#define DUMMY_CLOCKS         80
#define POWER_UP_DELAY       50
#define GO_IDLE_STATE_RETRY  10
#define SEND_IF_COND_RETRY   10
#define SEND_OP_COND_RETRY 1000 

typedef enum {
  ST_POWER_UP,
  ST_GO_IDLE_STATE,
  ST_SEND_IF_COND,
  ST_READ_OCR,
  ST_APP_CMD,
  ST_SEND_OP_COND,
  ST_SET_BLOCKLEN,
  ST_READY
} sd_init_state_t;

uint8_t sdcard_type;

uint8_t sd_type() {
  return sdcard_type;
}

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
  for (a = 8; a > 0; a++) 
    if (((r = spi_transfer(0xFF)) & 0x80) == 0)  
      return r;
  return 0xff;
}

/*
 * Error code to string function (optional, for debugging)
 * You can remove this if you want to keep code size minimal
 */
const char* sd_error_string(uint8_t error_code) {
  switch (error_code) {
  case ER_SUCCESS:            return "Success";
  case ER_ERROR:              return "Error";
  case ER_GO_IDLE_STATE:      return "CMD0   / GO_IDLE_STATE Failed";
  case ER_SEND_IF_COND:       return "CMD8   / SEND_IF_COND Failed";
  case ER_APP_CMD:            return "CMD55  / APP_CMD Failed";
  case ER_READ_OCR:           return "CMD58  / READ_OCR Failed";
  case ER_SEND_OP_COND:       return "ACMD41 / SEND_OP_COND Failed";
  case ER_READ_SINGLE_BLOCK:  return "CMD17  / READ_SINGLE_BLOCK Failed";
  case ER_WRITE_SINGLE_BLOCK: return "CMD24  / WRITE_SIGNLE_BLOCK Failed";
  case ER_CMD1:               return "CMD1 Failed";
  case ER_V1_CARD:            return "v1.0 sdcard not supported";
  case ER_ACMD41_TIMEOUT:     return "ACMD41 timeout";
  case ER_UNKNOWN_CMD8:       return "CMD8 returned unexpected response";
  case ER_READ_TOKEN:         return "Read data token timeout/error";
  case ER_READ_TIMEOUT:       return "Read operation timeout";
  case ER_WRITE_REJECT:       return "Write data rejected";
  case ER_WRITE_TIMEOUT:      return "Write operation timeout";
  case ER_CMD13:              return "CMD13 (READ_STATUS) failed";
  case ER_PROTECTED:          return "sdcard is write protected";
  case ER_LOCKED:             return "sdcard is locked";
  default:                       
    return NULL;
  };
}


 void sd_power_up() {
  uint8_t i;

  spi_init(100, CPOL, CPHA);
  spi_set_frequency_khz(SD_INIT_SPEED);
  sd_deselect();
  sd_delay(1);
  for (i = 0; i < DUMMY_CLOCKS/8; i++) 
    spi_transfer(0xFF);
  sd_deselect();
}

 uint8_t sd_go_idle_state() {
  uint8_t r1;
  
  sd_select();
  r1 = sd_cmd(GO_IDLE_STATE, 0x00, 0x00, 0x00, 0x00);
  sd_deselect();
  return r1;
}

 uint8_t sd_send_if_cond(uint8_t *r7_data) {
  uint8_t i, r1;
  
  sd_select();
  if ((r1 = sd_cmd(SEND_IF_COND, 0x00, 0x00, 0x01, 0xAA)) == R1_IDLE_STATE) {
    for (i = 0; i < 4; i++) 
      r7_data[i] = spi_transfer(0xFF);
  }
  sd_deselect();
  return r1;
}  
  
 uint8_t sd_read_ocr(uint8_t *ocr_data) {
  uint8_t i, r1;
  
  sd_select();
  if ((r1 = sd_cmd(READ_OCR, 0x00, 0x00, 0x00, 0x00)) == R1_READY) {
    for (i = 0; i < 4; i++) 
      ocr_data[i] = spi_transfer(0xFF);
  }
  sd_deselect();
  return r1;
}

 uint8_t sd_send_app()
{
  uint8_t r1;

  sd_select();
  r1 = sd_cmd(APP_CMD, 0x00, 0x00, 0x00, 0x00);
  sd_deselect();
  return r1;
}

 uint8_t sd_send_op_cond()
{
  uint8_t r1;

  sd_select();
  r1 = sd_cmd(SD_SEND_OP_COND, (sdcard_type != SDCARD_V1) ? 0x40 : 0x00, 0x00, 0x00, 0x00);
  sd_deselect();
  return r1;
}

uint8_t sd_set_blocklen()
{
  uint8_t r1;

  sd_select();
  r1 = sd_cmd(SET_BLOCKLEN,  0x00, 0x00, 0x02, 0x00);
  sd_deselect();
  return r1;
}

uint8_t sd_init(void) {
  sd_init_state_t state;
  int16_t retry;
  int16_t acmd41_retry;
  uint8_t r1;
  uint8_t r7_data[4];
  uint8_t ocr_data[4];

  state = ST_POWER_UP;
  for (;;) {
    switch(state) {
    case ST_POWER_UP:
      sd_power_up();
      sd_delay(POWER_UP_DELAY);
      state = ST_GO_IDLE_STATE;
      break;
      
    case ST_GO_IDLE_STATE:
      for (retry = 0; retry < GO_IDLE_STATE_RETRY; retry++) {
	if (sd_go_idle_state() == R1_IDLE_STATE) {
	  state = ST_SEND_IF_COND;
	  break;
	}
      }
      if (retry >= GO_IDLE_STATE_RETRY)  
	return ER_GO_IDLE_STATE;
      break;

    case ST_SEND_IF_COND:
      for (retry = 0; retry < SEND_IF_COND_RETRY; retry++) {
	r1 = sd_send_if_cond(r7_data);
	if (r1 == R1_IDLE_STATE) {
	  if (r7_data[3] != 0xAA || (r7_data[2] & 0x01) == 0)
	    return ER_SEND_IF_COND;
	  sdcard_type = SDCARD_V2;
	  state = ST_APP_CMD;
	  break;
	} else if (r1 == 0x05) {
	  sdcard_type = SDCARD_V1;
	  state = ST_APP_CMD;
	  break;
	} 
      }
      if (retry >= SEND_IF_COND_RETRY)  // Only error if all retries failed
	return ER_SEND_IF_COND;
      break;

    case ST_READ_OCR:
      r1 = sd_read_ocr(ocr_data);
      if (r1 == R1_READY) {
	if (sdcard_type != SDCARD_V1)
	  sdcard_type = (ocr_data[0] & 0x40) ? SDCARD_SDHC : SDCARD_SDSC;
	state = ST_SET_BLOCKLEN;
      } else {
	return ER_READ_OCR;
      }
      break;

    case ST_APP_CMD:
      if (sd_send_app() <= R1_IDLE_STATE) 
	state = ST_SEND_OP_COND;
      else if (acmd41_retry++ < SEND_OP_COND_RETRY)
	state = ST_APP_CMD;
      else
	return ER_APP_CMD;
      break;
      
    case ST_SEND_OP_COND:
      if (sd_send_op_cond() == R1_READY)
	state = ST_READ_OCR;
      else if (acmd41_retry++ < SEND_OP_COND_RETRY)
	state = ST_APP_CMD;
      else
	return ER_SEND_OP_COND;  
      break;
      
    case ST_SET_BLOCKLEN:
      if (sd_set_blocklen() == R1_READY) 
	state = ST_READY;
      else 
	return ER_SET_BLOCKLEN;
      break;

    case ST_READY:
      spi_set_frequency_khz(SD_FAST_SPEED);    
      sd_deselect(); 
      return ER_SUCCESS;
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
  if (sdcard_type != SDCARD_SDHC)
    block_num *= 512;
  if (sd_cmd(READ_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), 
	     (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return ER_READ_SINGLE_BLOCK;
  }
  for(a = 5000; a > 0; a--) {
    if ((token = spi_transfer(0xFF))  == DATA_START_TOKEN) {
      for (i = 0; i < SD_BLOCK_SIZE; i++) 
	buffer[i] = spi_transfer(0xFF);
      crc1 = spi_transfer(0xFF);
      crc2 = spi_transfer(0xFF);
      (void) crc1;
      (void) crc2;      
      sd_deselect();
      return ER_SUCCESS;
    } else if (token != 0xFF) {
      sd_deselect();
      return ER_READ_TOKEN;
    }
  }
  sd_deselect();
  return ER_READ_TIMEOUT;
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
      return ER_CMD13;
    }
  }
  if (sdcard_type != SDCARD_SDHC)
    block_num *= 512;
  if (sd_cmd(WRITE_SINGLE_BLOCK, (uint8_t)(block_num >> 24), (uint8_t)(block_num >> 16), (uint8_t)(block_num >> 8), (uint8_t)(block_num)) != 0x00) {
    sd_deselect();
    return ER_WRITE_SINGLE_BLOCK;
  }
  spi_transfer(DATA_START_TOKEN);
  for (i = 0; i < SD_BLOCK_SIZE; i++) 
    spi_transfer(buffer[i]);
  spi_transfer(0xFF);
  spi_transfer(0xFF);
  data_response = spi_transfer(0xFF);
  if ((data_response & 0x1F) != DATA_ACCEPT_TOKEN) {
    sd_deselect();
    return ER_WRITE_REJECT;
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
    return ER_WRITE_TIMEOUT;
  }
  sd_deselect();
  return ER_SUCCESS;
}
