#include <stdio.h>
#include <stdint.h>

uint8_t sd_crc7(uint8_t *cmd_buffer) {
  uint8_t crc;
  uint8_t i, j;
  uint8_t byte;
  
  crc = 0;
  
  /* Process each of the 5 command bytes */
  for (i = 0; i < 5; i++) {
    byte = cmd_buffer[i];
    
    /* Process each bit of the byte */
    for (j = 0; j < 8; j++) {
      /* Check if we need to XOR with polynomial */
      if ((crc & 0x40) != 0) 
	crc = (crc << 1) ^ 0x09;  /* CRC7 polynomial: x^7 + x^3 + 1 */
      else 
	crc = crc << 1;
      /* XOR with current data bit */
      if ((byte & 0x80) != 0) 
	crc = crc ^ 0x09;
      byte = byte << 1;
    }
  }
  /* Add stop bit and return */
  return (crc & 0x7F) | 0x01;
}

/*
 * Calculate CRC7 for specific SD card commands
 * cc65 compatible version using separate argument bytes
 */
uint8_t sd_get_crc(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3) {
  uint8_t cmd_buffer[5];
  
  /* Build command buffer */
  cmd_buffer[0] = cmd;
  cmd_buffer[1] = arg0;  /* MSB */
  cmd_buffer[2] = arg1;
  cmd_buffer[3] = arg2;
  cmd_buffer[4] = arg3;  /* LSB */
  
  return sd_crc7(cmd_buffer);
}

/*
 * Pre-calculated CRC7 values for common SD commands
 * Use these for faster execution
 */
uint8_t sd_common_crc(uint8_t cmd, uint8_t arg0, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
  /* CMD0 (GO_IDLE_STATE) with arg 0x00000000 */
  if (cmd == 0x40 && arg0 == 0x00 && arg1 == 0x00 && arg2 == 0x00 && arg3 == 0x00) 
    return 0x95;
  
  /* CMD8 (SEND_IF_COND) with arg 0x000001AA */
  if (cmd == 0x48 && arg0 == 0x00 && arg1 == 0x00 && arg2 == 0x01 && arg3 == 0xAA) 
    return 0x87;
  
  /* CMD55 (APP_CMD) with arg 0x00000000 */
  if (cmd == 0x77 && arg0 == 0x00 && arg1 == 0x00 && arg2 == 0x00 && arg3 == 0x00) 
    return 0x65;
  
  /* ACMD41 (SD_SEND_OP_COND) with arg 0x40000000 */
  if (cmd == 0x69 && arg0 == 0x40 && arg1 == 0x00 && arg2 == 0x00 && arg3 == 0x00) 
    return 0x77;
  
  /* Calculate dynamically for other commands */
  return sd_get_crc(cmd, arg0, arg1, arg2, arg3);
}
