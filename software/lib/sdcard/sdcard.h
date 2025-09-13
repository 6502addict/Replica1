/*
 * File: include/sdcard.h
 * SDCARD Library Header File
 */

#ifndef SDCARD_H
#define SDCARD_H

/* SD Card block size */
#define SD_BLOCK_SIZE            512

/* Data tokens */
#define DATA_START_TOKEN         0xFE
#define DATA_ACCEPT_TOKEN        0x05
#define DATA_REJECT_CRC          0x0B
#define DATA_REJECT_WRITE        0x0D

/* SD Card initialization error codes */
#define SD_SUCCESS               0x00  /* Success */
#define SD_ERROR_CMD0            0x01  /* CMD0 (GO_IDLE_STATE) failed */
#define SD_ERROR_CMD8            0x02  /* CMD8 (SEND_IF_COND) failed */
#define SD_ERROR_ACMD41          0x03  /* ACMD41 initialization failed */
#define SD_ERROR_V1_CARD         0x04  /* SD v1.x card detected (not supported) */
#define SD_ERROR_UNKNOWN_CMD8    0x05  /* CMD8 returned unexpected response */

/* Additional error codes for other functions */
#define SD_ERROR_CMD55           0x10  /* CMD55 (APP_CMD) failed */
#define SD_ERROR_ACMD41_TIMEOUT  0x11  /* ACMD41 timeout */
#define SD_ERROR_CMD17           0x20  /* CMD17 (READ_SINGLE_BLOCK) failed */
#define SD_ERROR_READ_TOKEN      0x21  /* Read data token timeout/error */
#define SD_ERROR_READ_TIMEOUT    0x22  /* Read operation timeout */
#define SD_ERROR_CMD24           0x30  /* CMD24 (WRITE_BLOCK) failed */
#define SD_ERROR_WRITE_REJECT    0x31  /* Write data rejected */
#define SD_ERROR_WRITE_TIMEOUT   0x32  /* Write operation timeout */
#define SD_ERROR_CMD13           0x40  /* CMD13 (READ_STATUS) failed */
#define SD_ERROR_PROTECTED       0x41  /* sdcard is write protected */
#define SD_ERROR_LOCKED          0x42  /* sdcard is locked */

uint8_t     sd_crc7(uint8_t *);
uint8_t     sd_get_crc(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_common_crc(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void        sd_cmd(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_read(unsigned long, uint8_t *);
uint8_t     sd_write(unsigned long , uint8_t *);
uint8_t     sd_r1_response(void);
uint8_t     sd_cmd8(uint8_t *);
uint8_t     sd_r1_data(uint8_t *, uint8_t);
void        sd_delay(void);
uint8_t     sd_cmd_response(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_acmd41(void);
uint8_t     sd_init(void);
uint8_t     sd_cmd0(void);
uint8_t     sd_cmd13(uint32_t *);
uint8_t     sd_protected();
uint8_t     sd_cmd55(void);
const char* sd_error_string(uint8_t);

#endif /* SDCARD_H */
