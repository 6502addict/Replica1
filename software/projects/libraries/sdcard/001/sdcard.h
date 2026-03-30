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
#define SD_ERROR                 0x01  /* General ERROR */
#define SD_ERROR_CMD0            0x02  /* CMD0 (GO_IDLE_STATE) failed */
#define SD_ERROR_CMD8            0x03  /* CMD8 (SEND_IF_COND) failed */
#define SD_ERROR_CMD55           0x04  /* CMD55 (APP_CMD) failed */
#define SD_ERROR_CMD58           0x05  /* CMD55 (APP_CMD) failed */
#define SD_ERROR_ACMD41          0x06  /* ACMD41 initialization failed */
#define SD_ERROR_CMD17           0x07  /* CMD17 (READ_SINGLE_BLOCK) failed */
#define SD_ERROR_CMD24           0x08  /* CMD24 (WRITE_BLOCK) failed */
#define SD_ERROR_V1_CARD         0x09  /* SD v1.x card detected (not supported) */

/* Additional error codes for other functions */
#define SD_ERROR_ACMD41_TIMEOUT  0x12  /* ACMD41 timeout */
#define SD_ERROR_UNKNOWN_CMD8    0x20  /* CMD8 returned unexpected response */
#define SD_ERROR_READ_TOKEN      0x21  /* Read data token timeout/error */
#define SD_ERROR_READ_TIMEOUT    0x22  /* Read operation timeout */
#define SD_ERROR_WRITE_REJECT    0x31  /* Write data rejected */
#define SD_ERROR_WRITE_TIMEOUT   0x32  /* Write operation timeout */
#define SD_ERROR_CMD13           0x40  /* CMD13 (READ_STATUS) failed */
#define SD_ERROR_PROTECTED       0x41  /* sdcard is write protected */
#define SD_ERROR_LOCKED          0x42  /* sdcard is locked */

//uint8_t     sd_crc7(uint8_t *);
//uint8_t     sd_get_crc(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
//uint8_t     sd_common_crc(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_cmd(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_read(unsigned long, uint8_t *);
uint8_t     sd_write(unsigned long , uint8_t *);
//uint8_t     sd_r1_response(void);
uint8_t     sd_cmd8(uint8_t *);
//uint8_t     sd_r1_data(uint8_t *, uint8_t);
void        sd_delay(uint16_t);
//uint8_t     sd_cmd_response(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
//uint8_t     sd_acmd41(uint8_t);
//uint8_t     sd_cmd58(uint8_t *);
uint8_t     sd_init(void);
//uint8_t     sd_cmd0(void);
uint8_t     sd_cmd1(void);
uint8_t     sd_cmd13(uint8_t *, uint8_t *);
uint8_t     sd_protected();
//uint8_t     sd_cmd55(void);
//uint8_t     sd_wait_ready(void);
//void        sd_select(void);
//void        sd_deselect(void);
const char* sd_error_string(uint8_t);

#define sd_delay(ms)    timer_delay_ms(ms)
#define sd_select()     spi_cs_low()
#define sd_deselect()   spi_cs_high()



#endif /* SDCARD_H */
