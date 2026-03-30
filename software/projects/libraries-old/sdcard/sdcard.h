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
#define SD_SUCCESS               0x00
#define SD_ERROR                 0x01
#define ER_SUCCESS               0x00  /* Success */
#define ER_ERROR                 0x01  /* General ERROR */
#define ER_GO_IDLE_STATE         0x02  /* CMD0 (GO_IDLE_STATE) failed */
#define ER_SEND_IF_COND          0x03  /* CMD8 (SEND_IF_COND) failed */
#define ER_APP_CMD               0x04  /* CMD55 (APP_CMD) failed */
#define ER_READ_OCR              0x05  /* CMD58 (READ_OCR) failed */
#define ER_SEND_OP_COND          0x06  /* ACMD41 initialization failed */
#define ER_READ_SINGLE_BLOCK     0x07  /* CMD17 (READ_SINGLE_BLOCK) failed */
#define ER_WRITE_SINGLE_BLOCK    0x08  /* CMD24 (WRITE_BLOCK) failed */
#define ER_CMD1                  0x09  /* CMD1 (SEND_OP_COND) failed */
#define ER_V1_CARD               0x0A  /* SD v1.x card detected (not supported) */

/* Additional error codes for other functions */
#define ER_ACMD41_TIMEOUT        0x12  /* ACMD41 timeout */
#define ER_UNKNOWN_CMD8          0x20  /* CMD8 returned unexpected response */
#define ER_READ_TOKEN            0x21  /* Read data token timeout/error */
#define ER_READ_TIMEOUT          0x22  /* Read operation timeout */
#define ER_WRITE_REJECT          0x31  /* Write data rejected */
#define ER_WRITE_TIMEOUT         0x32  /* Write operation timeout */
#define ER_CMD13                 0x40  /* CMD13 (READ_STATUS) failed */
#define ER_PROTECTED             0x41  /* sdcard is write protected */
#define ER_LOCKED                0x42  /* sdcard is locked */
#define ER_SET_BLOCKLEN          0x43  /* SET_BLOCKLEN failed */

#define R1_READY                 0x00
#define R1_IDLE_STATE            0x01
#define R1_ERASE_RESET           0x02
#define R1_ILLEGAL_COMMAND       0x04
#define R1_COM_CRC_ERROR         0x08
#define R1_ERASE_SEQ_ERROR       0x10
#define R1_ADDRESS_ERROR         0x20
#define R1_PARAMETER_ERROR       0x40
#define R1_ILLEGAL_CMD_IDLE      0x05
#define R1_ADDRESS_ERROR_IDLE    0x21
#define R1_NO_RESPONSE           0xFF

#define SDCARD_V1            0
#define SDCARD_V2            1
#define SDCARD_SDSC          2
#define SDCARD_SDHC          3

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

#define sd_delay(ms)    timer_delay_ms(ms)
#define sd_select()     spi_transfer(0xff); spi_cs_low();  spi_transfer(0xff)
#define sd_deselect()   spi_transfer(0xff); spi_cs_high(); spi_transfer(0xff)


uint8_t     sd_cmd(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
uint8_t     sd_read(unsigned long, uint8_t *);
uint8_t     sd_write(unsigned long , uint8_t *);
uint8_t     sd_init(void);
uint8_t     sd_type(void);
const char* sd_error_string(uint8_t);



#endif /* SDCARD_H */
