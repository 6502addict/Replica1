/*
 * File: include/spi.h
 * SPI Library Header File
 */

#ifndef SPI_H
#define SPI_H

/* SPI register addresses */
#define SPI_COMMAND  ((uint8_t*)0xC200)
#define SPI_STATUS   ((uint8_t*)0xC201)  
#define SPI_DATA     ((uint8_t*)0xC202)
#define SPI_DIVISOR  ((uint8_t*)0xC203)

/* Status register bits */
#define SPI_DATA_READY  0x01
#define SPI_BUSY_N      0x02

/* Function prototypes */
void __fastcall__ spi_init(uint8_t, uint8_t, uint8_t);
void __fastcall__ spi_set_divisor(uint8_t);
void __fastcall__ spi_set_mode(uint8_t, uint8_t);
void __fastcall__ spi_cs_low(void);
void __fastcall__ spi_cs_high(void);
uint8_t __fastcall__ spi_transfer(uint8_t);


#endif /* SPI_H */
