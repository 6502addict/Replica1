/*
 * File: include/spi.h
 * SPI Library Header File
 */

#ifndef SPI_H
#define SPI_H

/* SPI register addresses */
#define SPI_COMMAND  ((uint8_t*) SPI_ADDR + 0)
#define SPI_STATUS   ((uint8_t*) SPI_ADDR + 1)  
#define SPI_DATA     ((uint8_t*) SPI_ADDR + 2)
#define SPI_DIVISOR  ((uint8_t*) SPI_ADDR + 3)

/* Status register bits */
#define SPI_DATA_READY  0x01
#define SPI_BUSY_N      0x02

#ifdef PICO_BUILD
// Pico: Rename to avoid SDK conflict
void spi_pico_init(uint8_t divisor, uint8_t cpol, uint8_t cpha);
#define spi_init(d,p,h) spi_pico_init(d,p,h)
#else
// Normal platforms
void spi_init(uint8_t divisor, uint8_t cpol, uint8_t cpha);
#endif

/* Function prototypes */
//void    spi_init(uint8_t, uint8_t, uint8_t);
void    spi_set_divisor(uint8_t);
void    spi_set_mode(uint8_t, uint8_t);
void    spi_cs_low(void);
void    spi_cs_high(void);
uint8_t spi_calculate_divisor(uint16_t);
void    spi_set_frequency_khz(uint16_t);
uint8_t spi_transfer(uint8_t);


#endif /* SPI_H */
