#include <stdio.h>
#include <stdint.h>
#include <timer.h>
#include "../config.h"
#include "spi.h"

void spi_cs_low(void) {
    *SPI_COMMAND |= 0x04;  /* Set CS bit */
}

void  spi_cs_high(void)  {
    *SPI_COMMAND &= ~0x04;  /* Clear CS bit */
}

void spi_set_divisor(uint8_t divisor) {
    *SPI_DIVISOR = divisor;
}

void spi_set_mode(uint8_t cpol, uint8_t cpha) {
  *SPI_COMMAND = (*SPI_COMMAND & 0x03) | (cpol | (cpha << 1));
}

uint8_t spi_calculate_divisor(uint16_t target_khz) {
    const uint32_t spi_clock_khz = 30000;  // Fixed 60MHz SPI clock
    int16_t divisor;
    
    divisor = spi_clock_khz / (8 * target_khz);
    
    /* Clamp to valid range */
    if (divisor < 0) divisor = 0;
    if (divisor > 255) divisor = 255;
    
    printf("SPI Clock: 30 MHz, Target: %d kHz, Divisor: %d\n",
           target_khz, divisor);
    return (uint8_t)divisor;
}

void spi_set_frequency_khz(uint16_t target_khz) {
    uint8_t divisor = spi_calculate_divisor(target_khz);
    spi_set_divisor(divisor);
}

void spi_init(uint8_t divisor, uint8_t cpol, uint8_t cpha) {
  spi_set_divisor(divisor);
  spi_set_mode(cpol, cpha);
  spi_cs_high();
}

uint8_t  spi_transfer(uint8_t data)
{
  while (!(*SPI_STATUS & SPI_BUSY_N));
  *SPI_DATA = data;
  while (!(*SPI_STATUS & SPI_DATA_READY));
  return *SPI_DATA;
}
