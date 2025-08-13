#include <stdio.h>
#include <stdint.h>
#include "spi.h"

void __fastcall__ spi_cs_low(void) {
    *SPI_COMMAND |= 0x04;  /* Set CS bit */
}

void __fastcall__ spi_cs_high(void)  {
    *SPI_COMMAND &= ~0x04;  /* Clear CS bit */
}

void __fastcall__ spi_set_divisor(uint8_t divisor) {
    *SPI_DIVISOR = divisor;
}

void __fastcall__ spi_set_mode(uint8_t cpol, uint8_t cpha) {
  *SPI_COMMAND = (*SPI_COMMAND & 0x03) | (cpol | (cpha << 1));
}

void __fastcall__ spi_init(uint8_t divisor, uint8_t cpol, uint8_t cpha) {
  spi_set_divisor(divisor);
  spi_set_mode(cpol, cpha);
  spi_cs_high();
}

