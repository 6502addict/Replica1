#include <stdio.h>
#include <stdint.h>
#include "spi.h"

uint8_t  __fastcall__ spi_transfer(uint8_t data)
{
  while (!(*SPI_STATUS & SPI_BUSY_N));
  *SPI_DATA = data;
  while (!(*SPI_STATUS & SPI_DATA_READY));
  return *SPI_DATA;
}



