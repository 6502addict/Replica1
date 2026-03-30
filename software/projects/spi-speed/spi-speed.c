#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <conio.h>
#include "timer.h" 
#include "spi.h" 

/* Target SPI frequencies in kHz */
#define SPI_INIT_SPEED_KHZ    150   /* For SD card initialization */
#define SPI_FAST_SPEED_KHZ    500   /* For normal operation */


int main(void) {
  int i;
  uint8_t cpu_speed;
  static char input[128];
  
  printf("SPI Frequency Test Program\n");
  printf("==========================\n");
  
  /* Read CPU speed from register if available */
  cpu_speed = timer_cpu_speed();
  printf("CPU Speed: %d MHz\n\n", cpu_speed);
  
  /* Initialize SPI */
  spi_init(100, 0, 0); /* Start with slow divisor */

  printf("start logic analyser and press return\n");
  for(;;) {
    if (kbhit())
      break;
  }

  spi_set_frequency_khz(SPI_INIT_SPEED_KHZ);
  
  for (i = 0; i < 100; i++) 
    spi_transfer(0xAA);

  spi_set_frequency_khz(SPI_FAST_SPEED_KHZ);

  for (i = 0; i < 100; i++) 
    spi_transfer(0xAA);

  printf("start logic analyser and press return\n");

}
