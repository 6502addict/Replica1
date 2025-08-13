#include <stdio.h>
#include <stdint.h>
#include <spi.h>
#include <sdcard.h>


/*
 * Simple delay function (implement based on your system)
 */
void sd_delay(void)
{
    unsigned int i;
    for (i = 0; i < 1000; i++) {
        /* Simple delay loop */
    }
}
