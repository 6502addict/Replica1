#include <stdio.h>
#include <stdint.h>
#include <sdcard.h>


/*
 * Error code to string function (optional, for debugging)
 * You can remove this if you want to keep code size minimal
 */
const char* sd_error_string(uint8_t error_code) {
    switch (error_code) {
        case SD_SUCCESS:
	  return "Success";
        case SD_ERROR_CMD0:
	  return "CMD0 failed";
        case SD_ERROR_CMD8:
	  return "CMD8 failed";
        case SD_ERROR_ACMD41:
	  return "ACMD41 failed";
        case SD_ERROR_V1_CARD:
	  return "SD v1.x not supported";
        case SD_ERROR_UNKNOWN_CMD8:
	  return "CMD8 unexpected response";
        case SD_ERROR_CMD55:
	  return "CMD55 failed";
        case SD_ERROR_ACMD41_TIMEOUT:
	  return "ACMD41 timeout";
        case SD_ERROR_CMD17:
	  return "CMD17 failed";
        case SD_ERROR_READ_TOKEN:
	  return "Read token error";
        case SD_ERROR_READ_TIMEOUT:
	  return "Read timeout";
        case SD_ERROR_CMD24:
	  return "CMD24 failed";
        case SD_ERROR_WRITE_REJECT:
	  return "Write rejected";
        case SD_ERROR_WRITE_TIMEOUT:
	  return "Write timeout";
        default:
	  return "Unknown error";
    }
}
