# Requires cc65 toolchain installed

# Compiler and tools
CC65_HOME=/usr/local/share/cc65 


# Default target
all: sdcard.lib

# Build SPI library
sdcard.lib: sd_crc7.o sd_read.o sd_write.o sd_cmd.o sd_r1_response.o sd_cmd8.o sd_r1_data.o sd_acmd41.o sd_delay.o sd_cmd_response.o \
            sd_init.o sd_cmd0.o sd_cmd55.o sd_error.o sd_cmd13.o sd_protected.o
	ar65 r sdcard.lib sd_crc7.o sd_read.o sd_write.o sd_cmd.o sd_r1_response.o sd_cmd8.o sd_r1_data.o sd_acmd41.o sd_delay.o sd_cmd_response.o sd_init.o \
            sd_cmd0.o sd_cmd55.o sd_error.o sd_cmd13.o sd_protected.o
	@echo "SDCARD library created: sdcard.lib"

sd_crc7.s: sd_crc7.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_crc7.c 

sd_read.s: sd_read.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_read.c 

sd_write.s: sd_write.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_write.c 

sd_cmd.s: sd_cmd.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd.c 

sd_r1_response.s: sd_r1_response.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_r1_response.c

sd_cmd8.s: sd_cmd8.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd8.c

sd_r1_data.s: sd_r1_data.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_r1_data.c

sd_acmd41.s: sd_acmd41.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_acmd41.c

sd_delay.s: sd_delay.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_delay.c

sd_cmd_response.s: sd_cmd_response.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd_response.c

sd_init.s: sd_init.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_init.c

sd_cmd0.s: sd_cmd0.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd0.c

sd_cmd55.s: sd_cmd55.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd55.c

sd_error.s: sd_error.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_error.c

sd_cmd13.s: sd_cmd13.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_cmd13.c

sd_protected.s: sd_protected.c sdcard.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I. -I../spi -t replica1 sd_protected.c

sd_crc7.o: sd_crc7.s 
	CC65_HOME=/usr/local/share/cc65 ca65  sd_crc7.s

sd_read.o: sd_read.s 
	CC65_HOME=/usr/local/share/cc65 ca65  sd_read.s

sd_write.o: sd_write.s 
	CC65_HOME=/usr/local/share/cc65 ca65  sd_write.s

sd_cmd.o: sd_cmd.s 
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd.s

sd_r1_response.o: sd_r1_response.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_r1_response.s

sd_cmd8.o: sd_cmd8.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd8.s

sd_r1_data.o: sd_r1_data.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_r1_data.s

sd_acmd41.o: sd_acmd41.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_acmd41.s

sd_delay.o: sd_delay.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_delay.s

sd_cmd_response.o: sd_cmd_response.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd_response.s

sd_init.o: sd_init.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_init.s

sd_cmd0.o: sd_cmd0.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd0.s

sd_cmd55.o: sd_cmd55.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd55.s

sd_error.o: sd_error.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_error.s

sd_cmd13.o: sd_cmd13.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_cmd13.s

sd_protected.o: sd_protected.s
	CC65_HOME=/usr/local/share/cc65 ca65  sd_protected.s


# Clean build files
clean:
	rm -f *.o *.map  *~ *.s

# Install libraries to cc65 lib directory (optional)
install: $(SPI_LIB) $(SDCARD_LIB)
	cp $(SPI_LIB) $(SDCARD_LIB) $(CC65_HOME)/lib/
	cp include/*.h $(CC65_HOME)/include/

# Create directory structure
setup:
	mkdir -p src include examples

.PHONY: all clean install setup




