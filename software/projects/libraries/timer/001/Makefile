# Requires cc65 toolchain installed
include ../../common.mk

# Default target
all: timer.lib timer.txt

# Build SPI library
timer.lib: timer_start.o timer_stop.o timer_read.o timer_is_running.o timer_ticks_to_us.o timer_ticks_to_ms.o timer_delay_ticks.o timer_delay_us.o timer_delay_ms.o timer_cpu_speed.o
	ar65 r timer.lib timer_start.o timer_stop.o timer_read.o timer_is_running.o timer_ticks_to_us.o timer_ticks_to_ms.o timer_delay_ticks.o timer_delay_us.o timer_delay_ms.o timer_cpu_speed.o
	@echo "TIMER library created: timer.lib"

timer_start.s: timer_start.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_start.c 

timer_stop.s: timer_stop.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_stop.c 

timer_read.s: timer_read.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_read.c 

timer_is_running.s: timer_is_running.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_is_running.c 

timer_cpu_speed.s: timer_cpu_speed.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_cpu_speed.c 

timer_ticks_to_us.s: timer_ticks_to_us.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_ticks_to_us.c 

timer_ticks_to_ms.s: timer_ticks_to_ms.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_ticks_to_ms.c 

timer_delay_ticks.s: timer_delay_ticks.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_delay_ticks.c 

timer_delay_us.s: timer_delay_us.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_delay_us.c 

timer_delay_ms.s: timer_delay_ms.c timer.h
	CC65_HOME=/usr/local/share/cc65 cc65 -O -I.  -t $(TARGET) timer_delay_ms.c 

timer_start.o: timer_start.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_start.s

timer_stop.o: timer_stop.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_stop.s

timer_read.o: timer_read.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_read.s

timer_is_running.o: timer_is_running.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_is_running.s

timer_cpu_speed.o: timer_cpu_speed.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_cpu_speed.s

timer_ticks_to_us.o: timer_ticks_to_us.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_ticks_to_us.s

timer_ticks_to_ms.o: timer_ticks_to_ms.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_ticks_to_ms.s

timer_delay_ticks.o: timer_delay_ticks.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_delay_ticks.s

timer_delay_us.o: timer_delay_us.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_delay_us.s

timer_delay_ms.o: timer_delay_ms.s
	CC65_HOME=/usr/local/share/cc65 ca65  timer_delay_ms.s

timer.txt: timer.man
	nroff -man timer.txt >timer.txt

# Clean build files
clean:
	rm -f *.o *.map *.s *~

# Install libraries to cc65 lib directory (optional)
install: timer.lib timer.h timer.man timer.md timer.txt
	cp timer.h   $(INCLUDE_DIR)
	cp timer.lib $(LIB_DIR)
	cp timer.md  $(DOC_DIR)
	cp timer.man $(MAN_DIR)
	cp timer.txt $(MAN_DIR)

.PHONY: all clean install setup




