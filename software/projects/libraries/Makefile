# Top-level Makefile for 6502 libraries with dependencies
SUBDIRS = timer spi sdcard fatfs

.PHONY: all install clean $(SUBDIRS)

all: $(SUBDIRS)

# Build and install each library
$(SUBDIRS):
	$(MAKE) -C $@ all
	$(MAKE) -C $@ install

# Define dependencies between libraries
spi: timer
sdcard: spi timer
fatfs: sdcard spi timer

install: all

clean:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean || exit 1; \
	done
