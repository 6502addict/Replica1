# Simple makefile for conversion tools
CC = gcc
CFLAGS = -Wall -O2
COPY = cp

# List of executables to build
TARGETS = bin2mif bintovhdl hex2mif hextovhdl s19tovhdl

# Build all targets
all: $(TARGETS)

# Build each executable from corresponding .c file
bin2mif: bin2mif.c
	$(CC) $(CFLAGS) -o bin2mif bin2mif.c

bintovhdl: bintovhdl.c
	$(CC) $(CFLAGS) -o bintovhdl bintovhdl.c

hex2mif: hex2mif.c
	$(CC) $(CFLAGS) -o hex2mif hex2mif.c

hextovhdl: hextovhdl.c
	$(CC) $(CFLAGS) -o hextovhdl hextovhdl.c

s19tovhdl: s19tovhdl.c
	$(CC) $(CFLAGS) -o s19tovhdl s19tovhdl.c

# Clean built files
clean:
	rm -f $(TARGETS) *~ *.o 

# Install executables to ~/bin
install: $(TARGETS)
	mkdir -p ~/bin
	$(COPY) bin2mif ~/bin/
	$(COPY) hex2mif ~/bin/
	$(COPY) bintovhdl ~/bin/
	$(COPY) hextovhdl ~/bin/
	$(COPY) s19tovhdl ~/bin/

.PHONY: all clean install
