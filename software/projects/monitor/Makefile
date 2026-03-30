PLATFORM = none

all: mon6502.vhd

mon6502.vhd: mon6502.bin
	 bintovhdl mon6502.bin mon6502.vhd --start=FF00 --end=FFFF --reset=FF00 --name=mon6502

mon6502.o: mon6502.asm
	CC65_HOME=/usr/local/share/cc65 ca65 -o mon6502.o mon6502.asm

mon6502.bin: mon6502.o
	CC65_HOME=/usr/local/share/cc65 ld65 -o mon6502.bin -t $(PLATFORM) mon6502.o

clean:
	$(RM) *.lst *.map *.mon *.bin *.o

install:
	@true

