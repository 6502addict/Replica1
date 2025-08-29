asm6809 -H mon6809.asm -o mon6809.hex -l mon6809.lst
./hextovhdl  mon6809.hex mon6809.vhdl  --start=F800 --end=FFFF --name=MON6809 --reset=F800
