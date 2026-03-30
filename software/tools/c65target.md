# CC65 Target Addition Manual

## Overview
This manual explains how to add a new target system to the cc65 cross-compiler suite. Adding a target requires creating configuration files, library code, and modifying compiler source files.

## Prerequisites
- cc65 source code
- Basic knowledge of 6502 assembly
- Understanding of your target system's memory layout
- Knowledge of C and make build systems

## Step 1: Create Target Include File

**File:** `asminc/<target>.inc`

Define hardware addresses, memory locations, and ROM routines for your target system.

```assembly
; ---------------------------------------------------------------------------
; <Target> definitions
; ---------------------------------------------------------------------------

RAMSTART := $0300     ; Entry point

; ---------------------------------------------------------------------------
; Hardware Addresses
; ---------------------------------------------------------------------------

; I/O addresses specific to your system
KBD     := $D010      ; Keyboard input
KBDRDY  := $D011      ; Keyboard ready status
UART    := $8000      ; UART data register

; ---------------------------------------------------------------------------
; ROM routines
; ---------------------------------------------------------------------------

RESET   := $FF00      ; RESET entry point into monitor
PUTCHAR := $FFEF      ; Output character in A register
GETCHAR := $FFE0      ; Input character to A register
```

## Step 2: Create Linker Configuration

**File:** `cfg/<target>.cfg`

Define memory layout, segments, and linking rules.

```
# <target>.cfg
#
# <Target System Description>
#
# ld65 --config <target>.cfg -o <prog>.bin <prog>.o

FEATURES {
    STARTADDRESS: default = $0300;
    CONDES: segment = STARTUP,
            type    = constructor,
            label   = __CONSTRUCTOR_TABLE__,
            count   = __CONSTRUCTOR_COUNT__;
    CONDES: segment = STARTUP,
            type    = destructor,
            label   = __DESTRUCTOR_TABLE__,
            count   = __DESTRUCTOR_COUNT__;
}

SYMBOLS {
    __STACKSIZE__:    type = weak, value = $0100;   # Stack size
    __STARTADDRESS__: type = export, value = %S;
}

MEMORY {
    ZP:       file = %O, define = yes, start = $0000, size = $00EE;
    CPUSTACK: file = "", define = yes, start = $0100, size = $0100;
    RAM:      file = %O, define = yes, start = %S, size = $8000 - %S - __STACKSIZE__;
    IO:       file = "", define = yes, start = $8000, size = $0100;
    ROM:      file = "", define = yes, start = $8100, size = $7F00;
}

SEGMENTS {
    ZEROPAGE: load = ZP,  type = zp,  define = yes;
    STARTUP:  load = RAM, type = ro,  define = yes;
    CODE:     load = RAM, type = ro,  define = yes;
    RODATA:   load = RAM, type = ro,  define = yes;
    ONCE:     load = RAM, type = ro,  define = yes;
    DATA:     load = RAM, type = rw,  define = yes;
    BSS:      load = RAM, type = bss, define = yes;
}
```

### Memory Section Notes:
- **ZP**: Zero page memory
- **CPUSTACK**: 6502 hardware stack
- **RAM**: Main program memory
- **IO**: Memory-mapped I/O area
- **ROM**: Read-only memory area

### Segment Notes:
- **ZEROPAGE**: Zero page variables
- **STARTUP**: Startup/initialization code
- **CODE**: Main program code
- **RODATA**: Read-only data (constants)
- **DATA**: Initialized variables
- **BSS**: Uninitialized variables

## Step 3: Create Target Library Directory

**Directory:** `libsrc/<target>/`

### Required Files:

#### Startup Code: `crt0.s`
```assembly
;
; Startup code for cc65 (<TARGET> version)
;

        .export         _init, _exit
        .export         __STARTUP__ : absolute = 1      ; Mark as startup

        .import         _main
        .import         initlib, donelib, copydata, zerobss
        .import         __RAM_START__, __RAM_SIZE__     ; Linker generated
        .import         __STACKSIZE__                   ; Linker generated

        .include        "zeropage.inc"
        .include        "<target>.inc"

.segment  "STARTUP"

_init:    cld                          ; Clear decimal mode

; Set cc65 argument stack pointer
          lda     #<(__RAM_START__ + __RAM_SIZE__)
          sta     sp
          lda     #>(__RAM_START__ + __RAM_SIZE__)
          sta     sp+1

; Initialize memory storage
          jsr     zerobss              ; Clear BSS segment
          jsr     copydata             ; Initialize DATA segment
          jsr     initlib              ; Run constructors

; Call main()
          jsr     _main

; Back from main (this is also the _exit entry)
_exit:    jmp    RESET               ; Return to monitor/system
```

#### Character Type: `ctype.s`
```assembly
; Character specification table.
; Uses the "common" definition

        .include "ctype_common.inc"
```

#### Clear Screen: `clrscr.s`
```assembly
;
; void clrscr (void);
;

        .export         _clrscr
        .include        "<target>.inc"

_clrscr:
        lda     #$0C            ; Form feed character
        jsr     PUTCHAR         ; Send to output
        rts
```

#### Keyboard Hit: `kbhit.s`
```assembly
;
; unsigned char kbhit (void);
;

        .export _kbhit
        .include        "<target>.inc"

_kbhit:
        ldx     #$00
        lda     KBDRDY          ; Check keyboard status
        rol                     ; Move status bit to carry
        txa
        rol                     ; Move carry to result
        rts
```

#### Read Function: `read.s`
```assembly
;
; int __fastcall__ read (int fd, void* buf, unsigned count);
;

.include        "<target>.inc"
.import         popax, popptr1
.importzp       ptr1, ptr2, ptr3
.export         _read

.proc           _read
        sta     ptr3
        stx     ptr3+1           ; Count in ptr3
        inx
        stx     ptr2+1           ; Increment and store in ptr2
        tax
        inx
        stx     ptr2
        jsr     popptr1          ; Buffer address in ptr1
        jsr     popax

begin:  dec     ptr2
        bne     getch
        dec     ptr2+1
        beq     done             ; If buffer full, return

getch:  jsr     GETCHAR          ; Read character
        ldx     #$00
        jsr     PUTCHAR          ; Echo character

        cmp     #$0D             ; Check for carriage return
        bne     putch
        lda     #$0A             ; Replace with line feed
        jsr     PUTCHAR

putch:  ldy     #$00             ; Put char into return buffer
        sta     (ptr1),y
        inc     ptr1             ; Increment pointer
        bne     begin
        inc     ptr1+1
        bne     begin

done:   lda     ptr3
        ldx     ptr3+1
        rts                      ; Return count
.endproc
```

#### Write Function: `write.s`
```assembly
;
; int __fastcall__ write (int fd, const void* buf, int count);
;

.include        "<target>.inc"
.import         popax, popptr1
.importzp       ptr1, ptr2, ptr3
.export         _write

.proc           _write
        sta     ptr3
        stx     ptr3+1          ; Count in ptr3
        inx
        stx     ptr2+1          ; Increment and store in ptr2
        tax
        inx
        stx     ptr2
        jsr     popptr1         ; Buffer address in ptr1
        jsr     popax

begin:  dec     ptr2
        bne     outch
        dec     ptr2+1
        beq     done

outch:  ldy     #0
        lda     (ptr1),y
        jsr     PUTCHAR         ; Send character
        cmp     #$0A            ; Check for line feed
        bne     next
        lda     #$0D            ; Add carriage return
        jsr     PUTCHAR
next:   inc     ptr1
        bne     begin
        inc     ptr1+1
        jmp     begin

done:   lda     ptr3
        ldx     ptr3+1
        rts                     ; Return count
.endproc
```

## Step 4: Modify Compiler Source Files

### Add Target Enum

**File:** `src/common/target.h`

```c
typedef enum {
    // ... existing targets ...
    TGT_<TARGET>,               // Add your target here
    TGT_COUNT                   // Must be last
} target_t;
```

### Add Target Properties

**File:** `src/common/target.c`

Add to `TargetMap[]` array (maintain alphabetical order):
```c
static const TargetEntry TargetMap[] = {
    // ... other entries in alphabetical order ...
    {   "<target>",     TGT_<TARGET>     },
    // ... rest of entries ...
};
```

Add to `PropertyTable[]` array (same order as enum):
```c
static const TargetProperties PropertyTable[TGT_COUNT] = {
    // ... other entries ...
    { "<target>",       CPU_6502,       BINFMT_BINARY,      CTNone  },
    // ... rest of entries ...
};
```

**Important:** The `TargetMap[]` array uses binary search, so it **must** be sorted alphabetically by target name.

### Update C Compiler

**File:** `src/cc65/main.c`

Add case in `SetSys()` function:
```c
static void SetSys (const char* Sys)
{
    switch (Target = FindTarget (Sys)) {
        // ... existing cases ...
        
        case TGT_<TARGET>:
            DefineNumericMacro ("__<TARGET>__", 1);
            break;
            
        // ... rest of cases ...
    }
}
```

### Update Assembler

**File:** `src/ca65/main.c`

Add case in `SetSys()` function:
```c
static void SetSys (const char* Sys)
{
    switch (Target = FindTarget (Sys)) {
        // ... existing cases ...
        
        case TGT_<TARGET>:
            NewSymbol ("__<TARGET>__", 1);
            break;
            
        // ... rest of cases ...
    }
}
```

## Step 5: Add Target to Build System

**File:** `targettest/Makefile`

Add target to `TARGETS` list (maintain alphabetical order):
```makefile
TARGETS := \
  apple2       \
  apple2enh    \
  atari        \
  // ... other targets ...
  <target>     \
  // ... rest of targets ...
```

## Step 6: Build Process

### Rebuild Compiler Tools
```bash
make -C src clean
make -C src
```

### Build Target Libraries
```bash
make lib
```

### Test Your Target
```bash
echo 'int main() { return 0; }' > test.c
cc65 -t <target> test.c
ca65 -t <target> test.s
ld65 -C <target>.cfg -o test.bin test.o <target>.lib
```

## Step 7: Verification Checklist

- [ ] Target name appears in `cc65 --help` output
- [ ] `cc65 -t <target>` compiles without errors
- [ ] `ca65 -t <target>` assembles without errors
- [ ] Linker finds the configuration file
- [ ] Simple test program builds successfully
- [ ] Target-specific macros are defined (`__<TARGET>__`)

## Common Pitfalls

1. **Alphabetical Order**: `TargetMap[]` must be sorted alphabetically
2. **Enum Order**: `PropertyTable[]` must match enum order in `target.h`
3. **Case Sensitivity**: Target names are case-sensitive
4. **Memory Overlap**: Ensure memory regions don't overlap in config file
5. **Missing Includes**: Don't forget `#include` statements in `.inc` files
6. **Stack Size**: Ensure adequate stack space for your applications

## Memory Layout Considerations

- Reserve zero page locations for cc65 runtime
- Leave space for 6502 hardware stack ($0100-$01FF)
- Consider I/O memory mapping
- Plan for future expansion
- Document memory usage for users

## Example: Complete Target Addition

For a target called "mysystem":

1. Create `asminc/mysystem.inc`
2. Create `cfg/mysystem.cfg`  
3. Create directory `libsrc/mysystem/`
4. Add required library files
5. Add `TGT_MYSYSTEM` to `target.h`
6. Add "mysystem" to `TargetMap[]` (alphabetically)
7. Add properties to `PropertyTable[]`
8. Add cases to cc65 and ca65 `SetSys()`
9. Add to `targettest/Makefile`
10. Rebuild and test

## Advanced Features

- Custom segment types
- Multiple memory configurations
- Target-specific optimizations
- Hardware-specific library functions
- Custom startup sequences

## MAX6502 Example

Here's a complete example for the MAX6502 system with 32K RAM and 32K ROM:

### Memory Layout
- **RAM:** $0000-$7FFF (32KB)
- **I/O:** $8000-$80FF (256 bytes)  
- **ROM:** $8100-$FFFF (32KB - 256 bytes)

### asminc/max6502.inc
```assembly
; MAX6502 definitions - 32K RAM + 32K ROM system
RAMSTART := $0300     ; Entry point

; I/O Area ($8000-$80FF)
SPI_DATA    := $8000
SPI_STATUS  := $8001
SD_STATUS   := $8010
UART_DATA   := $8020
UART_STATUS := $8021

; ROM routines ($8100-$FFFF)
RESET       := $FF00   ; RESET entry point
PUTCHAR     := $FF20   ; Output character in A
GETCHAR     := $FF30   ; Input character to A
FAT_INIT    := $8100   ; Initialize FAT filesystem
SD_INIT     := $9000   ; Initialize SD card
```

### cfg/max6502.cfg
```
MEMORY {
    ZP:       file = %O, define = yes, start = $0000, size = $00EE;
    CPUSTACK: file = "", define = yes, start = $0100, size = $0100;
    RAM:      file = %O, define = yes, start = %S, size = $8000 - %S - __STACKSIZE__;
    IO:       file = "", define = yes, start = $8000, size = $0100;
    ROM:      file = "", define = yes, start = $8100, size = $7F00;
}
```

This manual provides the foundation for adding any 6502-based target to cc65. Adapt the memory layouts, I/O definitions, and library functions to match your specific hardware requirements.