#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define MAX_LINE_LENGTH 256
#define DEFAULT_PAD_VALUE 0xFF

// Function to parse hex string to int
unsigned int parse_hex(const char *hex_str) {
    unsigned int value = 0;
    
    // Skip "0x" or "$" prefix if present
    if (hex_str[0] == '0' && (hex_str[1] == 'x' || hex_str[1] == 'X'))
        hex_str += 2;
    else if (hex_str[0] == '$')
        hex_str += 1;
    
    while (*hex_str) {
        char c = toupper(*hex_str++);
        if (c >= '0' && c <= '9')
            value = (value << 4) | (c - '0');
        else if (c >= 'A' && c <= 'F')
            value = (value << 4) | (c - 'A' + 10);
        else
            break; // Invalid character
    }
    
    return value;
}

// Function to parse command line arguments
int parse_arg(char *arg, const char *prefix, unsigned int *value) {
    size_t prefix_len = strlen(prefix);
    
    if (strncmp(arg, prefix, prefix_len) == 0) {
        *value = parse_hex(arg + prefix_len);
        return 1;
    }
    
    return 0;
}

// Parse hex byte from S-Record string
unsigned int parse_hex_byte(const char *str) {
    char hex[3] = {str[0], str[1], 0};
    return (unsigned int)strtol(hex, NULL, 16);
}

// Parse hex word (16-bit) from S-Record string
unsigned int parse_hex_word(const char *str) {
    char hex[5] = {str[0], str[1], str[2], str[3], 0};
    return (unsigned int)strtol(hex, NULL, 16);
}

void print_usage(const char *program_name) {
    printf("Usage: %s input.s19 output.vhd [options]\n", program_name);
    printf("Options:\n");
    printf("  --start=XXXX   Override starting address of ROM (hex)\n");
    printf("  --end=YYYY     Override ending address of ROM (hex)\n");
    printf("  --pad=ZZ       Padding byte value (hex, default: FF)\n");
    printf("  --reset=RRRR   Address to set reset vector (hex, default: start address)\n");
    printf("  --name=NAME    Entity name for VHDL file (default: ROM)\n");
}

int main(int argc, char *argv[]) {
    FILE *input;
    FILE *output;
    char line[MAX_LINE_LENGTH];
    unsigned char *buffer = NULL;
    unsigned int start_addr = 0xFFFF;  // Will be set from S19 file unless overridden
    unsigned int end_addr = 0x0000;    // Will be calculated from S19 file unless overridden
    unsigned int override_start_addr = 0xFFFF;
    unsigned int override_end_addr = 0x0000;
    unsigned int has_override_start = 0;
    unsigned int has_override_end = 0;
    unsigned int pad_value = DEFAULT_PAD_VALUE;
    unsigned int reset_addr = 0;
    int reset_specified = 0;
    char entity_name[64] = "ROM";
    int i;
    
    // Check minimum command line arguments
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }
    
    // Parse optional arguments
    for (i = 3; i < argc; i++) {
        if (parse_arg(argv[i], "--start=", &override_start_addr)) {
            has_override_start = 1;
            continue;
        } else if (parse_arg(argv[i], "--end=", &override_end_addr)) {
            has_override_end = 1;
            continue;
        } else if (parse_arg(argv[i], "--pad=", &pad_value)) {
            pad_value &= 0xFF;  // Ensure it's a byte
            continue;
        } else if (parse_arg(argv[i], "--reset=", &reset_addr)) {
            reset_specified = 1;
            continue;
        } else if (strncmp(argv[i], "--name=", 7) == 0) {
            strncpy(entity_name, argv[i] + 7, sizeof(entity_name) - 1);
            entity_name[sizeof(entity_name) - 1] = '\0';
            continue;
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Open input file
    input = fopen(argv[1], "r");
    if (!input) {
        printf("Error: Could not open input file %s\n", argv[1]);
        return 1;
    }
    
    // First pass: Determine address range from the S-Record file
    while (fgets(line, MAX_LINE_LENGTH, input)) {
        // Skip non-S-Record lines or non-data records
        if (line[0] != 'S' || (line[1] != '1' && line[1] != '2')) continue;
        
        // Extract record type and count
        char record_type = line[1];
        unsigned int byte_count = parse_hex_byte(line + 2);
        
        // Calculate address offset and data offset based on record type
        int addr_digits = (record_type == '1') ? 4 : 6;  // S1 = 16-bit, S2 = 24-bit
        int addr_bytes = addr_digits / 2;
        
        // Extract the address (use only the lower 16 bits even for S2 records)
        unsigned int address = 0;
        for (i = 0; i < addr_bytes && i < 2; i++) {
            address = (address << 8) | parse_hex_byte(line + 4 + i*2);
        }
        
        // Calculate number of data bytes (count includes address and checksum)
        unsigned int data_bytes = byte_count - addr_bytes - 1;  // -1 for checksum
        
        // Update address range
        if (address < start_addr) start_addr = address;
        if (address + data_bytes - 1 > end_addr) end_addr = address + data_bytes - 1;
    }
    
    // Apply overrides if specified
    if (has_override_start) start_addr = override_start_addr;
    if (has_override_end) end_addr = override_end_addr;
    
    // If we still don't have a valid range, exit
    if (start_addr > end_addr) {
        printf("Error: Could not determine a valid address range from the S19 file.\n");
        fclose(input);
        return 1;
    }
    
    // Calculate size
    unsigned int rom_size = end_addr - start_addr + 1;
    
    // If reset not specified, use start address
    if (!reset_specified) {
        reset_addr = start_addr;
    }
    
    // Allocate buffer
    buffer = (unsigned char*)malloc(rom_size);
    if (!buffer) {
        printf("Error: Could not allocate memory for ROM data\n");
        fclose(input);
        return 1;
    }
    
    // Initialize buffer with padding value
    memset(buffer, pad_value, rom_size);
    
    // Reset file position
    rewind(input);
    
    // Second pass: Fill buffer with data
    while (fgets(line, MAX_LINE_LENGTH, input)) {
        // Skip non-S-Record lines or non-data records
        if (line[0] != 'S' || (line[1] != '1' && line[1] != '2')) continue;
        
        // Extract record type and count
        char record_type = line[1];
        unsigned int byte_count = parse_hex_byte(line + 2);
        
        // Calculate address offset and data offset based on record type
        int addr_digits = (record_type == '1') ? 4 : 6;  // S1 = 16-bit, S2 = 24-bit
        int addr_bytes = addr_digits / 2;
        int data_offset = 4 + addr_digits;
        
        // Extract the address (use only the lower 16 bits even for S2 records)
        unsigned int address = 0;
        for (i = 0; i < addr_bytes && i < 2; i++) {
            address = (address << 8) | parse_hex_byte(line + 4 + i*2);
        }
        
        // Calculate number of data bytes (count includes address and checksum)
        unsigned int data_bytes = byte_count - addr_bytes - 1;  // -1 for checksum
        
        // Copy data bytes to our buffer
        for (i = 0; i < data_bytes; i++) {
            unsigned int data_addr = address + i;
            if (data_addr >= start_addr && data_addr <= end_addr) {
                unsigned int buffer_offset = data_addr - start_addr;
                buffer[buffer_offset] = parse_hex_byte(line + data_offset + i*2);
            }
        }
    }
    
    fclose(input);
    
    // Set reset vector if it falls within our ROM range
    // Assuming 6502 reset vector at $FFFC-$FFFD
    if (start_addr <= 0xFFFC && end_addr >= 0xFFFD) {
        unsigned int reset_offset_fc = 0xFFFC - start_addr;
        unsigned int reset_offset_fd = 0xFFFD - start_addr;
        
        buffer[reset_offset_fc] = reset_addr & 0xFF;        // Low byte
        buffer[reset_offset_fd] = (reset_addr >> 8) & 0xFF; // High byte
    }
    
    // Open output file
    output = fopen(argv[2], "w");
    if (!output) {
        printf("Error: Could not open output file %s\n", argv[2]);
        free(buffer);
        return 1;
    }
    
    // Determine address width based on end address
    int addr_width = 8;
    unsigned int temp = end_addr;
    while (temp > 255) {
        addr_width += 8;
        temp >>= 8;
    }
    
    // Write VHDL header
    fprintf(output, "library ieee;\n");
    fprintf(output, "use ieee.std_logic_1164.all;\n");
    fprintf(output, "use ieee.numeric_std.all;\n\n");
    fprintf(output, "entity %s is\n", entity_name);
    fprintf(output, "    port (\n");
    fprintf(output, "        clock:    in std_logic;\n");
    fprintf(output, "        address:  in std_logic_vector(%d downto 0);\n", addr_width - 1);
    fprintf(output, "        cs_n:     in std_logic;\n");
    fprintf(output, "        data_out: out std_logic_vector(7 downto 0)\n");
    fprintf(output, "    );\n");
    fprintf(output, "end entity;\n\n");
    fprintf(output, "architecture rtl of %s is\n", entity_name);
    fprintf(output, "    -- ROM from $%04X to $%04X (%d bytes)\n", start_addr, end_addr, rom_size);
    fprintf(output, "    type rom_type is array(0 to %d) of std_logic_vector(7 downto 0);\n", rom_size - 1);
    fprintf(output, "    signal rom : rom_type := (\n");
    
    // Write ROM data
    for (i = 0; i < rom_size; i++) {
        // Start a new line after every 8 bytes for readability
        if (i % 8 == 0) {
            if (i > 0) {
                fprintf(output, "\n        ");
            } else {
                fprintf(output, "        ");
            }
        }
        
        // Print the byte in X"XX" format
        fprintf(output, "X\"%02X\"", buffer[i]);
        
        // Add a comma after each byte except the last one
        if (i < rom_size - 1) {
            fprintf(output, ", ");
        }
    }
    
    // Write VHDL footer
    fprintf(output, "\n    );\n");
    fprintf(output, "begin\n");
    fprintf(output, "    process(clock)\n");
    fprintf(output, "        variable addr_int : integer range 0 to %d;\n", rom_size - 1);
    fprintf(output, "    begin\n");
    fprintf(output, "        if rising_edge(clock) then\n");
    fprintf(output, "            if cs_n = '0' then\n");
    fprintf(output, "                -- Convert address to ROM offset\n");
    fprintf(output, "                addr_int := to_integer(unsigned(address)) - %d;\n", start_addr);
    fprintf(output, "                -- Check if address is in range\n");
    fprintf(output, "                if addr_int >= 0 and addr_int <= %d then\n", rom_size - 1);
    fprintf(output, "                    data_out <= rom(addr_int);\n");
    fprintf(output, "                else\n");
    fprintf(output, "                    data_out <= X\"%02X\"; -- Return padding value for out-of-range\n", pad_value);
    fprintf(output, "                end if;\n");
    fprintf(output, "            end if;\n");
    fprintf(output, "        end if;\n");
    fprintf(output, "    end process;\n");
    fprintf(output, "end rtl;\n");
    
    fclose(output);
    free(buffer);
    
    printf("Conversion complete. Created %s with ROM from $%04X to $%04X (%d bytes).\n", 
           argv[2], start_addr, end_addr, rom_size);
    
    if (start_addr <= 0xFFFC && end_addr >= 0xFFFD) {
        printf("Reset vector at $FFFC-$FFFD set to $%04X.\n", reset_addr);
    }
    
    return 0;
}
