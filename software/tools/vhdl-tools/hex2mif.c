#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define MAX_LINE_LENGTH 512
#define DEFAULT_DEPTH 2048
#define DEFAULT_WIDTH 8

// Convert a hex character to its integer value
int hex_char_to_int(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

// Convert two hex chars to a byte
int hex_to_byte(const char *hex) {
    return (hex_char_to_int(hex[0]) << 4) | hex_char_to_int(hex[1]);
}

// Convert four hex chars to a word (address)
int hex_to_word(const char *hex) {
    return (hex_char_to_int(hex[0]) << 12) | 
           (hex_char_to_int(hex[1]) << 8) |
           (hex_char_to_int(hex[2]) << 4) |
            hex_char_to_int(hex[3]);
}

void convert_hex_to_mif(const char *hex_filename, const char *mif_filename, int depth, int width) {
    FILE *hex_file, *mif_file;
    char line[MAX_LINE_LENGTH];
    unsigned char *memory;
    int byte_count, address, record_type, i, data_byte, extended_address = 0;
    
    // Allocate memory buffer initialized to zeros
    memory = (unsigned char*)calloc(depth, sizeof(unsigned char));
    if (!memory) {
        fprintf(stderr, "Memory allocation failed\n");
        return;
    }
    
    // Open input HEX file
    hex_file = fopen(hex_filename, "r");
    if (!hex_file) {
        fprintf(stderr, "Cannot open input file: %s\n", hex_filename);
        free(memory);
        return;
    }
    
    // Process each line in the HEX file
    while (fgets(line, MAX_LINE_LENGTH, hex_file)) {
        // Skip any line not starting with ':'
        if (line[0] != ':') continue;
        
        // Remove any trailing newline
        line[strcspn(line, "\r\n")] = 0;
        
        // Parse record
        byte_count = hex_to_byte(&line[1]);
        address = hex_to_word(&line[3]);
        record_type = hex_to_byte(&line[7]);
        
        // Process based on record type
        switch (record_type) {
            case 0: // Data record
                // Load data into memory
                for (i = 0; i < byte_count; i++) {
                    data_byte = hex_to_byte(&line[9 + i*2]);
                    int full_address = extended_address + address + i;
                    if (full_address < depth) {
                        memory[full_address] = data_byte;
                    }
                }
                break;
                
            case 1: // End of file record
                break;
                
            case 2: // Extended Segment Address Record
                extended_address = hex_to_byte(&line[9]) << 12 | hex_to_byte(&line[11]) << 4;
                break;
                
            case 4: // Extended Linear Address Record
                extended_address = hex_to_byte(&line[9]) << 24 | hex_to_byte(&line[11]) << 16;
                break;
        }
    }
    
    fclose(hex_file);
    
    // Create and write MIF file
    mif_file = fopen(mif_filename, "w");
    if (!mif_file) {
        fprintf(stderr, "Cannot open output file: %s\n", mif_filename);
        free(memory);
        return;
    }
    
    // Write MIF header
    fprintf(mif_file, "-- Memory Initialization File\n");
    fprintf(mif_file, "WIDTH = %d;\n", width);
    fprintf(mif_file, "DEPTH = %d;\n", depth);
    fprintf(mif_file, "ADDRESS_RADIX = HEX;\n");
    fprintf(mif_file, "DATA_RADIX = HEX;\n\n");
    fprintf(mif_file, "CONTENT BEGIN\n");
    
    // Write memory data
    for (i = 0; i < depth; i++) {
        fprintf(mif_file, "    %04X : %02X;\n", i, memory[i]);
    }
    
    fprintf(mif_file, "END;\n");
    fclose(mif_file);
    
    free(memory);
    printf("Conversion complete: %s -> %s\n", hex_filename, mif_filename);
}

int main(int argc, char *argv[]) {
    int depth = DEFAULT_DEPTH;
    int width = DEFAULT_WIDTH;
    
    if (argc < 3 || argc > 5) {
        printf("Usage: %s input.hex output.mif [depth] [width]\n", argv[0]);
        printf("  Default depth: %d\n", DEFAULT_DEPTH);
        printf("  Default width: %d bits\n", DEFAULT_WIDTH);
        return 1;
    }
    
    if (argc >= 4) {
        depth = atoi(argv[3]);
    }
    
    if (argc >= 5) {
        width = atoi(argv[4]);
    }
    
    convert_hex_to_mif(argv[1], argv[2], depth, width);
    
    return 0;
}