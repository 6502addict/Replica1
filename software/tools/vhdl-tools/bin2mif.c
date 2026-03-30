#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {
    FILE *bin_file, *mif_file;
    unsigned char *buffer;
    long file_size;
    int i, width = 8;
    
    // Check command line arguments
    if (argc < 3 || argc > 4) {
        printf("Usage: %s <input.bin> <output.mif> [width]\n", argv[0]);
        printf("  width: Data width in bits (default: 8)\n");
        return 1;
    }
    
    // Get optional width parameter
    if (argc == 4) {
        width = atoi(argv[3]);
    }
    
    // Open binary input file
    bin_file = fopen(argv[1], "rb");
    if (!bin_file) {
        printf("Error: Cannot open input file %s\n", argv[1]);
        return 1;
    }
    
    // Get file size
    fseek(bin_file, 0, SEEK_END);
    file_size = ftell(bin_file);
    rewind(bin_file);
    
    printf("Binary file size: %ld bytes\n", file_size);
    
    // Allocate memory for file content
    buffer = (unsigned char*)malloc(file_size);
    if (!buffer) {
        printf("Error: Memory allocation failed\n");
        fclose(bin_file);
        return 1;
    }
    
    // Read the entire file
    if (fread(buffer, 1, file_size, bin_file) != file_size) {
        printf("Error: Failed to read input file\n");
        free(buffer);
        fclose(bin_file);
        return 1;
    }
    
    fclose(bin_file);
    
    // Create MIF file
    mif_file = fopen(argv[2], "w");
    if (!mif_file) {
        printf("Error: Cannot open output file %s\n", argv[2]);
        free(buffer);
        return 1;
    }
    
    // Write MIF header
    fprintf(mif_file, "-- Memory Initialization File\n");
    fprintf(mif_file, "-- Converted from binary file: %s\n", argv[1]);
    fprintf(mif_file, "WIDTH=%d;\n", width);
    fprintf(mif_file, "DEPTH=%ld;\n", file_size);
    fprintf(mif_file, "ADDRESS_RADIX=HEX;\n");
    fprintf(mif_file, "DATA_RADIX=HEX;\n");
    fprintf(mif_file, "CONTENT BEGIN\n");
    
    // Write memory data
    for (i = 0; i < file_size; i++) {
        fprintf(mif_file, "    %04X : %02X;\n", i, buffer[i]);
    }
    
    fprintf(mif_file, "END;\n");
    
    fclose(mif_file);
    free(buffer);
    
    printf("Conversion complete: %s -> %s\n", argv[1], argv[2]);
    return 0;
}