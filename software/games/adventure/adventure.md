# Enhancing the Apple 1 Adventure Game

This guide explains how to expand the Crystal Cave adventure game with additional rooms, items, and gameplay features while maintaining compatibility with the Apple 1's limited memory.

## Memory Considerations

Before expanding, understand the memory constraints:
- **Standard Apple 1**: 8KB RAM (4KB available for programs after system overhead)
- **With expansions**: 32KB or 48KB allows much more content
- **String storage**: Room descriptions are the largest memory consumers

### Memory Usage Estimation
- Each room description: ~40-50 bytes
- Each item name: ~8-12 bytes  
- Room connection table: 4 bytes per room
- Game code: ~2-3KB

## Adding New Rooms

### Step 1: Update Constants

```c
#define NUM_ROOMS 10  // Increase from 6
```

### Step 2: Add Room Descriptions

Extend the `room_desc` array:

```c
const char* room_desc[NUM_ROOMS] = {
    "CAVE ENTRANCE. DARK TUNNEL TO NORTH.",
    "DARK TUNNEL. PASSAGES EAST AND WEST.",
    "CRYSTAL CHAMBER. GLOWING CRYSTALS. SOUTH EXIT.",
    "UNDERGROUND RIVER. BRIDGE TO NORTH.",
    "TREASURE ROOM. GOLD EVERYWHERE! WEST EXIT.",
    "DEEP PIT. ROPE HANGS DOWN. UP TO ESCAPE.",
    // New rooms:
    "ANCIENT LIBRARY. DUSTY BOOKS EVERYWHERE.",
    "WIZARD'S WORKSHOP. MAGICAL APPARATUS.",
    "SECRET PASSAGE. NARROW AND WINDING.",
    "DRAGON'S LAIR. BONES SCATTERED ABOUT."
};
```

### Step 3: Update Room Connections

Extend the `exits` array (N,E,S,W directions):

```c
const unsigned char exits[NUM_ROOMS][4] = {
    {1, 255, 255, 255},      // Room 0
    {255, 2, 0, 3},          // Room 1  
    {255, 255, 1, 255},      // Room 2
    {4, 1, 5, 255},          // Room 3
    {255, 255, 3, 255},      // Room 4
    {255, 255, 255, 3},      // Room 5
    // New room connections:
    {255, 7, 255, 2},        // Room 6: Library (E to workshop, W to crystal chamber)
    {255, 255, 255, 6},      // Room 7: Workshop (W to library)
    {9, 255, 255, 4},        // Room 8: Secret passage (N to lair, W to treasure)
    {255, 255, 8, 255}       // Room 9: Dragon's lair (S to passage)
};
```

### Step 4: Add Items to New Rooms

Update the `room_items` array:

```c
unsigned char room_items[NUM_ROOMS] = {
    1,    // Room 0: TORCH
    0,    // Room 1: nothing
    8,    // Room 2: CRYSTAL  
    2,    // Room 3: KEY
    0,    // Room 4: nothing
    4,    // Room 5: ROPE
    16,   // Room 6: BOOK (bit 4)
    32,   // Room 7: POTION (bit 5)
    0,    // Room 8: nothing
    64    // Room 9: SWORD (bit 6)
};
```

## Adding New Items

### Step 1: Update Item Count

```c
#define NUM_ITEMS 7  // Increase from 4
```

### Step 2: Add Item Names

```c
const char* items[NUM_ITEMS] = {
    "TORCH", "KEY", "ROPE", "CRYSTAL",
    // New items:
    "BOOK", "POTION", "SWORD"
};
```

### Step 3: Update Inventory System

The inventory system uses bitmasks, supporting up to 8 items with `unsigned char`. For more items, use `unsigned int`:

```c
unsigned int inventory = 0;  // Now supports 16+ items
```

## Advanced Features

### Conditional Room Access

Add more complex movement logic in `move_player()`:

```c
void move_player(unsigned char dir) {
    if (exits[room][dir] != 255) {
        // Existing checks...
        
        // New conditional access:
        if (room == 6 && dir == 1 && !(inventory & 16)) {
            printf("YOU NEED THE MAGIC BOOK TO ENTER!\n");
            return;
        }
        
        if (room == 8 && dir == 0 && !(inventory & 64)) {
            printf("THE DRAGON BLOCKS YOUR PATH!\n");
            printf("YOU NEED A WEAPON!\n");
            return;
        }
        
        room = exits[room][dir];
        print_room();
        
        // Multiple win conditions
        if (room == 4 && (inventory & 8)) {
            printf("\nFIRST TREASURE FOUND!\n");
        }
        if (room == 9 && (inventory & 64) && (inventory & 32)) {
            printf("\nYOU DEFEAT THE DRAGON!\n");
            printf("ULTIMATE VICTORY!\n");
            game_flags |= 1;
        }
    }
}
```

### Item Combinations

Add an `use_item()` function:

```c
void use_item(void) {
    if (strstr(input, "POTION") && (inventory & 32)) {
        if (room == 9) {
            printf("THE POTION WEAKENS THE DRAGON!\n");
            game_flags |= 4; // Set dragon-weakened flag
        } else {
            printf("NOTHING HAPPENS HERE.\n");
        }
    }
    // Add more item interactions...
}
```

### Random Events

Add simple randomness (using a counter as pseudo-random):

```c
unsigned char event_counter = 0;

void random_event(void) {
    event_counter++;
    if ((event_counter & 7) == 0) {  // Every 8th move
        switch (room) {
            case 1:
                printf("YOU HEAR STRANGE ECHOES...\n");
                break;
            case 5:
                printf("WATER DRIPS FROM ABOVE.\n");
                break;
        }
    }
}
```

## Memory Optimization Tips

### Compress Room Descriptions

Use abbreviations and shorter phrases:
```c
"LIB. BOOKS EVERYWR."  // Instead of "ANCIENT LIBRARY. DUSTY BOOKS EVERYWHERE."
"WKSHP. MAG APPARATUS" // Instead of "WIZARD'S WORKSHOP. MAGICAL APPARATUS."
```

### Share Common Strings

Define common phrases once:
```c
const char* common_phrases[] = {
    "NOTHING HERE.",
    "TOO DARK!",
    "LOCKED DOOR.",
    "YOU WIN!"
};
```

### Use Packed Data Structures

For many items, pack multiple flags into single bytes:
```c
// Pack room data: connections + items + flags
struct room_data {
    unsigned char exits[4];
    unsigned char items;
    unsigned char flags;
};
```

## Testing Your Expansions

### Memory Usage Check

Compile and check the output size:
```bash
cc65 -t apple1 -O adventure.c
size adventure
```

### Playtesting Checklist

- [ ] All rooms accessible
- [ ] No impossible puzzles  
- [ ] Items can be picked up
- [ ] Win condition reachable
- [ ] No memory corruption
- [ ] Game fits in target RAM

## Advanced Expansion Ideas

### Multi-Level Adventures
- Use room numbers 0-99 for level 1, 100-199 for level 2
- Teleporters between levels

### NPC Characters
- Simple state machines for character behavior
- Basic conversation trees

### Save/Load System
- Store game state in a compact format
- Use Apple 1's cassette interface for persistence

### Dynamic Content
- Procedurally generated room descriptions
- Random item placement

Remember: Always test on actual hardware or accurate emulation to ensure your enhancements work within the Apple 1's constraints!