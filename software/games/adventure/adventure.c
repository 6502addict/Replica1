#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* Enhanced adventure game for Apple 1 - "CRYSTAL CAVE DELUXE"
 * Compiled with cc65 for 6502
 * Optimized for 32KB+ memory systems
 * Features: Combat, monsters, magic, expanded world
 */

#define MAX_INPUT 30
#define NUM_ROOMS 12
#define NUM_ITEMS 10
#define NUM_MONSTERS 5

/* Game state */
unsigned char room = 0;
unsigned int inventory = 0;     /* Extended to 16-bit for more items */
unsigned char game_flags = 0;
unsigned char player_health = 20;
unsigned char player_max_health = 20;
unsigned char player_attack = 3;
unsigned char player_armor = 0;
unsigned char magic_points = 5;
unsigned char max_magic = 5;
unsigned char rng_seed = 17;

/* Monster structure */
struct monster {
    unsigned char health;
    unsigned char max_health;
    unsigned char attack;
    unsigned char armor;
    const char* name;
    const char* description;
};

/* Monster definitions */
struct monster monsters[NUM_MONSTERS] = {
    {8, 8, 2, 0, "GIANT RAT", "A DISEASED RODENT WITH GLOWING EYES"},
    {12, 12, 4, 1, "CAVE GOBLIN", "A SNEAKY CREATURE WITH SHARP CLAWS"},
    {18, 18, 6, 2, "STONE TROLL", "A MASSIVE BEAST OF ROCK AND FURY"},
    {25, 25, 8, 3, "SHADOW WRAITH", "AN UNDEAD SPIRIT FROM THE DEPTHS"},
    {40, 40, 12, 5, "CRYSTAL DRAGON", "THE ANCIENT GUARDIAN OF THE CAVE"}
};

/* Room descriptions */
const char* room_desc[NUM_ROOMS] = {
    "CAVE ENTRANCE. Sunlight filters through the opening. A dark tunnel leads north.",
    "DARK TUNNEL. The air is cold and damp. Passages branch east and west.",
    "CRYSTAL CHAMBER. Magnificent crystals glow with inner light. Ancient runes cover the walls.",
    "UNDERGROUND RIVER. Clear water flows over smooth stones. A sturdy bridge spans north.",
    "TREASURE VAULT. Golden coins and jewels sparkle in the torchlight. The dragon's hoard!",
    "DEEP PIT. A dangerous chasm with rope hanging down. Bones litter the bottom.",
    "ANCIENT LIBRARY. Dusty tomes line the walls. Knowledge of ages past rests here.",
    "WIZARD'S WORKSHOP. Magical apparatus and bubbling potions fill this mystical chamber.",
    "SECRET PASSAGE. A narrow, winding corridor carved by ancient hands.",
    "ARMORY. Weapons and armor hang on the walls, waiting for brave warriors.",
    "TEMPLE RUINS. Broken columns and a cracked altar. Divine magic still lingers here.",
    "DRAGON'S LAIR. Massive chamber with scorched walls. The air shimmers with heat."
};

/* Item definitions */
const char* items[NUM_ITEMS] = {
    "TORCH", "IRON KEY", "ROPE", "CRYSTAL ORB", "MAGIC BOOK",
    "HEALING POTION", "STEEL SWORD", "CHAIN ARMOR", "MAGIC RING", "DRAGON SCALE"
};

const char* item_descriptions[NUM_ITEMS] = {
    "A burning torch that lights the darkness",
    "A heavy iron key with intricate engravings", 
    "Strong rope for climbing dangerous heights",
    "A glowing orb pulsing with magical energy",
    "Ancient tome filled with mystical knowledge",
    "Red potion that restores health when drunk",
    "Sharp steel blade that gleams in the light",
    "Protective armor made of interlocked rings",
    "Silver ring inscribed with protective runes",
    "Shimmering scale from the crystal dragon"
};

/* Room connections: N,E,S,W (255 = no exit) */
const unsigned char exits[NUM_ROOMS][4] = {
    {1, 255, 255, 255},     /* 0: Cave entrance */
    {6, 2, 0, 3},           /* 1: Dark tunnel */
    {255, 255, 1, 7},       /* 2: Crystal chamber */
    {4, 1, 5, 8},           /* 3: Underground river */
    {255, 255, 3, 9},       /* 4: Treasure vault */
    {255, 255, 255, 3},     /* 5: Deep pit */
    {255, 255, 1, 10},      /* 6: Ancient library */
    {255, 2, 255, 255},     /* 7: Wizard's workshop */
    {255, 3, 255, 255},     /* 8: Secret passage */
    {255, 4, 255, 255},     /* 9: Armory */
    {255, 6, 255, 11},      /* 10: Temple ruins */
    {255, 255, 255, 10}     /* 11: Dragon's lair */
};

/* Items in rooms (bitmask) */
unsigned int room_items[NUM_ROOMS] = {
    1,      /* 0: TORCH */
    0,      /* 1: nothing */
    8,      /* 2: CRYSTAL ORB */
    2,      /* 3: IRON KEY */
    512,    /* 4: DRAGON SCALE */
    4,      /* 5: ROPE */
    16,     /* 6: MAGIC BOOK */
    32,     /* 7: HEALING POTION */
    0,      /* 8: nothing */
    192,    /* 9: STEEL SWORD + CHAIN ARMOR */
    256,    /* 10: MAGIC RING */
    0       /* 11: nothing */
};

/* Monster presence in rooms (monster ID + 1, 0 = no monster) */
unsigned char room_monsters[NUM_ROOMS] = {
    0, 1, 0, 2, 5, 3, 0, 0, 4, 0, 0, 5
};

/* Current monster health in each room */
unsigned char monster_health[NUM_ROOMS];

char input[MAX_INPUT];

/* Simple random number generator */
unsigned char simple_random(void) {
    rng_seed = (rng_seed * 9 + 7) & 0xFF;
    return rng_seed;
}

void init_game(void) {
    unsigned char i;
    /* Initialize monster health */
    for (i = 0; i < NUM_ROOMS; i++) {
        if (room_monsters[i] > 0) {
            monster_health[i] = monsters[room_monsters[i] - 1].max_health;
        }
    }
}

void print_status(void) {
    printf("\n=== STATUS ===\n");
    printf("HEALTH: %d/%d  MAGIC: %d/%d  ATTACK: %d  ARMOR: %d\n",
           player_health, player_max_health, magic_points, max_magic,
           player_attack, player_armor);
}

void print_room(void) {
    unsigned char i;
    
    printf("\n%s\n", room_desc[room]);
    
    /* Show monster */
    if (room_monsters[room] > 0 && monster_health[room] > 0) {
        unsigned char mid = room_monsters[room] - 1;
        printf("\nA %s blocks your path!\n", monsters[mid].name);
        printf("%s\n", monsters[mid].description);
        printf("MONSTER HEALTH: %d/%d\n", monster_health[room], monsters[mid].max_health);
    }
    
    /* Show items in room */
    for (i = 0; i < NUM_ITEMS; i++) {
        if (room_items[room] & (1 << i)) {
            printf("\nYou see a %s here.\n", items[i]);
        }
    }
    
    /* Show exits */
    printf("\nExits: ");
    if (exits[room][0] != 255) printf("North ");
    if (exits[room][1] != 255) printf("East ");
    if (exits[room][2] != 255) printf("South ");
    if (exits[room][3] != 255) printf("West ");
    printf("\n");
}

void show_inventory(void) {
    unsigned char i, count = 0;
    
    printf("\n=== INVENTORY ===\n");
    for (i = 0; i < NUM_ITEMS; i++) {
        if (inventory & (1 << i)) {
            printf("%s - %s\n", items[i], item_descriptions[i]);
            count++;
        }
    }
    if (count == 0) printf("Your pack is empty.\n");
}

unsigned char get_current_attack(void) {
    unsigned char attack = player_attack;
    if (inventory & 64) attack += 4;   /* Steel sword */
    if (inventory & 256) attack += 2;  /* Magic ring */
    return attack;
}

unsigned char get_current_armor(void) {
    unsigned char armor = player_armor;
    if (inventory & 128) armor += 3;   /* Chain armor */
    if (inventory & 256) armor += 1;   /* Magic ring */
    return armor;
}

void combat(void) {
    unsigned char monster_id = room_monsters[room] - 1;
    unsigned char player_fled = 0;
    
    printf("\n=== COMBAT ===\n");
    printf("You face the %s!\n", monsters[monster_id].name);
    
    while (monster_health[room] > 0 && player_health > 0 && !player_fled) {
        printf("\n[A]ttack, [M]agic, [R]un, [S]tatus? ");
        
        if (fgets(input, MAX_INPUT, stdin)) {
            char action = toupper(input[0]);
            
            switch (action) {
                case 'A': {
                    /* Player attacks */
                    unsigned char damage;
                    unsigned char monster_armor;
                    
                    damage = get_current_attack();
                    monster_armor = monsters[monster_id].armor;
                    
                    /* Critical hit chance */
                    if ((simple_random() & 7) == 0) {
                        printf("CRITICAL HIT! ");
                        damage *= 2;
                    }
                    
                    damage = (damage > monster_armor) ? damage - monster_armor : 1;
                    monster_health[room] = (monster_health[room] > damage) ? 
                                         monster_health[room] - damage : 0;
                    
                    printf("You hit for %d damage!\n", damage);
                    
                    if (monster_health[room] == 0) {
                        printf("The %s is defeated!\n", monsters[monster_id].name);
                        
                        /* Award experience/items for some monsters */
                        if (monster_id == 4) { /* Dragon */
                            printf("You have slain the Crystal Dragon!\n");
                            printf("The ancient curse is broken!\n");
                            game_flags |= 1; /* Victory! */
                        }
                        return;
                    }
                    break;
                }
                
                case 'M': {
                    char spell;
                    unsigned char damage;
                    
                    if (magic_points <= 0) {
                        printf("You have no magic points!\n");
                        continue;
                    }
                    
                    printf("Cast [H]eal or [F]ireball? ");
                    if (fgets(input, MAX_INPUT, stdin)) {
                        spell = toupper(input[0]);
                        
                        if (spell == 'H' && (inventory & 16)) { /* Has magic book */
                            magic_points--;
                            player_health += 8;
                            if (player_health > player_max_health) 
                                player_health = player_max_health;
                            printf("You heal yourself for 8 points!\n");
                        } else if (spell == 'F' && (inventory & 16)) {
                            damage = 10;
                            magic_points--;
                            monster_health[room] = (monster_health[room] > damage) ?
                                                 monster_health[room] - damage : 0;
                            printf("Fireball hits for %d damage!\n", damage);
                            
                            if (monster_health[room] == 0) {
                                printf("The %s is destroyed by magic!\n", monsters[monster_id].name);
                                return;
                            }
                        } else {
                            printf("You don't know that spell!\n");
                            continue;
                        }
                    }
                    break;
                }
                
                case 'R': {
                    if ((simple_random() & 1) == 0) {
                        printf("You escape safely!\n");
                        player_fled = 1;
                        return;
                    } else {
                        printf("You can't escape!\n");
                    }
                    break;
                }
                
                case 'S': {
                    print_status();
                    printf("MONSTER: %s (%d/%d HP)\n", 
                           monsters[monster_id].name, 
                           monster_health[room], 
                           monsters[monster_id].max_health);
                    continue;
                }
                
                default:
                    printf("Invalid action!\n");
                    continue;
            }
            
            /* Monster attacks back */
            if (monster_health[room] > 0) {
                unsigned char damage;
                unsigned char armor;
                
                damage = monsters[monster_id].attack;
                armor = get_current_armor();
                
                damage = (damage > armor) ? damage - armor : 1;
                player_health = (player_health > damage) ? player_health - damage : 0;
                
                printf("The %s attacks for %d damage!\n", monsters[monster_id].name, damage);
                
                if (player_health == 0) {
                    printf("\nYou have been slain!\n");
                    printf("GAME OVER!\n");
                    game_flags |= 2;
                    return;
                }
            }
        }
    }
}

unsigned char parse_direction(char c) {
    switch (toupper(c)) {
        case 'N': return 0;
        case 'E': return 1;
        case 'S': return 2;
        case 'W': return 3;
        default: return 255;
    }
}

void move_player(unsigned char dir) {
    if (exits[room][dir] != 255) {
        /* Check for monster blocking path */
        if (room_monsters[room] > 0 && monster_health[room] > 0) {
            printf("The %s blocks your way!\n", monsters[room_monsters[room] - 1].name);
            return;
        }
        
        /* Special movement requirements */
        if (room == 1 && dir == 0 && !(inventory & 1)) {
            printf("It's too dark without a torch!\n");
            return;
        }
        if (room == 3 && dir == 0 && !(inventory & 2)) {
            printf("The vault door is locked!\n");
            return;
        }
        if (room == 3 && dir == 2 && !(inventory & 4)) {
            printf("The pit is too deep without rope!\n");
            return;
        }
        if (room == 10 && dir == 3 && !(inventory & 8)) {
            printf("The crystal orb is needed to open this passage!\n");
            return;
        }
        
        room = exits[room][dir];
        print_room();
        
        /* Restore magic points when moving */
        if (magic_points < max_magic && (simple_random() & 3) == 0) {
            magic_points++;
            printf("You feel magical energy returning...\n");
        }
        
        /* Check for monster in new room */
        if (room_monsters[room] > 0 && monster_health[room] > 0) {
            combat();
        }
        
        /* Check win condition */
        if (room == 4 && (inventory & 8) && (inventory & 512)) {
            printf("\nWith the Crystal Orb and Dragon Scale, you unlock the ultimate treasure!\n");
            printf("The ancient magic recognizes you as the true hero!\n");
            printf("*** VICTORY ACHIEVED ***\n");
            game_flags |= 1;
        }
    } else {
        printf("You can't go that way.\n");
    }
}

void take_item(void) {
    unsigned char i;
    char* item_name;
    
    if (strlen(input) < 6) {
        printf("Take what?\n");
        return;
    }
    
    item_name = input + 5;
    
    /* Convert to uppercase */
    for (i = 0; item_name[i]; i++) {
        item_name[i] = toupper(item_name[i]);
    }
    
    /* Check each item */
    for (i = 0; i < NUM_ITEMS; i++) {
        if (strstr(items[i], item_name) && (room_items[room] & (1 << i))) {
            room_items[room] &= ~(1 << i);
            inventory |= (1 << i);
            printf("Taken: %s\n", items[i]);
            
            /* Special item effects */
            if (i == 7) { /* Chain armor */
                printf("You feel more protected!\n");
            }
            if (i == 8) { /* Magic ring */
                max_magic += 2;
                magic_points += 2;
                printf("Your magical power increases!\n");
            }
            return;
        }
    }
    
    printf("I don't see that here.\n");
}

void use_item(void) {
    if (strlen(input) < 5) {
        printf("Use what?\n");
        return;
    }
    
    if (strstr(input, "POTION") && (inventory & 32)) {
        inventory &= ~32;
        player_health += 15;
        if (player_health > player_max_health) player_health = player_max_health;
        printf("You drink the healing potion and feel much better!\n");
        printf("Health restored to %d/%d\n", player_health, player_max_health);
    } else {
        printf("You can't use that here.\n");
    }
}

void examine_item(void) {
    unsigned char i;
    char* item_name;
    
    if (strlen(input) < 9) {
        printf("Examine what?\n");
        return;
    }
    
    item_name = input + 8;
    
    for (i = 0; item_name[i]; i++) {
        item_name[i] = toupper(item_name[i]);
    }
    
    for (i = 0; i < NUM_ITEMS; i++) {
        if (strstr(items[i], item_name) && (inventory & (1 << i))) {
            printf("%s: %s\n", items[i], item_descriptions[i]);
            return;
        }
    }
    
    printf("You don't have that item.\n");
}

void process_input(void) {
    unsigned char dir;
    
    if (strlen(input) == 0) return;
    
    /* Convert to uppercase */
    input[0] = toupper(input[0]);
    
    /* Single letter directions */
    if (strlen(input) == 1) {
        dir = parse_direction(input[0]);
        if (dir != 255) {
            move_player(dir);
            return;
        }
    }
    
    /* Commands */
    if (input[0] == 'L') {
        print_room();
    } else if (input[0] == 'I') {
        show_inventory();
    } else if (strstr(input, "TAKE") == input) {
        take_item();
    } else if (strstr(input, "USE") == input) {
        use_item();
    } else if (strstr(input, "EXAMINE") == input) {
        examine_item();
    } else if (input[0] == 'S') {
        print_status();
    } else if (input[0] == 'H') {
        printf("\n=== COMMANDS ===\n");
        printf("Movement: N, E, S, W (or NORTH, EAST, SOUTH, WEST)\n");
        printf("L or LOOK - Examine surroundings\n");
        printf("I or INVENTORY - Show your items\n");
        printf("TAKE <item> - Pick up an item\n");
        printf("USE <item> - Use an item\n");
        printf("EXAMINE <item> - Get details about an item\n");
        printf("S or STATUS - Show your condition\n");
        printf("Q or QUIT - End the game\n");
        printf("\nCombat: A(ttack), M(agic), R(un), S(tatus)\n");
    } else if (input[0] == 'Q') {
        game_flags |= 2;
    } else {
        printf("I don't understand that command. Type H for help.\n");
    }
}

int main(void) {
    printf("*** CRYSTAL CAVE DELUXE ***\n");
    printf("An Enhanced Adventure for the Apple 1\n");
    printf("Featuring Combat, Magic, and Expanded World!\n");
    printf("\nType H for help.\n");
    
    init_game();
    print_room();
    
    while (!(game_flags & 3)) {
        printf("\n> ");
        
        if (fgets(input, MAX_INPUT, stdin)) {
            input[strcspn(input, "\n")] = 0;
            process_input();
        }
    }
    
    if (game_flags & 1) {
        printf("\nCongratulations, brave adventurer!\n");
        printf("You have completed the Crystal Cave!\n");
        printf("Your legend will be remembered forever!\n");
    } else {
        printf("\nFarewell, adventurer!\n");
    }
    
    printf("\nThanks for playing!\n");
    return 0;
}
