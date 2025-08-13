#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* Medieval Lords - Strategy Game for Apple 1
 * Compiled with cc65 for 6502
 * Manage your medieval kingdom, build armies, conquer lands
 */

#define MAX_INPUT 25
#define NUM_TERRITORIES 8
#define NUM_RESOURCES 4
#define NUM_BUILDINGS 5
#define NUM_UNIT_TYPES 4

/* Game state */
unsigned char current_territory = 0;
unsigned char game_turn = 1;
unsigned char game_flags = 0;
unsigned char player_gold = 100;
unsigned char player_fame = 0;
unsigned char rng_seed = 23;

/* Resource types: FOOD, WOOD, STONE, IRON */
unsigned char resources[NUM_RESOURCES] = {50, 30, 20, 10};
const char* resource_names[NUM_RESOURCES] = {
    "FOOD", "WOOD", "STONE", "IRON"
};

/* Building types: FARM, LUMBER MILL, QUARRY, MINE, BARRACKS */
unsigned char buildings[NUM_TERRITORIES][NUM_BUILDINGS];
const char* building_names[NUM_BUILDINGS] = {
    "FARM", "LUMBER MILL", "QUARRY", "MINE", "BARRACKS"
};

/* Unit types: PEASANTS, ARCHERS, KNIGHTS, CATAPULTS */
unsigned char units[NUM_TERRITORIES][NUM_UNIT_TYPES];
const char* unit_names[NUM_UNIT_TYPES] = {
    "PEASANTS", "ARCHERS", "KNIGHTS", "CATAPULTS"
};

/* Territory data */
struct territory {
    const char* name;
    unsigned char owner;        /* 0=player, 1=enemy, 2=neutral */
    unsigned char population;
    unsigned char defense;
    unsigned char prosperity;
};

struct territory territories[NUM_TERRITORIES] = {
    {"HOMELAND", 0, 100, 50, 80},
    {"GREENFIELD", 2, 60, 20, 40},
    {"IRONHOLD", 1, 80, 70, 60},
    {"WOODHAVEN", 2, 40, 30, 50},
    {"STONEWALL", 1, 90, 90, 30},
    {"GOLDVALE", 2, 70, 40, 90},
    {"DARKFOREST", 1, 30, 80, 20},
    {"DRAGONPEAK", 1, 50, 100, 10}
};

/* Building costs: FOOD, WOOD, STONE, IRON, GOLD */
const unsigned char building_costs[NUM_BUILDINGS][5] = {
    {5, 10, 0, 0, 20},      /* Farm */
    {10, 5, 5, 0, 30},      /* Lumber Mill */
    {15, 10, 0, 5, 40},     /* Quarry */
    {20, 15, 10, 0, 50},    /* Mine */
    {30, 20, 25, 15, 100}   /* Barracks */
};

/* Unit costs: FOOD, WOOD, STONE, IRON, GOLD */
const unsigned char unit_costs[NUM_UNIT_TYPES][5] = {
    {2, 1, 0, 0, 5},        /* Peasants */
    {5, 8, 0, 2, 15},       /* Archers */
    {10, 5, 0, 10, 50},     /* Knights */
    {20, 30, 40, 20, 200}   /* Catapults */
};

/* Unit combat values: ATTACK, DEFENSE */
const unsigned char unit_stats[NUM_UNIT_TYPES][2] = {
    {1, 1},     /* Peasants */
    {3, 2},     /* Archers */
    {5, 4},     /* Knights */
    {8, 3}      /* Catapults */
};

char input[MAX_INPUT];

/* Simple random number generator */
unsigned char simple_random(void) {
    rng_seed = (rng_seed * 13 + 7) & 0xFF;
    return rng_seed;
}

void init_game(void) {
    unsigned char i, j;
    
    /* Initialize buildings - player starts with basic infrastructure */
    buildings[0][0] = 2;  /* 2 farms in homeland */
    buildings[0][1] = 1;  /* 1 lumber mill */
    buildings[0][4] = 1;  /* 1 barracks */
    
    /* Initialize starting units */
    units[0][0] = 20;     /* 20 peasants */
    units[0][1] = 5;      /* 5 archers */
    units[0][2] = 2;      /* 2 knights */
    
    /* Clear other territories */
    for (i = 1; i < NUM_TERRITORIES; i++) {
        for (j = 0; j < NUM_BUILDINGS; j++) {
            buildings[i][j] = 0;
        }
        for (j = 0; j < NUM_UNIT_TYPES; j++) {
            units[i][j] = 0;
        }
    }
}

void print_header(void) {
    printf("\n=== MEDIEVAL LORDS - TURN %d ===\n", game_turn);
    printf("GOLD: %d  FAME: %d\n", player_gold, player_fame);
    printf("RESOURCES: ");
    printf("FOOD:%d WOOD:%d STONE:%d IRON:%d\n", 
           resources[0], resources[1], resources[2], resources[3]);
}

void print_territory_status(void) {
    unsigned char i;
    const char* owner_names[3] = {"YOURS", "ENEMY", "NEUTRAL"};
    
    printf("\n=== TERRITORY STATUS ===\n");
    for (i = 0; i < NUM_TERRITORIES; i++) {
        printf("%d. %s (%s) - POP:%d DEF:%d PROS:%d\n",
               i + 1, territories[i].name, owner_names[territories[i].owner],
               territories[i].population, territories[i].defense, territories[i].prosperity);
    }
}

void print_current_territory(void) {
    unsigned char i;
    
    printf("\n=== %s ===\n", territories[current_territory].name);
    printf("Population: %d  Defense: %d  Prosperity: %d\n",
           territories[current_territory].population,
           territories[current_territory].defense,
           territories[current_territory].prosperity);
    
    if (territories[current_territory].owner == 0) {
        printf("\nBUILDINGS:\n");
        for (i = 0; i < NUM_BUILDINGS; i++) {
            if (buildings[current_territory][i] > 0) {
                printf("%s: %d\n", building_names[i], buildings[current_territory][i]);
            }
        }
        
        printf("\nARMY:\n");
        for (i = 0; i < NUM_UNIT_TYPES; i++) {
            if (units[current_territory][i] > 0) {
                printf("%s: %d\n", unit_names[i], units[current_territory][i]);
            }
        }
    } else {
        printf("This territory is not under your control.\n");
    }
}

unsigned char can_afford(const unsigned char costs[5]) {
    unsigned char i;
    
    for (i = 0; i < NUM_RESOURCES; i++) {
        if (resources[i] < costs[i]) return 0;
    }
    if (player_gold < costs[4]) return 0;
    return 1;
}

void pay_costs(const unsigned char costs[5]) {
    unsigned char i;
    
    for (i = 0; i < NUM_RESOURCES; i++) {
        resources[i] -= costs[i];
    }
    player_gold -= costs[4];
}

void build_structure(void) {
    unsigned char building_type;
    
    if (territories[current_territory].owner != 0) {
        printf("You can only build in your own territories!\n");
        return;
    }
    
    printf("\nBUILD WHAT?\n");
    printf("1. FARM (Food:5 Wood:10 Gold:20)\n");
    printf("2. LUMBER MILL (Food:10 Wood:5 Stone:5 Gold:30)\n");
    printf("3. QUARRY (Food:15 Wood:10 Iron:5 Gold:40)\n");
    printf("4. MINE (Food:20 Wood:15 Stone:10 Gold:50)\n");
    printf("5. BARRACKS (Food:30 Wood:20 Stone:25 Iron:15 Gold:100)\n");
    printf("Choice (1-5): ");
    
    if (fgets(input, MAX_INPUT, stdin)) {
        building_type = input[0] - '1';
        
        if (building_type < NUM_BUILDINGS) {
            if (can_afford(building_costs[building_type])) {
                pay_costs(building_costs[building_type]);
                buildings[current_territory][building_type]++;
                printf("Built %s!\n", building_names[building_type]);
                
                /* Increase territory prosperity */
                if (territories[current_territory].prosperity < 100) {
                    territories[current_territory].prosperity += 5;
                }
            } else {
                printf("Insufficient resources!\n");
            }
        } else {
            printf("Invalid choice!\n");
        }
    }
}

void recruit_units(void) {
    unsigned char unit_type;
    unsigned char quantity;
    
    if (territories[current_territory].owner != 0) {
        printf("You can only recruit in your own territories!\n");
        return;
    }
    
    if (buildings[current_territory][4] == 0) {
        printf("You need a barracks to recruit units!\n");
        return;
    }
    
    printf("\nRECRUIT WHAT?\n");
    printf("1. PEASANTS (Food:2 Wood:1 Gold:5)\n");
    printf("2. ARCHERS (Food:5 Wood:8 Iron:2 Gold:15)\n");
    printf("3. KNIGHTS (Food:10 Wood:5 Iron:10 Gold:50)\n");
    printf("4. CATAPULTS (Food:20 Wood:30 Stone:40 Iron:20 Gold:200)\n");
    printf("Choice (1-4): ");
    
    if (fgets(input, MAX_INPUT, stdin)) {
        unit_type = input[0] - '1';
        
        if (unit_type < NUM_UNIT_TYPES) {
            printf("How many? ");
            if (fgets(input, MAX_INPUT, stdin)) {
                quantity = 0;
                /* Simple string to number conversion */
                if (input[0] >= '1' && input[0] <= '9') {
                    quantity = input[0] - '0';
                    if (input[1] >= '0' && input[1] <= '9') {
                        quantity = quantity * 10 + (input[1] - '0');
                    }
                }
                
                if (quantity > 0 && quantity <= 20) {
                    unsigned char total_costs[5];
                    unsigned char i;
                    
                    /* Calculate total costs */
                    for (i = 0; i < 5; i++) {
                        total_costs[i] = unit_costs[unit_type][i] * quantity;
                    }
                    
                    if (can_afford(total_costs)) {
                        pay_costs(total_costs);
                        units[current_territory][unit_type] += quantity;
                        printf("Recruited %d %s!\n", quantity, unit_names[unit_type]);
                    } else {
                        printf("Insufficient resources!\n");
                    }
                } else {
                    printf("Invalid quantity (1-20)!\n");
                }
            }
        } else {
            printf("Invalid choice!\n");
        }
    }
}

unsigned char calculate_army_strength(unsigned char territory_id) {
    unsigned char i;
    unsigned char strength = 0;
    
    for (i = 0; i < NUM_UNIT_TYPES; i++) {
        strength += units[territory_id][i] * unit_stats[i][0];
    }
    return strength;
}

void attack_territory(void) {
    unsigned char target;
    unsigned char player_strength;
    unsigned char enemy_strength;
    unsigned char casualties;
    unsigned char i;
    
    printf("Attack which territory (1-%d)? ", NUM_TERRITORIES);
    
    if (fgets(input, MAX_INPUT, stdin)) {
        target = input[0] - '1';
        
        if (target >= NUM_TERRITORIES) {
            printf("Invalid territory!\n");
            return;
        }
        
        if (territories[target].owner == 0) {
            printf("You already control that territory!\n");
            return;
        }
        
        player_strength = calculate_army_strength(current_territory);
        if (player_strength == 0) {
            printf("You have no army to attack with!\n");
            return;
        }
        
        /* Calculate enemy strength based on territory defense */
        enemy_strength = territories[target].defense + (simple_random() & 31);
        
        printf("\nBATTLE FOR %s!\n", territories[target].name);
        printf("Your strength: %d\n", player_strength);
        printf("Enemy strength: %d\n", enemy_strength);
        
        if (player_strength > enemy_strength) {
            printf("\nVICTORY! You have conquered %s!\n", territories[target].name);
            territories[target].owner = 0;
            player_fame += 10;
            player_gold += territories[target].prosperity;
            
            /* Move half your army to the conquered territory */
            for (i = 0; i < NUM_UNIT_TYPES; i++) {
                units[target][i] = units[current_territory][i] / 2;
                units[current_territory][i] -= units[target][i];
            }
        } else {
            printf("\nDEFEAT! Your army has been repelled!\n");
            
            /* Calculate casualties */
            casualties = (enemy_strength - player_strength) / 4;
            if (casualties > 10) casualties = 10;
            
            /* Remove casualties from weakest units first */
            for (i = 0; i < NUM_UNIT_TYPES && casualties > 0; i++) {
                if (units[current_territory][i] > 0) {
                    unsigned char lost = (casualties > units[current_territory][i]) ? 
                                       units[current_territory][i] : casualties;
                    units[current_territory][i] -= lost;
                    casualties -= lost;
                    printf("Lost %d %s\n", lost, unit_names[i]);
                }
            }
        }
    }
}

void collect_resources(void) {
    unsigned char i;
    unsigned char production[NUM_RESOURCES] = {0, 0, 0, 0};
    unsigned char gold_income = 0;
    
    /* Calculate resource production from all owned territories */
    for (i = 0; i < NUM_TERRITORIES; i++) {
        if (territories[i].owner == 0) {
            /* Each building type produces different resources */
            production[0] += buildings[i][0] * 10;  /* Farms produce food */
            production[1] += buildings[i][1] * 8;   /* Mills produce wood */
            production[2] += buildings[i][2] * 6;   /* Quarries produce stone */
            production[3] += buildings[i][3] * 4;   /* Mines produce iron */
            
            /* Base production from population */
            production[0] += territories[i].population / 20;
            production[1] += territories[i].population / 30;
        }
    }
    
    printf("\n=== RESOURCE COLLECTION ===\n");
    for (i = 0; i < NUM_RESOURCES; i++) {
        /* Check for overflow before adding */
        if (resources[i] > 255 - production[i]) {
            resources[i] = 255;  /* Cap at 255 */
        } else {
            resources[i] += production[i];
        }
        printf("Collected %d %s (Total: %d)\n", 
               production[i], resource_names[i], resources[i]);
    }
    
    /* Collect gold from prosperity */
    for (i = 0; i < NUM_TERRITORIES; i++) {
        if (territories[i].owner == 0) {
            gold_income += territories[i].prosperity / 10;
        }
    }
    /* Check for overflow before adding */
    if (player_gold > 255 - gold_income) {
        player_gold = 255;
    } else {
        player_gold += gold_income;
    }
    printf("Collected %d gold from prosperity\n", gold_income);
}

void enemy_turn(void) {
    unsigned char i, j, k;
    
    printf("\n=== ENEMY ACTIONS ===\n");
    
    /* Simple AI: enemies occasionally attack player territories */
    for (i = 0; i < NUM_TERRITORIES; i++) {
        if (territories[i].owner == 1 && (simple_random() & 7) == 0) {
            /* Look for adjacent player territory to attack */
            for (j = 0; j < NUM_TERRITORIES; j++) {
                if (territories[j].owner == 0 && (simple_random() & 3) == 0) {
                    unsigned char enemy_str = territories[i].defense;
                    unsigned char player_str = calculate_army_strength(j) + territories[j].defense;
                    
                    printf("%s attacks %s!\n", territories[i].name, territories[j].name);
                    
                    if (enemy_str > player_str) {
                        printf("DEFEAT! %s has been lost!\n", territories[j].name);
                        territories[j].owner = 1;
                        /* Clear player units */
                        for (k = 0; k < NUM_UNIT_TYPES; k++) {
                            units[j][k] = 0;
                        }
                        for (k = 0; k < NUM_BUILDINGS; k++) {
                            buildings[j][k] = 0;
                        }
                    } else {
                        printf("Your defenses hold!\n");
                    }
                    break;
                }
            }
            break;
        }
    }
}

unsigned char check_victory(void) {
    unsigned char player_territories = 0;
    unsigned char i;
    
    for (i = 0; i < NUM_TERRITORIES; i++) {
        if (territories[i].owner == 0) {
            player_territories++;
        }
    }
    
    if (player_territories >= 6) {
        printf("\n*** VICTORY! ***\n");
        printf("You have conquered most of the realm!\n");
        printf("Your fame: %d\n", player_fame);
        printf("Final gold: %d\n", player_gold);
        printf("You are now the supreme Medieval Lord!\n");
        return 1;
    }
    
    if (player_territories == 0) {
        printf("\n*** DEFEAT! ***\n");
        printf("You have lost all your territories!\n");
        printf("Your reign has ended...\n");
        return 1;
    }
    
    return 0;
}

void process_input(void) {
    char command;
    
    if (strlen(input) == 0) return;
    
    command = toupper(input[0]);
    
    switch (command) {
        case 'M': {
            unsigned char new_territory;
            printf("Move to which territory (1-%d)? ", NUM_TERRITORIES);
            if (fgets(input, MAX_INPUT, stdin)) {
                new_territory = input[0] - '1';
                if (new_territory < NUM_TERRITORIES) {
                    current_territory = new_territory;
                    print_current_territory();
                } else {
                    printf("Invalid territory!\n");
                }
            }
            break;
        }
        
        case 'B':
            build_structure();
            break;
            
        case 'R':
            recruit_units();
            break;
            
        case 'A':
            attack_territory();
            break;
            
        case 'L':
            print_current_territory();
            break;
            
        case 'T':
            print_territory_status();
            break;
            
        case 'E':
            /* End turn */
            collect_resources();
            enemy_turn();
            game_turn++;
            printf("\nTurn %d begins...\n", game_turn);
            break;
            
        case 'S':
            print_header();
            break;
            
        case 'H':
            printf("\n=== COMMANDS ===\n");
            printf("M - Move to territory\n");
            printf("B - Build structure\n");
            printf("R - Recruit units\n");
            printf("A - Attack territory\n");
            printf("L - Look at current territory\n");
            printf("T - Territory status\n");
            printf("E - End turn\n");
            printf("S - Show status\n");
            printf("Q - Quit game\n");
            break;
            
        case 'Q':
            game_flags |= 1;
            break;
            
        default:
            printf("Unknown command. Type H for help.\n");
            break;
    }
}

int main(void) {
    printf("*** MEDIEVAL LORDS ***\n");
    printf("A Strategy Game for the Apple 1\n");
    printf("Conquer the realm and become the supreme lord!\n\n");
    printf("Type H for help.\n");
    
    init_game();
    print_header();
    print_current_territory();
    
    while (!(game_flags & 1) && !check_victory()) {
        printf("\n> ");
        
        if (fgets(input, MAX_INPUT, stdin)) {
            input[strcspn(input, "\n")] = 0;
            process_input();
        }
    }
    
    printf("\nThanks for playing Medieval Lords!\n");
    return 0;
}
