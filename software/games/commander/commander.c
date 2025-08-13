#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* AI Commander - Strategy Game with AI Programming for Apple 1
 * Compiled with cc65 for 6502
 * Program AI behaviors and watch them battle!
 */

#define MAX_INPUT 30
#define GRID_SIZE 8
#define MAX_UNITS 12
#define MAX_BEHAVIORS 8
#define MAX_RULES 16

/* Game state */
unsigned char game_turn = 0;
unsigned char game_mode = 0;  /* 0=setup, 1=battle, 2=ended */
unsigned char current_player = 0;  /* 0=player1, 1=player2, 2=AI */
unsigned char rng_seed = 42;

/* Unit structure */
struct unit {
    unsigned char x, y;
    unsigned char health;
    unsigned char max_health;
    unsigned char attack;
    unsigned char player;
    unsigned char behavior_id;
    unsigned char active;
    unsigned char last_action;
};

/* AI Behavior rule structure */
struct ai_rule {
    unsigned char condition;    /* What to check */
    unsigned char parameter;    /* Condition parameter */
    unsigned char action;       /* What to do */
    unsigned char priority;     /* Rule priority */
};

/* AI Behavior sets */
struct ai_behavior {
    char name[12];
    unsigned char num_rules;
    struct ai_rule rules[4];
};

/* Game data */
struct unit units[MAX_UNITS];
struct ai_behavior behaviors[MAX_BEHAVIORS];
unsigned char grid[GRID_SIZE][GRID_SIZE];  /* 0=empty, 1=obstacle, 2=unit_id+2 */
unsigned char num_units = 0;
unsigned char num_behaviors = 0;

char input[MAX_INPUT];

/* Simple random number generator */
unsigned char simple_random(void) {
    rng_seed = (rng_seed * 17 + 13) & 0xFF;
    return rng_seed;
}

/* AI Conditions */
#define COND_ENEMY_NEAR     1
#define COND_HEALTH_LOW     2
#define COND_ALLY_NEAR      3
#define COND_AT_EDGE        4
#define COND_ENEMY_WEAK     5
#define COND_OUTNUMBERED    6

/* AI Actions */
#define ACT_MOVE_RANDOM     1
#define ACT_MOVE_TO_ENEMY   2
#define ACT_MOVE_AWAY       3
#define ACT_ATTACK_NEAREST  4
#define ACT_DEFEND          5
#define ACT_MOVE_TO_ALLY    6
#define ACT_WAIT            7

void init_game(void) {
    unsigned char i, j;
    
    /* Clear grid */
    for (i = 0; i < GRID_SIZE; i++) {
        for (j = 0; j < GRID_SIZE; j++) {
            grid[i][j] = 0;
        }
    }
    
    /* Add some obstacles */
    grid[3][3] = 1;
    grid[3][4] = 1;
    grid[4][3] = 1;
    grid[4][4] = 1;
    grid[1][6] = 1;
    grid[6][1] = 1;
    
    /* Initialize default AI behaviors */
    strcpy(behaviors[0].name, "AGGRESSIVE");
    behaviors[0].num_rules = 3;
    behaviors[0].rules[0].condition = COND_ENEMY_NEAR;
    behaviors[0].rules[0].parameter = 2;
    behaviors[0].rules[0].action = ACT_ATTACK_NEAREST;
    behaviors[0].rules[0].priority = 10;
    
    behaviors[0].rules[1].condition = COND_ENEMY_WEAK;
    behaviors[0].rules[1].parameter = 3;
    behaviors[0].rules[1].action = ACT_MOVE_TO_ENEMY;
    behaviors[0].rules[1].priority = 8;
    
    behaviors[0].rules[2].condition = 0;  /* Always true */
    behaviors[0].rules[2].parameter = 0;
    behaviors[0].rules[2].action = ACT_MOVE_RANDOM;
    behaviors[0].rules[2].priority = 1;
    
    strcpy(behaviors[1].name, "DEFENSIVE");
    behaviors[1].num_rules = 3;
    behaviors[1].rules[0].condition = COND_HEALTH_LOW;
    behaviors[1].rules[0].parameter = 3;
    behaviors[1].rules[0].action = ACT_MOVE_AWAY;
    behaviors[1].rules[0].priority = 10;
    
    behaviors[1].rules[1].condition = COND_ALLY_NEAR;
    behaviors[1].rules[1].parameter = 2;
    behaviors[1].rules[1].action = ACT_DEFEND;
    behaviors[1].rules[1].priority = 7;
    
    behaviors[1].rules[2].condition = 0;
    behaviors[1].rules[2].parameter = 0;
    behaviors[1].rules[2].action = ACT_MOVE_TO_ALLY;
    behaviors[1].rules[2].priority = 2;
    
    strcpy(behaviors[2].name, "SCOUT");
    behaviors[2].num_rules = 2;
    behaviors[2].rules[0].condition = COND_AT_EDGE;
    behaviors[2].rules[0].parameter = 0;
    behaviors[2].rules[0].action = ACT_MOVE_RANDOM;
    behaviors[2].rules[0].priority = 8;
    
    behaviors[2].rules[1].condition = 0;
    behaviors[2].rules[1].parameter = 0;
    behaviors[2].rules[1].action = ACT_MOVE_TO_ENEMY;
    behaviors[2].rules[1].priority = 5;
    
    num_behaviors = 3;
    
    /* Clear units */
    for (i = 0; i < MAX_UNITS; i++) {
        units[i].active = 0;
    }
    num_units = 0;
}

void print_grid(void) {
    unsigned char i, j;
    
    printf("\n  ");
    for (i = 0; i < GRID_SIZE; i++) {
        printf("%d ", i);
    }
    printf("\n");
    
    for (i = 0; i < GRID_SIZE; i++) {
        printf("%d ", i);
        for (j = 0; j < GRID_SIZE; j++) {
            if (grid[i][j] == 1) {
                printf("# ");  /* Obstacle */
            } else if (grid[i][j] >= 2) {
                /* Find unit at this position */
                unsigned char unit_id = grid[i][j] - 2;
                if (units[unit_id].active) {
                    if (units[unit_id].player == 0) {
                        printf("A ");  /* Player A unit */
                    } else if (units[unit_id].player == 1) {
                        printf("B ");  /* Player B unit */
                    } else {
                        printf("C ");  /* AI unit */
                    }
                } else {
                    printf(". ");
                }
            } else {
                printf(". ");  /* Empty */
            }
        }
        printf("\n");
    }
}

void print_units(void) {
    unsigned char i;
    
    printf("\n=== UNITS ===\n");
    for (i = 0; i < MAX_UNITS; i++) {
        if (units[i].active) {
            char player_name;
            if (units[i].player == 0) player_name = 'A';
            else if (units[i].player == 1) player_name = 'B';
            else player_name = 'C';
            
            printf("Unit %c%d: (%d,%d) HP:%d/%d ATK:%d AI:%s\n",
                   player_name, i,
                   units[i].x, units[i].y,
                   units[i].health, units[i].max_health,
                   units[i].attack,
                   behaviors[units[i].behavior_id].name);
        }
    }
}

unsigned char add_unit(unsigned char x, unsigned char y, unsigned char player, unsigned char behavior) {
    unsigned char i;
    
    if (grid[y][x] != 0) return 255;  /* Position occupied */
    
    for (i = 0; i < MAX_UNITS; i++) {
        if (!units[i].active) {
            units[i].x = x;
            units[i].y = y;
            units[i].health = 5;
            units[i].max_health = 5;
            units[i].attack = 2;
            units[i].player = player;
            units[i].behavior_id = behavior;
            units[i].active = 1;
            units[i].last_action = 0;
            
            grid[y][x] = i + 2;
            num_units++;
            return i;
        }
    }
    return 255;  /* No free slots */
}

unsigned char get_distance(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2) {
    unsigned char dx = (x1 > x2) ? x1 - x2 : x2 - x1;
    unsigned char dy = (y1 > y2) ? y1 - y2 : y2 - y1;
    return dx + dy;  /* Manhattan distance */
}

unsigned char find_nearest_enemy(unsigned char unit_id) {
    unsigned char i;
    unsigned char nearest = 255;
    unsigned char min_dist = 255;
    
    for (i = 0; i < MAX_UNITS; i++) {
        if (units[i].active && units[i].player != units[unit_id].player) {
            unsigned char dist = get_distance(units[unit_id].x, units[unit_id].y,
                                            units[i].x, units[i].y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
    }
    return nearest;
}

unsigned char find_nearest_ally(unsigned char unit_id) {
    unsigned char i;
    unsigned char nearest = 255;
    unsigned char min_dist = 255;
    
    for (i = 0; i < MAX_UNITS; i++) {
        if (units[i].active && units[i].player == units[unit_id].player && i != unit_id) {
            unsigned char dist = get_distance(units[unit_id].x, units[unit_id].y,
                                            units[i].x, units[i].y);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
    }
    return nearest;
}

unsigned char check_condition(unsigned char unit_id, unsigned char condition, unsigned char parameter) {
    unsigned char nearest;
    
    switch (condition) {
        case 0:
            return 1;  /* Always true */
            
        case COND_ENEMY_NEAR:
            nearest = find_nearest_enemy(unit_id);
            if (nearest != 255) {
                unsigned char dist = get_distance(units[unit_id].x, units[unit_id].y,
                                                units[nearest].x, units[nearest].y);
                return (dist <= parameter);
            }
            return 0;
            
        case COND_HEALTH_LOW:
            return (units[unit_id].health <= parameter);
            
        case COND_ALLY_NEAR:
            nearest = find_nearest_ally(unit_id);
            if (nearest != 255) {
                unsigned char dist = get_distance(units[unit_id].x, units[unit_id].y,
                                                units[nearest].x, units[nearest].y);
                return (dist <= parameter);
            }
            return 0;
            
        case COND_AT_EDGE:
            return (units[unit_id].x == 0 || units[unit_id].x == GRID_SIZE-1 ||
                   units[unit_id].y == 0 || units[unit_id].y == GRID_SIZE-1);
                   
        case COND_ENEMY_WEAK:
            nearest = find_nearest_enemy(unit_id);
            if (nearest != 255) {
                return (units[nearest].health <= parameter);
            }
            return 0;
            
        case COND_OUTNUMBERED: {
            unsigned char i;
            unsigned char allies = 0;
            unsigned char enemies = 0;
            for (i = 0; i < MAX_UNITS; i++) {
                if (units[i].active) {
                    unsigned char dist = get_distance(units[unit_id].x, units[unit_id].y,
                                                    units[i].x, units[i].y);
                    if (dist <= parameter) {
                        if (units[i].player == units[unit_id].player) {
                            allies++;
                        } else {
                            enemies++;
                        }
                    }
                }
            }
            return (enemies > allies);
        }
        default:
            break;
    }
    return 0;
}

void move_unit(unsigned char unit_id, unsigned char new_x, unsigned char new_y) {
    if (new_x >= GRID_SIZE || new_y >= GRID_SIZE) return;
    if (grid[new_y][new_x] != 0) return;  /* Position occupied */
    
    /* Clear old position */
    grid[units[unit_id].y][units[unit_id].x] = 0;
    
    /* Set new position */
    units[unit_id].x = new_x;
    units[unit_id].y = new_y;
    grid[new_y][new_x] = unit_id + 2;
}

void attack_unit(unsigned char attacker_id, unsigned char target_id) {
    unsigned char distance = get_distance(units[attacker_id].x, units[attacker_id].y,
                                        units[target_id].x, units[target_id].y);
    
    if (distance <= 1) {  /* Adjacent attack */
        unsigned char damage = units[attacker_id].attack;
        if (units[target_id].health > damage) {
            units[target_id].health -= damage;
        } else {
            /* Unit destroyed */
            grid[units[target_id].y][units[target_id].x] = 0;
            units[target_id].active = 0;
            num_units--;
            printf("Unit destroyed at (%d,%d)!\n", units[target_id].x, units[target_id].y);
        }
    }
}

void execute_action(unsigned char unit_id, unsigned char action) {
    unsigned char target_id;
    unsigned char new_x, new_y;
    unsigned char dx, dy;
    
    units[unit_id].last_action = action;
    
    switch (action) {
        case ACT_MOVE_RANDOM: {
            unsigned char direction = simple_random() & 3;
            new_x = units[unit_id].x;
            new_y = units[unit_id].y;
            
            switch (direction) {
                case 0: if (new_y > 0) new_y--; break;          /* North */
                case 1: if (new_x < GRID_SIZE-1) new_x++; break; /* East */
                case 2: if (new_y < GRID_SIZE-1) new_y++; break; /* South */
                case 3: if (new_x > 0) new_x--; break;          /* West */
            }
            move_unit(unit_id, new_x, new_y);
            break;
        }
        
        case ACT_MOVE_TO_ENEMY:
            target_id = find_nearest_enemy(unit_id);
            if (target_id != 255) {
                /* Move one step toward enemy */
                dx = (units[target_id].x > units[unit_id].x) ? 1 : 
                     (units[target_id].x < units[unit_id].x) ? -1 : 0;
                dy = (units[target_id].y > units[unit_id].y) ? 1 :
                     (units[target_id].y < units[unit_id].y) ? -1 : 0;
                     
                new_x = units[unit_id].x + dx;
                new_y = units[unit_id].y + dy;
                move_unit(unit_id, new_x, new_y);
            }
            break;
            
        case ACT_MOVE_AWAY:
            target_id = find_nearest_enemy(unit_id);
            if (target_id != 255) {
                /* Move away from enemy */
                dx = (units[target_id].x > units[unit_id].x) ? -1 : 
                     (units[target_id].x < units[unit_id].x) ? 1 : 0;
                dy = (units[target_id].y > units[unit_id].y) ? -1 :
                     (units[target_id].y < units[unit_id].y) ? 1 : 0;
                     
                new_x = units[unit_id].x + dx;
                new_y = units[unit_id].y + dy;
                move_unit(unit_id, new_x, new_y);
            }
            break;
            
        case ACT_ATTACK_NEAREST:
            target_id = find_nearest_enemy(unit_id);
            if (target_id != 255) {
                attack_unit(unit_id, target_id);
            }
            break;
            
        case ACT_MOVE_TO_ALLY:
            target_id = find_nearest_ally(unit_id);
            if (target_id != 255) {
                dx = (units[target_id].x > units[unit_id].x) ? 1 : 
                     (units[target_id].x < units[unit_id].x) ? -1 : 0;
                dy = (units[target_id].y > units[unit_id].y) ? 1 :
                     (units[target_id].y < units[unit_id].y) ? -1 : 0;
                     
                new_x = units[unit_id].x + dx;
                new_y = units[unit_id].y + dy;
                move_unit(unit_id, new_x, new_y);
            }
            break;
            
        case ACT_DEFEND:
        case ACT_WAIT:
            /* Do nothing */
            break;
    }
}

void ai_turn(unsigned char unit_id) {
    unsigned char behavior_id = units[unit_id].behavior_id;
    unsigned char i;
    unsigned char best_action = ACT_WAIT;
    unsigned char best_priority = 0;
    
    /* Evaluate all rules and find highest priority applicable action */
    for (i = 0; i < behaviors[behavior_id].num_rules; i++) {
        struct ai_rule* rule = &behaviors[behavior_id].rules[i];
        
        if (check_condition(unit_id, rule->condition, rule->parameter)) {
            if (rule->priority > best_priority) {
                best_priority = rule->priority;
                best_action = rule->action;
            }
        }
    }
    
    execute_action(unit_id, best_action);
}

void run_battle_turn(void) {
    unsigned char i;
    unsigned char player_counts[3];
    
    printf("\n=== TURN %d ===\n", game_turn + 1);
    
    /* Process all units */
    for (i = 0; i < MAX_UNITS; i++) {
        if (units[i].active) {
            ai_turn(i);
        }
    }
    
    game_turn++;
    print_grid();
    
    /* Check win condition */
    player_counts[0] = 0;
    player_counts[1] = 0;
    player_counts[2] = 0;
    
    for (i = 0; i < MAX_UNITS; i++) {
        if (units[i].active) {
            player_counts[units[i].player]++;
        }
    }
    
    if (player_counts[0] == 0 && player_counts[1] == 0) {
        printf("\nAI WINS!\n");
        game_mode = 2;
    } else if (player_counts[0] == 0) {
        printf("\nPLAYER B WINS!\n");
        game_mode = 2;
    } else if (player_counts[1] == 0 && player_counts[2] == 0) {
        printf("\nPLAYER A WINS!\n");
        game_mode = 2;
    }
}

void setup_units(void) {
    unsigned char x, y, player, behavior;
    
    printf("Add unit at position (x y player[0-2] behavior[0-2]): ");
    if (fgets(input, MAX_INPUT, stdin)) {
        if (sscanf(input, "%d %d %d %d", &x, &y, &player, &behavior) == 4) {
            if (x < GRID_SIZE && y < GRID_SIZE && player < 3 && behavior < num_behaviors) {
                if (add_unit(x, y, player, behavior) != 255) {
                    printf("Unit added!\n");
                } else {
                    printf("Could not add unit (position occupied or no slots)!\n");
                }
            } else {
                printf("Invalid parameters!\n");
            }
        } else {
            printf("Invalid format! Use: x y player behavior\n");
        }
    }
}

void show_behaviors(void) {
    unsigned char i, j;
    
    printf("\n=== AI BEHAVIORS ===\n");
    for (i = 0; i < num_behaviors; i++) {
        printf("%d. %s\n", i, behaviors[i].name);
        for (j = 0; j < behaviors[i].num_rules; j++) {
            struct ai_rule* rule = &behaviors[i].rules[j];
            printf("   Rule %d: Priority %d\n", j+1, rule->priority);
            printf("   Condition: %d (param: %d)\n", rule->condition, rule->parameter);
            printf("   Action: %d\n", rule->action);
        }
        printf("\n");
    }
}

void process_input(void) {
    char command;
    
    if (strlen(input) == 0) return;
    command = toupper(input[0]);
    
    switch (command) {
        case 'G':
            print_grid();
            break;
            
        case 'U':
            print_units();
            break;
            
        case 'A':
            if (game_mode == 0) {
                setup_units();
            } else {
                printf("Cannot add units during battle!\n");
            }
            break;
            
        case 'B':
            show_behaviors();
            break;
            
        case 'S':
            if (game_mode == 0 && num_units >= 2) {
                game_mode = 1;
                printf("Battle started!\n");
                print_grid();
            } else {
                printf("Need at least 2 units to start battle!\n");
            }
            break;
            
        case 'T':
            if (game_mode == 1) {
                run_battle_turn();
            } else {
                printf("Battle not in progress!\n");
            }
            break;
            
        case 'R':
            if (game_mode == 1) {
                while (game_mode == 1) {
                    run_battle_turn();
                    if (game_turn > 50) {  /* Prevent infinite battles */
                        printf("Battle timeout - draw!\n");
                        game_mode = 2;
                        break;
                    }
                }
            }
            break;
            
        case 'N':
            init_game();
            game_mode = 0;
            game_turn = 0;
            printf("New game started!\n");
            break;
            
        case 'H':
            printf("\n=== AI COMMANDER HELP ===\n");
            printf("Setup Phase:\n");
            printf("G - Show grid\n");
            printf("U - Show units\n");
            printf("A - Add unit\n");
            printf("B - Show AI behaviors\n");
            printf("S - Start battle\n");
            printf("\nBattle Phase:\n");
            printf("T - Next turn\n");
            printf("R - Run battle to completion\n");
            printf("\nGeneral:\n");
            printf("N - New game\n");
            printf("Q - Quit\n");
            break;
            
        case 'Q':
            game_mode = 3;  /* Quit */
            break;
            
        default:
            printf("Unknown command. Type H for help.\n");
            break;
    }
}

int main(void) {
    printf("*** AI COMMANDER ***\n");
    printf("Program AI behaviors and watch them battle!\n");
    printf("Type H for help.\n\n");
    
    init_game();
    print_grid();
    printf("\nSetup phase: Add units with A command, then S to start battle.\n");
    
    while (game_mode != 3) {
        printf("\n> ");
        
        if (fgets(input, MAX_INPUT, stdin)) {
            input[strcspn(input, "\n")] = 0;
            process_input();
        }
    }
    
    printf("\nThanks for playing AI Commander!\n");
    return 0;
}
