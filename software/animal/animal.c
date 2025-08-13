#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* Neural Animal Guesser - Apple 1 Compatible Version
 * Simplified without file I/O for replica1 target
 * Uses neural network to learn patterns and improve guessing
 */

#define MAX_INPUT 40
#define MAX_ANIMALS 16
#define MAX_QUESTIONS 12
#define NUM_FEATURES 6       /* Reduced for memory efficiency */
#define LEARNING_RATE 16

/* Animal knowledge base */
struct animal {
    char name[12];
    unsigned char features[NUM_FEATURES];
    unsigned char learned;
    unsigned char confidence;
};

/* Question database */
struct question {
    char text[32];
    unsigned char feature_index;
    unsigned char threshold;
    unsigned char asked_count;
    unsigned char success_rate;
};

/* Neural network for pattern recognition */
signed int feature_weights[NUM_FEATURES][NUM_FEATURES];
signed int animal_scores[MAX_ANIMALS];

/* Game data */
struct animal animals[MAX_ANIMALS];
struct question questions[MAX_QUESTIONS];
unsigned char num_animals = 0;
unsigned char num_questions = 0;
unsigned int  current_features[NUM_FEATURES];
unsigned char questions_asked[MAX_QUESTIONS];
unsigned char num_asked = 0;
unsigned char games_played = 0;
unsigned char games_won = 0;

char input[MAX_INPUT];
unsigned char rng_seed = 127;

/* Feature indices */
#define FEAT_SIZE       0    /* 0=tiny, 128=medium, 255=huge */
#define FEAT_HABITAT    1    /* 0=land, 128=amphibious, 255=water */
#define FEAT_DIET       2    /* 0=herbivore, 128=omnivore, 255=carnivore */
#define FEAT_ACTIVITY   3    /* 0=nocturnal, 128=crepuscular, 255=diurnal */
#define FEAT_DOMESTICATED 4  /* 0=wild, 255=domestic */
#define FEAT_INTELLIGENCE 5  /* 0=low, 255=high */

unsigned char simple_random(void) {
    rng_seed = (rng_seed * 31 + 19) & 0xFF;
    return rng_seed;
}

void init_animals(void) {
    unsigned char i, j;
    
    /* Initialize with basic animals */
    strcpy(animals[0].name, "DOG");
    animals[0].features[FEAT_SIZE] = 128;
    animals[0].features[FEAT_HABITAT] = 0;
    animals[0].features[FEAT_DIET] = 128;
    animals[0].features[FEAT_ACTIVITY] = 255;
    animals[0].features[FEAT_DOMESTICATED] = 255;
    animals[0].features[FEAT_INTELLIGENCE] = 200;
    animals[0].learned = 0;
    animals[0].confidence = 255;
    
    strcpy(animals[1].name, "CAT");
    animals[1].features[FEAT_SIZE] = 80;
    animals[1].features[FEAT_HABITAT] = 0;
    animals[1].features[FEAT_DIET] = 220;
    animals[1].features[FEAT_ACTIVITY] = 100;
    animals[1].features[FEAT_DOMESTICATED] = 255;
    animals[1].features[FEAT_INTELLIGENCE] = 180;
    animals[1].learned = 0;
    animals[1].confidence = 255;
    
    strcpy(animals[2].name, "ELEPHANT");
    animals[2].features[FEAT_SIZE] = 255;
    animals[2].features[FEAT_HABITAT] = 0;
    animals[2].features[FEAT_DIET] = 0;
    animals[2].features[FEAT_ACTIVITY] = 200;
    animals[2].features[FEAT_DOMESTICATED] = 50;
    animals[2].features[FEAT_INTELLIGENCE] = 255;
    animals[2].learned = 0;
    animals[2].confidence = 255;
    
    strcpy(animals[3].name, "FISH");
    animals[3].features[FEAT_SIZE] = 100;
    animals[3].features[FEAT_HABITAT] = 255;
    animals[3].features[FEAT_DIET] = 150;
    animals[3].features[FEAT_ACTIVITY] = 128;
    animals[3].features[FEAT_DOMESTICATED] = 100;
    animals[3].features[FEAT_INTELLIGENCE] = 80;
    animals[3].learned = 0;
    animals[3].confidence = 255;
    
    num_animals = 4;
    
    /* Clear remaining slots */
    for (i = num_animals; i < MAX_ANIMALS; i++) {
        animals[i].confidence = 0;
        animals[i].name[0] = '\0';
        for (j = 0; j < NUM_FEATURES; j++) {
            animals[i].features[j] = 0;
        }
    }
}

void init_questions(void) {
    unsigned char i;
    
    strcpy(questions[0].text, "Is it larger than a cat?");
    questions[0].feature_index = FEAT_SIZE;
    questions[0].threshold = 120;
    
    strcpy(questions[1].text, "Does it live in water?");
    questions[1].feature_index = FEAT_HABITAT;
    questions[1].threshold = 180;
    
    strcpy(questions[2].text, "Does it eat meat?");
    questions[2].feature_index = FEAT_DIET;
    questions[2].threshold = 150;
    
    strcpy(questions[3].text, "Is it active during day?");
    questions[3].feature_index = FEAT_ACTIVITY;
    questions[3].threshold = 180;
    
    strcpy(questions[4].text, "Is it a pet?");
    questions[4].feature_index = FEAT_DOMESTICATED;
    questions[4].threshold = 180;
    
    strcpy(questions[5].text, "Is it very smart?");
    questions[5].feature_index = FEAT_INTELLIGENCE;
    questions[5].threshold = 180;
    
    strcpy(questions[6].text, "Is it huge?");
    questions[6].feature_index = FEAT_SIZE;
    questions[6].threshold = 200;
    
    strcpy(questions[7].text, "Does it live on land?");
    questions[7].feature_index = FEAT_HABITAT;
    questions[7].threshold = 80;
    
    strcpy(questions[8].text, "Does it eat plants?");
    questions[8].feature_index = FEAT_DIET;
    questions[8].threshold = 100;
    
    strcpy(questions[9].text, "Is it found in homes?");
    questions[9].feature_index = FEAT_DOMESTICATED;
    questions[9].threshold = 200;
    
    num_questions = 10;
    
    /* Initialize statistics */
    for (i = 0; i < num_questions; i++) {
        questions[i].asked_count = 0;
        questions[i].success_rate = 128;
    }
}

void init_neural_network(void) {
    unsigned char i, j;
    
    for (i = 0; i < NUM_FEATURES; i++) {
        for (j = 0; j < NUM_FEATURES; j++) {
            if (i == j) {
                feature_weights[i][j] = 256;
            } else {
                feature_weights[i][j] = 0;
            }
        }
    }
}

void update_feature_from_answer(unsigned char question_id, unsigned char answer) {
    unsigned char feature = questions[question_id].feature_index;
    unsigned char threshold = questions[question_id].threshold;
    
    if (answer) {
        if (current_features[feature] < threshold) {
            current_features[feature] = threshold + 20;
        } else {
            /* Clamp to prevent overflow */
            if (current_features[feature] > 245) {
                current_features[feature] = 255;
            } else {
                current_features[feature] += 10;
            }
        }
    } else {
        if (current_features[feature] > threshold) {
            current_features[feature] = threshold - 20;
        } else {
            if (current_features[feature] >= 10) {
                current_features[feature] -= 10;
            } else {
                current_features[feature] = 0;
            }
        }
    }
    
    if (current_features[feature] > 255) current_features[feature] = 255;
}

void calculate_animal_scores(void) {
    unsigned char i, j;
    signed int score;
    signed int diff;
    signed int correlation;
    
    for (i = 0; i < num_animals; i++) {
        if (!animals[i].confidence) continue;
        
        score = 1000;
        
        for (j = 0; j < NUM_FEATURES; j++) {
            diff = (signed int)current_features[j] - (signed int)animals[i].features[j];
            if (diff < 0) diff = -diff;
            score -= diff * 2;
        }
        
        for (j = 0; j < NUM_FEATURES; j++) {
            correlation = (feature_weights[j][j] * current_features[j]) / 256;
            score += correlation / 4;
        }
        
        score += animals[i].confidence;
        animal_scores[i] = score;
    }
}

unsigned char find_best_question(void) {
    unsigned char i, j;
    unsigned char best_question = 255;
    signed int best_score = -1000;
    unsigned char already_asked;
    signed int score;
    unsigned char feature;
    unsigned char uncertainty;
    
    for (i = 0; i < num_questions; i++) {
        already_asked = 0;
        
        for (j = 0; j < num_asked; j++) {
            if (questions_asked[j] == i) {
                already_asked = 1;
                break;
            }
        }
        
        if (!already_asked) {
            score = questions[i].success_rate;
            score += (simple_random() & 31);
            
            feature = questions[i].feature_index;
            uncertainty = 128;
            if (current_features[feature] > 192 || current_features[feature] < 64) {
                uncertainty = 64;
            }
            score += uncertainty;
            
            if (score > best_score) {
                best_score = score;
                best_question = i;
            }
        }
    }
    
    return best_question;
}

unsigned char get_yes_no(void) {
    printf("(Y/N)? ");
    if (fgets(input, MAX_INPUT, stdin)) {
        return (toupper(input[0]) == 'Y');
    }
    return 0;
}

void learn_new_animal(void) {
    unsigned char i;
    
    if (num_animals >= MAX_ANIMALS) {
        printf("My brain is full!\n");
        return;
    }
    
    printf("You got me! What animal? ");
    if (fgets(input, MAX_INPUT, stdin)) {
        input[strcspn(input, "\n")] = 0;
        
        for (i = 0; input[i]; i++) {
            input[i] = toupper(input[i]);
        }
        
        strcpy(animals[num_animals].name, input);
        for (i = 0; i < NUM_FEATURES; i++) {
            animals[num_animals].features[i] = current_features[i];
        }
        animals[num_animals].learned = 1;
        animals[num_animals].confidence = 128;
        
        printf("Thanks! I learned about %s!\n", input);
        num_animals++;
        
        for (i = 0; i < num_asked; i++) {
            unsigned char q_id = questions_asked[i];
            if (questions[q_id].success_rate < 245) {
                questions[q_id].success_rate += 10;
            }
        }
    }
}

void update_neural_weights(unsigned char correct_animal) {
    unsigned char i, j;
    signed int correlation;
    
    for (i = 0; i < NUM_FEATURES; i++) {
        for (j = 0; j < NUM_FEATURES; j++) {
            correlation = (signed int)animals[correct_animal].features[i] * 
                         (signed int)animals[correct_animal].features[j];
            correlation /= 256;
            
            feature_weights[i][j] += (LEARNING_RATE * correlation) / 256;
            
            /* Prevent overflow with safer bounds */
            if (feature_weights[i][j] > 16000) feature_weights[i][j] = 16000;
            if (feature_weights[i][j] < -16000) feature_weights[i][j] = -16000;
        }
    }
    
    if (animals[correct_animal].confidence < 245) {
        animals[correct_animal].confidence += 10;
    }
}

void play_game(void) {
    unsigned char i;
    unsigned char question_id;
    unsigned char answer;
    unsigned char guessed_animal;
    signed int best_score;
    signed int second_best;
    unsigned char second_animal;
    
    games_played++;
    
    printf("\n=== GAME %d ===\n", games_played);
    printf("Think of an animal!\n\n");
    
    for (i = 0; i < NUM_FEATURES; i++) {
        current_features[i] = 128;
    }
    num_asked = 0;
    
    while (num_asked < 8) {
        question_id = find_best_question();
        
        if (question_id == 255) break;
        
        printf("%s ", questions[question_id].text);
        answer = get_yes_no();
        
        questions_asked[num_asked] = question_id;
        num_asked++;
        
        questions[question_id].asked_count++;
        update_feature_from_answer(question_id, answer);
        
        calculate_animal_scores();
        
        best_score = -1000;
        guessed_animal = 0;
        for (i = 0; i < num_animals; i++) {
            if (animal_scores[i] > best_score) {
                best_score = animal_scores[i];
                guessed_animal = i;
            }
        }
        
        if (best_score > 700 && num_asked >= 3) {
            break;
        }
    }
    
    calculate_animal_scores();
    best_score = -1000;
    guessed_animal = 0;
    
    for (i = 0; i < num_animals; i++) {
        if (animal_scores[i] > best_score) {
            best_score = animal_scores[i];
            guessed_animal = i;
        }
    }
    
    printf("\nIs it a %s? ", animals[guessed_animal].name);
    answer = get_yes_no();
    
    if (answer) {
        printf("Great! Got it in %d questions!\n", num_asked + 1);
        games_won++;
        
        update_neural_weights(guessed_animal);
        
        for (i = 0; i < num_asked; i++) {
            unsigned char q_id = questions_asked[i];
            if (questions[q_id].success_rate < 250) {
                questions[q_id].success_rate += 5;
            }
        }
    } else {
        printf("Let me try again...\n");
        
        second_best = -1000;
        second_animal = 0;
        for (i = 0; i < num_animals; i++) {
            if (i != guessed_animal && animal_scores[i] > second_best) {
                second_best = animal_scores[i];
                second_animal = i;
            }
        }
        
        if (second_best > 400) {
            printf("Is it a %s? ", animals[second_animal].name);
            answer = get_yes_no();
            
            if (answer) {
                printf("Got it on try 2!\n");
                games_won++;
                update_neural_weights(second_animal);
            } else {
                learn_new_animal();
            }
        } else {
            learn_new_animal();
        }
    }
    
    printf("\nScore: %d wins out of %d games", games_won, games_played);
    if (games_played > 0) {
        printf(" (%d%%)", (games_won * 100) / games_played);
    }
    printf("\n");
}

void show_knowledge(void) {
    unsigned char i;
    
    printf("\n=== KNOWLEDGE BASE ===\n");
    printf("I know %d animals:\n\n", num_animals);
    
    for (i = 0; i < num_animals; i++) {
        printf("%d. %s (Confidence: %d%s)\n", 
               i+1, animals[i].name, 
               (animals[i].confidence * 100) / 255,
               animals[i].learned ? " - Learned!" : "");
    }
    
    printf("\n=== QUESTION STATS ===\n");
    for (i = 0; i < num_questions && i < 6; i++) {
        printf("%s\n", questions[i].text);
        printf("  Asked %d times, Success: %d%%\n\n",
               questions[i].asked_count,
               (questions[i].success_rate * 100) / 255);
    }
}

void show_neural_state(void) {
    unsigned char i;
    const char* feature_names[NUM_FEATURES] = {
        "Size", "Habitat", "Diet", "Activity", "Domestic", "Intelligence"
    };
    
    printf("\n=== NEURAL NETWORK ===\n");
    printf("Current learning state:\n");
    
    for (i = 0; i < NUM_FEATURES; i++) {
        if (feature_weights[i][i] != 256) {
            printf("%s weight: %d\n", 
                   feature_names[i], feature_weights[i][i]);
        }
    }
    
    printf("\nLast game features:\n");
    for (i = 0; i < NUM_FEATURES; i++) {
        printf("%s: %d\n", feature_names[i], current_features[i]);
    }
}

void dump_knowledge_text(void) {
    unsigned char i, j;
    const char* feature_names[NUM_FEATURES] = {
        "Size", "Habitat", "Diet", "Activity", "Domestic", "Intelligence"
    };
    
    printf("\n=== KNOWLEDGE DUMP ===\n");
    printf("Neural Animal Guesser Knowledge Base\n");
    printf("Games: %d won / %d played\n\n", games_won, games_played);
    
    printf("ANIMALS:\n");
    for (i = 0; i < num_animals; i++) {
        printf("%s: ", animals[i].name);
        for (j = 0; j < NUM_FEATURES; j++) {
            printf("%d ", animals[i].features[j]);
        }
        printf("(conf:%d learned:%d)\n", 
               animals[i].confidence, animals[i].learned);
    }
    
    printf("\nQUESTIONS:\n");
    for (i = 0; i < num_questions; i++) {
        printf("\"%s\" feat:%d thresh:%d asked:%d success:%d\n",
               questions[i].text, questions[i].feature_index,
               questions[i].threshold, questions[i].asked_count,
               questions[i].success_rate);
    }
    
    printf("\nNEURAL WEIGHTS:\n");
    for (i = 0; i < NUM_FEATURES; i++) {
        printf("%s: ", feature_names[i]);
        for (j = 0; j < NUM_FEATURES; j++) {
            printf("%d ", feature_weights[i][j]);
        }
        printf("\n");
    }
    printf("\n=== END DUMP ===\n");
}

void process_input(void) {
    char command;
    
    if (strlen(input) == 0) return;
    command = toupper(input[0]);
    
    switch (command) {
        case 'P':
            play_game();
            break;
            
        case 'K':
            show_knowledge();
            break;
            
        case 'N':
            show_neural_state();
            break;
            
        case 'D':
            dump_knowledge_text();
            break;
            
        case 'H':
            printf("\n=== NEURAL ANIMAL GUESSER ===\n");
            printf("P - Play guessing game\n");
            printf("K - Show knowledge base\n");
            printf("N - Show neural network state\n");
            printf("D - Dump knowledge as text\n");
            printf("H - This help\n");
            printf("Q - Quit\n\n");
            printf("I learn from every game!\n");
            break;
            
        case 'Q':
            printf("\nThanks for teaching me!\n");
            printf("Final: %d/%d games won\n", games_won, games_played);
            printf("I learned %d animals!\n", num_animals);
            break;
            
        default:
            printf("Type H for help.\n");
            break;
    }
}

int main(void) {
    printf("*** NEURAL ANIMAL GUESSER ***\n");
    printf("Apple 1 Learning AI Version\n");
    printf("I get smarter every game!\n");
    printf("Type H for help, P to play!\n\n");
    
    init_animals();
    init_questions();
    init_neural_network();
    
    while (1) {
        printf("> ");
        
        if (fgets(input, MAX_INPUT, stdin)) {
            input[strcspn(input, "\n")] = 0;
            
            if (toupper(input[0]) == 'Q') {
                process_input();
                break;
            }
            
            process_input();
        }
    }
    
    return 0;
}
