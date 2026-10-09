#include <stdbool.h>
#include <stddef.h>

#define MAX_STATES 64
#define MAX_TRANSITIONS 256
#define MAX_SYMBOL_LEN 32

typedef enum {
  SYMBOL_LETTER = 'L',
  SYMBOL_DIGIT = 'D',
  SYMBOL_UNDERSCORE = 'U',
  SYMBOL_POINT = 'P',
  SYMBOL_QUOTE = 'Q',
  SYMBOL_CHAR = 'C'
} SymbolType;

typedef struct {
  char from_state[MAX_SYMBOL_LEN];
  char to_state[MAX_SYMBOL_LEN];
  SymbolType symbol;
} Transition;

typedef struct {
  char name[MAX_SYMBOL_LEN];
  char states[MAX_STATES][MAX_SYMBOL_LEN];
  SymbolType alphabet[MAX_STATES];
  Transition transitions[MAX_TRANSITIONS];
  char initial_state[MAX_SYMBOL_LEN];
  char final_states[MAX_STATES][MAX_SYMBOL_LEN];
  size_t state_count;
  size_t alphabet_size;
  size_t transition_count;
  size_t final_state_count;
} FiniteAutomaton;

void fa_init(FiniteAutomaton *fa, const char *name);

bool fa_load_from_file(FiniteAutomaton *fa, const char *filename);

bool fa_validate_string(const FiniteAutomaton *fa, const char *input);

void fa_print(const FiniteAutomaton *fa);

void fa_convert_to_grammar(const FiniteAutomaton *fa);
