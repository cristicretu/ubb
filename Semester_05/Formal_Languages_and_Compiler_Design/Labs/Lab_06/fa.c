#include "fa.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 512

static char *trim_whitespace(char *str) {
  char *end;
  while (isspace((unsigned char)*str))
    str++;
  if (*str == 0)
    return str;
  end = str + strlen(str) - 1;
  while (end > str && isspace((unsigned char)*end))
    end--;
  *(end + 1) = 0;
  return str;
}

static bool is_comment_or_empty(const char *line) {
  return line[0] == '#' || line[0] == '\0';
}

void fa_init(FiniteAutomaton *fa, const char *name) {
  memset(fa, 0, sizeof(FiniteAutomaton));
  strncpy(fa->name, name, MAX_SYMBOL_LEN - 1);
}

static bool fa_is_final_state(const FiniteAutomaton *fa, const char *state) {
  for (size_t i = 0; i < fa->final_state_count; i++) {
    if (strcmp(fa->final_states[i], state) == 0) {
      return true;
    }
  }
  return false;
}

static const Transition *fa_find_transition(const FiniteAutomaton *fa,
                                            const char *from_state,
                                            SymbolType symbol) {
  for (size_t i = 0; i < fa->transition_count; i++) {
    if (strcmp(fa->transitions[i].from_state, from_state) == 0) {
      if (fa->transitions[i].symbol == symbol) {
        return &fa->transitions[i];
      }
      if (fa->transitions[i].symbol == SYMBOL_CHAR && symbol != SYMBOL_QUOTE &&
          symbol != 0) {
        return &fa->transitions[i];
      }
    }
  }
  return NULL;
}

static SymbolType classify_character(char c) {
  if (c == '"')
    return SYMBOL_QUOTE;
  if (c == '\n')
    return 0;
  if (isalpha(c))
    return SYMBOL_LETTER;
  if (isdigit(c))
    return SYMBOL_DIGIT;
  if (c == '_')
    return SYMBOL_UNDERSCORE;
  if (c == '.')
    return SYMBOL_POINT;
  if (isprint(c))
    return SYMBOL_CHAR;
  return 0;
}

static bool parse_states(FiniteAutomaton *fa, char *line) {
  char *token = strtok(line, ",");
  while (token != NULL && fa->state_count < MAX_STATES) {
    token = trim_whitespace(token);
    if (strlen(token) > 0) {
      strncpy(fa->states[fa->state_count++], token, MAX_SYMBOL_LEN - 1);
    }
    token = strtok(NULL, ",");
  }
  return fa->state_count > 0;
}

static bool parse_alphabet(FiniteAutomaton *fa, char *line) {
  char *token = strtok(line, ",");
  while (token != NULL && fa->alphabet_size < MAX_STATES) {
    token = trim_whitespace(token);
    if (strlen(token) > 0) {
      fa->alphabet[fa->alphabet_size++] = (SymbolType)token[0];
    }
    token = strtok(NULL, ",");
  }
  return fa->alphabet_size > 0;
}

static bool parse_transition(FiniteAutomaton *fa, char *line) {
  char from[MAX_SYMBOL_LEN], symbol_str[MAX_SYMBOL_LEN], to[MAX_SYMBOL_LEN];

  if (sscanf(line, "%[^,], %[^,], %s", from, symbol_str, to) == 3) {
    if (fa->transition_count >= MAX_TRANSITIONS)
      return false;

    Transition *t = &fa->transitions[fa->transition_count];
    strncpy(t->from_state, trim_whitespace(from), MAX_SYMBOL_LEN - 1);
    strncpy(t->to_state, trim_whitespace(to), MAX_SYMBOL_LEN - 1);
    t->symbol = (SymbolType)trim_whitespace(symbol_str)[0];
    fa->transition_count++;
    return true;
  }
  return false;
}

static bool parse_final_states(FiniteAutomaton *fa, char *line) {
  char *token = strtok(line, ",");
  while (token != NULL && fa->final_state_count < MAX_STATES) {
    token = trim_whitespace(token);
    if (strlen(token) > 0) {
      strncpy(fa->final_states[fa->final_state_count++], token,
              MAX_SYMBOL_LEN - 1);
    }
    token = strtok(NULL, ",");
  }
  return fa->final_state_count > 0;
}

bool fa_load_from_file(FiniteAutomaton *fa, const char *filename) {
  FILE *file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "Error: Cannot open file '%s'\n", filename);
    return false;
  }

  char buffer[BUFFER_SIZE];
  bool in_transitions = false;

  while (fgets(buffer, BUFFER_SIZE, file)) {
    char *line = trim_whitespace(buffer);
    if (is_comment_or_empty(line))
      continue;

    if (strncmp(line, "states:", 7) == 0) {
      parse_states(fa, line + 7);
    } else if (strncmp(line, "alphabet:", 9) == 0) {
      parse_alphabet(fa, line + 9);
    } else if (strncmp(line, "transitions:", 12) == 0) {
      in_transitions = true;
    } else if (strncmp(line, "initial:", 8) == 0) {
      char *initial = trim_whitespace(line + 8);
      strncpy(fa->initial_state, initial, MAX_SYMBOL_LEN - 1);
      in_transitions = false;
    } else if (strncmp(line, "final:", 6) == 0) {
      parse_final_states(fa, line + 6);
      in_transitions = false;
    } else if (in_transitions) {
      parse_transition(fa, line);
    }
  }

  fclose(file);
  return fa->state_count > 0 && fa->transition_count > 0;
}

void fa_print(const FiniteAutomaton *fa) {
  printf("\n--- Finite Automaton: %s ---\n", fa->name);

  printf("States: { ");
  for (size_t i = 0; i < fa->state_count; i++) {
    printf("%s%s", fa->states[i], i < fa->state_count - 1 ? ", " : "");
  }
  printf(" }\n");

  printf("Alphabet: { ");
  for (size_t i = 0; i < fa->alphabet_size; i++) {
    printf("%c%s", fa->alphabet[i], i < fa->alphabet_size - 1 ? ", " : "");
  }
  printf(" }\n");

  printf("Initial state: %s\n", fa->initial_state);

  printf("Final states: { ");
  for (size_t i = 0; i < fa->final_state_count; i++) {
    printf("%s%s", fa->final_states[i],
           i < fa->final_state_count - 1 ? ", " : "");
  }
  printf(" }\n");

  printf("Transitions:\n");
  for (size_t i = 0; i < fa->transition_count; i++) {
    printf("  δ(%s, %c) -> %s\n", fa->transitions[i].from_state,
           fa->transitions[i].symbol, fa->transitions[i].to_state);
  }
}

void fa_convert_to_grammar(const FiniteAutomaton *fa) {
  printf("\n--- Regular Grammar for %s ---\n", fa->name);
  printf("Non-terminals: { ");
  for (size_t i = 0; i < fa->state_count; i++) {
    printf("%s%s", fa->states[i], i < fa->state_count - 1 ? ", " : "");
  }
  printf(" }\n");

  printf("Terminals: { ");
  for (size_t i = 0; i < fa->alphabet_size; i++) {
    printf("%c%s", fa->alphabet[i], i < fa->alphabet_size - 1 ? ", " : "");
  }
  printf(" }\n");

  printf("Start symbol: %s\n", fa->initial_state);
  printf("Productions:\n");

  for (size_t i = 0; i < fa->transition_count; i++) {
    const Transition *t = &fa->transitions[i];
    printf("  %s -> %c %s\n", t->from_state, t->symbol, t->to_state);

    if (fa_is_final_state(fa, t->to_state)) {
      printf("  %s -> %c\n", t->from_state, t->symbol);
    }
  }

  if (fa_is_final_state(fa, fa->initial_state)) {
    printf("  %s -> ε\n", fa->initial_state);
  }
}

bool fa_validate_string(const FiniteAutomaton *fa, const char *input) {
  if (!input || strlen(input) == 0)
    return false;

  char current_state[MAX_SYMBOL_LEN];
  strncpy(current_state, fa->initial_state, MAX_SYMBOL_LEN - 1);

  for (const char *p = input; *p; p++) {
    SymbolType symbol = classify_character(*p);
    if (!symbol) {
      return false;
    }

    const Transition *transition =
        fa_find_transition(fa, current_state, symbol);
    if (!transition) {
      return false;
    }

    strncpy(current_state, transition->to_state, MAX_SYMBOL_LEN - 1);
  }

  return fa_is_final_state(fa, current_state);
}
