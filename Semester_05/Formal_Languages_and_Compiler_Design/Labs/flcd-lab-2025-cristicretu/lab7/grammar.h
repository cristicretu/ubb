#ifndef GRAMMAR_H
#define GRAMMAR_H

#include <stdbool.h>
#include <stddef.h>

#define MAX_SYMBOLS 256
#define MAX_PRODUCTIONS 512
#define MAX_RHS_LENGTH 32
#define MAX_SYMBOL_LEN 64

typedef struct {
  char name[MAX_SYMBOL_LEN];
  bool is_terminal;
} Symbol;

typedef struct {
  int lhs;
  int rhs[MAX_RHS_LENGTH];
  int rhs_len;
} Production;

typedef struct {
  Symbol symbols[MAX_SYMBOLS];
  int symbol_count;
  Production productions[MAX_PRODUCTIONS];
  int production_count;
  int start_symbol;
  int epsilon_idx;
  int eof_idx;
} Grammar;

void grammar_init(Grammar *g);
int grammar_add_symbol(Grammar *g, const char *name, bool is_terminal);
int grammar_find_symbol(Grammar *g, const char *name);
int grammar_add_production(Grammar *g, int lhs, int *rhs, int rhs_len);
bool grammar_load_from_file(Grammar *g, const char *filename);
void grammar_print(Grammar *g);
const char *grammar_symbol_name(Grammar *g, int idx);
bool grammar_is_terminal(Grammar *g, int idx);
bool grammar_is_nonterminal(Grammar *g, int idx);

#endif

