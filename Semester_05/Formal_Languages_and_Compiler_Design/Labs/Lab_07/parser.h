#ifndef PARSER_H
#define PARSER_H

#include "grammar.h"
#include "ll1.h"
#include "tree.h"

#define MAX_TOKENS 4096
#define MAX_STACK 4096

typedef struct {
  char token[64];
  int symbol_idx;
} Token;

typedef struct {
  int symbol;
  int tree_node;
} StackEntry;

typedef struct {
  Grammar *grammar;
  LL1Table *table;
  Token tokens[MAX_TOKENS];
  int token_count;
  int productions_used[MAX_PRODUCTIONS];
  int production_count;
  ParseTree tree;
} Parser;

void parser_init(Parser *p, Grammar *g, LL1Table *t);
bool parser_load_pif(Parser *p, const char *filename);
bool parser_parse(Parser *p);
void parser_print_productions(Parser *p, FILE *out);

#endif

