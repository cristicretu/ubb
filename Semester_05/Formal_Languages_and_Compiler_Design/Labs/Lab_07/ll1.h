#ifndef LL1_H
#define LL1_H

#include "grammar.h"

#define MAX_SET_SIZE 128

typedef struct {
  int elements[MAX_SET_SIZE];
  int size;
} SymbolSet;

typedef struct {
  Grammar *grammar;
  SymbolSet first[MAX_SYMBOLS];
  SymbolSet follow[MAX_SYMBOLS];
  int parse_table[MAX_SYMBOLS][MAX_SYMBOLS];
} LL1Table;

void set_init(SymbolSet *s);
bool set_add(SymbolSet *s, int elem);
bool set_contains(SymbolSet *s, int elem);
bool set_union(SymbolSet *dest, SymbolSet *src);
bool set_union_except(SymbolSet *dest, SymbolSet *src, int except);

void ll1_init(LL1Table *t, Grammar *g);
void ll1_compute_first(LL1Table *t);
void ll1_compute_follow(LL1Table *t);
bool ll1_build_table(LL1Table *t);
void ll1_print_first(LL1Table *t);
void ll1_print_follow(LL1Table *t);
void ll1_print_table(LL1Table *t);

void ll1_first_of_sequence(LL1Table *t, int *seq, int len, SymbolSet *result);

#endif

