#ifndef GRAMMAR_H
#define GRAMMAR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAX_SYMBOLS 256
#define MAX_PRODUCTIONS 512
#define MAX_RHS_LENGTH 32
#define MAX_SYMBOL_LEN 64

// Hash table size for O(1) symbol lookup (power of 2)
#define HASH_TABLE_SIZE 512
#define HASH_TABLE_MASK (HASH_TABLE_SIZE - 1)

// Branch prediction hints - geohot style
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

// Cache-aligned symbol structure
typedef struct __attribute__((aligned(64))) {
  char name[MAX_SYMBOL_LEN];
  bool is_terminal;
  int8_t _pad[7];  // explicit padding for alignment
} Symbol;

typedef struct {
  int lhs;
  int rhs_len;
  int rhs[MAX_RHS_LENGTH];
} Production;

// Hash table entry for O(1) symbol lookup
typedef struct {
  int symbol_idx;  // -1 if empty
  uint32_t hash;
} HashEntry;

typedef struct {
  Symbol symbols[MAX_SYMBOLS];
  int symbol_count;
  Production productions[MAX_PRODUCTIONS];
  int production_count;
  int start_symbol;
  int epsilon_idx;
  int eof_idx;
  
  // Hash table for O(1) symbol lookup
  HashEntry hash_table[HASH_TABLE_SIZE];
} Grammar;

// Core functions
void grammar_init(Grammar *g);
int grammar_add_symbol(Grammar *g, const char *name, bool is_terminal);
int grammar_find_symbol(Grammar *g, const char *name);
int grammar_add_production(Grammar *g, int lhs, int *rhs, int rhs_len);
bool grammar_load_from_file(Grammar *g, const char *filename);
void grammar_print(Grammar *g);

// Inline accessors for hot paths
static inline const char *grammar_symbol_name(Grammar *g, int idx) {
  if (unlikely(idx < 0 || idx >= g->symbol_count))
    return "<?>";
  return g->symbols[idx].name;
}

static inline bool grammar_is_terminal(Grammar *g, int idx) {
  if (unlikely(idx < 0 || idx >= g->symbol_count))
    return false;
  return g->symbols[idx].is_terminal;
}

static inline bool grammar_is_nonterminal(Grammar *g, int idx) {
  if (unlikely(idx < 0 || idx >= g->symbol_count))
    return false;
  return !g->symbols[idx].is_terminal;
}

#endif
