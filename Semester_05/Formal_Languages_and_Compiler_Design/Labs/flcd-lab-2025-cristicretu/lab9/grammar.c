#include "grammar.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FNV-1a hash - fast and good distribution
static inline uint32_t hash_string(const char *str) {
  uint32_t hash = 2166136261u;
  while (*str) {
    hash ^= (uint8_t)*str++;
    hash *= 16777619u;
  }
  return hash;
}

void grammar_init(Grammar *g) {
  g->symbol_count = 0;
  g->production_count = 0;
  g->start_symbol = -1;

  // Initialize hash table with memset - much faster than loop
  memset(g->hash_table, 0xFF, sizeof(g->hash_table)); // -1 for symbol_idx

  g->epsilon_idx = grammar_add_symbol(g, "epsilon", true);
  g->eof_idx = grammar_add_symbol(g, "$", true);
}

int grammar_add_symbol(Grammar *g, const char *name, bool is_terminal) {
  // Check if already exists via hash table - O(1)
  uint32_t hash = hash_string(name);
  uint32_t idx = hash & HASH_TABLE_MASK;
  uint32_t start = idx;

  // Linear probing
  do {
    if (g->hash_table[idx].symbol_idx == -1) {
      // Empty slot - symbol doesn't exist, add it
      break;
    }
    if (g->hash_table[idx].hash == hash) {
      int sym_idx = g->hash_table[idx].symbol_idx;
      if (strcmp(g->symbols[sym_idx].name, name) == 0) {
        return sym_idx; // Found existing
      }
    }
    idx = (idx + 1) & HASH_TABLE_MASK;
  } while (idx != start);

  if (unlikely(g->symbol_count >= MAX_SYMBOLS)) {
    fprintf(stderr, "Error: too many symbols\n");
    return -1;
  }

  // Add new symbol
  int sym_idx = g->symbol_count++;
  strncpy(g->symbols[sym_idx].name, name, MAX_SYMBOL_LEN - 1);
  g->symbols[sym_idx].name[MAX_SYMBOL_LEN - 1] = '\0';
  g->symbols[sym_idx].is_terminal = is_terminal;

  // Insert into hash table
  g->hash_table[idx].symbol_idx = sym_idx;
  g->hash_table[idx].hash = hash;

  return sym_idx;
}

int grammar_find_symbol(Grammar *g, const char *name) {
  uint32_t hash = hash_string(name);
  uint32_t idx = hash & HASH_TABLE_MASK;
  uint32_t start = idx;

  // Linear probing - O(1) average case
  do {
    if (g->hash_table[idx].symbol_idx == -1) {
      return -1; // Not found
    }
    if (g->hash_table[idx].hash == hash) {
      int sym_idx = g->hash_table[idx].symbol_idx;
      if (strcmp(g->symbols[sym_idx].name, name) == 0) {
        return sym_idx;
      }
    }
    idx = (idx + 1) & HASH_TABLE_MASK;
  } while (idx != start);

  return -1;
}

int grammar_add_production(Grammar *g, int lhs, int *rhs, int rhs_len) {
  if (unlikely(g->production_count >= MAX_PRODUCTIONS)) {
    fprintf(stderr, "Error: too many productions\n");
    return -1;
  }

  int idx = g->production_count++;
  g->productions[idx].lhs = lhs;
  g->productions[idx].rhs_len = rhs_len;

  // Use memcpy for bulk copy - faster than loop
  memcpy(g->productions[idx].rhs, rhs, rhs_len * sizeof(int));

  return idx;
}

static inline bool is_terminal_name(const char *name) {
  if (name[0] == 'e' && strcmp(name, "epsilon") == 0)
    return true;
  for (const char *p = name; *p; p++) {
    if (islower(*p) || *p == '_')
      return false;
  }
  return true;
}

static inline char *trim(char *str) {
  while (isspace((unsigned char)*str))
    str++;
  if (*str == '\0')
    return str;
  char *end = str + strlen(str) - 1;
  while (end > str && isspace((unsigned char)*end))
    end--;
  *(end + 1) = '\0';
  return str;
}

bool grammar_load_from_file(Grammar *g, const char *filename) {
  FILE *fp = fopen(filename, "r");
  if (unlikely(!fp)) {
    fprintf(stderr, "Error: cannot open grammar file '%s'\n", filename);
    return false;
  }

  char line[1024];
  bool first_production = true;

  while (fgets(line, sizeof(line), fp)) {
    char *trimmed = trim(line);
    if (trimmed[0] == '\0' || trimmed[0] == '#')
      continue;

    char *arrow = strstr(trimmed, "->");
    if (!arrow)
      continue;

    *arrow = '\0';
    char *lhs_str = trim(trimmed);
    char *rhs_str = trim(arrow + 2);

    int lhs = grammar_find_symbol(g, lhs_str);
    if (lhs < 0) {
      lhs = grammar_add_symbol(g, lhs_str, false);
    }

    if (first_production) {
      g->start_symbol = lhs;
      first_production = false;
    }

    char *alt = rhs_str;
    char *pipe;

    do {
      pipe = strchr(alt, '|');
      if (pipe)
        *pipe = '\0';

      char *alt_trimmed = trim(alt);
      int rhs[MAX_RHS_LENGTH];
      int rhs_len = 0;

      char *saveptr;
      char *token = strtok_r(alt_trimmed, " \t", &saveptr);
      while (token && rhs_len < MAX_RHS_LENGTH) {
        char *tok_trimmed = trim(token);
        if (tok_trimmed[0] == '\0') {
          token = strtok_r(NULL, " \t", &saveptr);
          continue;
        }

        int sym_idx = grammar_find_symbol(g, tok_trimmed);
        if (sym_idx < 0) {
          bool is_term = is_terminal_name(tok_trimmed);
          sym_idx = grammar_add_symbol(g, tok_trimmed, is_term);
        }
        rhs[rhs_len++] = sym_idx;
        token = strtok_r(NULL, " \t", &saveptr);
      }

      if (rhs_len > 0) {
        grammar_add_production(g, lhs, rhs, rhs_len);
      }

      if (pipe)
        alt = pipe + 1;
    } while (pipe);
  }

  fclose(fp);
  return g->production_count > 0;
}

void grammar_print(Grammar *g) {
  printf("Grammar:\n");
  printf("  Symbols (%d):\n", g->symbol_count);
  for (int i = 0; i < g->symbol_count; i++) {
    printf("    %d: %s (%s)\n", i, g->symbols[i].name,
           g->symbols[i].is_terminal ? "terminal" : "non-terminal");
  }

  printf("  Productions (%d):\n", g->production_count);
  for (int i = 0; i < g->production_count; i++) {
    Production *p = &g->productions[i];
    printf("    %d: %s ->", i, g->symbols[p->lhs].name);
    for (int j = 0; j < p->rhs_len; j++) {
      printf(" %s", g->symbols[p->rhs[j]].name);
    }
    printf("\n");
  }

  printf("  Start symbol: %s\n", g->symbols[g->start_symbol].name);
}
