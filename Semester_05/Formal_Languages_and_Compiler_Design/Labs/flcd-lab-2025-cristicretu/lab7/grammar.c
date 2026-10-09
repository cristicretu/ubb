#include "grammar.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void grammar_init(Grammar *g) {
  g->symbol_count = 0;
  g->production_count = 0;
  g->start_symbol = -1;
  g->epsilon_idx = grammar_add_symbol(g, "epsilon", true);
  g->eof_idx = grammar_add_symbol(g, "$", true);
}

int grammar_add_symbol(Grammar *g, const char *name, bool is_terminal) {
  int idx = grammar_find_symbol(g, name);
  if (idx >= 0)
    return idx;

  if (g->symbol_count >= MAX_SYMBOLS) {
    fprintf(stderr, "Error: too many symbols\n");
    return -1;
  }

  idx = g->symbol_count++;
  strncpy(g->symbols[idx].name, name, MAX_SYMBOL_LEN - 1);
  g->symbols[idx].name[MAX_SYMBOL_LEN - 1] = '\0';
  g->symbols[idx].is_terminal = is_terminal;
  return idx;
}

int grammar_find_symbol(Grammar *g, const char *name) {
  for (int i = 0; i < g->symbol_count; i++) {
    if (strcmp(g->symbols[i].name, name) == 0) {
      return i;
    }
  }
  return -1;
}

int grammar_add_production(Grammar *g, int lhs, int *rhs, int rhs_len) {
  if (g->production_count >= MAX_PRODUCTIONS) {
    fprintf(stderr, "Error: too many productions\n");
    return -1;
  }

  int idx = g->production_count++;
  g->productions[idx].lhs = lhs;
  g->productions[idx].rhs_len = rhs_len;
  for (int i = 0; i < rhs_len; i++) {
    g->productions[idx].rhs[i] = rhs[i];
  }
  return idx;
}

static bool is_terminal_name(const char *name) {
  if (strcmp(name, "epsilon") == 0)
    return true;
  for (int i = 0; name[i]; i++) {
    if (islower(name[i]) || name[i] == '_')
      return false;
  }
  return true;
}

static char *trim(char *str) {
  while (isspace(*str))
    str++;
  if (*str == '\0')
    return str;
  char *end = str + strlen(str) - 1;
  while (end > str && isspace(*end))
    end--;
  *(end + 1) = '\0';
  return str;
}

bool grammar_load_from_file(Grammar *g, const char *filename) {
  FILE *fp = fopen(filename, "r");
  if (!fp) {
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

      char *token = strtok(alt_trimmed, " \t");
      while (token && rhs_len < MAX_RHS_LENGTH) {
        char *tok_trimmed = trim(token);
        if (tok_trimmed[0] == '\0') {
          token = strtok(NULL, " \t");
          continue;
        }

        int sym_idx = grammar_find_symbol(g, tok_trimmed);
        if (sym_idx < 0) {
          bool is_term = is_terminal_name(tok_trimmed);
          sym_idx = grammar_add_symbol(g, tok_trimmed, is_term);
        }
        rhs[rhs_len++] = sym_idx;
        token = strtok(NULL, " \t");
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
    printf("    %d: %s ->", i, g->symbols[g->productions[i].lhs].name);
    for (int j = 0; j < g->productions[i].rhs_len; j++) {
      printf(" %s", g->symbols[g->productions[i].rhs[j]].name);
    }
    printf("\n");
  }

  printf("  Start symbol: %s\n", g->symbols[g->start_symbol].name);
}

const char *grammar_symbol_name(Grammar *g, int idx) {
  if (idx < 0 || idx >= g->symbol_count)
    return "<?>";
  return g->symbols[idx].name;
}

bool grammar_is_terminal(Grammar *g, int idx) {
  if (idx < 0 || idx >= g->symbol_count)
    return false;
  return g->symbols[idx].is_terminal;
}

bool grammar_is_nonterminal(Grammar *g, int idx) {
  if (idx < 0 || idx >= g->symbol_count)
    return false;
  return !g->symbols[idx].is_terminal;
}
