#include "parser.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void parser_init(Parser *p, Grammar *g, LL1Table *t) {
  p->grammar = g;
  p->table = t;
  p->token_count = 0;
  p->production_count = 0;
  tree_init(&p->tree, g);
}

static inline char *trim_str(char *str) {
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

bool parser_load_pif(Parser *p, const char *filename) {
  FILE *fp = fopen(filename, "r");
  if (unlikely(!fp)) {
    fprintf(stderr, "Error: cannot open PIF file '%s'\n", filename);
    return false;
  }

  char line[256];
  bool in_pif = false;

  while (fgets(line, sizeof(line), fp)) {
    char *trimmed = trim_str(line);

    if (strstr(trimmed, "PIF (") != NULL) {
      in_pif = true;
      continue;
    }
    if (strcmp(trimmed, "End PIF") == 0) {
      break;
    }
    if (!in_pif)
      continue;

    int idx;
    char token_name[64];
    int bucket;
    char symbol_val[64];

    if (sscanf(trimmed, "%d | %63s | %d | %63s", &idx, token_name, &bucket,
               symbol_val) >= 2) {
      char *clean_token = trim_str(token_name);

      int sym_idx = grammar_find_symbol(p->grammar, clean_token);
      if (sym_idx < 0) {
        sym_idx = grammar_add_symbol(p->grammar, clean_token, true);
      }

      Token *tok = &p->tokens[p->token_count];
      strncpy(tok->token, clean_token, 63);
      tok->token[63] = '\0';
      tok->symbol_idx = sym_idx;
      p->token_count++;
    }
  }

  // Append EOF token
  int eof_sym = grammar_find_symbol(p->grammar, "$");
  if (eof_sym < 0) {
    eof_sym = grammar_add_symbol(p->grammar, "$", true);
  }
  Token *eof_tok = &p->tokens[p->token_count];
  strncpy(eof_tok->token, "$", 63);
  eof_tok->symbol_idx = eof_sym;
  p->token_count++;

  fclose(fp);
  return p->token_count > 1;
}

bool parser_parse(Parser *p) {
  Grammar *g = p->grammar;
  LL1Table *t = p->table;

  // Stack allocated on... stack. Cache friendly.
  StackEntry stack[MAX_STACK];
  int stack_top = 0;

  int root = tree_add_node(&p->tree, g->start_symbol, 0);

  // Push EOF and start symbol
  stack[stack_top++] = (StackEntry){.symbol = g->eof_idx, .tree_node = -1};
  stack[stack_top++] = (StackEntry){.symbol = g->start_symbol, .tree_node = root};

  int input_pos = 0;
  const int token_count = p->token_count;  // Cache for hot loop

  while (likely(stack_top > 0)) {
    StackEntry top = stack[--stack_top];
    int top_sym = top.symbol;
    int top_node = top.tree_node;

    int input_sym = p->tokens[input_pos].symbol_idx;

    // Skip epsilon
    if (top_sym == g->epsilon_idx) {
      continue;
    }

    if (grammar_is_terminal(g, top_sym)) {
      if (likely(top_sym == input_sym)) {
        input_pos++;
      } else {
        fprintf(stderr, "Parse error: expected '%s', got '%s'\n",
                grammar_symbol_name(g, top_sym),
                grammar_symbol_name(g, input_sym));
        return false;
      }
    } else {
      int prod_idx = t->parse_table[top_sym][input_sym];
      if (unlikely(prod_idx < 0)) {
        fprintf(stderr, "Parse error: no production for [%s, %s]\n",
                grammar_symbol_name(g, top_sym),
                grammar_symbol_name(g, input_sym));
        return false;
      }

      p->productions_used[p->production_count++] = prod_idx;

      Production *prod = &g->productions[prod_idx];
      int rhs_len = prod->rhs_len;

      // Build children
      int child_nodes[MAX_RHS_LENGTH];
      for (int i = 0; i < rhs_len; i++) {
        child_nodes[i] = tree_add_node(&p->tree, prod->rhs[i], top_node + 1);
      }

      // Set siblings
      for (int i = 0; i < rhs_len - 1; i++) {
        tree_set_sibling(&p->tree, child_nodes[i], child_nodes[i + 1] + 1);
      }

      // Push children in reverse order
      for (int i = rhs_len - 1; i >= 0; i--) {
        stack[stack_top++] = (StackEntry){.symbol = prod->rhs[i], .tree_node = child_nodes[i]};
      }
    }
  }

  if (unlikely(input_pos != token_count)) {
    fprintf(stderr, "Parse error: input not fully consumed\n");
    return false;
  }

  return true;
}

void parser_print_productions(Parser *p, FILE *out) {
  Grammar *g = p->grammar;

  fprintf(out, "Productions used during parsing:\n");
  for (int i = 0; i < p->production_count; i++) {
    int prod_idx = p->productions_used[i];
    Production *prod = &g->productions[prod_idx];

    fprintf(out, "%d: %s ->", i + 1, grammar_symbol_name(g, prod->lhs));
    for (int j = 0; j < prod->rhs_len; j++) {
      fprintf(out, " %s", grammar_symbol_name(g, prod->rhs[j]));
    }
    fprintf(out, "\n");
  }
}
