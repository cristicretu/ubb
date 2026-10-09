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

static char *trim_str(char *str) {
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

bool parser_load_pif(Parser *p, const char *filename) {
  FILE *fp = fopen(filename, "r");
  if (!fp) {
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

    if (sscanf(trimmed, "%d | %s | %d | %s", &idx, token_name, &bucket,
               symbol_val) >= 2) {
      char *clean_token = trim_str(token_name);

      int sym_idx = grammar_find_symbol(p->grammar, clean_token);
      if (sym_idx < 0) {
        sym_idx = grammar_add_symbol(p->grammar, clean_token, true);
      }

      strncpy(p->tokens[p->token_count].token, clean_token, 63);
      p->tokens[p->token_count].token[63] = '\0';
      p->tokens[p->token_count].symbol_idx = sym_idx;
      p->token_count++;
    }
  }

  int eof_sym = grammar_find_symbol(p->grammar, "$");
  if (eof_sym < 0) {
    eof_sym = grammar_add_symbol(p->grammar, "$", true);
  }
  strncpy(p->tokens[p->token_count].token, "$", 63);
  p->tokens[p->token_count].symbol_idx = eof_sym;
  p->token_count++;

  fclose(fp);
  return p->token_count > 1;
}

bool parser_parse(Parser *p) {
  Grammar *g = p->grammar;
  LL1Table *t = p->table;

  StackEntry stack[MAX_STACK];
  int stack_top = 0;

  int root = tree_add_node(&p->tree, g->start_symbol, 0);

  stack[stack_top].symbol = g->eof_idx;
  stack[stack_top].tree_node = -1;
  stack_top++;

  stack[stack_top].symbol = g->start_symbol;
  stack[stack_top].tree_node = root;
  stack_top++;

  int input_pos = 0;

  while (stack_top > 0) {
    stack_top--;
    int top_sym = stack[stack_top].symbol;
    int top_node = stack[stack_top].tree_node;

    int input_sym = p->tokens[input_pos].symbol_idx;

    if (top_sym == g->epsilon_idx) {
      continue;
    }

    if (grammar_is_terminal(g, top_sym)) {
      if (top_sym == input_sym) {
        input_pos++;
      } else {
        fprintf(stderr, "Parse error: expected '%s', got '%s'\n",
                grammar_symbol_name(g, top_sym),
                grammar_symbol_name(g, input_sym));
        return false;
      }
    } else {
      int prod_idx = t->parse_table[top_sym][input_sym];
      if (prod_idx < 0) {
        fprintf(stderr, "Parse error: no production for [%s, %s]\n",
                grammar_symbol_name(g, top_sym),
                grammar_symbol_name(g, input_sym));
        return false;
      }

      p->productions_used[p->production_count++] = prod_idx;

      Production *prod = &g->productions[prod_idx];

      int child_nodes[MAX_RHS_LENGTH];
      int child_count = 0;

      for (int i = 0; i < prod->rhs_len; i++) {
        int child = tree_add_node(&p->tree, prod->rhs[i], top_node + 1);
        child_nodes[child_count++] = child;
      }

      for (int i = 0; i < child_count - 1; i++) {
        tree_set_sibling(&p->tree, child_nodes[i], child_nodes[i + 1] + 1);
      }

      for (int i = prod->rhs_len - 1; i >= 0; i--) {
        stack[stack_top].symbol = prod->rhs[i];
        stack[stack_top].tree_node = child_nodes[i];
        stack_top++;
      }
    }
  }

  if (input_pos != p->token_count) {
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

