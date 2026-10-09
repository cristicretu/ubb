#include "ll1.h"
#include <stdio.h>
#include <string.h>

void set_init(SymbolSet *s) { s->size = 0; }

bool set_add(SymbolSet *s, int elem) {
  if (set_contains(s, elem))
    return false;
  if (s->size >= MAX_SET_SIZE)
    return false;
  s->elements[s->size++] = elem;
  return true;
}

bool set_contains(SymbolSet *s, int elem) {
  for (int i = 0; i < s->size; i++) {
    if (s->elements[i] == elem)
      return true;
  }
  return false;
}

bool set_union(SymbolSet *dest, SymbolSet *src) {
  bool changed = false;
  for (int i = 0; i < src->size; i++) {
    if (set_add(dest, src->elements[i]))
      changed = true;
  }
  return changed;
}

bool set_union_except(SymbolSet *dest, SymbolSet *src, int except) {
  bool changed = false;
  for (int i = 0; i < src->size; i++) {
    if (src->elements[i] != except) {
      if (set_add(dest, src->elements[i]))
        changed = true;
    }
  }
  return changed;
}

void ll1_init(LL1Table *t, Grammar *g) {
  t->grammar = g;
  for (int i = 0; i < MAX_SYMBOLS; i++) {
    set_init(&t->first[i]);
    set_init(&t->follow[i]);
    for (int j = 0; j < MAX_SYMBOLS; j++) {
      t->parse_table[i][j] = -1;
    }
  }
}

void ll1_compute_first(LL1Table *t) {
  Grammar *g = t->grammar;

  for (int i = 0; i < g->symbol_count; i++) {
    if (grammar_is_terminal(g, i)) {
      set_add(&t->first[i], i);
    }
  }

  bool changed = true;
  while (changed) {
    changed = false;

    for (int p = 0; p < g->production_count; p++) {
      int lhs = g->productions[p].lhs;
      int *rhs = g->productions[p].rhs;
      int rhs_len = g->productions[p].rhs_len;

      if (rhs_len == 1 && rhs[0] == g->epsilon_idx) {
        if (set_add(&t->first[lhs], g->epsilon_idx))
          changed = true;
        continue;
      }

      bool all_have_epsilon = true;
      for (int i = 0; i < rhs_len; i++) {
        int sym = rhs[i];
        if (set_union_except(&t->first[lhs], &t->first[sym], g->epsilon_idx)) {
          changed = true;
        }
        if (!set_contains(&t->first[sym], g->epsilon_idx)) {
          all_have_epsilon = false;
          break;
        }
      }

      if (all_have_epsilon) {
        if (set_add(&t->first[lhs], g->epsilon_idx))
          changed = true;
      }
    }
  }
}

void ll1_compute_follow(LL1Table *t) {
  Grammar *g = t->grammar;

  set_add(&t->follow[g->start_symbol], g->eof_idx);

  bool changed = true;
  while (changed) {
    changed = false;

    for (int p = 0; p < g->production_count; p++) {
      int lhs = g->productions[p].lhs;
      int *rhs = g->productions[p].rhs;
      int rhs_len = g->productions[p].rhs_len;

      for (int i = 0; i < rhs_len; i++) {
        int B = rhs[i];
        if (!grammar_is_nonterminal(g, B))
          continue;

        SymbolSet first_beta;
        set_init(&first_beta);

        bool beta_derives_epsilon = true;
        for (int j = i + 1; j < rhs_len; j++) {
          set_union_except(&first_beta, &t->first[rhs[j]], g->epsilon_idx);
          if (!set_contains(&t->first[rhs[j]], g->epsilon_idx)) {
            beta_derives_epsilon = false;
            break;
          }
        }

        if (i == rhs_len - 1) {
          beta_derives_epsilon = true;
        }

        if (set_union(&t->follow[B], &first_beta))
          changed = true;

        if (beta_derives_epsilon) {
          if (set_union(&t->follow[B], &t->follow[lhs]))
            changed = true;
        }
      }
    }
  }
}

void ll1_first_of_sequence(LL1Table *t, int *seq, int len, SymbolSet *result) {
  Grammar *g = t->grammar;
  set_init(result);

  if (len == 0) {
    set_add(result, g->epsilon_idx);
    return;
  }

  bool all_have_epsilon = true;
  for (int i = 0; i < len; i++) {
    set_union_except(result, &t->first[seq[i]], g->epsilon_idx);
    if (!set_contains(&t->first[seq[i]], g->epsilon_idx)) {
      all_have_epsilon = false;
      break;
    }
  }

  if (all_have_epsilon) {
    set_add(result, g->epsilon_idx);
  }
}

bool ll1_build_table(LL1Table *t) {
  Grammar *g = t->grammar;
  bool is_ll1 = true;

  for (int p = 0; p < g->production_count; p++) {
    int A = g->productions[p].lhs;
    int *rhs = g->productions[p].rhs;
    int rhs_len = g->productions[p].rhs_len;

    SymbolSet first_alpha;
    ll1_first_of_sequence(t, rhs, rhs_len, &first_alpha);

    for (int i = 0; i < first_alpha.size; i++) {
      int a = first_alpha.elements[i];
      if (a == g->epsilon_idx)
        continue;

      if (t->parse_table[A][a] != -1 && t->parse_table[A][a] != p) {
        fprintf(stderr, "LL(1) conflict at [%s, %s]: productions %d and %d\n",
                grammar_symbol_name(g, A), grammar_symbol_name(g, a),
                t->parse_table[A][a], p);
        is_ll1 = false;
      }
      t->parse_table[A][a] = p;
    }

    if (set_contains(&first_alpha, g->epsilon_idx)) {
      for (int i = 0; i < t->follow[A].size; i++) {
        int b = t->follow[A].elements[i];
        if (t->parse_table[A][b] != -1 && t->parse_table[A][b] != p) {
          fprintf(stderr, "LL(1) conflict at [%s, %s]: productions %d and %d\n",
                  grammar_symbol_name(g, A), grammar_symbol_name(g, b),
                  t->parse_table[A][b], p);
          is_ll1 = false;
        }
        t->parse_table[A][b] = p;
      }
    }
  }

  return is_ll1;
}

void ll1_print_first(LL1Table *t) {
  Grammar *g = t->grammar;
  printf("FIRST sets:\n");
  for (int i = 0; i < g->symbol_count; i++) {
    if (!grammar_is_nonterminal(g, i))
      continue;
    printf("  FIRST(%s) = {", grammar_symbol_name(g, i));
    for (int j = 0; j < t->first[i].size; j++) {
      if (j > 0)
        printf(", ");
      printf("%s", grammar_symbol_name(g, t->first[i].elements[j]));
    }
    printf("}\n");
  }
}

void ll1_print_follow(LL1Table *t) {
  Grammar *g = t->grammar;
  printf("FOLLOW sets:\n");
  for (int i = 0; i < g->symbol_count; i++) {
    if (!grammar_is_nonterminal(g, i))
      continue;
    printf("  FOLLOW(%s) = {", grammar_symbol_name(g, i));
    for (int j = 0; j < t->follow[i].size; j++) {
      if (j > 0)
        printf(", ");
      printf("%s", grammar_symbol_name(g, t->follow[i].elements[j]));
    }
    printf("}\n");
  }
}

void ll1_print_table(LL1Table *t) {
  Grammar *g = t->grammar;
  printf("LL(1) Parse Table:\n");

  for (int i = 0; i < g->symbol_count; i++) {
    if (!grammar_is_nonterminal(g, i))
      continue;
    for (int j = 0; j < g->symbol_count; j++) {
      if (!grammar_is_terminal(g, j))
        continue;
      if (t->parse_table[i][j] >= 0) {
        int p = t->parse_table[i][j];
        printf("  M[%s, %s] = %s ->", grammar_symbol_name(g, i),
               grammar_symbol_name(g, j),
               grammar_symbol_name(g, g->productions[p].lhs));
        for (int k = 0; k < g->productions[p].rhs_len; k++) {
          printf(" %s", grammar_symbol_name(g, g->productions[p].rhs[k]));
        }
        printf("\n");
      }
    }
  }
}

