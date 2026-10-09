#include "ll1.h"
#include <stdio.h>
#include <string.h>

void ll1_init(LL1Table *t, Grammar *g) {
  t->grammar = g;
  
  // Use memset for bulk initialization - way faster than nested loops
  memset(t->first, 0, sizeof(t->first));
  memset(t->follow, 0, sizeof(t->follow));
  memset(t->parse_table, 0xFF, sizeof(t->parse_table));  // -1
}

void ll1_compute_first(LL1Table *t) {
  Grammar *g = t->grammar;

  // Terminals: FIRST(a) = {a}
  for (int i = 0; i < g->symbol_count; i++) {
    if (grammar_is_terminal(g, i)) {
      bitset_add(&t->first[i], i);
    }
  }

  // Fixed-point iteration
  bool changed = true;
  while (changed) {
    changed = false;

    for (int p = 0; p < g->production_count; p++) {
      int lhs = g->productions[p].lhs;
      int *rhs = g->productions[p].rhs;
      int rhs_len = g->productions[p].rhs_len;

      // A -> epsilon
      if (rhs_len == 1 && rhs[0] == g->epsilon_idx) {
        if (bitset_add(&t->first[lhs], g->epsilon_idx))
          changed = true;
        continue;
      }

      // A -> X1 X2 ... Xn
      bool all_have_epsilon = true;
      for (int i = 0; i < rhs_len; i++) {
        int sym = rhs[i];
        if (bitset_union_except(&t->first[lhs], &t->first[sym], g->epsilon_idx)) {
          changed = true;
        }
        if (!bitset_contains(&t->first[sym], g->epsilon_idx)) {
          all_have_epsilon = false;
          break;
        }
      }

      if (all_have_epsilon) {
        if (bitset_add(&t->first[lhs], g->epsilon_idx))
          changed = true;
      }
    }
  }
}

void ll1_compute_follow(LL1Table *t) {
  Grammar *g = t->grammar;

  // FOLLOW(S) contains $
  bitset_add(&t->follow[g->start_symbol], g->eof_idx);

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

        BitSet first_beta;
        bitset_init(&first_beta);

        bool beta_derives_epsilon = true;
        for (int j = i + 1; j < rhs_len; j++) {
          bitset_union_except(&first_beta, &t->first[rhs[j]], g->epsilon_idx);
          if (!bitset_contains(&t->first[rhs[j]], g->epsilon_idx)) {
            beta_derives_epsilon = false;
            break;
          }
        }

        if (i == rhs_len - 1) {
          beta_derives_epsilon = true;
        }

        if (bitset_union(&t->follow[B], &first_beta))
          changed = true;

        if (beta_derives_epsilon) {
          if (bitset_union(&t->follow[B], &t->follow[lhs]))
            changed = true;
        }
      }
    }
  }
}

void ll1_first_of_sequence(LL1Table *t, int *seq, int len, BitSet *result) {
  Grammar *g = t->grammar;
  bitset_init(result);

  if (len == 0) {
    bitset_add(result, g->epsilon_idx);
    return;
  }

  bool all_have_epsilon = true;
  for (int i = 0; i < len; i++) {
    bitset_union_except(result, &t->first[seq[i]], g->epsilon_idx);
    if (!bitset_contains(&t->first[seq[i]], g->epsilon_idx)) {
      all_have_epsilon = false;
      break;
    }
  }

  if (all_have_epsilon) {
    bitset_add(result, g->epsilon_idx);
  }
}

bool ll1_build_table(LL1Table *t) {
  Grammar *g = t->grammar;
  bool is_ll1 = true;

  for (int p = 0; p < g->production_count; p++) {
    int A = g->productions[p].lhs;
    int *rhs = g->productions[p].rhs;
    int rhs_len = g->productions[p].rhs_len;

    BitSet first_alpha;
    ll1_first_of_sequence(t, rhs, rhs_len, &first_alpha);

    // For each terminal a in FIRST(alpha)
    BitSetIter iter;
    bitset_iter_init(&iter, &first_alpha);
    int a;
    while ((a = bitset_iter_next(&iter)) >= 0) {
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

    // If epsilon in FIRST(alpha), add for each b in FOLLOW(A)
    if (bitset_contains(&first_alpha, g->epsilon_idx)) {
      bitset_iter_init(&iter, &t->follow[A]);
      int b;
      while ((b = bitset_iter_next(&iter)) >= 0) {
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
    bool first = true;
    BitSetIter iter;
    bitset_iter_init(&iter, &t->first[i]);
    int elem;
    while ((elem = bitset_iter_next(&iter)) >= 0) {
      if (!first) printf(", ");
      printf("%s", grammar_symbol_name(g, elem));
      first = false;
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
    bool first = true;
    BitSetIter iter;
    bitset_iter_init(&iter, &t->follow[i]);
    int elem;
    while ((elem = bitset_iter_next(&iter)) >= 0) {
      if (!first) printf(", ");
      printf("%s", grammar_symbol_name(g, elem));
      first = false;
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
        Production *prod = &g->productions[p];
        printf("  M[%s, %s] = %s ->", grammar_symbol_name(g, i),
               grammar_symbol_name(g, j),
               grammar_symbol_name(g, prod->lhs));
        for (int k = 0; k < prod->rhs_len; k++) {
          printf(" %s", grammar_symbol_name(g, prod->rhs[k]));
        }
        printf("\n");
      }
    }
  }
}
