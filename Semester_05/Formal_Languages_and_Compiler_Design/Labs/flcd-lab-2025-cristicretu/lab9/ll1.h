#ifndef LL1_H
#define LL1_H

#include "grammar.h"

// Bit-vector set for FIRST/FOLLOW - O(1) operations instead of O(n)
// Supports up to 256 symbols (4 * 64 bits)
#define BITVEC_WORDS 4

typedef struct {
  uint64_t bits[BITVEC_WORDS];
} BitSet;

typedef struct {
  Grammar *grammar;
  BitSet first[MAX_SYMBOLS];
  BitSet follow[MAX_SYMBOLS];
  int16_t parse_table[MAX_SYMBOLS][MAX_SYMBOLS];  // int16_t saves memory
} LL1Table;

// Bit-vector set operations - all O(1)
static inline void bitset_init(BitSet *s) {
  s->bits[0] = s->bits[1] = s->bits[2] = s->bits[3] = 0;
}

static inline bool bitset_add(BitSet *s, int elem) {
  int word = elem >> 6;
  uint64_t mask = 1ULL << (elem & 63);
  if (s->bits[word] & mask) return false;
  s->bits[word] |= mask;
  return true;
}

static inline bool bitset_contains(BitSet *s, int elem) {
  int word = elem >> 6;
  uint64_t mask = 1ULL << (elem & 63);
  return (s->bits[word] & mask) != 0;
}

static inline bool bitset_union(BitSet *dest, BitSet *src) {
  uint64_t old0 = dest->bits[0], old1 = dest->bits[1];
  uint64_t old2 = dest->bits[2], old3 = dest->bits[3];
  dest->bits[0] |= src->bits[0];
  dest->bits[1] |= src->bits[1];
  dest->bits[2] |= src->bits[2];
  dest->bits[3] |= src->bits[3];
  return (dest->bits[0] != old0 || dest->bits[1] != old1 ||
          dest->bits[2] != old2 || dest->bits[3] != old3);
}

static inline bool bitset_union_except(BitSet *dest, BitSet *src, int except) {
  BitSet tmp = *src;
  int word = except >> 6;
  tmp.bits[word] &= ~(1ULL << (except & 63));
  return bitset_union(dest, &tmp);
}

static inline bool bitset_is_empty(BitSet *s) {
  return (s->bits[0] | s->bits[1] | s->bits[2] | s->bits[3]) == 0;
}

// Iterator for bitset elements
typedef struct {
  BitSet *set;
  int current;
} BitSetIter;

static inline void bitset_iter_init(BitSetIter *iter, BitSet *s) {
  iter->set = s;
  iter->current = -1;
}

static inline int bitset_iter_next(BitSetIter *iter) {
  for (int i = iter->current + 1; i < MAX_SYMBOLS; i++) {
    if (bitset_contains(iter->set, i)) {
      iter->current = i;
      return i;
    }
  }
  return -1;
}

// LL1 table functions
void ll1_init(LL1Table *t, Grammar *g);
void ll1_compute_first(LL1Table *t);
void ll1_compute_follow(LL1Table *t);
bool ll1_build_table(LL1Table *t);
void ll1_print_first(LL1Table *t);
void ll1_print_follow(LL1Table *t);
void ll1_print_table(LL1Table *t);
void ll1_first_of_sequence(LL1Table *t, int *seq, int len, BitSet *result);

#endif
