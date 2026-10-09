#include "symbol_table.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static size_t first_hash(SymbolTable *st, const char *key);
static size_t second_hash(SymbolTable *st, const char *key);
static size_t hash_function(SymbolTable *st, const char *key, size_t pos);
static void put(SymbolTable *st, const char *key, size_t value);
static void rehash(SymbolTable *st);

SymbolTable *symbol_table_new(size_t capacity) {
  if (capacity == 0) {
    capacity = 16;
  }

  SymbolTable *st = (SymbolTable *)malloc(sizeof(SymbolTable));
  if (!st)
    return NULL;

  st->hash = (HashEntry *)calloc(capacity, sizeof(HashEntry));
  if (!st->hash) {
    free(st);
    return NULL;
  }

  st->capacity = capacity;
  st->position = 0;

  for (size_t i = 0; i < capacity; i++) {
    st->hash[i].key = NULL;
    st->hash[i].value = 0;
    st->hash[i].is_occupied = false;
  }

  return st;
}

void symbol_table_free(SymbolTable *st) {
  if (!st)
    return;

  if (st->hash) {
    for (size_t i = 0; i < st->capacity; i++) {
      if (st->hash[i].key) {
        free(st->hash[i].key);
      }
    }
    free(st->hash);
  }

  free(st);
}

static size_t first_hash(SymbolTable *st, const char *key) {
  size_t hash = 0;
  size_t len = strlen(key);

  if (len == 0)
    return 0;

  hash = (size_t)key[0];
  for (size_t i = 1; i < len; i++) {
    hash = ((hash * 41) + (size_t)key[i]) % st->capacity;
  }

  return hash;
}

static size_t second_hash(SymbolTable *st, const char *key) {
  size_t hash = 0;
  size_t len = strlen(key);

  if (len == 0)
    return 1;

  hash = (size_t)key[0];
  for (size_t i = 1; i < len; i++) {
    hash = ((hash * 61) + (size_t)key[i]) % st->capacity;
  }

  return (2 * hash + 1) % st->capacity;
}

static size_t hash_function(SymbolTable *st, const char *key, size_t pos) {
  size_t h1 = first_hash(st, key);
  size_t h2 = second_hash(st, key);
  return (h1 + pos * h2) % st->capacity;
}

static void put(SymbolTable *st, const char *key, size_t value) {
  if (st->capacity == st->position) {
    rehash(st);
  }

  size_t pos = 0;
  while (1) {
    size_t hash_pos = hash_function(st, key, pos);

    if (!st->hash[hash_pos].is_occupied) {
      st->hash[hash_pos].key = strdup(key);
      st->hash[hash_pos].value = value;
      st->hash[hash_pos].is_occupied = true;
      st->position++;
      break;
    }

    if (st->hash[hash_pos].is_occupied &&
        strcmp(st->hash[hash_pos].key, key) == 0) {
      break;
    }

    pos++;
  }
}

static void rehash(SymbolTable *st) {
  HashEntry *old_hash = st->hash;
  size_t old_capacity = st->capacity;

  st->capacity = old_capacity * 2;
  st->hash = (HashEntry *)calloc(st->capacity, sizeof(HashEntry));
  st->position = 0;

  for (size_t i = 0; i < st->capacity; i++) {
    st->hash[i].key = NULL;
    st->hash[i].value = 0;
    st->hash[i].is_occupied = false;
  }

  for (size_t i = 0; i < old_capacity; i++) {
    if (old_hash[i].is_occupied) {
      put(st, old_hash[i].key, old_hash[i].value);
      free(old_hash[i].key);
    }
  }

  free(old_hash);
}

void symbol_table_insert(SymbolTable *st, const char *key) {
  put(st, key, st->position);
}

int symbol_table_get(SymbolTable *st, const char *key) {
  size_t pos = 0;

  while (1) {
    size_t hash_pos = hash_function(st, key, pos);

    if (!st->hash[hash_pos].is_occupied) {
      return -1;
    }

    if (strcmp(st->hash[hash_pos].key, key) == 0) {
      return (int)st->hash[hash_pos].value;
    }

    pos++;

    if (pos >= st->capacity) {
      return -1;
    }
  }
}

static int compare_entries(const void *a, const void *b) {
  HashEntry *entry_a = *(HashEntry **)a;
  HashEntry *entry_b = *(HashEntry **)b;

  if (!entry_a->is_occupied && !entry_b->is_occupied)
    return 0;
  if (!entry_a->is_occupied)
    return 1;
  if (!entry_b->is_occupied)
    return -1;

  if (entry_a->value < entry_b->value)
    return -1;
  if (entry_a->value > entry_b->value)
    return 1;
  return 0;
}

HashEntry **symbol_table_get_sorted(SymbolTable *st, size_t *out_size) {
  HashEntry **sorted = (HashEntry **)malloc(st->capacity * sizeof(HashEntry *));
  if (!sorted)
    return NULL;

  for (size_t i = 0; i < st->capacity; i++) {
    sorted[i] = &st->hash[i];
  }

  qsort(sorted, st->capacity, sizeof(HashEntry *), compare_entries);

  if (out_size) {
    *out_size = st->position;
  }

  return sorted;
}

int symbol_table_get_bucket(SymbolTable *st, const char *key) {
  size_t pos = 0;

  while (1) {
    size_t hash_pos = hash_function(st, key, pos);

    if (!st->hash[hash_pos].is_occupied) {
      return -1;
    }

    if (strcmp(st->hash[hash_pos].key, key) == 0) {
      return (int)hash_pos;
    }

    pos++;

    if (pos >= st->capacity) {
      return -1;
    }
  }
}
