#include <stdbool.h>
#include <stddef.h>

typedef struct {
  char *key;
  size_t value;
  bool is_occupied;
} HashEntry;

typedef struct {
  HashEntry *hash;
  size_t capacity;
  size_t position;
} SymbolTable;

SymbolTable *symbol_table_new(size_t capacity);

void symbol_table_free(SymbolTable *st);

void symbol_table_insert(SymbolTable *st, const char *key);

int symbol_table_get(SymbolTable *st, const char *key);

int symbol_table_get_bucket(SymbolTable *st, const char *key);

HashEntry **symbol_table_get_sorted(SymbolTable *st, size_t *out_size);
