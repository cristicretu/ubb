#include "grammar.h"
#include "ll1.h"
#include "parser.h"
#include "tree.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <grammar.txt> <pif.txt> [output.txt]\n",
            argv[0]);
    return 1;
  }

  const char *grammar_file = argv[1];
  const char *pif_file = argv[2];
  const char *output_file = argc > 3 ? argv[3] : "output.txt";

  Grammar grammar;
  grammar_init(&grammar);

  if (!grammar_load_from_file(&grammar, grammar_file)) {
    fprintf(stderr, "Failed to load grammar from '%s'\n", grammar_file);
    return 1;
  }

  printf("Loaded grammar with %d symbols, %d productions\n",
         grammar.symbol_count, grammar.production_count);

  LL1Table table;
  ll1_init(&table, &grammar);

  ll1_compute_first(&table);
  ll1_compute_follow(&table);

  if (!ll1_build_table(&table)) {
    fprintf(stderr, "Warning: Grammar is not LL(1)\n");
  }

  Parser parser;
  parser_init(&parser, &grammar, &table);

  if (!parser_load_pif(&parser, pif_file)) {
    fprintf(stderr, "Failed to load PIF from '%s'\n", pif_file);
    return 1;
  }

  printf("Loaded %d tokens from PIF\n", parser.token_count);

  if (!parser_parse(&parser)) {
    fprintf(stderr, "Parsing failed\n");
    return 1;
  }

  printf("Parsing successful!\n");

  FILE *out = fopen(output_file, "w");
  if (!out) {
    fprintf(stderr, "Cannot open output file '%s'\n", output_file);
    return 1;
  }

  parser_print_productions(&parser, out);
  fprintf(out, "\n");
  tree_print_table(&parser.tree, out);

  fclose(out);

  printf("Output written to '%s'\n", output_file);

  return 0;
}

