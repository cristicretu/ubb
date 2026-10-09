#ifndef TREE_H
#define TREE_H

#include "grammar.h"
#include <stdio.h>

#define MAX_TREE_NODES 4096

typedef struct {
  int index;
  int symbol;
  int parent;
  int right_sibling;
} TreeNode;

typedef struct {
  TreeNode nodes[MAX_TREE_NODES];
  int node_count;
  Grammar *grammar;
} ParseTree;

void tree_init(ParseTree *tree, Grammar *g);
int tree_add_node(ParseTree *tree, int symbol, int parent);
void tree_set_sibling(ParseTree *tree, int node, int sibling);
void tree_print_table(ParseTree *tree, FILE *out);

#endif

