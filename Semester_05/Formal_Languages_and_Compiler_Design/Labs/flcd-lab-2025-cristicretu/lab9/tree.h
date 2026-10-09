#ifndef TREE_H
#define TREE_H

#include "grammar.h"
#include <stdio.h>

#define MAX_TREE_NODES 4096

// Packed tree node structure - better cache utilization
typedef struct __attribute__((packed)) {
  int32_t index;
  int32_t symbol;
  int32_t parent;
  int32_t right_sibling;
} TreeNode;

typedef struct {
  TreeNode nodes[MAX_TREE_NODES];
  int node_count;
  Grammar *grammar;
} ParseTree;

void tree_init(ParseTree *tree, Grammar *g);
int tree_add_node(ParseTree *tree, int symbol, int parent);

static inline void tree_set_sibling(ParseTree *tree, int node, int sibling) {
  if (likely(node >= 0 && node < tree->node_count)) {
    tree->nodes[node].right_sibling = sibling;
  }
}

void tree_print_table(ParseTree *tree, FILE *out);

#endif
