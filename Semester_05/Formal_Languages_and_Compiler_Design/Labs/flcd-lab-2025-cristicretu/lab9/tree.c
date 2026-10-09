#include "tree.h"
#include <stdio.h>
#include <string.h>

void tree_init(ParseTree *tree, Grammar *g) {
  tree->node_count = 0;
  tree->grammar = g;
}

int tree_add_node(ParseTree *tree, int symbol, int parent) {
  if (unlikely(tree->node_count >= MAX_TREE_NODES)) {
    fprintf(stderr, "Error: tree overflow\n");
    return -1;
  }

  int idx = tree->node_count++;
  TreeNode *n = &tree->nodes[idx];
  n->index = idx + 1;
  n->symbol = symbol;
  n->parent = parent;
  n->right_sibling = 0;
  return idx;
}

void tree_print_table(ParseTree *tree, FILE *out) {
  fprintf(out, "Parse Tree (Father-Sibling Representation)\n");
  fprintf(out, "%-6s | %-20s | %-6s | %-12s\n", "Index", "Symbol", "Parent",
          "RightSibling");
  fprintf(out, "------------------------------------------------------\n");

  for (int i = 0; i < tree->node_count; i++) {
    TreeNode *n = &tree->nodes[i];
    fprintf(out, "%-6d | %-20s | %-6d | %-12d\n", n->index,
            grammar_symbol_name(tree->grammar, n->symbol), n->parent,
            n->right_sibling);
  }
}
