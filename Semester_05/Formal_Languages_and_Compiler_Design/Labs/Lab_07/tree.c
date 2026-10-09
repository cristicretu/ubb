#include "tree.h"
#include <stdio.h>
#include <string.h>

void tree_init(ParseTree *tree, Grammar *g) {
  tree->node_count = 0;
  tree->grammar = g;
}

int tree_add_node(ParseTree *tree, int symbol, int parent) {
  if (tree->node_count >= MAX_TREE_NODES) {
    fprintf(stderr, "Error: tree overflow\n");
    return -1;
  }

  int idx = tree->node_count++;
  tree->nodes[idx].index = idx + 1;
  tree->nodes[idx].symbol = symbol;
  tree->nodes[idx].parent = parent;
  tree->nodes[idx].right_sibling = 0;
  return idx;
}

void tree_set_sibling(ParseTree *tree, int node, int sibling) {
  if (node >= 0 && node < tree->node_count) {
    tree->nodes[node].right_sibling = sibling;
  }
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

