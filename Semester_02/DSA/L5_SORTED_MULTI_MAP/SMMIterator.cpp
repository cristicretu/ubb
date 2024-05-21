#include "SMMIterator.h"

#include "SortedMultiMap.h"

SMMIterator::SMMIterator(const SortedMultiMap& d) : map(d) { first(); }

void SMMIterator::first() {
  while (!stack.empty()) {
    stack.pop();
  }

  current = map.root;
  while (current != nullptr) {
    stack.push(current);
    current = current->left;
  }

  if (!stack.empty()) {
    current = stack.peek();
  } else {
    current = nullptr;
  }
}

void SMMIterator::next() {
  if (!valid()) {
    throw std::exception();
  }

  Node* node = stack.pop();

  if (node->right != nullptr) {
    node = node->right;
    while (node != nullptr) {
      stack.push(node);
      node = node->left;
    }
  }

  if (stack.empty()) {
    current = nullptr;
  } else {
    current = stack.peek();
  }
}

bool SMMIterator::valid() const { return current != nullptr; }

TElem SMMIterator::getCurrent() const {
  // TODO - Implementation
  return NULL_TELEM;
}
