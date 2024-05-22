#include "SMMIterator.h"

#include "SortedMultiMap.h"
SMMIterator::SMMIterator(const SortedMultiMap& d)
    : map(d), current(nullptr), currentValueIndex(0) {
  first();
}

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
    currentValueIndex = 0;
  } else {
    current = nullptr;
  }
}

void SMMIterator::next() {
  if (!valid()) {
    throw std::exception();
  }

  if (currentValueIndex <
      current->size - 1) {  /// if we still have elements in the current node
    currentValueIndex++;
  } else {
    Node* node = stack.pop();  /// move to the next node

    if (node->right !=
        nullptr) {  /// look for the leftmost node in the right subtree
      node = node->right;
      while (node != nullptr) {
        stack.push(node);
        node = node->left;
      }
    }

    if (!stack.empty()) {
      current = stack.peek();
      currentValueIndex = 0;
    } else {
      current = nullptr;
    }
  }
}

bool SMMIterator::valid() const { return current != nullptr; }

TElem SMMIterator::getCurrent() const {
  if (!valid()) {
    throw std::exception();
  }

  return std::make_pair(current->key, current->elems[currentValueIndex]);
}
