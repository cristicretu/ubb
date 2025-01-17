#include "SMMIterator.h"

#include "SortedMultiMap.h"

/*
BC: Theta(1)
WC: Theta(n)
AC: O(n)
*/
SMMIterator::SMMIterator(const SortedMultiMap& d)
    : map(d), currentValueIndex(0) {
  stack = new TElem[map.size()];  /// number of elements in the bst

  Node* current = map.root;
  int index = 0;

  while (current != nullptr || !st.empty()) {  /// go through the bst
    while (current != nullptr) {               /// exhaust the left branch
      st.push(current);
      current = current->left;
    }

    current = st.pop();  /// get the last node from the stack, populate the arr
                         /// with its elements
    for (int i = 0; i < current->size; ++i) {
      stack[index++] = std::make_pair(current->key, current->elems[i]);
    }
    current = current->right;
  }
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
void SMMIterator::first() { currentValueIndex = 0; }

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
void SMMIterator::next() {
  if (!valid()) {
    throw std::exception();
  }

  currentValueIndex++;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
bool SMMIterator::valid() const {
  return currentValueIndex < map.size() && stack[currentValueIndex].first != -1;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
TElem SMMIterator::getCurrent() const {
  if (!valid()) {
    throw std::exception();
  }

  return stack[currentValueIndex];
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
SMMIterator::~SMMIterator() {
  delete[] stack;
  while (!st.empty()) {
    Node* node = st.pop();
    delete node;
  }
}
