#include "SetIterator.h"

#include "Set.h"

SetIterator::SetIterator(const Set& m) : set(m), current(0) { first(); }

void SetIterator::first() {
  current = 0;
  while (current < set.capacity && (set.elements[current] == NULL_TELEM ||
                                    set.elements[current] == DELETED_TELEM)) {
    current++;
  }
}

void SetIterator::next() {
  if (!valid()) {
    throw std::exception();
  }
  current++;
  while (current < set.capacity && (set.elements[current] == NULL_TELEM ||
                                    set.elements[current] == DELETED_TELEM)) {
    current++;
  }
}

TElem SetIterator::getCurrent() {
  if (!valid()) {
    throw std::exception();
  }
  return set.elements[current];
}

bool SetIterator::valid() const {
  return current < set.capacity && set.elements[current] != NULL_TELEM &&
         set.elements[current] != DELETED_TELEM;
}

void SetIterator::jumpForward(int k) {
  if (k <= 0) {
    throw std::exception();
  }

  int steps = 0;
  while (steps < k && current < set.capacity) {
    current++;
    while (current < set.capacity && (set.elements[current] == NULL_TELEM ||
                                      set.elements[current] == DELETED_TELEM)) {
      current++;
    }
    steps++;
  }

  if (steps < k) {
    throw std::exception();
  }
}