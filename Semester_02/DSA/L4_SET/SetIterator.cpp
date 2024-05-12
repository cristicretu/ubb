#include "SetIterator.h"

#include "Set.h"

SetIterator::SetIterator(const Set& m) : set(m), current(0) { first(); }

void SetIterator::first() {
  current = 0;
  while (current < set.capacity && set.elements[current] == NULL_TELEM) {
    current++;
  }
}

void SetIterator::next() {
  if (!valid()) {
    throw std::exception();
  }
  current++;
  while (current < set.capacity && set.elements[current] == NULL_TELEM) {
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
  return current < set.capacity && set.elements[current] != NULL_TELEM;
}
