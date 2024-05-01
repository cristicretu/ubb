#include "SetIterator.h"

#include "Set.h"

SetIterator::SetIterator(const Set& m) : set(m) {
  while (set.elements[current] == NULL_TELEM && current < set.capacity) {
    current++;
  }
}

void SetIterator::first() {
  current = 0;
  while (set.elements[current] == NULL_TELEM && current < set.capacity) {
    current++;
  }
}

void SetIterator::next() {
  current++;
  while (set.elements[current] == NULL_TELEM && current < set.capacity) {
    current++;
  }
}

TElem SetIterator::getCurrent() {
  if (valid()) {
    return set.elements[current];
  }
}

bool SetIterator::valid() const {
  return current < set.capacity && set.elements[current] != NULL_TELEM;
}
