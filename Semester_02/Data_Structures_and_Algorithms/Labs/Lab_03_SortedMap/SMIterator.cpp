#include "SMIterator.h"

#include <exception>

#include "SortedMap.h"

using namespace std;

SMIterator::SMIterator(const SortedMap& m) : map(m) {
  this->current = this->map.head;
}

void SMIterator::first() { this->current = this->map.head; }

void SMIterator::next() {
  if (!this->valid()) {
    throw exception();
  }
  this->current = this->map.next[this->current];
}

bool SMIterator::valid() const { return this->current != -1; }

TElem SMIterator::getCurrent() const {
  if (!this->valid()) {
    throw exception();
  }
  return this->map.elements[this->current];
}
