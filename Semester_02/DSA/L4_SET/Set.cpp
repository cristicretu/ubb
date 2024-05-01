#include "Set.h"

#include "SetIterator.h"

Set::Set() {
  // TODO - Implementation
}

bool Set::add(TElem elem) {
  // TODO - Implementation
  return false;
}

bool Set::remove(TElem elem) {
  // TODO - Implementation
  return false;
}

bool Set::search(TElem elem) const {
  // TODO - Implementation
  return false;
}

int Set::size() const {
  // TODO - Implementation
  return 0;
}

bool Set::isEmpty() const {
  // TODO - Implementation
  return false;
}

Set::~Set() {
  // TODO - Implementation
}

SetIterator Set::iterator() const { return SetIterator(*this); }
