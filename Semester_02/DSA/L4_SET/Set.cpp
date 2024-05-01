#include "Set.h"

#include "SetIterator.h"

void Set::resize() {
  TElem *newElements = new TElem[capacity * 2];
  for (int i = 0; i < capacity * 2; i++) {
    newElements[i] = NULL_TELEM;
  }

  for (int i = 0; i < capacity; i++) {
    if (elements[i] != NULL_TELEM) {
      int index = hash(elements[i]);
      int step = 1;
      int secondaryStep = hash2(elements[i]);

      while (newElements[index] != NULL_TELEM) {
        index = (index + secondaryStep * step) % (capacity * 2);
        step++;
      }

      newElements[index] = elements[i];
    }
  }

  delete[] elements;
  elements = newElements;
  capacity *= 2;
}

Set::Set() {
  elements = new TElem[INITIAL_CAPACITY];
  capacity = INITIAL_CAPACITY;
  length = 0;
}

bool Set::add(TElem elem) {
  if (search(elem)) {
    return false;
  }

  if (length == capacity) {
    resize();
  }

  int index = hash(elem);
  int step = 1;
  int secondaryStep = hash2(elem);

  for (int i = 0; i < capacity; i++, step++) {
    if (elements[index] == NULL_TELEM) {
      elements[index] = elem;
      length++;
      return true;
    }
    index = (index + secondaryStep * step) % capacity;
  }

  elements[index] = elem;
  length++;
  return true;
}

int Set::hash(TElem elem) const { return elem % capacity; }
int Set::hash2(TElem elem) const { return 1 + (elem % (capacity - 1)); }

bool Set::remove(TElem elem) {
  // TODO - Implementation
  return false;
}

bool Set::search(TElem elem) const {
  int index = hash(elem);
  int step = 1;
  int secondaryStep = hash2(elem);

  for (int i = 0; i < capacity; i++, step++) {
    if (elements[index] == elem) {
      return true;
    }
    index = (index + secondaryStep * step) % capacity;
  }

  return false;
}

int Set::size() const { return length; }

bool Set::isEmpty() const { return length == 0; }

Set::~Set() {
  delete[] elements;
  elements = nullptr;
  capacity = 0;
  length = 0;
}

SetIterator Set::iterator() const { return SetIterator(*this); }
