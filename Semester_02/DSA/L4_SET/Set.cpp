#include "Set.h"

#include "SetIterator.h"

/*
BC: Theta(n)
WC: Theta(n)
TC: Theta(n)
*/
Set::Set() : capacity(INITIAL_CAPACITY), length(0) {
  elements = new TElem[capacity];
  for (int i = 0; i < capacity; i++) {
    elements[i] = NULL_TELEM;
  }
}

/*
BC: Theta(n)
WC: Theta(n)
TC: Theta(n)
*/
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

/*
BC: Theta(1) - when no collision occurs
WC: Theta(n) - when probing has to go through all the elements
TC: O(n)
*/
bool Set::add(TElem elem) {
  if (search(elem)) {
    return false;
  }

  if (length >= capacity * LOAD_FACTOR_THRESHOLD) {
    resize();
  }

  int index = hash(elem);
  int step = 1;
  int secondaryStep = hash2(elem);

  for (int i = 0; i < capacity; i++, step++) {
    if (elements[index] == NULL_TELEM) {
      break;
    }
    index = (index + secondaryStep * step) % capacity;
  }

  elements[index] = elem;
  length++;
  return true;
}

int Set::hash(TElem elem) const {
  // return elem % capacity;
  return (elem ^ (elem << 1) ^ (elem << 4) ^ (elem << 7)) % capacity;
}
int Set::hash2(TElem elem) const { return 1 + (elem % (capacity - 1)); }

/*
BC: Theta(1)
WC: Theta(n)
TC: O(n)
*/
bool Set::remove(TElem elem) {
  if (isEmpty() || !search(elem)) {
    return false;
  }

  int index = hash(elem);
  int step = 1;
  int secondaryStep = hash2(elem);

  for (int i = 0; i < capacity; i++, step++) {
    if (elements[index] == elem) {
      elements[index] = NULL_TELEM;
      length--;
      return true;
    }
    index = (index + secondaryStep * step) % capacity;
  }

  return false;
}

/*
BC: Theta(1)
WC: Theta(n)
TC: O(n)
*/
bool Set::search(int elem) const {
  int index = hash(elem);
  int step = 1;
  int secondaryStep = hash2(elem);

  for (int i = 0; i < capacity; i++, step++) {
    if (elements[index] == elem) {
      return true;
    }
    // if (elements[index] == NULL_TELEM) {
    //   return false;
    // }
    index = (index + secondaryStep * step) % capacity;
  }

  return false;
}

/*
BC: Theta(1)
WC: Theta(1)
TC: Theta(1)
*/
int Set::size() const { return length; }

/*
BC: Theta(1)
WC: Theta(1)
TC: Theta(1)
*/
bool Set::isEmpty() const { return length == 0; }

Set::~Set() {
  delete[] elements;
  elements = nullptr;
  capacity = 0;
  length = 0;
}

SetIterator Set::iterator() const { return SetIterator(*this); }
