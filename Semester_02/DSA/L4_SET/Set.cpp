#include "Set.h"

#include "SetIterator.h"

Set::Set() : capacity(INITIAL_CAPACITY), length(0) {
  elements = new TElem[capacity];
  std::fill(elements, elements + capacity, NULL_TELEM);
}
/*
BC: Theta(n)
WC: Theta(n)
AC: Theta(n)
*/
void Set::resize() {
  int newCapacity = findNextPrime(capacity * 2);
  TElem *newElements = new TElem[newCapacity];
  std::fill(newElements, newElements + newCapacity, NULL_TELEM);

  for (int i = 0; i < capacity; i++) {
    if (elements[i] != NULL_TELEM && elements[i] != DELETED_TELEM) {
      int index = hash(elements[i], newCapacity);
      int secondaryStep = hash2(elements[i], newCapacity);
      while (newElements[index] != NULL_TELEM &&
             newElements[index] != DELETED_TELEM) {
        index = (index + secondaryStep) % newCapacity;
      }
      newElements[index] = elements[i];
    }
  }

  delete[] elements;
  elements = newElements;
  capacity = newCapacity;
}

/*
BC: Theta(1)
WC: Theta(alfa)
AC: O(alfa)
*/
bool Set::add(TElem elem) {
  int index = hash(elem, capacity);
  int step = hash2(elem, capacity);
  int originalIndex = index;

  while (elements[index] != NULL_TELEM && elements[index] != DELETED_TELEM) {
    if (elements[index] == elem) {
      return false;  // element already exists
    }
    index = (index + step) % capacity;
  }

  elements[index] = elem;
  ++length;

  if (length >=
      capacity * LOAD_FACTOR_THRESHOLD) {  // if the load factor is greater than
                                           // the threshold
    resize();
  }
  return true;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/

int Set::hash(TElem elem, int cap) const { return std::abs(elem) % cap; }

// /*
// BC: Theta(1)
// WC: Theta(1)
// AC: Theta(1)
// */

int Set::hash2(TElem elem, int cap) const {
  return 1 + (abs(elem) % (cap - 1));
}

/*
BC: Theta(1)
WC: Theta(alfa)
AC: O(alfa)
*/
bool Set::remove(TElem elem) {
  int index = hash(elem, capacity);
  int step = hash2(elem, capacity);
  int originalIndex = index;

  while (elements[index] != NULL_TELEM && elements[index] != elem) {
    if (elements[index] == DELETED_TELEM) {
      index = (index + step) % capacity;
      continue;
    }
    index = (index + step) % capacity;
  }

  if (elements[index] != elem) {  // element not found
    return false;
  }

  elements[index] = DELETED_TELEM;  // mark as DELETED
  length--;

  return true;
}

/*
BC: Theta(1) - if the first hash is the correct one
WC: Theta(alfa) - if the element is not in the set
AC: O(alfa) - if the element is in the set, but not at the first hash index
where n is the number of elements in the set
*/
bool Set::search(TElem elem) const {
  int index = hash(elem, capacity);
  int secondaryStep = hash2(elem, capacity);
  int originalIndex = index;

  do {
    if (elements[index] == elem) {
      return true;
    }
    if (elements[index] == NULL_TELEM) {  // we finished probing the set, the
                                          // element is not in the set
      return false;
    }
    if (elements[index] == DELETED_TELEM) {
      index = (index + secondaryStep) % capacity;  // skip over deleted elements
      continue;
    }
    index = (index + secondaryStep) % capacity;  // move to the next index
  } while (index !=
           originalIndex);  // while we didn't loop through the entire set

  return false;
}

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
int Set::size() const { return length; }

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
bool Set::isEmpty() const { return length == 0; }

/*
BC: Theta(1)
WC: Theta(1)
AC: Theta(1)
*/
Set::~Set() {
  delete[] elements;
  elements = nullptr;
  capacity = 0;
  length = 0;
}

SetIterator Set::iterator() const { return SetIterator(*this); }