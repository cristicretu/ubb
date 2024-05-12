#include "Set.h"

#include "SetIterator.h"

Set::Set() : capacity(INITIAL_CAPACITY), length(0) {
  elements = new TElem[capacity];
  std::fill(elements, elements + capacity, NULL_TELEM);
}

void Set::resize() {
  int newCapacity = findNextPrime(capacity * 2);
  TElem *newElements = new TElem[newCapacity];
  std::fill(newElements, newElements + newCapacity, NULL_TELEM);

  for (int i = 0; i < capacity; i++) {
    if (elements[i] != NULL_TELEM) {
      int index = hash(elements[i], newCapacity);
      int secondaryStep = hash2(elements[i], newCapacity);
      while (newElements[index] != NULL_TELEM) {
        index = (index + secondaryStep) % newCapacity;
      }
      newElements[index] = elements[i];
    }
  }

  delete[] elements;
  elements = newElements;
  capacity = newCapacity;
}

bool Set::add(TElem elem) {
  if (search(elem)) {
    return false;
  }

  if (length >= capacity * LOAD_FACTOR_THRESHOLD) {
    resize();
  }

  int index = hash(elem, capacity);
  int secondaryStep = hash2(elem, capacity);

  while (elements[index] != NULL_TELEM) {
    index = (index + secondaryStep) % capacity;
  }

  elements[index] = elem;
  ++length;
  return true;
}

int Set::hash(TElem elem, int cap) const {
  return std::abs(static_cast<int>(std::hash<int>{}(elem) % cap));
}

int Set::hash2(TElem elem, int cap) const {
  int result = 1 + (std::abs(static_cast<int>(std::hash<int>{}(elem) / cap)) %
                    (cap - 1));
  return result;
}

bool Set::remove(TElem elem) {
  int index = hash(elem, capacity);
  int secondaryStep = hash2(elem, capacity);
  int probeIndex = index;

  while (elements[probeIndex] != NULL_TELEM && elements[probeIndex] != elem) {
    probeIndex = (probeIndex + secondaryStep) % capacity;
    if (probeIndex == index) {
      return false;
    }
  }

  if (elements[probeIndex] != elem) {
    return false;
  }

  elements[probeIndex] = NULL_TELEM;
  length--;

  int nextIndex = (probeIndex + secondaryStep) % capacity;
  while (elements[nextIndex] != NULL_TELEM) {
    TElem toRehash = elements[nextIndex];
    elements[nextIndex] = NULL_TELEM;
    int newIndex = hash(toRehash, capacity);
    int newStep = hash2(toRehash, capacity);

    while (elements[newIndex] != NULL_TELEM) {
      newIndex = (newIndex + newStep) % capacity;
    }

    elements[newIndex] = toRehash;
    nextIndex = (nextIndex + secondaryStep) % capacity;
  }

  return true;
}

bool Set::search(TElem elem) const {
  int index = hash(elem, capacity);
  int secondaryStep = hash2(elem, capacity);
  int originalIndex = index;

  do {
    if (elements[index] == elem) {
      return true;
    }
    if (elements[index] == NULL_TELEM) {
      return false;
    }
    index = (index + secondaryStep) % capacity;
  } while (index != originalIndex);

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