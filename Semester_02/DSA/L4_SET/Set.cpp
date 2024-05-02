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
  int newCapacity = findNextPrime(capacity * 2);
  TElem *newElements = new TElem[newCapacity];
  for (int i = 0; i < newCapacity; i++) {
    newElements[i] = NULL_TELEM;
  }

  for (int i = 0; i < capacity; i++) {
    if (elements[i] != NULL_TELEM) {
      int index = hash(elements[i]) % newCapacity;
      int step = 1;
      int secondaryStep = hash2(elements[i]) % newCapacity;

      while (newElements[index] != NULL_TELEM) {
        index = (index + secondaryStep * step++) % newCapacity;
      }

      newElements[index] = elements[i];
    }
  }

  delete[] elements;
  elements = newElements;
  capacity = newCapacity;
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

  if (elements[index] == NULL_TELEM) {
    elements[index] = elem;
    ++length;
    return true;
  }

  for (int i = 0; i < capacity && elements[index] != NULL_TELEM; i++, step++) {
    index = (index + secondaryStep * step) % capacity;
  }

  // if (elements[index] != NULL_TELEM) {
  //   std::cout << "i am elem " << elem << " and i am not added\n";
  //   return false;
  // }
  elements[index] = elem;
  ++length;
  return true;
}

int Set::hash(TElem elem) const {
  // return elem % capacity;
  // return (elem ^ (elem << 1) ^ (elem << 4) ^ (elem << 7)) % capacity;
  return (std::hash<int>{}(elem)) % capacity;

  // std::string s = std::to_string(elem);
  // long sum = 0, mul = 1;
  // for (int i = 0; i < s.length(); i++) {
  //   mul = (i % 4 == 0) ? 1 : mul * 256;
  //   sum += s[i] * mul;
  // }
  // return int(abs(sum) % capacity);
  // return floor(capacity * (elem * 1337 % 1));
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
    if (elements[index] == NULL_TELEM) {
      return false;
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

    if (elements[index] == NULL_TELEM) {
      return false;
    }

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
