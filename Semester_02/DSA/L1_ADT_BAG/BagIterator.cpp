#include "BagIterator.h"

#include <exception>
#include <iostream>

#include "Bag.h"

using namespace std;

BagIterator::BagIterator(const Bag &c) : bag(c) {
  // TODO - Implementation
  this->position = 0;
  this->occurrences = this->bag.frequencies[this->position];
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
void BagIterator::first() {
  this->position = 0;
  this->occurrences = this->bag.frequencies[this->position];
}

/*
BC: Θ(1)
WC: Θ(n) - when we are on the minimum element and we need to go to the maximum
TC: O(n)
*/
void BagIterator::next() {
  if (!this->valid()) {
    this->position = -1;
    throw exception();
  }

  if (this->occurrences > 1) {
    this->occurrences--;
    return;
  }

  do {
    this->position++;
  } while (this->position < this->bag.capacity &&
           this->bag.frequencies[this->position] == 0);

  if (this->position < this->bag.capacity) {
    this->occurrences = this->bag.frequencies[this->position];
  } else {
    this->position = -1;
  }
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool BagIterator::valid() const {
  return this->bag.length != 0 && this->position != -1 &&
         this->position < this->bag.capacity;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
TElem BagIterator::getCurrent() const {
  if (!this->valid()) {
    throw exception();
  }

  return this->position + this->bag.minimum;
}
