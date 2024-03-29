#include "Bag.h"

#include <cmath>
#include <exception>
#include <iostream>

#include "BagIterator.h"
using namespace std;

Bag::Bag() {
  this->frequencies = new int[1];
  this->capacity = 1;
  this->length = 0;
  this->minimum = NULL_TELEM;
  this->maximum = NULL_TELEM;
}

/*
BC: Θ(1) - element is between minimum and maximum
WC: Θ(n) - element is not between minimum and maximum, so we need to resize the
array TC: O(n)
*/
void Bag::add(TElem elem) {
  int oldMinimum = this->minimum, oldMaximum = this->maximum;
  // exception if it's the first element, so both maximum and minimum are the
  // same
  if (Bag::size() == 0) {
    this->minimum = elem;
    this->maximum = elem;

    this->frequencies[0] = 1;
    this->length = 1;
    return;
  }

  // If we got a new mininum or a new maximum, we have to resize the array,
  // otherwise we just increment the frequencies
  bool resize = false;
  if (elem < this->minimum) {
    this->minimum = elem;
    resize = true;
  }
  if (elem > this->maximum) {
    this->maximum = elem;
    resize = true;
  }

  if (!resize) {
    // Just increment the frequencies
    this->frequencies[elem - this->minimum]++;
    this->length = this->length + 1;
    return;
  }

  // the new cpacity adjusted for the new maximum or minimum
  int newCapacity = maximum - minimum + 1;

  int *newFrequencies = new int[newCapacity];

  for (int i = 0; i < newCapacity; i++) {
    newFrequencies[i] = 0;
  }

  // if we have a new minimum ,we have to shirt to the right all element by the
  // difference of the old minimum and the new minimum
  if (oldMinimum > this->minimum) {
    int j = 0;
    for (int i = oldMinimum - this->minimum; i < newCapacity; i++) {
      newFrequencies[i] = this->frequencies[j++];
    }
    newFrequencies[0]++;
  }
  if (oldMaximum < this->maximum) {
    // otherwise we just copy everything
    for (int i = 0; i < this->capacity; i++) {
      newFrequencies[i] = this->frequencies[i];
    }

    // and adjust the last element frequency (the maximum)
    newFrequencies[newCapacity - 1]++;
  }

  // cleanup after resze
  delete[] this->frequencies;
  this->frequencies = newFrequencies;
  this->capacity = newCapacity;
  this->length = this->length + 1;
}

/*
BC: Θ(1) - element is between minimum and maximum, or does not belong to the bag
                 - or the element is the minimum or maximum, but it has more
than one occurences WC: Θ(n) - element is the minimum or maximum and has only
one occurence, so we need to readjust the indexes again TC: O(n)
*/

bool Bag::remove(TElem elem) {
  if (this->length == 0 || search(elem) == false) {
    return false;
  }

  if (elem != this->minimum && elem != this->maximum) {
    this->frequencies[elem - this->minimum]--;
    this->length = this->length - 1;
    return true;
  }

  if (elem == this->minimum) {
    if (this->frequencies[elem - this->minimum] > 1) {
      this->frequencies[elem - this->minimum]--;
      this->length = this->length - 1;
      return true;
    } else {
      // If we only have one element, then go back to the initial structure
      if (this->length == 1) {
        this->minimum = NULL_TELEM;
        this->maximum = NULL_TELEM;
        this->length = 0;
        this->capacity = 1;
        delete[] this->frequencies;
        this->frequencies = new int[1];
        return true;
      }

      // Find the new minimum
      int oldMinimum = this->minimum;
      for (int i = 1; i < this->capacity; i++) {
        if (this->frequencies[i] > 0) {
          this->minimum = i + this->minimum;
          break;
        }
      }

      // Calculate the new capacity based on the new minimum element
      int newCapacity = this->maximum - this->minimum + 1;
      int *newFrequencies = new int[newCapacity];

      // Init a new list with 0
      for (int i = 0; i < newCapacity; i++) {
        newFrequencies[i] = 0;
      }

      // Logic for copying all frequencies starting from the new minimum element
      int j = 0;
      for (int i = this->minimum - oldMinimum; i < this->capacity; i++) {
        newFrequencies[j++] = this->frequencies[i];
      }

      // cleanup
      delete[] this->frequencies;
      this->frequencies = newFrequencies;
      this->capacity = newCapacity;
      this->length = this->length - 1;
    }
  }
  if (elem == this->maximum) {
    // If the element appears more than 1 time, good, just decrement
    if (this->frequencies[elem - this->minimum] > 1) {
      this->frequencies[elem - this->minimum]--;
      this->length = this->length - 1;
      return true;
    } else {
      // Else check if it's the only element, and go back to the initial
      // structure
      if (this->length == 1) {
        this->minimum = NULL_TELEM;
        this->maximum = NULL_TELEM;
        this->length = 0;
        this->capacity = 1;
        delete[] this->frequencies;
        this->frequencies = new int[1];
        return true;
      }

      // find the new maximum for the array
      for (int i = this->capacity - 2; i >= 0; i--) {
        if (this->frequencies[i] > 0) {
          this->maximum = i + this->minimum;
          break;
        }
      }

      // calculate the new capacity for the new maximum
      int newCapacity = this->maximum - this->minimum + 1;

      int *newFrequencies = new int[newCapacity];

      // just copy everything as it was until we reach the new maximum element
      for (int i = 0; i < newCapacity; i++) {
        newFrequencies[i] = this->frequencies[i];
      }

      // cleanup
      delete[] this->frequencies;
      this->frequencies = newFrequencies;
      this->capacity = newCapacity;
      this->length = this->length - 1;
    }
  }

  return true;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool Bag::search(TElem elem) const {
  // cout << "I AM SEARCHING FOR " <<elem << "having F[] " <<
  // this->frequencies[elem - this->minimum] << " reuslt is" << (elem >=
  // this->minimum && elem <= this->maximum && this->frequencies[elem -
  // this->minimum] > 0) << endl;
  return elem >= this->minimum && elem <= this->maximum &&
         this->frequencies[elem - this->minimum] > 0;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Bag::nrOccurrences(TElem elem) const {
  if (elem < this->minimum || elem > this->maximum) {
    return 0;
  }
  return this->frequencies[elem - this->minimum];
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Bag::size() const { return this->length; }

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool Bag::isEmpty() const { return this->length == 0; }

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)

*/
BagIterator Bag::iterator() const { return BagIterator(*this); }

void Bag::printBag() {
  // cout << "I AM CAPACITY " << this->capacity << endl;
  // cout << "I AM LENGTH " << this->length << endl;
  // for (int i = 0; i < this->capacity; i++)
  // {
  // if (this->frequencies[i] > 0)
  // cout << i + this->minimum << " ";
  // cout << i + this->minimum << " ";
  // cout << i << " ";
  // }
  // cout << endl;
  // cout << this->capacity << endl;
  // for (int i = 0; i < this->capacity; i++)
  // {
  // if (this->frequencies[i] > 0)
  // cout << this->frequencies[i] << " ";
  // cout << this->frequencies[i] << " ";
  // }
  cout << endl;
}

Bag::~Bag() { delete[] this->frequencies; }
