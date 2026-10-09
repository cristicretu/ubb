#include "SortedMap.h"

#include <exception>

#include "SMIterator.h"
using namespace std;

void SortedMap::resize() {
  int newCapacity = capacity * 2;
  TElem* newElements = new TElem[newCapacity];
  int* newNext = new int[newCapacity];

  for (int i = 0; i < capacity; i++) {
    newElements[i] = elements[i];  /// Copy the elements and next pointers
    newNext[i] = next[i];
  }
  for (int i = capacity; i < newCapacity - 1;
       i++) {  /// Initialize the next pointers for the new elements
    newNext[i] = i + 1;
  }
  newNext[newCapacity - 1] = -1;  /// The last element has no next, so it's -1

  delete[] elements;  /// Delete the old arrays
  delete[] next;

  elements = newElements;
  next = newNext;
  firstEmpty = capacity;  /// The first empty space is the first new element
  capacity = newCapacity;
}

// BC: Theta(1)
// WC: Theta(1)
// AC: Theta(1)
SortedMap::SortedMap(Relation r) {
  this->relation = r;
  this->capacity = 1;
  this->elements = new TElem[this->capacity];
  this->next = new int[this->capacity];
  this->firstEmpty = 0;
  this->head = -1;  /// No new elements
  this->len = 0;
  this->next[0] = -1;  /// No next element
}

// BC: Theta(1), when the key is the first element or the map is empty
// WC: Theta(n), when the key is the last element, so we have to iterate through
// all the elements
// AC : O(n)
TValue SortedMap::add(TKey k, TValue v) {
  if (firstEmpty == -1) {  /// No more empty spaces, resize
    resize();
  }

  int current = head;
  int previous = -1;

  while (current != -1 &&
         relation(elements[current].first,
                  k)) {  /// Find the position to insert the new element
    previous = current;
    current = next[current];
  }

  if (current != -1 && elements[current].first ==
                           k) {  /// If the key already exists, update the value
    TValue old = elements[current].second;
    elements[current].second = v;
    return old;  /// Return the old value
  }

  int newElem = firstEmpty;       /// The new element will be inserted here
  firstEmpty = next[firstEmpty];  /// Update the first empty space
  elements[newElem] = make_pair(k, v);  /// Insert the new element
  next[newElem] = current;  /// Update the next pointer of the new element

  if (previous ==
      -1) {  /// If the new element is the first element, update the head
    head = newElem;
  } else {  /// Otherwise, update the next pointer of the previous element
    next[previous] = newElem;
  }
  len++;
  return NULL_TVALUE;
}

// BC: Theta(1), when the key is the first element
// WC: Theta(n), when the key is the last element, so we have to iterate through
// all the elements
// AC: O(n)
TValue SortedMap::search(TKey k) const {
  int current = head;
  while (current != -1) {  /// Iterate through the elements
    if (elements[current].first ==
        k) {  /// If the key is found, return the value
      return elements[current].second;
    }
    current = next[current];  /// Move to the next element
  }
  return NULL_TVALUE;
}

// BC: Theta(1), when the key is the first element, or the map is empty
// WC: Theta(n), when the key is the last element, so we have to iterate through
// all the elements
// AC: O(n)
TValue SortedMap::remove(TKey k) {
  int current = head;  /// Use 2 pointers to keep track of the current and
                       /// previous element
  int previous = -1;

  while (current != -1 &&
         elements[current].first != k) {  /// Find the element to remove
    previous = current;
    current = next[current];
  }

  if (current == -1) {  /// If the element is not found, return NULL_TVALUE
    return NULL_TVALUE;
  }

  if (previous ==
      -1) {  /// If the element is the first element, update the head
    head = next[head];
  } else {
    next[previous] = next[current];  /// Otherwise, update the next pointer of
                                     /// the previous element
  }

  TValue removedValue = elements[current].second;  /// Save the value to return
  next[current] =
      firstEmpty;        /// Update the next pointer of the removed element
  firstEmpty = current;  /// Update the first empty space
  len--;                 /// Decrease the length

  return removedValue;
}

// BC: Theta(1)
// WC: Theta(1)
// AC: Theta(1)
int SortedMap::size() const { return this->len; }

// BC: Theta(1)
// WC: Theta(1)
// AC: Theta(1)
bool SortedMap::isEmpty() const { return this->len == 0; }

int SortedMap::addIfNotPresent(SortedMap& sm) {
  SMIterator itSelf = this->iterator();
  SMIterator itSm = sm.iterator();
  int ans = 0;

  int current = head;
  int previous = -1;

  TKey currentSelfKey, currentSmKey;
  TValue currentSmValue;

  while (itSelf.valid() && itSm.valid()) {
    currentSelfKey = itSelf.getCurrent().first;
    currentSmKey = itSm.getCurrent().first;
    currentSmValue = itSm.getCurrent().second;

    if (currentSmKey < currentSelfKey) {
      int newElem = firstEmpty;
      if (newElem == -1) {
        resize();
        newElem = firstEmpty;
      }
      firstEmpty = next[firstEmpty];

      elements[newElem] = make_pair(currentSmKey, currentSmValue);
      next[newElem] = current;

      if (previous == -1) {
        head = newElem;
      } else {
        next[previous] = newElem;
      }

      ans++;
      len++;
      itSm.next();
    } else if (currentSmKey == currentSelfKey) {
      previous = current;
      current = next[current];
      itSelf.next();
      itSm.next();
    } else {
      previous = current;
      current = next[current];
      itSelf.next();
    }
  }

  while (itSm.valid()) {
    currentSmKey = itSm.getCurrent().first;
    currentSmValue = itSm.getCurrent().second;

    int newElem = firstEmpty;
    if (newElem == -1) {
      resize();
      newElem = firstEmpty;
    }
    firstEmpty = next[firstEmpty];

    elements[newElem] = make_pair(currentSmKey, currentSmValue);
    next[newElem] = -1;

    if (previous == -1) {
      head = newElem;
    } else {
      next[previous] = newElem;
    }

    ans++;
    len++;
    itSm.next();
  }

  return ans;
}

SMIterator SortedMap::iterator() const { return SMIterator(*this); }

SortedMap::~SortedMap() {
  delete[] this->elements;
  delete[] this->next;
}
