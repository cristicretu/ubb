#pragma once
#include <algorithm>
#include <iostream>
#include <set>
// DO NOT INCLUDE SETITERATOR

// DO NOT CHANGE THIS PART
#define NULL_TELEM -111111
typedef int TElem;
class SetIterator;

class Set {
  // DO NOT CHANGE THIS PART
  friend class SetIterator;

 private:
  const int INITIAL_CAPACITY = 17;
  const double LOAD_FACTOR_THRESHOLD = 0.5;

  TElem *elements;
  int capacity;
  int length;

  int hash(TElem elem, int cap) const;
  int hash2(TElem elem, int cap) const;
  void resize();

 public:
  // implicit constructor
  Set();

  // adds an element to the set
  // returns true if the element was added, false otherwise (if the element was
  // already in the set and it was not added)
  bool add(TElem e);

  // removes an element from the set
  // returns true if e was removed, false otherwise
  bool remove(TElem e);

  // checks whether an element belongs to the set or not
  bool search(TElem elem) const;

  // returns the number of elements;
  int size() const;

  // check whether the set is empty or not;
  bool isEmpty() const;

  void print() {
    std::cout << capacity << " " << length << std::endl;
    std::vector<int> s;
    for (int i = 0; i < capacity; i++) {
      s.push_back(elements[i]);
    }

    std::sort(s.begin(), s.end());

    for (auto elem : s) {
      std::cout << elem << " ";
    }
    std::cout << std::endl;
  }

  // return an iterator for the set
  SetIterator iterator() const;

  int findNextPrime(int n) {
    while (true) {
      bool isPrime = true;
      for (int i = 2; i <= n / 2; i++) {
        if (n % i == 0) {
          isPrime = false;
          break;
        }
      }

      if (isPrime) {
        return n;
      }

      n++;
    }
  }

  // destructor
  ~Set();
};
