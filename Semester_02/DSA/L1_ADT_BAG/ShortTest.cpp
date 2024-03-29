#include "ShortTest.h"

#include <assert.h>

#include "Bag.h"
#include "BagIterator.h"

void testAll() {
  Bag b;
  assert(b.isEmpty() == true);
  assert(b.size() == 0);
  b.add(5);
  b.add(1);
  b.add(10);
  b.add(7);
  b.add(1);
  b.add(11);
  b.add(-3);
  assert(b.size() == 7);
  assert(b.search(10) == true);
  assert(b.search(16) == false);
  assert(b.nrOccurrences(1) == 2);
  assert(b.nrOccurrences(7) == 1);
  assert(b.remove(1) == true);
  assert(b.remove(6) == false);
  assert(b.size() == 6);
  assert(b.nrOccurrences(1) == 1);
  BagIterator it = b.iterator();
  it.first();
  while (it.valid()) {
    TElem e = it.getCurrent();
    it.next();
  }

  b.addOccurences(20, 6);
  assert(b.nrOccurrences(6) == 20);

  b.addOccurences(200, 60);
  assert(b.nrOccurrences(60) == 200);

  try {
    b.addOccurences(-200, 6);
    assert(false);
  } catch (int nr) {
    assert(true);
  }

  b.addOccurences(99999, 99999);
  assert(b.nrOccurrences(99999) == 99999);

  b.addOccurences(10, 10);
  assert(b.nrOccurrences(10) == 11);
}
