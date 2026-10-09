#include "ShortTest.h"

#include <assert.h>

#include <exception>

#include "SMIterator.h"
#include "SortedMap.h"
using namespace std;

bool relatie1(TKey cheie1, TKey cheie2) {
  if (cheie1 < cheie2) {
    return true;
  } else {
    return false;
  }
}

void testAll() {
  SortedMap sm(relatie1);
  assert(sm.size() == 0);
  assert(sm.isEmpty());
  sm.add(1, 2);
  assert(sm.size() == 1);
  assert(!sm.isEmpty());
  assert(sm.search(1) != NULL_TVALUE);
  TValue v = sm.add(1, 3);
  assert(v == 2);
  assert(sm.search(1) == 3);
  SMIterator it = sm.iterator();
  it.first();
  while (it.valid()) {
    TElem e = it.getCurrent();
    assert(e.second != NULL_TVALUE);
    it.next();
  }
  assert(sm.remove(1) == 3);
  assert(sm.isEmpty());

  SortedMap sm1(relatie1);
  sm1.add(1, 2);
  sm1.add(2, 3);
  sm1.add(3, 4);
  sm1.add(4, 5);

  SortedMap sm2(relatie1);
  sm2.add(1, 2);
  sm2.add(1, 2);
  sm2.add(1, 2);
  sm2.add(1, 2);

  assert(sm1.addIfNotPresent(sm2) == 0);
  assert(sm1.size() == 4);

  sm2.add(5, 6);
  sm2.add(6, 7);
  sm2.add(7, 8);
  sm2.add(2, 3);
  sm2.add(3, 4);

  assert(sm1.addIfNotPresent(sm2) == 3);
  assert(sm1.size() == 7);
}
