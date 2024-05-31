#include <assert.h>

#include <exception>
#include <iostream>
#include <vector>

#include "SMMIterator.h"
#include "SortedMultiMap.h"

using namespace std;

bool relation1(TKey cheie1, TKey cheie2) {
  if (cheie1 <= cheie2) {
    return true;
  } else {
    return false;
  }
}

void testAll() {
  SortedMultiMap smm = SortedMultiMap(relation1);
  assert(smm.size() == 0);
  assert(smm.isEmpty());
  smm.add(1, 2);
  smm.add(1, 3);
  assert(smm.size() == 2);
  assert(!smm.isEmpty());
  vector<TValue> v = smm.search(1);
  assert(v.size() == 2);
  v = smm.search(3);
  assert(v.size() == 0);
  SMMIterator it = smm.iterator();
  it.first();
  while (it.valid()) {
    TElem e = it.getCurrent();
    it.next();
  }
  assert(smm.remove(1, 2) == true);
  assert(smm.remove(1, 3) == true);
  assert(smm.remove(2, 1) == false);
  assert(smm.isEmpty());

  smm.add(1, 2);
  smm.add(1, 3);
  smm.add(1, 4);

  smm.add(2, 5);
  smm.add(2, 6);

  smm.add(3, 7);
  smm.add(3, 8);
  smm.add(3, 9);

  assert(smm.size() == 8);

  smm.add(4, 10);
  smm.add(4, 11);
  smm.add(4, 12);

  smm.add(5, 13);
  smm.add(5, 14);
  smm.add(5, 15);

  auto searched = smm.search(3);
  auto elems = smm.removeKey(3);

  assert(smm.search(3).size() == 0);

  assert(elems.size() == 3);
  assert(smm.size() == 11);

  assert(searched == elems);
}
