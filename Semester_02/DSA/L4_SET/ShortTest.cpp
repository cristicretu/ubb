#include "ShortTest.h"

#include <assert.h>

#include "Set.h"
#include "SetIterator.h"

void testAll() {
  Set s;
  assert(s.isEmpty() == true);
  assert(s.size() == 0);
  assert(s.add(5) == true);
  assert(s.add(1) == true);
  assert(s.add(10) == true);
  assert(s.add(7) == true);
  assert(s.add(1) == false);
  assert(s.add(10) == false);
  assert(s.add(-3) == true);
  assert(s.size() == 5);
  assert(s.search(10) == true);
  assert(s.search(16) == false);
  assert(s.remove(1) == true);
  assert(s.remove(6) == false);
  assert(s.size() == 4);

  SetIterator it = s.iterator();
  it.first();
  int sum = 0;
  while (it.valid()) {
    TElem e = it.getCurrent();
    sum += e;
    it.next();
  }
  assert(sum == 19);

  s.add(9);
  std::vector<int> v;

  SetIterator it3 = s.iterator();
  it3.first();

  while (it3.valid()) {
    TElem e = it3.getCurrent();
    v.push_back(e);
    it3.next();
  }

  // for (int i = 0; i < v.size(); i++) {
  //   std::cout << v[i] << std::endl;
  // }
  // std::cout << std::endl;

  SetIterator it2 = s.iterator();
  it2.first();
  assert(it2.valid() == true);
  int count = 0;
  while (it2.valid()) {
    TElem e = it2.getCurrent();
    assert(e == v[count]);
    try {
      it2.jumpForward(2);
    } catch (std::exception&) {
      assert(count + 2 >= v.size());
    }
    count += 2;
  }
}
