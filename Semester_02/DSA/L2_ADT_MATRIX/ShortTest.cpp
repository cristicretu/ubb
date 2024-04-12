#include <assert.h>

#include "Matrix.h"

using namespace std;

void testAll() {
  Matrix m(4, 4);
  assert(m.nrLines() == 4);
  assert(m.nrColumns() == 4);
  m.modify(1, 1, 5);
  assert(m.element(1, 1) == 5);
  TElem old = m.modify(1, 1, 6);
  assert(m.element(1, 2) == NULL_TELEM);
  assert(old == 5);

  m.setElemsOnLine(1, 3);
  assert(m.element(1, 1) == 3);
  assert(m.element(1, 2) == 3);
  assert(m.element(1, 3) == 3);

  m.setElemsOnLine(1, 0);
  assert(m.element(1, 1) == NULL_TELEM);
  assert(m.element(1, 2) == NULL_TELEM);
  assert(m.element(1, 3) == NULL_TELEM);

  m.setElemsOnLine(1, 1);
  assert(m.element(1, 1) == 1);

  try {
    m.setElemsOnLine(-202002, 3);
    assert(false);
  } catch (int line) {
    assert(true);
  }

  m.setElemsOnLine(3, 10000);
  assert(m.element(3, 3) == 10000);
}