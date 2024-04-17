#include <iostream>

#include "ExtendedTest.h"
#include "ShortTest.h"
#include "SortedMap.h"
using namespace std;

int main() {
  testAll();
  testAllExtended();

  cout << "That's all!" << endl;
  return 0;
}
