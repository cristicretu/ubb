#include <iostream>

#include "Bag.h"
#include "ExtendedTest.h"
#include "ShortTest.h"

using namespace std;

int main() {
  testAll();
  cout << "Short tests over" << endl;
  testAllExtended();

  cout << "All test over" << endl;
}