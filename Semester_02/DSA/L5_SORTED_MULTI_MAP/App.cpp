#include <iostream>

#include "ExtendedTest.h"
#include "ShortTest.h"

int main() {
  testAll();
  testAllExtended();

  std::cout << "Finished SMM Tests!" << std::endl;
}
