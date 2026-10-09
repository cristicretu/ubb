#include <cstdlib>
#include <ctime>
#include <iostream>

#include "headers/tests/tests.h"
#include "headers/ui/ui.h"

int main(int argc, char *argv[]) {
  // test_all();
  srand(static_cast<unsigned int>(time(NULL)));

  UI ui;
  ui.run();
  return 0;
}