#include <QApplication>
#include <QMainWindow>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include "headers/qt/window.h"
#include "headers/tests/tests.h"
#include "headers/ui/ui.h"

int main(int argc, char *argv[]) {
  // test_all();
  srand(static_cast<unsigned int>(time(NULL)));

  QApplication app(argc, argv);
  QCoreApplication::setApplicationName("Dog Adoption");
  DogApp window;
  window.resize(400, 300);
  window.show();

  return app.exec();
}