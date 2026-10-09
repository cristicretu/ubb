#include <QApplication>

#include "gui.h"

int main(int argc, char **argv) {
  QApplication app(argc, argv);

  GUI gui;
  gui.show();

  return app.exec();
}