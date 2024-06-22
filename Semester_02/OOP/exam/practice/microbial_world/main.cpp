#include <QApplication>

#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  Session session;

  for (auto &biologist : session.get_biologists()) {
    auto *window = new Window(session, biologist);
    window->show();
  }
  return a.exec();
}