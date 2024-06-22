#include <QApplication>

#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  Session session;
  return a.exec();
}