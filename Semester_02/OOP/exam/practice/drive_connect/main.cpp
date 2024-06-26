#include <QApplication>

#include "repository.h"
#include "session.h"
#include "window.h"
#include "winmap.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Session session(repo);

  for (auto &x : repo.getDrivers()) {
    auto window = new Window(session, x);
    window->show();
  }

  auto wnmap = new WinMap(session);
  wnmap->show();

  return a.exec();
}