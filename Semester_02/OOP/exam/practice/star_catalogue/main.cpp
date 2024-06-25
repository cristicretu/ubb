#include <QApplication>

#include "repository.h"
#include "session.h"
#include "starItemModel.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  Repository repo;
  Session session(repo);

  auto *model = new starItemModel(repo);

  for (auto x : repo.getAstronomers()) {
    auto window = new Window(session, x, model);
    window->show();
  }
  return a.exec();
}