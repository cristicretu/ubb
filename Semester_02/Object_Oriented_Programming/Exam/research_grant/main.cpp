#include <QAbstractItemModel>
#include <QApplication>

#include "ideaModel.h"
#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;
  Service service(repo);

  auto *model = new ideaModel(repo);

  for (auto &x : repo.getResearchers()) {
    auto window = new Window(service, x, model);
    window->show();
  }
  return a.exec();
}