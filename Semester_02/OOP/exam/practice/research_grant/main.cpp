#include <QApplication>
#include <QSortFilterProxyModel>

#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Session session(repo);

  for (auto r : repo.getResearchers()) {
    std::cout << r.getName() << std::endl;
    QSortFilterProxyModel *filterModel = new QSortFilterProxyModel();
    filterModel->setSourceModel(new IdeaTableModel(repo));
    filterModel->setFilterKeyColumn(3);
    filterModel->setFilterFixedString(QString::fromStdString(r.getName()));
    Window *w = new Window(filterModel, session, r);
    w->show();
  }
  return a.exec();
}