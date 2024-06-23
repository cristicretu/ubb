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
    // QSortFilterProxyModel *filterModel = new QSortFilterProxyModel();
    // filterModel->setSourceModel(new IdeaTableModel(repo));
    // filterModel->setFilterKeyColumn(3);
    // filterModel->setFilterFixedString(QString::fromStdString(r.getName()));
    // Window *w = new Window(filterModel, session, r);

    Window *w = new Window(new IdeaTableModel(repo), session, r);
    w->show();
  }
  return a.exec();
}