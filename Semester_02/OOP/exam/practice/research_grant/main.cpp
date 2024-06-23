#include <QApplication>

#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Session session(repo);

  for (auto r : repo.getResearchers()) {
    Window *w = new Window(new IdeaTableModel(repo), session, r);
    w->show();
  }
  return a.exec();
}