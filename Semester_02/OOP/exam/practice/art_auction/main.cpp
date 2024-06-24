#include <QApplication>

#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Session session(repo);

  for (auto& user : repo.getUsers()) {
    Window* window = new Window(session, user.getId());
    window->show();
  }

  return a.exec();
}