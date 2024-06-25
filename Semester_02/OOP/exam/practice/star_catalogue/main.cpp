#include <QApplication>

#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  Repository repo;
  Session session(repo);

  for (auto x : repo.getAstronomers()) {
    std::cout << x.getName() << " " << x.getConstellation() << std::endl;
  }
  return a.exec();
}