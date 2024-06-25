#include <QApplication>

#include "repository.h"
#include "session.h"
#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Session session(repo);

  for (auto x : repo.getPersons()) {
    cout << x.getName() << " " << x.getLatitude() << " " << x.getLongitude()
         << " " << x.getStatus() << endl;
  }

  return a.exec();
}