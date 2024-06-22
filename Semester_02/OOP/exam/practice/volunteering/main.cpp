#include <QApplication>

#include "window.h"

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  Repository repo;

  Service service(repo);

  for (Department department : service.getDepartments()) {
    // std::cout << department.getName() << std::endl;
    Window *window = new Window(service, department);
    window->show();
  }

  return a.exec();
}