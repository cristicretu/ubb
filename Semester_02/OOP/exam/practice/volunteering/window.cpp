#include "window.h"

Window::Window(Service &service, Department department, QWidget *parent)
    : service{service}, department{department}, QWidget{parent} {
  service.registerObserver(this);
  setWindowTitle(QString::fromStdString(department.getName()));

  QVBoxLayout *layout = new QVBoxLayout{this};

  description = new QLabel{QString::fromStdString(department.getDescription())};
  layout->addWidget(description);
}

void Window::update() {}

void Window::addVolunteer() {}

void Window::assignVolunteer() {}

void Window::displayMostSuitable() {}