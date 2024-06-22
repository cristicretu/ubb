#include "window.h"

Window::Window(Session &session, Biologist &biologist, QWidget *parent)
    : session(session), biologist(biologist), QWidget(parent) {
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(biologist.get_name()));

  QVBoxLayout *layout = new QVBoxLayout(this);
}

void Window::update() const {}