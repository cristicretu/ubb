#include "window.h"

Window::Window(Session &session, Biologist &biologist, QWidget *parent)
    : session(session), biologist(biologist), QWidget(parent) {
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(biologist.get_name()));
}

void Window::update() const {}