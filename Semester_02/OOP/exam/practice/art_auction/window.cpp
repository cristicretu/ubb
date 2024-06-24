#include "window.h"

Window::Window(Session& session, int userId, QWidget* parent)
    : QWidget(parent), session(session), userId(userId) {
  session.registerObserver(this);
  auto users = session.getUsers();
  auto user = std::find_if(
      users.begin(), users.end(),
      [userId](const User& user) { return user.getId() == userId; });
  setWindowTitle(QString::fromStdString(user->getName()));
  setFixedSize(400, 200);

  QVBoxLayout* layout = new QVBoxLayout(this);

  QLabel* label = new QLabel("Items", this);
  layout->addWidget(label);

  setLayout(layout);
}

void Window::update() {}