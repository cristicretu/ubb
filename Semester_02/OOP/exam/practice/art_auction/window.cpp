#include "window.h"

Window::Window(Session& session, int userId, QWidget* parent)
    : QWidget(parent), session(session), userId(userId) {
  session.registerObserver(this);
  auto users = session.getUsers();
  auto user = std::find_if(
      users.begin(), users.end(),
      [userId](const User& user) { return user.getId() == userId; });
  setWindowTitle(QString::fromStdString(user->getName()));

  QVBoxLayout* layout = new QVBoxLayout(this);

  QLabel* label = new QLabel("Items", this);
  layout->addWidget(label);

  itemsList = new QListWidget(this);

  layout->addWidget(itemsList);

  update();

  setLayout(layout);
}

void Window::update() const {
  itemsList->clear();
  auto items = session.getItems();

  sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.getPrice() < b.getPrice();
  });
  for (const auto& item : items) {
    itemsList->addItem(QString::fromStdString(item.toString()));
  }
}