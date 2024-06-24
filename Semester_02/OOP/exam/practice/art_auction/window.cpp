#include "window.h"

#include <set>

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

  combobox = new QComboBox(this);
  // populated  with the categories
  layout->addWidget(combobox);
  // populate the combobox with the categories
  auto items = session.getItems();
  set<string> categories;
  for (const auto& item : items) {
    categories.insert(item.getCategory());
  }

  combobox->addItem("All");
  for (const auto& category : categories) {
    combobox->addItem(QString::fromStdString(category));
  }

  update();

  setLayout(layout);

  QObject::connect(combobox,
                   QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                   &Window::update);
}

void Window::update() const {
  itemsList->clear();
  auto items = session.getItems();

  sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.getPrice() < b.getPrice();
  });
  for (const auto& item : items) {
    if (item.getCategory() == combobox->currentText().toStdString() ||
        combobox->currentText().toStdString() == "All") {
      itemsList->addItem(QString::fromStdString(item.toString()));
    }
  }
}