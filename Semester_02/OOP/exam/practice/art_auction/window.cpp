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
  layout->addWidget(combobox);
  auto items = session.getItems();
  set<string> categories;
  for (const auto& item : items) {
    categories.insert(item.getCategory());
  }

  combobox->addItem("All");
  for (const auto& category : categories) {
    combobox->addItem(QString::fromStdString(category));
  }

  if (user->getType() == "admin") {
    auto hlayout = new QHBoxLayout();

    auto nameL = new QLabel("Name:", this);
    nameE = new QLineEdit(this);

    auto categoryL = new QLabel("Category:", this);
    categoryE = new QLineEdit(this);

    auto priceL = new QLabel("Price:", this);
    priceE = new QLineEdit(this);

    hlayout->addWidget(nameL);
    hlayout->addWidget(nameE);
    hlayout->addWidget(categoryL);
    hlayout->addWidget(categoryE);
    hlayout->addWidget(priceL);
    hlayout->addWidget(priceE);

    layout->addLayout(hlayout);
    addButton = new QPushButton("Add");
    layout->addWidget(addButton);

    QObject::connect(addButton, &QPushButton::clicked, this, &Window::addItem);
  }

  offersList = new QListWidget(this);

  layout->addWidget(offersList);

  update();

  setLayout(layout);

  QObject::connect(combobox,
                   QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                   &Window::update);

  QObject::connect(itemsList, &QListWidget::itemSelectionChanged, this,
                   &Window::selectItem);
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

void Window::addItem() {
  auto name = nameE->text().toStdString();
  auto cateogry = categoryE->text().toStdString();
  auto price = priceE->text().toStdString();

  try {
    if (name.empty() || cateogry.empty() || price.empty()) {
      throw runtime_error("Invalid item");
    }
    session.addItem(name, cateogry, stoi(price));

    nameE->clear();
    categoryE->clear();
    priceE->clear();

  } catch (runtime_error& e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void Window::selectItem() {
  auto row = itemsList->currentRow();
  std::cout << row << std::endl;
  if (row < 0) {
    return;
  }

  auto items = session.getItems();
  sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
    return a.getPrice() < b.getPrice();
  });
  auto item = items[row];
  offersList->clear();
  std::cout << item.toString() << std::endl;
  for (const auto& offer : item.getOffers()) {
    offersList->addItem(QString::fromStdString(to_string(get<0>(offer)) +
                                               " | " + get<1>(offer) + " | " +
                                               to_string(get<2>(offer))));
  }
}