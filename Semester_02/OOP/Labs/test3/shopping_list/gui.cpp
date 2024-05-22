#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  service = Service();

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();

  mainLayout->addWidget(mainItems);

  setLayout(mainLayout);
  populateMainItems();
}

void GUI::populateMainItems() {
  mainItems->clear();
  std::vector<Item> items = service.getItems();

  for (auto item : items) {
    mainItems->addItem(QString::fromStdString(item.to_string()));
  }
}