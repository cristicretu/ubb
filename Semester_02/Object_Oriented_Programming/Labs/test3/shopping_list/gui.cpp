#include "gui.h"

#include <iostream>

GUI::GUI(QWidget *parent) : QWidget(parent) {
  service = Service();

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();
  inputField = new QLineEdit();
  categoryField = new QLineEdit();
  searchButton = new QPushButton("Filter by category");
  QLabel *searchLabel = new QLabel("Search by category or name");
  QLabel *categoryLabel = new QLabel("Filter by category");

  mainLayout->addWidget(searchLabel);
  mainLayout->addWidget(inputField);
  mainLayout->addWidget(mainItems);
  mainLayout->addWidget(categoryLabel);
  mainLayout->addWidget(categoryField);
  mainLayout->addWidget(searchButton);

  setLayout(mainLayout);
  populateMainItems();

  QObject::connect(searchButton, &QPushButton::clicked, this,
                   &GUI::filterCategory);
  QObject::connect(inputField, &QLineEdit::textChanged, this,
                   &GUI::populateFilteredItems);
}

void GUI::populateMainItems() {
  mainItems->clear();
  std::vector<Item> items = service.getItems();

  for (auto item : items) {
    mainItems->addItem(QString::fromStdString(item.to_string()));
  }
}

void GUI::populateFilteredItems() {
  if (inputField->text().size() == 0) {
    populateMainItems();
    return;
  }

  std::string categoryOrName = inputField->text().toStdString();

  std::cout << "i got here!\n";

  mainItems->clear();
  std::vector<Item> filteredItems = service.search(categoryOrName);

  for (auto item : filteredItems) {
    mainItems->addItem(QString::fromStdString(item.to_string()));
  }
}

void GUI::filterCategory() {
  std::string category = categoryField->text().toStdString();
  if (category == "") {
    populateMainItems();
    return;
  }

  std::vector<Item> filteredItems = service.search(category);

  QWidget *filteredItemsWindow = new QWidget();
  QVBoxLayout *layout = new QVBoxLayout();
  QListWidget *filteredItemsList = new QListWidget();

  for (auto item : filteredItems) {
    filteredItemsList->addItem(QString::fromStdString(item.to_string()));
  }

  layout->addWidget(filteredItemsList);
  filteredItemsWindow->setLayout(layout);
  filteredItemsWindow->show();
}

void GUI::printItem() {
  int index = mainItems->currentRow();
  if (index == -1) {
    return;
  }

  std::vector<Item> items = service.getItems();
  Item item = items[index];
  std::string message = item.to_string();
  QMessageBox::information(this, "Item", QString::fromStdString(message));
}