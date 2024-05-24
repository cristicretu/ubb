#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();
  QLabel *label = new QLabel("Input field");
  inputField = new QLineEdit();
  showBestMatching = new QPushButton("Show best matching");

  mainLayout->addWidget(mainItems);
  mainLayout->addWidget(label);
  mainLayout->addWidget(inputField);
  mainLayout->addWidget(showBestMatching);

  setLayout(mainLayout);
  populateList();
}

void GUI::populateList() {
  mainItems->clear();

  std::vector<Document> documents = service.getAll();

  for (auto x : documents) {
    mainItems->addItem(QString::fromStdString(x.toString()));
  }
}