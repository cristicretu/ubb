#include "window.h"

#include <iostream>

Window::Window(Session &session, Biologist &biologist, QWidget *parent)
    : session(session), biologist(biologist), QWidget(parent) {
  std::cout << "Window constructor\n";
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(biologist.get_name()));

  auto *layout = new QVBoxLayout;

  bacteriaTable = new QTableView(this);

  speciesComboBox = new QComboBox(this);

  auto inputLayout = new QHBoxLayout;

  nameLabel = new QLabel("Name:", this);
  nameLineEdit = new QLineEdit(this);
  inputLayout->addWidget(nameLabel);
  inputLayout->addWidget(nameLineEdit);

  speciesLabel = new QLabel("Species:", this);
  speciesLineEdit = new QLineEdit(this);
  inputLayout->addWidget(speciesLabel);
  inputLayout->addWidget(speciesLineEdit);

  sizeLabel = new QLabel("Size:", this);
  sizeLineEdit = new QLineEdit(this);
  inputLayout->addWidget(sizeLabel);
  inputLayout->addWidget(sizeLineEdit);

  diseasesLabel = new QLabel("Diseases:", this);
  diseasesLineEdit = new QLineEdit(this);
  inputLayout->addWidget(diseasesLabel);
  inputLayout->addWidget(diseasesLineEdit);

  layout->addWidget(bacteriaTable);
  layout->addWidget(speciesComboBox);
  layout->addLayout(inputLayout);

  addButton = new QPushButton("Add", this);

  layout->addWidget(addButton);

  setLayout(layout);

  update();
}

void Window::update() const {}