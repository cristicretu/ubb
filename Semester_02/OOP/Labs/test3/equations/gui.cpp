#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout();

  mainItems = new QListWidget();
  a = new QLineEdit();
  b = new QLineEdit();
  c = new QLineEdit();
  updateBtn = new QPushButton("Update");
  computeSolutions = new QPushButton("Compute Solution");

  mainLayout->addWidget(mainItems);
  mainLayout->addWidget(a);
  mainLayout->addWidget(b);
  mainLayout->addWidget(c);
  mainLayout->addWidget(updateBtn);
  mainLayout->addWidget(computeSolutions);

  setLayout(mainLayout);
}