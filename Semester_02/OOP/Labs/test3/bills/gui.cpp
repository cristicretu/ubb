#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();
  showPaid = new QCheckBox("Show paid");
  showNotPaid = new QCheckBox("Show not paid");
  totalButton = new QPushButton("Total");

  mainLayout->addWidget(mainItems);
  QHBoxLayout *filterLayout = new QHBoxLayout();
  filterLayout->addWidget(showPaid);
  filterLayout->addWidget(showNotPaid);
  mainLayout->addLayout(filterLayout);
  mainLayout->addWidget(totalButton);

  setLayout(mainLayout);
  populateList();
}