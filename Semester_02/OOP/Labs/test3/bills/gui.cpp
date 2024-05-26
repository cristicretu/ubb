#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();
  showPaid = new QCheckBox("Show paid");
  showNotPaid = new QCheckBox("Show not paid");
  companyInput = new QLineEdit();
  totalButton = new QPushButton("Total");

  mainLayout->addWidget(mainItems);
  QHBoxLayout *filterLayout = new QHBoxLayout();
  filterLayout->addWidget(showPaid);
  filterLayout->addWidget(showNotPaid);
  mainLayout->addLayout(filterLayout);
  mainLayout->addWidget(totalButton);

  setLayout(mainLayout);
  populateList();

  QObject::connect(showPaid, &QCheckBox::clicked, this, &GUI::populateList);
  QObject::connect(showNotPaid, &QCheckBox::clicked, this, &GUI::populateList);
  QObject::connect(totalButton, &QPushButton::clicked, this, &GUI::showTotal);
}

void GUI::populateList() {
  mainItems->clear();
  auto bills = service.getBills();

  for (const auto &bill : bills) {
    if (showPaid->isChecked() && !bill.getIsPaid()) continue;
    if (showNotPaid->isChecked() && bill.getIsPaid()) continue;

    auto item = new QListWidgetItem(QString::fromStdString(bill.toString()));
    if (!bill.getIsPaid()) item->setBackground(Qt::red);
    mainItems->addItem(item);
  }
}

void GUI::showTotal() {
  auto company = companyInput->text().toStdString();
  auto total = service.getTotal(company);

  if (total == -1) {
    QMessageBox::warning(this, "Error", "Company not found");
    return;
  }

  QMessageBox::information(this, "Total", "Total: " + QString::number(total));
}