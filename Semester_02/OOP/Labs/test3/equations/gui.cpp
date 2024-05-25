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

  populateItems();

  QObject::connect(mainItems, &QListWidget::itemSelectionChanged, this,
                   &GUI::populateFields);
  QObject::connect(updateBtn, &QPushButton::clicked, this,
                   &GUI::updateEquation);

  QObject::connect(computeSolutions, &QPushButton::clicked, this,
                   &GUI::showSolution);
}

void GUI::populateItems() {
  mainItems->clear();

  auto eq = service.getEquations();

  for (auto &e : eq) {
    QListWidgetItem *item = new QListWidgetItem(e.toString().c_str());
    if (e.hasRealRoots()) {
      QFont font = item->font();
      font.setBold(true);
      item->setFont(font);
    }
    mainItems->addItem(item);
  }
}

void GUI::populateFields() {
  auto items = mainItems->selectedItems();
  if (items.empty()) {
    return;
  }

  selectedIndex = mainItems->row(items[0]);

  int index = selectedIndex;
  auto eq = service.getEquations()[index];

  a->setText(QString::number(eq.getA()));
  b->setText(QString::number(eq.getB()));
  c->setText(QString::number(eq.getC()));
}

void GUI::updateEquation() {
  double aVal = a->text().toDouble();
  double bVal = b->text().toDouble();
  double cVal = c->text().toDouble();

  service.updateEquation(selectedIndex, Equation(aVal, bVal, cVal));

  populateItems();
}

void GUI::showSolution() {
  auto items = mainItems->selectedItems();
  if (items.empty()) {
    return;
  }

  int index = mainItems->row(items[0]);
  auto eq = service.getEquations()[index];

  QMessageBox msgBox;
  msgBox.setText(eq.solve().c_str());
  msgBox.exec();
}