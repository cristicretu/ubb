#include "window.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QStandardItem>
#include <iostream>

Window::Window(Session &session, Biologist &biologist, QWidget *parent)
    : session(session), biologist(biologist), QWidget(parent) {
  std::cout << "Window constructor\n";
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(biologist.get_name()));

  auto *layout = new QVBoxLayout;

  auto *horizontalLayout = new QHBoxLayout;

  bacteriaTable = new QTableView(this);
  auto *model = new QStandardItemModel(this);
  model->setColumnCount(4);
  model->setHeaderData(0, Qt::Horizontal, "Name");
  model->setHeaderData(1, Qt::Horizontal, "Species");
  model->setHeaderData(2, Qt::Horizontal, "Size");
  model->setHeaderData(3, Qt::Horizontal, "Diseases");

  for (const auto &bacterium :
       session.get_bacteria_by_biologist(biologist.get_name())) {
    QList<QStandardItem *> row;
    row << new QStandardItem(QString::fromStdString(bacterium.get_name()));
    row << new QStandardItem(QString::fromStdString(bacterium.get_spacies()));
    row << new QStandardItem(QString::number(bacterium.get_size()));
    std::string diseases;
    for (const auto &disease : bacterium.get_diseases()) {
      diseases += disease + " ";
    }
    row << new QStandardItem(QString::fromStdString(diseases));
    model->appendRow(row);
  }

  bacteriaTable->setModel(model);

  horizontalLayout->addWidget(bacteriaTable);

  bacteriumDiseasesList = new QListWidget(parent);
  horizontalLayout->addWidget(bacteriumDiseasesList);

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

  layout->addLayout(horizontalLayout);
  layout->addWidget(speciesComboBox);
  layout->addLayout(inputLayout);

  addButton = new QPushButton("Add", this);

  layout->addWidget(addButton);

  setLayout(layout);

  update();

  connect(addButton, &QPushButton::clicked, this, &Window::addBacterium);
  connect(bacteriaTable, &QTableView::clicked,
          [this, &session](const QModelIndex &index) {
            auto name =
                bacteriaTable->model()
                    ->data(bacteriaTable->model()->index(index.row(), 0))
                    .toString()
                    .toStdString();
            auto bacterium = session.get_bacterium_by_name(name);
            bacteriumDiseasesList->clear();
            for (const auto &disease : bacterium.get_diseases()) {
              bacteriumDiseasesList->addItem(QString::fromStdString(disease));
            }
          });
}

void Window::update() const {}

void Window::addBacterium() {
  auto name = nameLineEdit->text().toStdString();
  auto species = speciesLineEdit->text().toStdString();
  auto size = sizeLineEdit->text().toDouble();
  auto diseases = diseasesLineEdit->text().toStdString();

  try {
    session.add_bacterium(name, species, size, {diseases});

    std::cout << "Bacterium added\n";
    nameLineEdit->clear();
    speciesLineEdit->clear();
    sizeLineEdit->clear();
    diseasesLineEdit->clear();

  } catch (const std::runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}
