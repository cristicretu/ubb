#include "window.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QStandardItem>
#include <iostream>

Window::Window(Session &session, Biologist &biologist, QWidget *parent)
    : session(session), biologist(biologist), QWidget(parent) {
  name = biologist.get_name();
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(biologist.get_name()));

  auto *layout = new QVBoxLayout;

  auto *horizontalLayout = new QHBoxLayout;

  speciesComboBox = new QComboBox(this);
  updateSpeciesComboBox();

  bacteriaTable = new QTableView(this);
  model = new QStandardItemModel(this);
  model->setColumnCount(4);
  model->setHorizontalHeaderLabels({"Name", "Species", "Size", "Diseases"});
  bacteriaTable->setModel(model);
  updateBacteriaTable();

  horizontalLayout->addWidget(bacteriaTable);

  bacteriumDiseasesList = new QListWidget(parent);
  horizontalLayout->addWidget(bacteriumDiseasesList);

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

  diseaseLabel = new QLabel("Disease:", this);
  diseaseLineEdit = new QLineEdit(this);

  addDiseaseButton = new QPushButton("Add Disease", this);

  layout->addWidget(diseaseLabel);
  layout->addWidget(diseaseLineEdit);
  layout->addWidget(addDiseaseButton);

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
            selectedBacteriumStr = name;
            bacteriumDiseasesList->clear();
            for (const auto &disease : bacterium.get_diseases()) {
              bacteriumDiseasesList->addItem(QString::fromStdString(disease));
            }
          });
  connect(speciesComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &Window::updateBacteriaTable);
  connect(addDiseaseButton, &QPushButton::clicked, [this, &session]() {
    auto disease = diseaseLineEdit->text().toStdString();

    if (disease.empty() || selectedBacteriumStr.empty()) {
      QMessageBox::warning(this, "Error", QString("Invalid disease"));
      return;
    }

    session.add_bacterium_disease(selectedBacteriumStr, disease);

    diseaseLineEdit->clear();
  });
  connect(model, &QStandardItemModel::dataChanged, this,
          &Window::updateBacterium);
}

void Window::updateBacterium() const {
  int row = bacteriaTable->currentIndex().row();
  auto name = model->item(row, 0)->text().toStdString();
  auto species = model->item(row, 1)->text().toStdString();
  auto size = model->item(row, 2)->text().toDouble();
  auto diseases = model->item(row, 3)->text().toStdString();

  try {
    session.update_bacterium(name, species, size, diseases);
  } catch (const std::runtime_error &e) {
    return;
  }
}

void Window::updateSpeciesComboBox() const {
  speciesComboBox->clear();
  speciesComboBox->addItem("All Species");
  for (const auto &species :
       session.get_species_by_biologist(biologist.get_name())) {
    speciesComboBox->addItem(QString::fromStdString(species));
  }
}

void Window::updateBacteriaTable() const {
  model->removeRows(0, model->rowCount());

  for (auto bacterium : session.get_bacteria_by_biologist(name)) {
    bool found = false;
    if (speciesComboBox->currentText() == "All Species" ||
        bacterium.get_spacies() ==
            speciesComboBox->currentText().toStdString()) {
      found = true;
    }

    if (!found) {
      continue;
    }
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
}

void Window::update() const {}

void Window::addBacterium() {
  auto name = nameLineEdit->text().toStdString();
  auto species = speciesLineEdit->text().toStdString();
  auto size = sizeLineEdit->text().toDouble();
  auto diseases = diseasesLineEdit->text().toStdString();

  try {
    session.add_bacterium(name, species, size, {diseases});

    nameLineEdit->clear();
    speciesLineEdit->clear();
    sizeLineEdit->clear();
    diseasesLineEdit->clear();

    updateBacteriaTable();

  } catch (const std::runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}
