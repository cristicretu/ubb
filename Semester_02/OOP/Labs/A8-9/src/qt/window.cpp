#include "../../headers/qt/window.h"

#include <QtGui/qaction.h>

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QToolBar>
#include <QtCharts/QPieSeries>

#include "../../headers/repository/csv_repository.h"
#include "../../headers/repository/db_repository.h"
#include "../../headers/repository/html_repository.h"

DogApp::DogApp(QWidget* parent)
    : QMainWindow(parent), service(new Service()), chart(nullptr) {
  setWindowTitle("Dog Adoption");

  dogTypeRepository = DogTypeRepository();
  dogDetails = new QTextBrowser();

  QTabWidget* tabWidget = new QTabWidget(this);

  setupToolbar();

  // Tab 1
  QWidget* dogListWidget = adminTab();
  QWidget* tab1 = new QWidget();
  QVBoxLayout* layout1 = new QVBoxLayout();
  QLabel* label1 = new QLabel("Admin Mode");

  layout1->addWidget(label1);
  layout1->addWidget(dogListWidget);
  tab1->setLayout(layout1);

  // Tab 2
  QWidget* tab2 = new QWidget();
  layout2 = new QVBoxLayout();
  userButtons = createUserButtons();
  layout2->addLayout(userButtons);

  currentStack = new QStackedWidget();
  layout2->addWidget(currentStack);
  tab2->setLayout(layout2);

  // Tab 3
  QWidget* tab3 = new QWidget();
  layout3 = new QVBoxLayout();
  QLabel* label3 = new QLabel("Dog charts");
  layout3->addWidget(label3);
  tab3->setLayout(layout3);

  auto updateChart = [this]() {
    if (chart != nullptr) {
      layout3->removeWidget(chart);
      delete chart;
    }

    chart = chartDogs();
    layout3->addWidget(chart);
  };

  auto onTabChange = [this, updateChart](int index) {
    if (index == 2) {
      updateChart();
    }
  };

  tabWidget->addTab(tab1, "Admin");
  tabWidget->addTab(tab2, "User");
  tabWidget->addTab(tab3, "Chart");

  connect(tabWidget, &QTabWidget::currentChanged, onTabChange);

  setCentralWidget(tabWidget);
}

void DogApp::populateTable() {
  int numDogs = service->getNumberOfDogs();
  dogTable->setRowCount(numDogs);

  for (int i = 0; i < numDogs; ++i) {
    Dog dog = service->getDog(i);
    dogTable->setItem(
        i, 0, new QTableWidgetItem(QString::fromStdString(dog.getName())));
    dogTable->setItem(
        i, 1, new QTableWidgetItem(QString::fromStdString(dog.getBreed())));
    dogTable->setItem(i, 2,
                      new QTableWidgetItem(QString::number(dog.getAge())));

    QTableWidgetItem* linkItem = new QTableWidgetItem("Link");
    linkItem->setToolTip(QString::fromStdString(dog.getPhotograph()));
    linkItem->setFlags(linkItem->flags() | Qt::ItemIsEditable);
    dogTable->setItem(i, 3, linkItem);
  }

  connect(dogTable, &QTableWidget::cellClicked, this,
          &DogApp::expandCellWithUrl);
  connect(dogTable, &QTableWidget::cellDoubleClicked, this, &DogApp::openLink);

  if (numDogs == 0) {
    dogTable->setRowCount(1);
    dogTable->setItem(0, 0, new QTableWidgetItem("No dogs in the database!"));
    dogTable->setSpan(0, 0, 1, 4);
  }
}

void DogApp::expandCellWithUrl(int row, int column) {
  if (column == 3) {
    QTableWidgetItem* item = dogTable->item(row, column);
    if (item) {
      dogTable->blockSignals(true);
      QString url =
          QString::fromStdString(service->getDog(row).getPhotograph());
      item->setText(url);
      item->setToolTip(url);
      dogTable->blockSignals(false);
    }
  }
}

void DogApp::openLink(int row, int column) {
  if (column == 3) {
    QTableWidgetItem* item = dogTable->item(row, column);
    QString url = item->toolTip();
    QProcess::startDetached("open", QStringList() << url);
  }
}

QWidget* DogApp::adminTab() {
  QWidget* dogListWidget = new QWidget();
  QVBoxLayout* dogListLayout = new QVBoxLayout(dogListWidget);

  dogTable = new QTableWidget();

  QPushButton* addButton = new QPushButton("Add");
  connect(addButton, &QPushButton::clicked, this, &DogApp::showAddDogPopup);
  dogListLayout->addWidget(addButton);

  QPushButton* removeButton = new QPushButton("Remove");
  connect(removeButton, &QPushButton::clicked, this, &DogApp::removeDog);
  dogListLayout->addWidget(removeButton);

  dogTable->setColumnCount(4);
  dogTable->setHorizontalHeaderLabels(QStringList() << "Name" << "Breed"
                                                    << "Age" << "Photograph");
  dogListLayout->addWidget(dogTable);

  dogTable->blockSignals(true);
  populateTable();
  dogTable->blockSignals(false);

  connect(dogTable, &QTableWidget::itemChanged, this,
          &DogApp::updateDogFromTable);

  return dogListWidget;
}

void DogApp::updateDogFromTable(QTableWidgetItem* item) {
  QTableWidget* dogTable = qobject_cast<QTableWidget*>(item->tableWidget());

  if (!dogTable) return;

  int row = item->row();
  int column = item->column();
  if (column == 3) {
    dogTable->blockSignals(true);
    QString newUrl = item->text();
    item->setToolTip(newUrl);
    item->setText("Link");
    dogTable->blockSignals(false);
  }
  Dog dog = service->getDog(row);

  QString initialName = QString::fromStdString(dog.getName());
  QString initialBreed = QString::fromStdString(dog.getBreed());
  int initialAge = dog.getAge();
  QString initialPhotograph = QString::fromStdString(dog.getPhotograph());

  int index =
      service->findDog(initialBreed.toStdString(), initialName.toStdString(),
                       initialAge, initialPhotograph.toStdString());

  QString name = dogTable->item(row, 0)->text();
  QString breed = dogTable->item(row, 1)->text();
  QString age = dogTable->item(row, 2)->text();
  QString photograph = dogTable->item(row, 3)->toolTip();

  std::cout << "I am updating dog " << index << " " << breed.toStdString()
            << " " << name.toStdString() << " " << age.toInt() << " "
            << photograph.toStdString() << "\n";

  if (!validateAndUpdateDog(breed, name, age, photograph, index)) {
    dogTable->item(row, 0)->setText(initialName);
    dogTable->item(row, 1)->setText(initialBreed);
    dogTable->item(row, 2)->setText(QString::number(initialAge));
    dogTable->item(row, 3)->setText(initialPhotograph);
  }
}

bool DogApp::validateAndUpdateDog(const QString& breed, const QString& name,
                                  const QString& age, const QString& photograph,
                                  int index) {
  std::string breedStr = breed.toStdString();
  std::string nameStr = name.toStdString();
  int ageInt;
  try {
    ageInt = std::stoi(age.toStdString());
  } catch (std::exception& e) {
    QMessageBox::warning(this, "Error", "Invalid age format!");
    return false;
  }

  std::string breedToLowercase = "";
  for (char c : breedStr) {
    breedToLowercase += std::tolower(c);
  }

  std::string photographStr = photograph.toStdString();
  // if (!photographStr.empty()) {
  //   photographStr = "https://images.dog.ceo/breeds/" + breedToLowercase + "/"
  //   +
  //                   photographStr;
  // }

  std::string newBreed = breedStr;
  std::string newName = nameStr;
  int newAge = ageInt;
  std::string newPhotograph = photographStr;

  try {
    this->service->updateDog(index, newBreed, newName, newAge, newPhotograph);
    QMessageBox::information(this, "Success", "Dog updated successfully!");
    return true;
  } catch (std::invalid_argument& e) {
    QMessageBox::warning(this, "Error", e.what());
    return false;
  }
}

void DogApp::showAddDogPopup() {
  QDialog dialog;
  dialog.setWindowTitle("Add Dog");

  QVBoxLayout* layout = new QVBoxLayout(&dialog);

  QLabel* breedLabel = new QLabel("Breed:");
  QLineEdit* breedInput = new QLineEdit();
  layout->addWidget(breedLabel);
  layout->addWidget(breedInput);

  QLabel* nameLabel = new QLabel("Name:");
  QLineEdit* nameInput = new QLineEdit();
  layout->addWidget(nameLabel);
  layout->addWidget(nameInput);

  QLabel* ageLabel = new QLabel("Age:");
  QLineEdit* ageInput = new QLineEdit();
  layout->addWidget(ageLabel);
  layout->addWidget(ageInput);

  QLabel* photographLabel = new QLabel("Photograph:");
  QLineEdit* photographInput = new QLineEdit();
  layout->addWidget(photographLabel);
  layout->addWidget(photographInput);

  QPushButton* addButton = new QPushButton("Add Dog");
  connect(addButton, &QPushButton::clicked, this,
          [this, breedInput, nameInput, ageInput, photographInput]() {
            validateAndAddDog(breedInput->text(), nameInput->text(),
                              ageInput->text(), photographInput->text());
          });
  layout->addWidget(addButton);

  dialog.exec();
}

void DogApp::setupToolbar() {
  QToolBar* toolbar = new QToolBar("Repositories", this);

  QLabel* dogsRepoLabel = new QLabel("Dogs Repo:");
  dogsRepoCombo = new QComboBox();
  dogsRepoCombo->addItem("TXT");
  dogsRepoCombo->addItem("SQL");

  QLabel* adoptedDogsRepoLabel = new QLabel("Adopted Dogs Repo:");
  adoptedDogsRepoCombo = new QComboBox();
  adoptedDogsRepoCombo->addItem("CSV");
  adoptedDogsRepoCombo->addItem("HTML");

  toolbar->addWidget(dogsRepoLabel);
  toolbar->addWidget(dogsRepoCombo);
  toolbar->addSeparator();
  toolbar->addWidget(adoptedDogsRepoLabel);
  toolbar->addWidget(adoptedDogsRepoCombo);

  this->switchDogRepo(-1);
  this->switchAdoptedDogRepo(0);

  connect(dogsRepoCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &DogApp::switchDogRepo);
  connect(adoptedDogsRepoCombo,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &DogApp::switchAdoptedDogRepo);

  addToolBar(Qt::BottomToolBarArea, toolbar);
}

void DogApp::switchDogRepo(int index) {
  if (index == -1) {
    service->setRepository(new FileRepository("../dogs.txt"));
    return;
  }
  if (index == 0) {
    service->setRepository(new FileRepository("../dogs.txt"));
  } else {
    service->setRepository(new DBRepository("../dogs.db"));
  }
  dogTable->blockSignals(true);
  populateTable();
  dogTable->blockSignals(false);
}

void DogApp::switchAdoptedDogRepo(int index) {
  if (index == 0) {
    service->setAdoptedRepository(new CSVRepository("../adopted.csv"));
  } else {
    service->setAdoptedRepository(new HTMLRepository("../adopted.html"));
  }
}

QVBoxLayout* DogApp::createUserButtons() {
  QVBoxLayout* userLayout = new QVBoxLayout();

  QPushButton* adoptButton = new QPushButton("Adopt Dog");
  QPushButton* filterButton = new QPushButton("Filter Dogs by Breed and Age");
  QPushButton* adoptedListButton = new QPushButton("List Adopted Dogs");
  QPushButton* openFileButton = new QPushButton("Open Adopted File");

  userLayout->addWidget(adoptButton);
  userLayout->addWidget(filterButton);
  userLayout->addWidget(adoptedListButton);
  userLayout->addWidget(openFileButton);

  connect(adoptButton, &QPushButton::clicked, this, &DogApp::adoptDogMenu);
  connect(filterButton, &QPushButton::clicked, this, &DogApp::filterDogs);
  connect(adoptedListButton, &QPushButton::clicked, this,
          &DogApp::listAdoptedDogs);
  connect(openFileButton, &QPushButton::clicked, this,
          &DogApp::openAdoptedFile);

  return userLayout;
}

void DogApp::hideMainMenuButtons() {
  for (int i = 0; i < userButtons->count(); i++) {
    QWidget* widget = userButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(false);
    }
  }
}

void DogApp::hideOrShowUserButtons(bool show) {
  for (int i = 0; i < userButtons->count(); i++) {
    QWidget* widget = userButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(show);
    }
  }
}

QWidget* DogApp::chartDogs() {
  std::unordered_map<std::string, int> breedCounts;
  auto dogs = service->getDogs();

  for (const Dog& dog : dogs) {
    breedCounts[dog.getBreed()]++;
  }

  QPieSeries* pieSeries = new QPieSeries();

  for (const auto& pair : breedCounts) {
    pieSeries->append(QString::fromStdString(pair.first), pair.second);
  }

  for (auto slice : pieSeries->slices()) {
    slice->setLabel(QString("%1: %2").arg(slice->label()).arg(slice->value()));
    slice->setLabelVisible(true);
  }

  QChart* chart = new QChart();
  chart->addSeries(pieSeries);
  chart->setTitle("Dog Breed Distribution");
  chart->setAnimationOptions(QChart::AllAnimations);

  QChartView* chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);

  return chartView;
}

void DogApp::validateAndAddDog(const QString& breed, const QString& name,
                               const QString& age, const QString& photograph) {
  std::string breedStr = breed.toStdString();
  std::string nameStr = name.toStdString();
  int ageInt;
  try {
    ageInt = std::stoi(age.toStdString());
  } catch (std::exception& e) {
    QMessageBox::warning(this, "Error", "Invalid age format!");
    return;
  }

  if (ageInt < 0) {
    QMessageBox::warning(this, "Error", "Age cannot be negative!");
    return;
  }

  if (this->dogTypeRepository.findBreed(breedStr) == -1) {
    QMessageBox::warning(this, "Error", "Invalid breed!");
    return;
  }

  std::string photographStr = photograph.toStdString();

  if (photographStr.empty()) {
    auto [photoUrl, success] = this->dogTypeRepository.getDogImageUrl(breedStr);
    if (success) {
      photographStr = photoUrl;
    } else {
      photographStr =
          "https://images.dog.ceo/breeds/hound-basset/n02088238_10473.jpg";
    }
  }

  try {
    (*this->service).addDog(breedStr, nameStr, ageInt, photographStr);
    QMessageBox::information(this, "Success", "Dog added successfully!");
    dogTable->blockSignals(true);
    populateTable();
    dogTable->blockSignals(false);

  } catch (std::invalid_argument& e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void DogApp::findAndPopulateDog(const QString& breed, const QString& name,
                                const QString& age, const QString& photograph,
                                QLineEdit* breedInput, QLineEdit* nameInput,
                                QLineEdit* ageInput,
                                QLineEdit* photographInput) {
  std::string breedStr = breed.toStdString();
  std::string nameStr = name.toStdString();
  int ageInt;
  try {
    ageInt = std::stoi(age.toStdString());
  } catch (std::exception& e) {
    QMessageBox::warning(this, "Error", "Invalid age format!");
    return;
  }

  std::string breedToLowercase = "";
  for (char c : breedStr) {
    breedToLowercase += std::tolower(c);
  }

  std::string photographStr = photograph.toStdString();
  if (!photographStr.empty()) {
    photographStr = "https://images.dog.ceo/breeds/" + breedToLowercase + "/" +
                    photographStr;
  }

  int index = this->service->findDog(breedStr, nameStr, ageInt, photographStr);

  if (index == -1) {
    QMessageBox::warning(this, "Error", "Dog not found!");
    return;
  }

  Dog dog = this->service->getDog(index);

  breedInput->setText(QString::fromStdString(dog.getBreed()));
  nameInput->setText(QString::fromStdString(dog.getName()));
  ageInput->setText(QString::number(dog.getAge()));
  photographInput->setText(QString::fromStdString(dog.getPhotograph()));
}

void DogApp::removeDog() {
  int row = dogTable->currentRow();

  auto breedItem = dogTable->item(row, 1);
  auto nameItem = dogTable->item(row, 0);
  auto ageItem = dogTable->item(row, 2);
  auto photographItem = dogTable->item(row, 3)->toolTip();

  int index = service->findDog(
      breedItem->text().toStdString(), nameItem->text().toStdString(),
      ageItem->text().toInt(), photographItem.toStdString());

  if (row != -1 && index != -1) {
    service->removeDog(index);
    dogTable->blockSignals(true);
    populateTable();
    dogTable->blockSignals(false);
  }
}

void DogApp::validateAndRemoveDog(const QString& breed, const QString& name,
                                  const QString& age,
                                  const QString& photograph) {
  std::string breedStr = breed.toStdString();
  std::string nameStr = name.toStdString();
  int ageInt;
  try {
    ageInt = std::stoi(age.toStdString());
  } catch (std::exception& e) {
    QMessageBox::warning(this, "Error", "Invalid age format!");
    return;
  }

  std::string breedToLowercase = "";
  for (char c : breedStr) {
    breedToLowercase += std::tolower(c);
  }

  std::string photographStr = photograph.toStdString();
  if (!photographStr.empty()) {
    photographStr = "https://images.dog.ceo/breeds/" + breedToLowercase + "/" +
                    photographStr;
  }

  int index = this->service->findDog(breedStr, nameStr, ageInt, photographStr);

  try {
    this->service->removeDog(index);
    QMessageBox::information(this, "Success", "Dog removed successfully!");
  } catch (std::invalid_argument& e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void DogApp::adoptDogMenu() {
  hideMainMenuButtons();
  auto dogs = service->getDogs();
  currentDogIndex = 0;

  if (dogs.empty()) {
    QMessageBox::warning(this, "Info", "No dogs in the database!");
    return;
  }

  QWidget* adoptWidget = new QWidget();
  QVBoxLayout* adoptLayout = new QVBoxLayout(adoptWidget);

  QPushButton* backButton = new QPushButton("Back to Menu");

  adoptLayout->addWidget(dogDetails);

  QPushButton* adoptButton = new QPushButton("Adopt");
  QPushButton* skipButton = new QPushButton("Skip");

  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addWidget(adoptButton);
  buttonLayout->addWidget(skipButton);
  buttonLayout->addWidget(backButton);

  adoptLayout->addLayout(buttonLayout);

  connect(adoptButton, &QPushButton::clicked, this, &DogApp::adoptCurrentDog);
  connect(skipButton, &QPushButton::clicked, this, &DogApp::nextDog);
  connect(backButton, &QPushButton::clicked, [=]() {
    currentStack->removeWidget(adoptWidget);
    hideOrShowUserButtons(true);
  });

  displayCurrentDogDetails();

  currentStack->addWidget(adoptWidget);
  currentStack->setCurrentWidget(adoptWidget);
}

void DogApp::adoptCurrentDog() {
  auto dogs = service->getDogs();
  Dog dog = dogs[currentDogIndex];
  int index = service->findDog(dog.getBreed(), dog.getName(), dog.getAge(),
                               dog.getPhotograph());

  if (index != -1) {
    service->adoptDog(index);
    dogs.erase(dogs.begin() + currentDogIndex);
    QMessageBox::information(this, "Success", "Dog adopted successfully!");
  }

  if (dogs.empty()) {
    QMessageBox::warning(this, "Info", "No more dogs available.");
    return;
  }

  nextDog();
}

void DogApp::nextDog() {
  auto dogs = service->getDogs();
  if (++currentDogIndex >= dogs.size()) {
    currentDogIndex = 0;
  }

  displayCurrentDogDetails();
}

void DogApp::displayCurrentDogDetails() {
  auto dogs = service->getDogs();
  if (dogs.empty()) {
    return;
  }

  Dog dog = dogs[currentDogIndex];
  QString details = QString("Name: %1\nBreed: %2\nAge: %3\nPhotograph: %4")
                        .arg(QString::fromStdString(dog.getName()))
                        .arg(QString::fromStdString(dog.getBreed()))
                        .arg(dog.getAge())
                        .arg(QString::fromStdString(dog.getPhotograph()));

  dogDetails->setText(details);
}

void DogApp::filterDogs() {
  hideMainMenuButtons();
  QWidget* filterWidget = new QWidget();
  QVBoxLayout* filterLayout = new QVBoxLayout(filterWidget);

  QLabel* breedLabel = new QLabel("Breed:");
  QLineEdit* breedInput = new QLineEdit();
  filterLayout->addWidget(breedLabel);
  filterLayout->addWidget(breedInput);

  QLabel* ageLabel = new QLabel("Age:");
  QLineEdit* ageInput = new QLineEdit();
  filterLayout->addWidget(ageLabel);
  filterLayout->addWidget(ageInput);

  QPushButton* searchButton = new QPushButton("Search");
  filterLayout->addWidget(searchButton);

  QTableWidget* filteredDogsTable = new QTableWidget();
  filteredDogsTable->setColumnCount(5);
  filteredDogsTable->setHorizontalHeaderLabels(
      QStringList() << "ID" << "Name" << "Breed" << "Age" << "Photograph");
  filterLayout->addWidget(filteredDogsTable);

  QPushButton* backButton = new QPushButton("Back to Menu");
  filterLayout->addWidget(backButton);

  connect(searchButton, &QPushButton::clicked, [=]() {
    QString breed = breedInput->text();
    bool ageOk;
    int age = ageInput->text().toInt(&ageOk);

    if (!ageOk || age < 0) {
      QMessageBox::warning(this, "Error", "Invalid age!");
      return;
    }

    auto dogs = service->filterDogs(breed.toStdString(), age);

    filteredDogsTable->setRowCount(dogs.size());

    if (dogs.empty()) {
      filteredDogsTable->setRowCount(1);
      QTableWidgetItem* noDogsItem = new QTableWidgetItem("No dogs found!");
      noDogsItem->setFlags(noDogsItem->flags() & ~Qt::ItemIsEditable);
      filteredDogsTable->setSpan(0, 0, 1, 5);
    } else {
      for (int i = 0; i < dogs.size(); ++i) {
        Dog dog = dogs[i];
        filteredDogsTable->setItem(
            i, 0, new QTableWidgetItem(QString::number(i + 1)));
        filteredDogsTable->setItem(
            i, 1, new QTableWidgetItem(QString::fromStdString(dog.getName())));
        filteredDogsTable->setItem(
            i, 2, new QTableWidgetItem(QString::fromStdString(dog.getBreed())));
        filteredDogsTable->setItem(
            i, 3, new QTableWidgetItem(QString::number(dog.getAge())));

        QTableWidgetItem* linkItem = new QTableWidgetItem("Link");
        linkItem->setToolTip(QString::fromStdString(dog.getPhotograph()));
        linkItem->setFlags(linkItem->flags() & ~Qt::ItemIsEditable);
        filteredDogsTable->setItem(i, 4, linkItem);
      }
    }
  });

  connect(filteredDogsTable, &QTableWidget::cellDoubleClicked, this,
          [=](int row, int column) {
            if (column == 4) {
              QTableWidgetItem* item = filteredDogsTable->item(row, column);
              QString url = item->toolTip();
              QProcess::startDetached("open", QStringList() << url);
            }
          });

  connect(backButton, &QPushButton::clicked, [=]() {
    currentStack->removeWidget(filterWidget);
    hideOrShowUserButtons(true);
  });

  displayCurrentDogDetails();

  currentStack->addWidget(filterWidget);
  currentStack->setCurrentWidget(filterWidget);
}

void DogApp::listAdoptedDogs() {
  hideMainMenuButtons();

  QWidget* dogListWidget = new QWidget();
  QVBoxLayout* dogListLayout = new QVBoxLayout(dogListWidget);

  QTableWidget* adoptedDogsTable = new QTableWidget();
  adoptedDogsTable->setColumnCount(5);
  adoptedDogsTable->setHorizontalHeaderLabels(
      QStringList() << "ID" << "Name" << "Breed" << "Age" << "Photograph");
  dogListLayout->addWidget(adoptedDogsTable);

  QPushButton* backButton = new QPushButton("Back to Menu");
  dogListLayout->addWidget(backButton);

  auto adoptedDogs = service->getAdoptedDogs();

  adoptedDogsTable->setRowCount(adoptedDogs.size());

  if (adoptedDogs.empty()) {
    adoptedDogsTable->setRowCount(1);
    QTableWidgetItem* noDogsItem = new QTableWidgetItem("No dogs found!");
    noDogsItem->setFlags(noDogsItem->flags() & ~Qt::ItemIsEditable);
    adoptedDogsTable->setItem(0, 0, noDogsItem);
    adoptedDogsTable->setSpan(0, 0, 1, 5);
  } else {
    for (int i = 0; i < adoptedDogs.size(); ++i) {
      Dog dog = adoptedDogs[i];
      adoptedDogsTable->setItem(i, 0,
                                new QTableWidgetItem(QString::number(i + 1)));
      adoptedDogsTable->setItem(
          i, 1, new QTableWidgetItem(QString::fromStdString(dog.getName())));
      adoptedDogsTable->setItem(
          i, 2, new QTableWidgetItem(QString::fromStdString(dog.getBreed())));
      adoptedDogsTable->setItem(
          i, 3, new QTableWidgetItem(QString::number(dog.getAge())));

      QTableWidgetItem* linkItem = new QTableWidgetItem("Link");
      linkItem->setToolTip(QString::fromStdString(dog.getPhotograph()));
      linkItem->setFlags(linkItem->flags() & ~Qt::ItemIsEditable);
      adoptedDogsTable->setItem(i, 4, linkItem);
    }
  }

  connect(adoptedDogsTable, &QTableWidget::cellDoubleClicked, this,
          [=](int row, int column) {
            if (column == 4) {
              QTableWidgetItem* item = adoptedDogsTable->item(row, column);
              QString url = item->toolTip();
              QProcess::startDetached("open", QStringList() << url);
            }
          });

  connect(backButton, &QPushButton::clicked, [=]() {
    currentStack->removeWidget(dogListWidget);
    hideOrShowUserButtons(true);
  });

  displayCurrentDogDetails();

  currentStack->addWidget(dogListWidget);
  currentStack->setCurrentWidget(dogListWidget);
}

void DogApp::openAdoptedFile() {
  std::string filename =
      adoptedDogsRepoCombo->currentText().toStdString() == "CSV"
          ? "../adopted.csv"
          : "../adopted.html";
  this->service->openAdoptedFile(filename);
}