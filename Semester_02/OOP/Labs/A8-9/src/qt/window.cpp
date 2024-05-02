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

DogApp::DogApp(QWidget* parent) : QMainWindow(parent), service(new Service()) {
  setWindowTitle("Dog Adoption");

  dogTypeRepository = DogTypeRepository();

  QWidget* centralWidget = new QWidget(this);
  QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
  dogDetails = new QTextBrowser();

  currentStack = new QStackedWidget();

  setupToolbar();

  QWidget* mainMenuWidget = new QWidget();
  QVBoxLayout* mainMenuLayout = new QVBoxLayout(mainMenuWidget);

  QHBoxLayout* modeLayout = new QHBoxLayout();
  modeCombo = new QComboBox();
  modeCombo->addItem("Administrator mode");
  modeCombo->addItem("User mode");
  modeLayout->addWidget(new QLabel("Select Mode:"));
  modeLayout->addWidget(modeCombo);
  mainLayout->addLayout(modeLayout);

  adminButtons = createAdminButtons();
  userButtons = createUserButtons();

  mainLayout->addLayout(adminButtons);
  mainLayout->addLayout(userButtons);

  currentStack->addWidget(mainMenuWidget);
  mainLayout->addWidget(currentStack);

  connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &DogApp::switchMode);
  switchMode(0);

  setCentralWidget(centralWidget);
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

  this->switchDogRepo(0);
  this->switchAdoptedDogRepo(0);

  connect(dogsRepoCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &DogApp::switchDogRepo);
  connect(adoptedDogsRepoCombo,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &DogApp::switchAdoptedDogRepo);

  addToolBar(Qt::BottomToolBarArea, toolbar);
}

void DogApp::switchDogRepo(int index) {
  if (index == 0) {
    service->setRepository(new FileRepository("../dogs.txt"));
  } else {
    service->setRepository(new DBRepository("../dogs.db"));
  }
}

void DogApp::switchAdoptedDogRepo(int index) {
  if (index == 0) {
    service->setAdoptedRepository(new CSVRepository("../adopted.csv"));
  } else {
    service->setAdoptedRepository(new HTMLRepository("../adopted.html"));
  }
}

QVBoxLayout* DogApp::createAdminButtons() {
  QVBoxLayout* adminLayout = new QVBoxLayout();

  QPushButton* addButton = new QPushButton("Add Dog");
  QPushButton* updateButton = new QPushButton("Update Dog");
  QPushButton* removeButton = new QPushButton("Remove Dog");
  QPushButton* listButton = new QPushButton("List Dogs");
  QPushButton* chartButton = new QPushButton("Chart Dogs");

  adminLayout->addWidget(addButton);
  adminLayout->addWidget(updateButton);
  adminLayout->addWidget(removeButton);
  adminLayout->addWidget(listButton);
  adminLayout->addWidget(chartButton);

  connect(addButton, &QPushButton::clicked, this, &DogApp::addDog);
  connect(updateButton, &QPushButton::clicked, this, &DogApp::updateDog);
  connect(removeButton, &QPushButton::clicked, this, &DogApp::removeDog);
  connect(listButton, &QPushButton::clicked, this, &DogApp::listDogs);
  connect(chartButton, &QPushButton::clicked, this, &DogApp::chartDogs);

  return adminLayout;
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

void DogApp::switchMode(int index) {
  bool isAdmin = (index == 0);

  for (int i = 0; i < adminButtons->count(); i++) {
    QWidget* widget = adminButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(isAdmin);
    }
  }

  for (int i = 0; i < userButtons->count(); i++) {
    QWidget* widget = userButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(!isAdmin);
    }
  }
}

void DogApp::hideMainMenuButtons() {
  for (int i = 0; i < adminButtons->count(); i++) {
    QWidget* widget = adminButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(false);
    }
  }

  for (int i = 0; i < userButtons->count(); i++) {
    QWidget* widget = userButtons->itemAt(i)->widget();
    if (widget != nullptr) {
      widget->setVisible(false);
    }
  }
}

void DogApp::chartDogs() {
  hideMainMenuButtons();

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

  QWidget* dogListWidget = new QWidget();
  QVBoxLayout* dogListLayout = new QVBoxLayout(dogListWidget);

  QChartView* chartView = new QChartView(chart);
  chartView->setRenderHint(QPainter::Antialiasing);
  dogListLayout->addWidget(chartView);

  QPushButton* backButton = new QPushButton("Back to Menu");
  dogListLayout->addWidget(backButton);
  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(dogListWidget);
  currentStack->setCurrentWidget(dogListWidget);
}

void DogApp::addDog() {
  hideMainMenuButtons();

  QWidget* addDogWidget = new QWidget();
  QVBoxLayout* addDogLayout = new QVBoxLayout(addDogWidget);

  QLabel* breedLabel = new QLabel("Breed:");
  QLineEdit* breedInput = new QLineEdit();
  addDogLayout->addWidget(breedLabel);
  addDogLayout->addWidget(breedInput);

  QLabel* nameLabel = new QLabel("Name:");
  QLineEdit* nameInput = new QLineEdit();
  addDogLayout->addWidget(nameLabel);
  addDogLayout->addWidget(nameInput);

  QLabel* ageLabel = new QLabel("Age:");
  QLineEdit* ageInput = new QLineEdit();
  addDogLayout->addWidget(ageLabel);
  addDogLayout->addWidget(ageInput);

  QLabel* photographLabel = new QLabel("Photograph:");
  QLineEdit* photographInput = new QLineEdit();
  addDogLayout->addWidget(photographLabel);
  addDogLayout->addWidget(photographInput);

  QPushButton* addButton = new QPushButton("Add Dog");
  addDogLayout->addWidget(addButton);

  QPushButton* backButton = new QPushButton("Back to Menu");
  addDogLayout->addWidget(backButton);

  connect(addButton, &QPushButton::clicked, this,
          [this, breedInput, nameInput, ageInput, photographInput]() {
            validateAndAddDog(breedInput->text(), nameInput->text(),
                              ageInput->text(), photographInput->text());
          });

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(addDogWidget);
  currentStack->setCurrentWidget(addDogWidget);
}

// For adding
void DogApp::validateAndAddDog(const QString& breed, const QString& name,
                               const QString& age, const QString& photograph) {
  // Convert to standard types
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

    showMainMenu();
  } catch (std::invalid_argument& e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void DogApp::updateDog() {
  hideMainMenuButtons();

  QWidget* updateDogWidget = new QWidget();
  QVBoxLayout* updateDogLayout = new QVBoxLayout(updateDogWidget);

  QLabel* breedLabel = new QLabel("Breed:");
  QLineEdit* breedInput = new QLineEdit();
  updateDogLayout->addWidget(breedLabel);
  updateDogLayout->addWidget(breedInput);

  QLabel* nameLabel = new QLabel("Name:");
  QLineEdit* nameInput = new QLineEdit();
  updateDogLayout->addWidget(nameLabel);
  updateDogLayout->addWidget(nameInput);

  QLabel* ageLabel = new QLabel("Age:");
  QLineEdit* ageInput = new QLineEdit();
  updateDogLayout->addWidget(ageLabel);
  updateDogLayout->addWidget(ageInput);

  QLabel* photographLabel = new QLabel("Photograph:");
  QLineEdit* photographInput = new QLineEdit();
  updateDogLayout->addWidget(photographLabel);
  updateDogLayout->addWidget(photographInput);

  QPushButton* findButton = new QPushButton("Find Dog");
  updateDogLayout->addWidget(findButton);

  QPushButton* updateButton = new QPushButton("Update Dog");
  updateDogLayout->addWidget(updateButton);

  QPushButton* backButton = new QPushButton("Back to Menu");
  updateDogLayout->addWidget(backButton);

  connect(findButton, &QPushButton::clicked, this,
          [this, breedInput, nameInput, ageInput, photographInput]() {
            findAndPopulateDog(breedInput->text(), nameInput->text(),
                               ageInput->text(), photographInput->text(),
                               breedInput, nameInput, ageInput,
                               photographInput);
          });

  connect(updateButton, &QPushButton::clicked, this,
          [this, breedInput, nameInput, ageInput, photographInput]() {
            validateAndUpdateDog(breedInput->text(), nameInput->text(),
                                 ageInput->text(), photographInput->text());
          });

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(updateDogWidget);
  currentStack->setCurrentWidget(updateDogWidget);
}

void DogApp::findAndPopulateDog(const QString& breed, const QString& name,
                                const QString& age, const QString& photograph,
                                QLineEdit* breedInput, QLineEdit* nameInput,
                                QLineEdit* ageInput,
                                QLineEdit* photographInput) {
  // Convert inputs to standard types
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

void DogApp::validateAndUpdateDog(const QString& breed, const QString& name,
                                  const QString& age,
                                  const QString& photograph) {
  // Convert to standard types
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

  std::string newBreed = breedStr;
  std::string newName = nameStr;
  int newAge = ageInt;
  std::string newPhotograph = photographStr;

  try {
    this->service->updateDog(index, newBreed, newName, newAge, newPhotograph);
    QMessageBox::information(this, "Success", "Dog updated successfully!");
  } catch (std::invalid_argument& e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void DogApp::removeDog() {
  hideMainMenuButtons();

  QWidget* removeDogWidget = new QWidget();
  QVBoxLayout* removeDogLayout = new QVBoxLayout(removeDogWidget);

  QLabel* breedLabel = new QLabel("Breed:");
  QLineEdit* breedInput = new QLineEdit();
  removeDogLayout->addWidget(breedLabel);
  removeDogLayout->addWidget(breedInput);

  QLabel* nameLabel = new QLabel("Name:");
  QLineEdit* nameInput = new QLineEdit();
  removeDogLayout->addWidget(nameLabel);
  removeDogLayout->addWidget(nameInput);

  QLabel* ageLabel = new QLabel("Age:");
  QLineEdit* ageInput = new QLineEdit();
  removeDogLayout->addWidget(ageLabel);
  removeDogLayout->addWidget(ageInput);

  QLabel* photographLabel = new QLabel("Photograph:");
  QLineEdit* photographInput = new QLineEdit();
  removeDogLayout->addWidget(photographLabel);
  removeDogLayout->addWidget(photographInput);

  QPushButton* removeButton = new QPushButton("Remove Dog");
  removeDogLayout->addWidget(removeButton);

  QPushButton* backButton = new QPushButton("Back to Menu");
  removeDogLayout->addWidget(backButton);

  connect(removeButton, &QPushButton::clicked, this,
          [this, breedInput, nameInput, ageInput, photographInput]() {
            validateAndRemoveDog(breedInput->text(), nameInput->text(),
                                 ageInput->text(), photographInput->text());
          });

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(removeDogWidget);
  currentStack->setCurrentWidget(removeDogWidget);
}

void DogApp::validateAndRemoveDog(const QString& breed, const QString& name,
                                  const QString& age,
                                  const QString& photograph) {
  // Convert to standard types
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

void DogApp::listDogs() {
  hideMainMenuButtons();

  // new widget for the list
  QWidget* dogListWidget = new QWidget();
  QVBoxLayout* dogListLayout = new QVBoxLayout(dogListWidget);

  QListWidget* dogList = new QListWidget();
  dogListLayout->addWidget(dogList);

  QPushButton* backButton = new QPushButton("Back to Menu");
  dogListLayout->addWidget(backButton);

  if (service->getNumberOfDogs() > 0) {
    for (int i = 0; i < service->getNumberOfDogs(); i++) {
      Dog dog = service->getDog(i);
      QString itemText =
          QString("ID: %1, Name: %2, Breed: %3, Age: %4, Photograph: %5")
              .arg(i + 1)
              .arg(QString::fromStdString(dog.getName()))
              .arg(QString::fromStdString(dog.getBreed()))
              .arg(dog.getAge())
              .arg(QString::fromStdString(dog.getPhotograph()));
      dogList->addItem(itemText);
    }
  } else {
    dogList->addItem("No dogs in the database!");
  }

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(dogListWidget);
  currentStack->setCurrentWidget(dogListWidget);
}

void DogApp::showMainMenu() {
  // show main menu widget
  currentStack->setCurrentWidget(currentStack->widget(0));
  // reset the mode
  switchMode(modeCombo->currentIndex());
}

void DogApp::adoptDogMenu() {
  hideMainMenuButtons();

  auto dogs = service->getDogs();
  currentDogIndex = 0;

  if (dogs.empty()) {
    QMessageBox::warning(this, "Info", "No dogs in the database!");
    showMainMenu();
    return;
  }

  QWidget* adoptWidget = new QWidget();
  QVBoxLayout* adoptLayout = new QVBoxLayout(adoptWidget);

  adoptLayout->addWidget(dogDetails);

  QPushButton* adoptButton = new QPushButton("Adopt");
  QPushButton* skipButton = new QPushButton("Skip");
  QPushButton* backButton = new QPushButton("Back to Menu");

  QHBoxLayout* buttonLayout = new QHBoxLayout();
  buttonLayout->addWidget(adoptButton);
  buttonLayout->addWidget(skipButton);
  buttonLayout->addWidget(backButton);

  adoptLayout->addLayout(buttonLayout);

  connect(adoptButton, &QPushButton::clicked, this, &DogApp::adoptCurrentDog);
  connect(skipButton, &QPushButton::clicked, this, &DogApp::nextDog);
  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  displayCurrentDogDetails();

  currentStack->addWidget(adoptWidget);
  currentStack->setCurrentWidget(adoptWidget);
}

void DogApp::adoptDog() {}

void DogApp::adoptCurrentDog() {
  auto dogs = service->getDogs();
  Dog dog = dogs[currentDogIndex];
  int index = service->findDog(dog.getBreed(), dog.getName(), dog.getAge(),
                               dog.getPhotograph());

  if (index != -1) {
    service->adoptDog(index);
    dogs.erase(dogs.begin() + currentDogIndex);  // Remove from the local list
    QMessageBox::information(this, "Success", "Dog adopted successfully!");
  }

  if (dogs.empty()) {
    QMessageBox::warning(this, "Info", "No more dogs available.");
    showMainMenu();
    return;
  }

  nextDog();
}

void DogApp::nextDog() {
  auto dogs = service->getDogs();
  if (++currentDogIndex >= dogs.size()) {
    currentDogIndex = 0;  // Wrap around
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

  QListWidget* filteredDogsList = new QListWidget();
  filterLayout->addWidget(filteredDogsList);

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

    filteredDogsList->clear();

    if (dogs.empty()) {
      filteredDogsList->addItem("No dogs found!");
    } else {
      for (int i = 0; i < dogs.size(); i++) {
        Dog dog = dogs[i];
        QString itemText =
            QString("ID: %1 | Name: %2 | Breed: %3 | Age: %4 | Photo: %5")
                .arg(i + 1)
                .arg(QString::fromStdString(dog.getName()))
                .arg(QString::fromStdString(dog.getBreed()))
                .arg(dog.getAge())
                .arg(QString::fromStdString(dog.getPhotograph()));
        filteredDogsList->addItem(itemText);
      }
    }
  });

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(filterWidget);
  currentStack->setCurrentWidget(filterWidget);
}

void DogApp::listAdoptedDogs() {
  hideMainMenuButtons();

  // new widget for the list
  QWidget* dogListWidget = new QWidget();
  QVBoxLayout* dogListLayout = new QVBoxLayout(dogListWidget);

  QListWidget* dogList = new QListWidget();
  dogListLayout->addWidget(dogList);

  QPushButton* backButton = new QPushButton("Back to Menu");
  dogListLayout->addWidget(backButton);

  if (service->getAdoptedDogs().size() > 0) {
    for (auto dog : service->getAdoptedDogs()) {
      QString itemText =
          QString("ID: %1, Name: %2, Breed: %3, Age: %4, Photograph: %5")
              .arg(QString::fromStdString(dog.getName()))
              .arg(QString::fromStdString(dog.getBreed()))
              .arg(dog.getAge())
              .arg(QString::fromStdString(dog.getPhotograph()));
      dogList->addItem(itemText);
    }
  } else {
    dogList->addItem("No dogs in the database!");
  }

  connect(backButton, &QPushButton::clicked, this, &DogApp::showMainMenu);

  currentStack->addWidget(dogListWidget);
  currentStack->setCurrentWidget(dogListWidget);
}

void DogApp::openAdoptedFile() {
  // check if the combobox is set to csv or html
  // call the service method to open the file

  std::string filename =
      adoptedDogsRepoCombo->currentText().toStdString() == "CSV"
          ? "../adopted.csv"
          : "../adopted.html";
  this->service->openAdoptedFile(filename);
}