#ifndef WINDOW_H
#define WINDOW_H
#pragma once

#include <QComboBox>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <QWidget>
#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>

#include "../service/service.h"

class DogApp : public QMainWindow {
  Q_OBJECT

 public:
  DogApp(QWidget* parent = nullptr);

 private:
  Service* service;
  QComboBox* modeCombo;
  QComboBox* dogsRepoCombo;
  QComboBox* adoptedDogsRepoCombo;
  QStackedWidget* currentStack;
  QVBoxLayout* adminButtons;
  QVBoxLayout* userButtons;
  DogTypeRepository dogTypeRepository;

  QVBoxLayout* createAdminButtons();
  QVBoxLayout* createUserButtons();
  int currentDogIndex = 0;
  QTextBrowser* dogDetails;

  void switchMode(int index);

  void addDog();
  void updateDog();
  void removeDog();
  void listDogs();
  void chartDogs();

  void adoptDogMenu();
  void filterDogs();
  void listAdoptedDogs();
  void openAdoptedFile();

  void setupToolbar();
  void switchDogRepo(int index);
  void switchAdoptedDogRepo(int index);

  void hideMainMenuButtons();
  void showMainMenu();

  void validateAndAddDog(const QString& breed, const QString& name,
                         const QString& age, const QString& photograph);
  void validateAndRemoveDog(const QString& breed, const QString& name,
                            const QString& age, const QString& photograph);
  void findAndPopulateDog(const QString& breed, const QString& name,
                          const QString& age, const QString& photograph,
                          QLineEdit* breedInput, QLineEdit* nameInput,
                          QLineEdit* ageInput, QLineEdit* photographInput);
  void validateAndUpdateDog(const QString& breed, const QString& name,
                            const QString& age, const QString& photograph);
  void adoptDog();
  void nextDog();
  void adoptCurrentDog();
  void displayCurrentDogDetails();
};

#endif  // WINDOW_H