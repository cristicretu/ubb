#ifndef WINDOW_H
#define WINDOW_H
#pragma once

#include <qprocess.h>
#include <qspinbox.h>

#include <QComboBox>
#include <QHBoxLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableWidget>
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

  QTabWidget* tabWidget;
  QVBoxLayout* layout3;
  QVBoxLayout* layout2;

  QVBoxLayout* createUserButtons();
  int currentDogIndex = 0;
  QTextBrowser* dogDetails;

  QTableWidget* dogTable;

  QWidget* adminTab();
  QWidget* chart;

  void switchMode(int index);
  void updateDogFromTable(QTableWidgetItem* item);

  void addDog();
  void updateDog();
  void removeDog();
  void listDogs();
  QWidget* chartDogs();

  void adoptDogMenu();
  void filterDogs();
  void listAdoptedDogs();
  void openAdoptedFile();

  void setupToolbar();
  void switchDogRepo(int index);
  void switchAdoptedDogRepo(int index);

  void hideMainMenuButtons();
  void showAddDogPopup();
  void populateTable();

  void validateAndAddDog(const QString& breed, const QString& name,
                         const QString& age, const QString& photograph);
  void validateAndRemoveDog(const QString& breed, const QString& name,
                            const QString& age, const QString& photograph);
  void findAndPopulateDog(const QString& breed, const QString& name,
                          const QString& age, const QString& photograph,
                          QLineEdit* breedInput, QLineEdit* nameInput,
                          QLineEdit* ageInput, QLineEdit* photographInput);
  bool validateAndUpdateDog(const QString& breed, const QString& name,
                            const QString& age, const QString& photograph,
                            int index);
  void adoptDog();
  void nextDog();
  void adoptCurrentDog();
  void displayCurrentDogDetails();

  void expandCellWithUrl(int row, int column);

  void hideOrShowUserButtons(bool show);

  void openLink(int row, int column);
};

#endif  // WINDOW_H