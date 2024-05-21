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

class DogTableModel : public QAbstractTableModel {
  Q_OBJECT

 private:
  std::vector<Dog> dogs;
  bool readOnly;

 public:
  DogTableModel(QObject* parent = nullptr, bool readOnly = false)
      : QAbstractTableModel(parent), readOnly(readOnly) {}

  void setDogs(const std::vector<Dog>& newDogs) {
    beginResetModel();
    dogs = newDogs;
    endResetModel();
  }

  int rowCount(const QModelIndex& parent = QModelIndex()) const override {
    return static_cast<int>(dogs.size());
  }

  int columnCount(const QModelIndex& parent = QModelIndex()) const override {
    return 4;
  }

  QVariant data(const QModelIndex& index, int role) const override {
    if (!index.isValid()) return QVariant();

    if (index.row() >= static_cast<int>(dogs.size()) || index.row() < 0)
      return QVariant();

    const Dog& dog = dogs.at(index.row());

    if (role == Qt::DisplayRole || role == Qt::EditRole) {
      switch (index.column()) {
        case 0:
          return QString::fromStdString(dog.getName());
        case 1:
          return QString::fromStdString(dog.getBreed());
        case 2:
          return dog.getAge();
        case 3:
          return QString::fromStdString(dog.getPhotograph());
        default:
          return QVariant();
      }
    }

    return QVariant();
  }

  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override {
    if (role != Qt::DisplayRole) return QVariant();

    if (orientation == Qt::Horizontal) {
      switch (section) {
        case 0:
          return tr("Name");
        case 1:
          return tr("Breed");
        case 2:
          return tr("Age");
        case 3:
          return tr("Photograph");
        default:
          return QVariant();
      }
    }
    return QVariant();
  }

  bool setData(const QModelIndex& index, const QVariant& value,
               int role) override {
    if (index.isValid() && role == Qt::EditRole) {
      Dog& dog = dogs[index.row()];
      switch (index.column()) {
        case 0:
          dog.setName(value.toString().toStdString());
          break;
        case 1:
          dog.setBreed(value.toString().toStdString());
          break;
        case 2:
          dog.setAge(value.toInt());
          break;
        case 3:
          dog.setPhotograph(value.toString().toStdString());
          break;
        default:
          return false;
      }
      emit dataChanged(index, index, {role});
      return true;
    }
    return false;
  }

  Qt::ItemFlags flags(const QModelIndex& index) const override {
    if (!index.isValid()) return Qt::ItemIsEnabled;

    if (readOnly) return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
  }
};

class DogApp : public QMainWindow {
  Q_OBJECT

 public:
  DogApp(QWidget* parent = nullptr);

 protected:
  void keyPressEvent(QKeyEvent* event) override {
    if (event->modifiers() == Qt::ControlModifier &&
        event->key() == Qt::Key_Z) {
      try {
        service.undo();
        // dogTable->blockSignals(true);
        populateTable();
        // dogTable->blockSignals(false);
      } catch (std::exception& e) {
        QMessageBox::warning(this, "Error", e.what());
      }
    } else if (event->modifiers() ==
                   (Qt::ControlModifier | Qt::ShiftModifier) &&
               event->key() == Qt::Key_Z) {
      try {
        service.redo();
        // dogTable->blockSignals(true);
        populateTable();
        // dogTable->blockSignals(false);
      } catch (std::exception& e) {
        QMessageBox::warning(this, "Error", e.what());
      }
    }
  }

 private:
  Service service;
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

  QPushButton* undoButton;
  QPushButton* redoButton;

  QVBoxLayout* createUserButtons();
  int currentDogIndex = 0;
  QTextBrowser* dogDetails;

  QTableView* dogTable;
  DogTableModel* dogTableModel;

  QWidget* adminTab();
  QWidget* chart;

  void updateDogFromTable(const QModelIndex& topLeft,
                          const QModelIndex& bottomRight,
                          const QVector<int>& roles);

  void removeDog();
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
  void nextDog();
  void adoptCurrentDog();
  void displayCurrentDogDetails();

  void expandCellWithUrl(const QModelIndex& index);

  void hideOrShowUserButtons(bool show);

  void openLink(const QModelIndex& index);
};

#endif  // WINDOW_H