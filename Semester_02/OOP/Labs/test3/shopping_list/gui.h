#pragma once
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "service.h"

class GUI : public QWidget {
 private:
  Service service;
  QListWidget *mainItems;
  QListWidget *filteredItems;
  QLineEdit *inputField;
  QLineEdit *categoryField;
  QPushButton *searchButton;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

  void populateMainItems();
 public slots:
  void populateFilteredItems();
  void filterCategory();
  void printItem();
};
