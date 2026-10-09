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
  QLineEdit *filterInput;

  QLineEdit *hourInput;
  QLineEdit *descriptionInput;
  QPushButton *showButton;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

  void populateList();

  void showAnswer();
};
