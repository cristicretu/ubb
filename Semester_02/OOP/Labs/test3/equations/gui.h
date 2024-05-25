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

  QLineEdit *a;
  QLineEdit *b;
  QLineEdit *c;

  QPushButton *updateBtn;
  QPushButton *computeSolutions;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;
};
