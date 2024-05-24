#pragma once
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "service.h"

class GUI : public QWidget {
 private:
  Service service;

  QListWidget *mainItems;
  QLineEdit *inputField;
  QPushButton *showBestMatching;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

  void populateList();
 public slots:
  void onChangeText();
  void showBestMatchingFunc();
};
