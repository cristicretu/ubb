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
  QListWidget *filteredItems;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

  void populateMainItems();
};
