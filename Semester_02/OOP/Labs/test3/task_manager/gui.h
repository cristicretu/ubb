#pragma once
#include <QLineEdit>
#include <QList>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include "service.h"

class GUI : public QWidget {
 private:
  Service service;

  QListWidget *mainTasks;
  QPushButton *toggleBold;
  QLineEdit *inputField;
  QPushButton *showTasks;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

  void populateTasks();
 public slots:
  void toggleBoldFunc(int bold);
  void showTasksByPriority(int priority);
};
