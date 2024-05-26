#pragma once

#include <QCheckBox>
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
  QCheckBox *showPaid;
  QCheckBox *showNotPaid;
  QPushButton *totalButton;
  QLineEdit *companyInput;

 public:
  GUI(QWidget *parent = Q_NULLPTR);
  ~GUI() override = default;

 public slots:
  void populateList();
  void showTotal();
};
