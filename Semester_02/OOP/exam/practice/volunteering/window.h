#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "domain.h"
#include "observer.h"
#include "service.h"

class Window : public Observer, public QWidget {
 private:
  Service &service;
  Department department;
  QLabel *description;

  QLineEdit *name, email, interests;
  QLabel *nameLabel, *emailLabel, *interestsLabel;

 public:
  Window(Service &service, Department department, QWidget *parent = nullptr);
  void update() override;
 public slots:
  void addVolunteer();
  void assignVolunteer();
  void displayMostSuitable();
};
