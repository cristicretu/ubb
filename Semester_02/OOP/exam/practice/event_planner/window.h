#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;
  Person person;

  QListWidget *list;

  QLabel *nameLabel, *descriptionLabel, *latitudeLabel, *longitudeLabel,
      *dateLabel;
  QLineEdit *nameEdit, *descriptionEdit, *latitudeEdit, *longitudeEdit,
      *dateEdit;

  QPushButton *addButton;

  vector<Event> initialEvents;

 public:
  Window(Session &session, Person person, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
 public slots:
  void addEvent();
};
