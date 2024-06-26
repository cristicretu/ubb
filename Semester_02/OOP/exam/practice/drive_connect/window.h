#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "QListWidget"
#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;
  Driver &driver;

  QListWidget *list, *messages;
  QLabel *location, *score;

  QLineEdit *message;
  QPushButton *send;

  QLabel *description, *lat, *lg;
  QLineEdit *descriptionInput, *latInput, *lgInput;

  QPushButton *validateBtn;

 public:
  Window(Session &session, Driver &driver, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
 public slots:
  void sendMessage();
  void sendReport();
  void validateReport();
};
