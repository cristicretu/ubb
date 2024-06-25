#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;

 public:
  Window(Session &session, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
 public slots:
  void addEvent();
};
