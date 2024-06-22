#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;
  Biologist &biologist;

 public:
  Window(Session &session, Biologist &biologist, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
 public slots:
};
