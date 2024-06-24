#pragma once

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session& session;
  int userId;

  QListWidget* itemsList;
  QComboBox* combobox;

  QPushButton* addButton;

  QLineEdit *nameE, *categoryE, *priceE;

 public:
  Window(Session& session, int userId, QWidget* parent = Q_NULLPTR);
  ~Window() override = default;

  void update() const override;
 public slots:
  void addItem();
};
