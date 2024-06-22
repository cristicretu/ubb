#pragma once

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>
#include <QWidget>

#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;
  Biologist &biologist;

  QTableView *bacteriaTable;
  QComboBox *speciesComboBox;

  QLabel *nameLabel, *speciesLabel, *sizeLabel, *diseasesLabel;
  QLineEdit *nameLineEdit, *speciesLineEdit, *sizeLineEdit, *diseasesLineEdit;

  QPushButton *addButton, *removeButton, *updateButton;

 public:
  Window(Session &session, Biologist &biologist, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
 public slots:
};
