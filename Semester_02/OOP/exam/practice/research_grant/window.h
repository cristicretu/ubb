#pragma once

#include <qabstractitemmodel.h>

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>
#include <QWidget>

#include "ideaTableModel.h"
#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  QAbstractItemModel *model;
  Session &session;
  Researcher researcher;

  QTableView *tableView;

  QLabel *title, *description, *duration;
  QLineEdit *titleEdit, *descriptionEdit, *durationEdit;

  QPushButton *addButton, *acceptButton;

 public:
  Window(QAbstractItemModel *model, Session &session, Researcher researcher,
         QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() const override;
};
