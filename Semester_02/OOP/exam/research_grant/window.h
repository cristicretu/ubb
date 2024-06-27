#pragma once

#include <QAbstractItemModel>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>
#include <QWidget>

#include "ideaModel.h"
#include "ideaWindow.h"
#include "session.h"

class Window : public QWidget {
 private:
  Service &service;
  Researcher &researcher;
  ideaModel *model;

  QLabel *position;

  QTableView *table;

  QLabel *titleL, *descL, *durationL;
  QLineEdit *titleE, *descE, *durationE;

  QPushButton *createBtn;

  vector<Idea> acceptedIdeas;

 public:
  Window(Service &service, Researcher &researcher, ideaModel *model,
         QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
 public slots:
  void addIdea();
  void saveAllAcceptedIdeas();
  void acceptIdea();

  void showAcceptedIdeas();
};
