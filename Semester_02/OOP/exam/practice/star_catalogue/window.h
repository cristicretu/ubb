#pragma once

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QSortFilterProxyModel>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <set>

#include "session.h"

class Window : public QWidget {
 private:
  Session &session;
  Astronomer &astronomer;
  QAbstractItemModel *model;
  QSortFilterProxyModel *filterModel;

  QTableView *table;

  QComboBox *constellations;

  QLineEdit *name, *ra, *dec, *diameter;
  QPushButton *addStar;

  QLineEdit *starNameFilter;
  QListWidget *filteredStars;

 public:
  Window(Session &session, Astronomer &astronomer, QAbstractItemModel *model,
         QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
};
