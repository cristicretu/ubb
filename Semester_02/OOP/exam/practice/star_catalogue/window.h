#pragma once

#include <QCheckBox>
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

#include "session.h"
#include "starItemModel.h"

class Window : public QWidget {
 private:
  Session &session;
  Astronomer astronomer;
  starItemModel *stars;
  QSortFilterProxyModel *model;

  QTableView *table;

  QCheckBox *constellations{};

  QLineEdit *name, *ra, *dec, *diameter;
  QPushButton *addStar;

  QLineEdit *starNameFilter;
  QListWidget *filteredStars;

 public:
  Window(Session &session, Astronomer astronomer, starItemModel *model,
         QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
};
