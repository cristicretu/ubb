#pragma once
#include <QAbstractTableModel>

#include "repository.h"

class starItemModel : public QAbstractTableModel {
  Q_OBJECT
 private:
  Repository &repo;

 public:
  explicit starItemModel(Repository &repository) : repo(repository) {}
  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    return this->repo.getStars().size();
  }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override {
    return 5;
  }
  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override {
    int row = index.row(), column = index.column();
    Star star = this->repo.getStars()[row];
    if (role == Qt::DisplayRole) switch (column) {
        case 0:
          return QString::fromStdString(star.getName());
        case 1:
          return QString::fromStdString(star.getConstellation());
        case 2:
          return QString::fromStdString(std::to_string(star.getRa()));
        case 3:
          return QString::fromStdString(std::to_string(star.getDec()));
        case 4:
          return QString::fromStdString(std::to_string(star.getDiameter()));
      }
    return QVariant();
  }
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role) const override {
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole) {
      switch (section) {
        case 0:
          return "Name of Star";
        case 1:
          return "Constellation";
        case 2:
          return "RA";
        case 3:
          return "Dec";
        case 4:
          return "Diameter";
      }
    }
    return QVariant();
  }
  void updateData() {
    beginResetModel();
    endResetModel();
    emit this->dataChanged(index(0, 0),
                           index(rowCount() - 1, columnCount() - 1));
  }
};