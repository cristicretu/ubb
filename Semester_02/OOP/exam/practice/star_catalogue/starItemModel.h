#pragma once
#include <QAbstractTableModel>

#include "repository.h"

class starItemModel : public QAbstractTableModel {
  Q_OBJECT
 private:
  Repository &repo;

 public:
  starItemModel(Repository &repo) : repo(repo) {}
  ~starItemModel() override = default;

  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    return repo.getStars().size();
  }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override {
    return 5;
  }
  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override {
    if (role == Qt::DisplayRole || role == Qt::EditRole) {
      auto stars = repo.getStars();
      auto star = stars[index.row()];

      switch (index.column()) {
        case 0:
          return QString::fromStdString(star.getName());
        case 1:
          return QString::fromStdString(star.getConstellation());
        case 2:
          return (star.getRa());
        case 3:
          return (star.getDec());
        case 4:
          return (star.getDiameter());
        default:
          break;
      }
    }

    return QVariant();
  }
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override {
    if (role == Qt::DisplayRole) {
      if (orientation == Qt::Horizontal) {
        switch (section) {
          case 0:
            return "Name";
          case 1:
            return "Constellation";
          case 2:
            return "RA";
          case 3:
            return "DEC";
          case 4:
            return "Diameter";
          default:
            break;
        }
      }
    }
    return QVariant();
  }

  bool setData(const QModelIndex &index, const QVariant &value,
               int role = Qt::EditRole) override {
    if (role != Qt::EditRole) {
      return false;
    }
    Star &star = repo.getStars()[index.row()];
    switch (index.column()) {
      case 0:
        star.setName(value.toString().toStdString());
        break;
      case 1:
        star.setConstellation(value.toString().toStdString());
        break;
      case 2:
        star.setRa(value.toInt());
        break;
      case 3:
        star.setDec(value.toInt());
        break;
      case 4:
        star.setDiameter(value.toDouble());
        break;
      default:
        break;
    }
    emit dataChanged(index, index, {Qt::DisplayRole, Qt::EditRole});

    return true;
  }
  Qt::ItemFlags flags(const QModelIndex &index) const override {
    if (!index.isValid()) return Qt::NoItemFlags;
    return Qt::ItemIsEditable | QAbstractTableModel::flags(index);
  }

  void refreshModel() {
    beginResetModel();
    endResetModel();
  }
};