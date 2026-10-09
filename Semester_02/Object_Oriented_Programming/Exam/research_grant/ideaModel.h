#pragma once
#include <QAbstractTableModel>

#include "repository.h"

class ideaModel : public QAbstractTableModel {
  Q_OBJECT
 private:
  Repository &repo;

 public:
  ideaModel(Repository &repo) : repo(repo) {}
  ~ideaModel() override = default;

  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    return repo.getIdeas().size();
  }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override {
    return 4;
  }

  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override {
    if (role == Qt::DisplayRole || role == Qt::EditRole) {
      Idea idea = repo.getIdeas()[index.row()];

      switch (index.column()) {
        case 0:
          return QString::fromStdString(idea.getTitle());
        case 1:
          return QString::fromStdString(idea.getStatus() == 0 ? "Proposed"
                                                              : "ACCEPTED!");
        case 2:
          return QString::fromStdString(idea.getCreator());
        case 3:
          return QString::fromStdString(to_string(idea.getDuration()));
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
            return "Title";

          case 1:
            return "Status";
          case 2:
            return "Creator";
          case 3:
            return "Duration";
          default:
            break;
        }
      }
    }
    return QVariant();
  }

  Qt::ItemFlags flags(const QModelIndex &index) const override {
    return Qt::ItemIsSelectable | Qt::ItemIsEnabled;
  }

  void updateData() {
    beginResetModel();
    endResetModel();
    emit this->dataChanged(index(0, 0),
                           index(rowCount() - 1, columnCount() - 1));
  }
};