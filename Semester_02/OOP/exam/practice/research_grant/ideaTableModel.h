#pragma once
#include <QAbstractTableModel>

#include "repository.h"

class IdeaTableModel : public QAbstractTableModel {
 private:
  Repository &repo;

 public:
  IdeaTableModel(Repository &repo) : repo(repo){};

  int rowCount(const QModelIndex &parent = QModelIndex()) const override {
    return repo.getIdeas().size();
  }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override {
    return 5;
  }

  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override {
    if (role == Qt::DisplayRole) {
      Idea idea = repo.getIdeas()[index.row()];

      switch (index.column()) {
        case 0:
          return QString::fromStdString(idea.getTitle());
        case 1:
          return QString::fromStdString(idea.getDescription());
        case 2:
          return QString::fromStdString(idea.getCreator());
        case 3:
          return idea.getStatus();
        case 4:
          return idea.getDuration();
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
            return "Description";
          case 2:
            return "Creator";
          case 3:
            return "Status";
          case 4:
            return "Duration";
          default:
            break;
        }
      }
    }

    return QVariant();
  }

  bool setData(const QModelIndex &index, const QVariant &value,
               int role = Qt::EditRole) override {
    if (role == Qt::EditRole) {
      Idea idea = repo.getIdeas()[index.row()];

      switch (index.column()) {
        case 0:
          idea.setTitle(value.toString().toStdString());
          break;
        case 1:
          idea.setDescription(value.toString().toStdString());
          break;
        case 2:
          idea.setCreator(value.toString().toStdString());
          break;
        case 3:
          idea.setStatus(value.toInt());
          break;
        case 4:
          idea.setDuration(value.toInt());
          break;
        default:
          break;
      }

      repo.getIdeas()[index.row()] = idea;
      emit dataChanged(index, index);
      return true;
    }

    return false;
  }

  Qt::ItemFlags flags(const QModelIndex &index) const override {
    return Qt::ItemIsEditable | QAbstractTableModel::flags(index);
  }
};
