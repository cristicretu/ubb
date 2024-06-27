#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "session.h"

class IdeaWindow : public QWidget {
 private:
  Service &service;
  Idea &idea;

  QLineEdit *description;
  QPushButton *saveBtn, *saveIdeaToFile;

  QLabel *title, *desc, *duration;

 public:
  IdeaWindow(Service &service, Idea &idea, QWidget *parent = Q_NULLPTR)
      : service(service), idea(idea), QWidget(parent) {
    setWindowTitle(
        QString::fromStdString(idea.getTitle() + " by " + idea.getCreator()));

    auto layout = new QVBoxLayout();

    title = new QLabel(QString::fromStdString("Title: " + idea.getTitle()));
    desc = new QLabel(
        QString::fromStdString("Description: " + idea.getDescription()));
    duration = new QLabel(
        QString::fromStdString("Duration: " + to_string(idea.getDuration())));

    layout->addWidget(title);
    layout->addWidget(desc);
    layout->addWidget(duration);

    auto hlay = new QHBoxLayout();

    auto lbl = new QLabel("Change Description: ");

    description = new QLineEdit(QString::fromStdString(idea.getDescription()));

    saveBtn = new QPushButton("Save new descriptio");

    hlay->addWidget(lbl);
    hlay->addWidget(description);
    hlay->addWidget(saveBtn);

    layout->addLayout(hlay);

    saveIdeaToFile = new QPushButton("Save idea to file");

    layout->addWidget(saveIdeaToFile);

    setLayout(layout);
    connect(saveBtn, &QPushButton::clicked, this, &IdeaWindow::saveIdea);
    connect(saveIdeaToFile, &QPushButton::clicked, this,
            &IdeaWindow::saveIdeaToFileFn);
  }
  ~IdeaWindow() override = default;

 public slots:
  void saveIdea() {
    string newDesc = description->text().toStdString();

    try {
      service.changeDescription(idea.getTitle(), idea.getDescription(),
                                newDesc);

      desc->clear();
      desc->setText(QString::fromStdString("Description: " + newDesc));
    } catch (runtime_error &e) {
      QMessageBox::critical(this, "Invalid", e.what());
    }
  }

  void saveIdeaToFileFn() {
    auto description = desc->text().toStdString();

    try {
      service.saveIdeaToFile(idea.getTitle(), "a");
    } catch (runtime_error &e) {
      QMessageBox::critical(this, "invalid", e.what());
    }
  }
};
