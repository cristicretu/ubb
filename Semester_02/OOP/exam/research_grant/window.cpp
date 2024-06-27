#include "window.h"

Window::Window(Service &service, Researcher &researcher, ideaModel *model,
               QWidget *parent)
    : service(service), researcher(researcher), QWidget(parent), model(model) {
  setWindowTitle(QString::fromStdString(researcher.getName()));
  auto layout = new QVBoxLayout();

  position = new QLabel(
      QString::fromStdString("Position: " + researcher.getPosition()));

  layout->addWidget(position);

  table = new QTableView();
  table->setModel(model);

  layout->addWidget(table);

  auto *addLayout = new QHBoxLayout();

  titleL = new QLabel("Title: ");
  descL = new QLabel("Description: ");
  durationL = new QLabel("Duration: ");

  titleE = new QLineEdit();
  descE = new QLineEdit();
  durationE = new QLineEdit();

  addLayout->addWidget(titleL);
  addLayout->addWidget(titleE);
  addLayout->addWidget(descL);
  addLayout->addWidget(descE);
  addLayout->addWidget(durationL);
  addLayout->addWidget(durationE);

  createBtn = new QPushButton("Add Idea");
  addLayout->addWidget(createBtn);

  QLabel *addIdeas = new QLabel("Add Idea: ");
  layout->addWidget(addIdeas);
  layout->addLayout(addLayout);

  if (researcher.getPosition() == "senior") {
    auto saveAllBtn = new QPushButton("Save all");

    layout->addWidget(saveAllBtn);

    connect(saveAllBtn, &QPushButton::clicked, this,
            &Window::saveAllAcceptedIdeas);

    connect(table, &QTableView::clicked, this, &Window::acceptIdea);
  }

  auto *btn = new QPushButton("Develop");

  layout->addWidget(btn);

  connect(btn, &QPushButton::clicked, this, &Window::showAcceptedIdeas);

  setLayout(layout);

  connect(createBtn, &QPushButton::clicked, this, &Window::addIdea);
}

void Window::addIdea() {
  auto title = titleE->text().toStdString();
  auto desc = descE->text().toStdString();
  auto duration = durationE->text().toInt();

  try {
    service.addIdea(title, desc, false, researcher.getName(), duration);

    model->updateData();

    titleE->clear();
    descE->clear();
    durationE->clear();

  } catch (runtime_error &e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void Window::saveAllAcceptedIdeas() { service.saveAcceptedIdeas(); }

void Window::acceptIdea() {
  auto idx = table->currentIndex().row();

  auto title = model->data(model->index(idx, 0)).toString().toStdString();
  auto desc = model->data(model->index(idx, 1)).toString().toStdString();

  try {
    service.acceptIdea(title, desc);

    model->updateData();
  } catch (runtime_error &e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void Window::showAcceptedIdeas() {
  acceptedIdeas.clear();

  for (auto x : service.getIdeas()) {
    if (x.getCreator() == researcher.getName() && x.getStatus() == 1) {
      acceptedIdeas.emplace_back(x);
    }
  }

  if (acceptedIdeas.size() == 0) {
    QMessageBox::critical(this, "Error", "You don't have any accepted ideas");
  }

  for (auto &x : acceptedIdeas) {
    auto *win = new IdeaWindow(service, x);
    win->show();
  }
}