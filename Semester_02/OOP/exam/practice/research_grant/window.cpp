#include "window.h"

Window::Window(QAbstractItemModel *model, Session &session,
               Researcher researcher, QWidget *parent)
    : model(model), session(session), researcher(researcher), QWidget(parent) {
  // this->model = dynamic_cast<IdeaTableModel *>(model);
  session.registerObserver(this);
  QVBoxLayout *layout = new QVBoxLayout(this);

  auto name =
      new QLabel("Name: " + QString::fromStdString(researcher.getName()));

  setWindowTitle(QString::fromStdString(researcher.getName()));

  tableView = new QTableView(this);
  tableView->setModel(model);

  layout->addWidget(name);
  layout->addWidget(tableView);

  auto horizontalLayout = new QHBoxLayout;

  title = new QLabel("Title:");
  titleEdit = new QLineEdit(this);

  description = new QLabel("Description:");
  descriptionEdit = new QLineEdit(this);

  creator = new QLabel("Creator:");
  creatorEdit = new QLineEdit(this);

  duration = new QLabel("Duration:");
  durationEdit = new QLineEdit(this);

  addButton = new QPushButton("Add", this);

  horizontalLayout->addWidget(title);
  horizontalLayout->addWidget(titleEdit);
  horizontalLayout->addWidget(description);
  horizontalLayout->addWidget(descriptionEdit);
  horizontalLayout->addWidget(creator);
  horizontalLayout->addWidget(creatorEdit);
  horizontalLayout->addWidget(duration);
  horizontalLayout->addWidget(durationEdit);

  layout->addLayout(horizontalLayout);
  layout->addWidget(addButton);

  setLayout(layout);

  update();
}

void Window::update() const {
  // model->beginResetModel();
  // model->endResetModel();
}