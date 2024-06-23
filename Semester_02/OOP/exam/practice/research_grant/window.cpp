#include "window.h"

Window::Window(QAbstractItemModel *model, Session &session,
               Researcher researcher, QWidget *parent)
    : model(model), session(session), researcher(researcher), QWidget(parent) {
  // this->model = dynamic_cast<IdeaTableModel *>(model);
  session.registerObserver(this);
  QVBoxLayout *layout = new QVBoxLayout(this);

  setWindowTitle(QString::fromStdString(researcher.getName()));

  tableView = new QTableView(this);
  tableView->setModel(model);

  layout->addWidget(tableView);

  setLayout(layout);

  update();
}

void Window::update() const {
  // model->beginResetModel();
  // model->endResetModel();
}