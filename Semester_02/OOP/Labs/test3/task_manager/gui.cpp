#include "gui.h"

#include <QLabel>
#include <QVBoxLayout>

GUI::GUI(QWidget *parent) : QWidget(parent) {
  service = Service();

  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainTasks = new QListWidget();
  toggleBold = new QPushButton("Toggle bold for priority 1");
  QLabel *label = new QLabel("Search by priority");
  inputField = new QLineEdit();
  showTasks = new QPushButton("Search by priority");

  mainLayout->addWidget(mainTasks);
  mainLayout->addWidget(toggleBold);
  mainLayout->addWidget(label);
  mainLayout->addWidget(inputField);
  mainLayout->addWidget(showTasks);

  setLayout(mainLayout);
  populateTasks();

  QObject::connect(toggleBold, &QPushButton::clicked, this,
                   &GUI::toggleBoldFunc);
}

void GUI::populateTasks() {
  mainTasks->clear();
  std::vector<Task> tasks = service.getAllTasks();

  for (auto t : tasks) {
    mainTasks->addItem(QString::fromStdString(t.to_string()));
  }
}

void GUI::toggleBoldFunc() {
  bold = !bold;
  for (int i = 0; i < mainTasks->count(); i++) {
    QListWidgetItem *item = mainTasks->item(i);
    Task task = service.getAllTasks()[i];
    if (task.getPriority() == 1) {
      QFont font = item->font();
      font.setBold(bold);
      item->setFont(font);
    }
  }
}