#include "window.h"

#include <QSortFilterProxyModel>

Window::Window(QAbstractItemModel *model, Session &session,
               Researcher researcher, QWidget *parent)
    : model(model), session(session), researcher(researcher), QWidget(parent) {
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

  duration = new QLabel("Duration:");
  durationEdit = new QLineEdit(this);

  addButton = new QPushButton("Add", this);

  horizontalLayout->addWidget(title);
  horizontalLayout->addWidget(titleEdit);
  horizontalLayout->addWidget(description);
  horizontalLayout->addWidget(descriptionEdit);
  horizontalLayout->addWidget(duration);
  horizontalLayout->addWidget(durationEdit);

  layout->addLayout(horizontalLayout);
  layout->addWidget(addButton);

  bool researcherHasAcceptedIdeas = false;

  for (auto idea : session.getIdeas()) {
    if (idea.getCreator() == researcher.getName() && idea.getStatus() == 1) {
      researcherHasAcceptedIdeas = true;
      break;
    }
  }

  if (researcherHasAcceptedIdeas) {
    developButton = new QPushButton("Develop", this);
    layout->addWidget(developButton);
  }

  if (researcher.getPosition() == "senior") {
    saveButton = new QPushButton("Save", this);

    std::cout << researcher.getName() << std::endl;
    layout->addWidget(saveButton);
    connect(saveButton, &QPushButton::clicked, this, &Window::saveIdea);
  }

  setLayout(layout);

  update();

  connect(addButton, &QPushButton::clicked, this,
          [this, &session, &researcher]() {
            try {
              session.addIdea(titleEdit->text().toStdString(),
                              descriptionEdit->text().toStdString(),
                              researcher.getName(), 0,
                              durationEdit->text().toInt());
            } catch (const std::invalid_argument &e) {
              QMessageBox::warning(this, "Error", e.what());
            }
          });
}

void Window::update() const {}

void Window::developIdea() {
  auto window = new QWidget;

  auto layout = new QVBoxLayout(window);
}

void Window::saveIdea() {
  ofstream fout(
      "../"
      "accepted_ideas.txt");

  std::cout << session.getIdeas().size() << std::endl;
  for (auto idea : session.getIdeas()) {
    if (idea.getStatus() == 1) {
      fout << idea.getTitle() << " (" << idea.getCreator() << ") "
           << idea.getDuration() << " " << idea.getDescription() << "\n";
    }
  }

  fout.close();
}