#include "window.h"

Window::Window(Service &service, Department department, QWidget *parent)
    : service{service}, department{department}, QWidget{parent} {
  service.registerObserver(this);
  setWindowTitle(QString::fromStdString(department.getName()));

  QVBoxLayout *layout = new QVBoxLayout();

  description = new QLabel{QString::fromStdString(department.getDescription())};
  layout->addWidget(description);

  auto *layoutLists = new QHBoxLayout();
  volunteersList = new QListWidget();
  unassignedVolunteers = new QListWidget();

  layoutLists->addWidget(volunteersList);
  layoutLists->addWidget(unassignedVolunteers);

  auto *layoutVolunteer = new QHBoxLayout();

  nameLabel = new QLabel{"Name"};
  layoutVolunteer->addWidget(nameLabel);

  name = new QLineEdit();
  layoutVolunteer->addWidget(name);

  emailLabel = new QLabel{"Email"};
  layoutVolunteer->addWidget(emailLabel);

  email = new QLineEdit();
  layoutVolunteer->addWidget(email);

  interestsLabel = new QLabel{"Interests"};
  layoutVolunteer->addWidget(interestsLabel);

  interests = new QLineEdit();
  layoutVolunteer->addWidget(interests);

  addVolunteer = new QPushButton{"Add volunteer"};
  layoutVolunteer->addWidget(addVolunteer);

  layout->addLayout(layoutLists);
  layout->addLayout(layoutVolunteer);

  assignVolunteer = new QPushButton{"Assign volunteer"};
  layout->addWidget(assignVolunteer);

  mostSuitable = new QPushButton{"Most suitable"};
  layout->addWidget(mostSuitable);

  setLayout(layout);

  update();

  QObject::connect(addVolunteer, &QPushButton::clicked, this,
                   &Window::addVolunteerFunc);
  QObject::connect(assignVolunteer, &QPushButton::clicked, this,
                   &Window::assignVolunteerFunc);
  QObject::connect(mostSuitable, &QPushButton::clicked, this,
                   &Window::displayMostSuitable);
}

void Window::update() {
  volunteersList->clear();
  unassignedVolunteers->clear();

  for (Volunteer volunteer : service.getVolunteers()) {
    if (volunteer.getDepartmentName() == department.getName()) {
      volunteersList->addItem(QString::fromStdString(volunteer.getName()));
    } else {
      unassignedVolunteers->addItem(
          QString::fromStdString(volunteer.getName()));
    }
  }
}

void Window::addVolunteerFunc() {
  std::string nameText = name->text().toStdString();
  std::string emailText = email->text().toStdString();
  std::string interestsText = interests->text().toStdString();

  service.addVolunteer(nameText, emailText, department.getName(),
                       interestsText);
}

void Window::assignVolunteerFunc() {
  int index = volunteersList->currentRow();
  service.assignVolunteer(index, department.getName());
}

void Window::displayMostSuitable() {
  unassignedVolunteers->clear();
  for (auto vol : service.getMostSuitable(department.getName()))
    unassignedVolunteers->addItem(QString::fromStdString(vol));
}