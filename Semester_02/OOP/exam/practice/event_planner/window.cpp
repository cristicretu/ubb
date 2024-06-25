#include "window.h"

Window::Window(Session &session, Person person, QWidget *parent)
    : session(session), person(person), QWidget(parent) {
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(person.getName()));

  initialEvents = session.getEvents();

  bool isOrganiser = person.getStatus() == 1;

  auto layout = new QVBoxLayout(this);

  list = new QListWidget(this);

  for (const auto &ev : initialEvents) {
    list->addItem(QString::fromStdString(
        ev.getName() + " " + ev.getDate() + " " + ev.getOrganiser() + " " +
        to_string(ev.getLatitude()) + " " + to_string(ev.getLongitude())));

    if (ev.getOrganiser() == person.getName() && isOrganiser) {
      list->item(list->count() - 1)->setBackground(Qt::green);
    }
  }

  layout->addWidget(list);

  seeNearEvents = new QCheckBox("See near events", this);
  layout->addWidget(seeNearEvents);

  if (isOrganiser) {
    auto hlayout = new QHBoxLayout;

    nameLabel = new QLabel("Name", this);
    hlayout->addWidget(nameLabel);

    nameEdit = new QLineEdit(this);
    hlayout->addWidget(nameEdit);

    descriptionLabel = new QLabel("Description", this);
    hlayout->addWidget(descriptionLabel);

    descriptionEdit = new QLineEdit(this);
    hlayout->addWidget(descriptionEdit);

    latitudeLabel = new QLabel("Latitude", this);
    hlayout->addWidget(latitudeLabel);

    latitudeEdit = new QLineEdit(this);
    hlayout->addWidget(latitudeEdit);

    longitudeLabel = new QLabel("Longitude", this);
    hlayout->addWidget(longitudeLabel);

    longitudeEdit = new QLineEdit(this);
    hlayout->addWidget(longitudeEdit);

    dateLabel = new QLabel("Date", this);
    hlayout->addWidget(dateLabel);

    dateEdit = new QLineEdit(this);
    hlayout->addWidget(dateEdit);

    layout->addLayout(hlayout);

    addButton = new QPushButton("Add", this);
    connect(addButton, &QPushButton::clicked, this, &Window::addEvent);

    layout->addWidget(addButton);
  } else {
    auto hlayout = new QHBoxLayout;

    description = new QLabel("Description", this);
    hlayout->addWidget(description);

    goingButton = new QPushButton("Going", this);
    hlayout->addWidget(goingButton);

    layout->addLayout(hlayout);

    connect(list, &QListWidget::clicked, this, [this, &session]() {
      auto ev = session.getEvents()[list->currentRow()];
      description->setText(QString::fromStdString(ev.getDescription()));
    });
  }

  setLayout(layout);

  QObject::connect(seeNearEvents, &QCheckBox::stateChanged, this,
                   &Window::update);
}

void Window::update() const {
  list->clear();

  auto events = session.getEvents();

  for (const auto &ev : events) {
    string descr = "";
    if (seeNearEvents->checkState() == Qt::Checked) {
      if (abs(ev.getLatitude() - person.getLatitude()) < 10 &&
          abs(ev.getLongitude() - person.getLongitude()) < 10) {
        list->addItem(QString::fromStdString(
            ev.getName() + " " + ev.getDate() + " " + ev.getOrganiser() + " " +
            to_string(ev.getLatitude()) + " " + to_string(ev.getLongitude()) +
            " " + descr));
      }

      if (ev.getOrganiser() == person.getName() && person.getStatus() == 1) {
        list->item(list->count() - 1)->setBackground(Qt::green);
      }
    } else {
      list->addItem(QString::fromStdString(
          ev.getName() + " " + ev.getDate() + " " + ev.getOrganiser() + " " +
          to_string(ev.getLatitude()) + " " + to_string(ev.getLongitude()) +
          " " + descr));

      if (ev.getOrganiser() == person.getName() && person.getStatus() == 1) {
        list->item(list->count() - 1)->setBackground(Qt::green);
      }
    }
  }
}

void Window::addEvent() {
  try {
    session.addEvent(
        person.getName(), nameEdit->text().toStdString(),
        descriptionEdit->text().toStdString(), latitudeEdit->text().toInt(),
        longitudeEdit->text().toInt(), dateEdit->text().toStdString());

    nameEdit->clear();
    descriptionEdit->clear();
    latitudeEdit->clear();
    longitudeEdit->clear();
    dateEdit->clear();
  } catch (const runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

bool Window::wasEventInInitialList(Event ev) const {
  for (auto x : initialEvents) {
    if (x.getName() == ev.getName() && x.getDate() == ev.getDate() &&
        x.getOrganiser() == ev.getOrganiser() &&
        x.getLatitude() == ev.getLatitude() &&
        x.getLongitude() == ev.getLongitude()) {
      return true;
    }
  }
  return false;
}