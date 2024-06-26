#include "window.h"

Window::Window(Session &session, Driver &driver, QWidget *parent)
    : session(session), driver(driver), QWidget(parent) {
  session.registerObserver(this);
  setWindowTitle(QString::fromStdString(driver.getName()));

  auto layout = new QVBoxLayout();

  //
  auto topLayout = new QHBoxLayout();
  location = new QLabel(QString::fromStdString(
      to_string(driver.getLat()) + "-" + to_string(driver.getLg())));
  score = new QLabel(
      QString::fromStdString(to_string(driver.getScore()) + " Stars"));
  topLayout->addWidget(location);
  topLayout->addWidget(score);
  //

  auto listLayout = new QVBoxLayout();
  auto lbl = new QLabel("Reports:");
  list = new QListWidget();

  listLayout->addWidget(lbl);
  listLayout->addWidget(list);
  //

  auto msgLayout = new QVBoxLayout();
  auto msgLbl = new QLabel("Messages:");
  messages = new QListWidget();

  auto msgInputLayout = new QHBoxLayout();
  message = new QLineEdit();
  send = new QPushButton("Send");

  msgInputLayout->addWidget(message);
  msgInputLayout->addWidget(send);

  msgLayout->addWidget(msgLbl);
  msgLayout->addWidget(messages);
  msgLayout->addLayout(msgInputLayout);

  //

  auto llbl = new QLabel("Reports:");
  auto reportLayout = new QHBoxLayout();
  description = new QLabel("Description:");
  lat = new QLabel("Lat:");
  lg = new QLabel("Lg:");

  descriptionInput = new QLineEdit();
  latInput = new QLineEdit();
  lgInput = new QLineEdit();

  auto reportBtn = new QPushButton("Report");

  reportLayout->addWidget(description);
  reportLayout->addWidget(descriptionInput);

  reportLayout->addWidget(lat);
  reportLayout->addWidget(latInput);

  reportLayout->addWidget(lg);
  reportLayout->addWidget(lgInput);

  reportLayout->addWidget(reportBtn);

  //
  layout->addLayout(topLayout);
  layout->addLayout(listLayout);
  layout->addLayout(msgLayout);
  layout->addWidget(llbl);
  layout->addLayout(reportLayout);
  //

  setLayout(layout);

  update();

  //
  connect(send, &QPushButton::clicked, this, &Window::sendMessage);
  connect(reportBtn, &QPushButton::clicked, this, &Window::sendReport);
  connect(list, &QListWidget::itemClicked, this, &Window::validateReport);
};

void Window::update() {
  list->clear();

  for (auto x : session.getInterestReports(driver.getName())) {
    list->addItem(QString::fromStdString(
        x.getDescription() + " / " + x.getReporter() + " / " +
        to_string(x.getLat()) + "-" + to_string(x.getLg()) + " / " +
        to_string(x.getStatus())));

    if (x.getStatus() == true) {
      list->item(list->count() - 1)->setFont(QFont("Arial", -1, QFont::Bold));
    }
  }

  //

  messages->clear();

  for (auto x : session.getMessages()) {
    messages->addItem(QString::fromStdString(x));

    if (x.find("[" + driver.getName() + "]") != string::npos) {
      messages->item(messages->count() - 1)
          ->setFont(QFont("Arial", -1, QFont::Bold));
    }
  }

  score->clear();

  if (driver.getName() == "Craig")
    std::cout << driver.getName() << driver.getScore() << std::endl;

  score->setText(
      QString::fromStdString(to_string(driver.getScore()) + " Stars"));
}

void Window::sendMessage() {
  try {
    session.addMessage(driver.getName(), message->text().toStdString());
  } catch (const runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void Window::sendReport() {
  try {
    auto description = descriptionInput->text().toStdString();
    auto lat = latInput->text().toInt();
    auto lg = lgInput->text().toInt();

    session.addReport(description, driver.getName(), lat, lg, false);
  } catch (const runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}

void Window::validateReport() {
  try {
    auto selected = list->selectedItems().at(0)->text().toStdString();
    auto description = selected.substr(0, selected.find(" / "));
    session.addValidation(description, driver.getName());
  } catch (const runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}
