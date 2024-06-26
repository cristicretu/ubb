#include "window.h"

Window::Window(Session &session, Driver driver, QWidget *parent)
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
  layout->addLayout(topLayout);
  layout->addLayout(listLayout);
  layout->addLayout(msgLayout);
  //

  setLayout(layout);

  update();

  //
  connect(send, &QPushButton::clicked, this, &Window::sendMessage);
};

void Window::update() const {
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
}

void Window::sendMessage() {
  try {
    session.addMessage(driver.getName(), message->text().toStdString());
  } catch (const runtime_error &e) {
    QMessageBox::warning(this, "Error", e.what());
  }
}
