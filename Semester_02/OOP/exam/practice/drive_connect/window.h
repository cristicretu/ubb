#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "QListWidget"
#include "observer.h"
#include "session.h"

class Window : public QWidget, public Observer {
 private:
  Session &session;
  Driver &driver;

  QListWidget *list, *messages;
  QLabel *location, *score;

  QLineEdit *message;
  QPushButton *send;

  QLabel *description, *lat, *lg;
  QLineEdit *descriptionInput, *latInput, *lgInput;

  QPushButton *validateBtn, *viewBtn;

 public:
  Window(Session &session, Driver &driver, QWidget *parent = Q_NULLPTR);
  ~Window() override = default;
  void update() override;
 public slots:
  void sendMessage();
  void sendReport();
  void validateReport();
  void openWinMap();

 protected:
  void paintEvent(QPaintEvent *event) override {
    QPainter painter(this);

    painter.setPen(Qt::red);

    auto reports = session.getReports();
    for (const auto &report : reports) {
      if (report.getStatus()) {
        int x = static_cast<int>((report.getLat() + 180) * (width() / 360.0));
        int y = static_cast<int>((90 - report.getLg()) * (height() / 180.0));

        painter.drawEllipse(QPoint(x, y), 50, 50);
        painter.drawText(x + 55, y + 5,
                         QString::fromStdString(report.getDescription()));
      }
    }
  }
};
