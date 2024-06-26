#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>

#include "QListWidget"
#include "observer.h"
#include "session.h"

class WinMap : public QWidget, public Observer {
 private:
  Session &session;

 public:
  WinMap(Session &session) : session(session) {
    session.registerObserver(this);

    setWindowTitle("Map");
  }
  ~WinMap() override = default;
  void update() override { repaint(); }

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
      }
    }
  }
};
