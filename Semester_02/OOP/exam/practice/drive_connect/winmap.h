#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include "QListWidget"
#include "observer.h"
#include "session.h"

class WinMap : public QWidget, public Observer {
 private:
  Session &session;

 public:
  WinMap(Session &session) : session(session) {
    /*
    7. A new window will show the "map" of all valid reports. Each reports
    location and destication will be shown (use circles/rectangles/any geometry
    figure). This is shown when the application starts. (Hint: method paintEvent
    of the Widget class and the Painter class). (1p)*/
    session.registerObserver(this);

    setWindowTitle("Map");
  }
  ~WinMap() override = default;
  void update() const override {};
};
