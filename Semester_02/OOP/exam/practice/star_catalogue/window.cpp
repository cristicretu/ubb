#include "window.h"

Window::Window(Session& session, Astronomer& astronomer,
               QAbstractItemModel* model, QWidget* parent)
    : session(session), astronomer(astronomer), model(model), QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);

  setWindowTitle(QString::fromStdString(astronomer.getName()));

  filterModel = new QSortFilterProxyModel();
  filterModel->setSourceModel(model);
  filterModel->setFilterKeyColumn(1);
  filterModel->setFilterRegularExpression(QRegularExpression(".*"));

  constellations = new QComboBox(this);

  constellations->addItem("All");
  set<string> con;

  for (auto x : session.getStars()) {
    con.insert(x.getConstellation());
  }

  for (auto x : con) {
    constellations->addItem(QString::fromStdString(x));
  }

  table = new QTableView(this);
  table->setModel(filterModel);

  layout->addWidget(table);

  layout->addWidget(constellations);

  setLayout(layout);

  connect(constellations, &QComboBox::currentIndexChanged, this, [this]() {
    auto regularEXP = constellations->currentText().toStdString() == "All"
                          ? ".*"
                          : constellations->currentText().toStdString();
    filterModel->setFilterRegularExpression(
        QRegularExpression(regularEXP.c_str()));
  });
}