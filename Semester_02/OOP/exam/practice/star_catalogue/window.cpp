#include "window.h"

#include "starItemModel.h"

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

  auto hlayout = new QHBoxLayout();

  auto nameL = new QLabel("Name:");
  auto raL = new QLabel("Ra:");
  auto decL = new QLabel("Dec:");
  auto diameterL = new QLabel("Diameter:");

  name = new QLineEdit();
  ra = new QLineEdit();
  dec = new QLineEdit();
  diameter = new QLineEdit();

  hlayout->addWidget(nameL);
  hlayout->addWidget(name);
  hlayout->addWidget(raL);
  hlayout->addWidget(ra);
  hlayout->addWidget(decL);
  hlayout->addWidget(dec);
  hlayout->addWidget(diameterL);
  hlayout->addWidget(diameter);

  layout->addLayout(hlayout);

  addStar = new QPushButton("Add star");

  layout->addWidget(addStar);

  starNameFilter = new QLineEdit();
  filteredStars = new QListWidget();

  layout->addWidget(starNameFilter);
  layout->addWidget(filteredStars);

  setLayout(layout);

  connect(constellations, &QComboBox::currentIndexChanged, this, [this]() {
    auto regularEXP = constellations->currentText().toStdString() == "All"
                          ? ".*"
                          : constellations->currentText().toStdString();
    filterModel->setFilterRegularExpression(
        QRegularExpression(regularEXP.c_str()));
  });

  connect(addStar, &QPushButton::clicked, this,
          [this, &session, &astronomer, &model]() {
            auto nameStr = name->text().toStdString();
            auto raStr = ra->text().toInt();
            auto decStr = dec->text().toInt();
            auto diameterStr = diameter->text().toDouble();

            try {
              if (raStr < 0 || raStr > 24 || decStr < -90 || decStr > 90 ||
                  diameterStr < 0) {
                throw invalid_argument("Invalid star data");
              }

              session.addStar(nameStr, astronomer.getConstellation(), raStr,
                              decStr, diameterStr);

              name->clear();
              ra->clear();
              dec->clear();
              diameter->clear();

            } catch (const invalid_argument& e) {
              QMessageBox::warning(this, "Error", e.what());
            }
          });

  connect(starNameFilter, &QLineEdit::textChanged, this, [this, &session]() {
    filteredStars->clear();
    auto filter = starNameFilter->text().toStdString();

    for (auto x : session.getStars()) {
      if (x.getName().find(filter) != string::npos) {
        filteredStars->addItem(QString::fromStdString(x.getName()));
      }
    }
  });
}
