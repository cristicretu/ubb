#include "window.h"

Window::Window(Session& session, Astronomer astronomer,
               starItemModel* starsModel, QWidget* parent)
    : session(session),
      astronomer(astronomer),
      stars(starsModel),
      QWidget(parent) {
  QVBoxLayout* layout = new QVBoxLayout(this);

  setWindowTitle(QString::fromStdString(astronomer.getName()));

  model = new QSortFilterProxyModel();
  model->setSourceModel(stars);

  constellations = new QCheckBox();

  table = new QTableView();
  table->setModel(model);

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

  connect(constellations, &QCheckBox::stateChanged, this,
          [this, &astronomer]() {
            if (constellations->isChecked()) {
              model->setFilterRegularExpression(QRegularExpression(
                  QString::fromStdString(this->astronomer.getConstellation()),
                  QRegularExpression::CaseInsensitiveOption));
              model->setFilterKeyColumn(1);
            } else {
              model->setFilterFixedString("");
            }
          });

  // connect(addStar, &QPushButton::clicked, this,
  //         [this, &session, &astronomer, &model]() {
  //           auto nameStr = name->text().toStdString();
  //           auto raStr = ra->text().toInt();
  //           auto decStr = dec->text().toInt();
  //           auto diameterStr = diameter->text().toDouble();

  //           try {
  //             if (raStr < 0 || raStr > 24 || decStr < -90 || decStr > 90 ||
  //                 diameterStr < 0) {
  //               throw invalid_argument("Invalid star data");
  //             }

  //             session.addStar(nameStr, astronomer.getConstellation(), raStr,
  //                             decStr, diameterStr);

  //             name->clear();
  //             ra->clear();
  //             dec->clear();
  //             diameter->clear();

  //           } catch (const invalid_argument& e) {
  //             QMessageBox::warning(this, "Error", e.what());
  //           }
  //         });

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
