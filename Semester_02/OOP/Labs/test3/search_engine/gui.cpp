#include "gui.h"

GUI::GUI(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *mainLayout = new QVBoxLayout();
  mainItems = new QListWidget();
  QLabel *label = new QLabel("Input field");
  inputField = new QLineEdit();
  showBestMatching = new QPushButton("Show best matching");

  mainLayout->addWidget(mainItems);
  mainLayout->addWidget(label);
  mainLayout->addWidget(inputField);
  mainLayout->addWidget(showBestMatching);

  setLayout(mainLayout);
  populateList();

  connect(inputField, &QLineEdit::textChanged, this, &GUI::onChangeText);
  connect(showBestMatching, &QPushButton::clicked, this,
          &GUI::showBestMatchingFunc);
}

void GUI::populateList() {
  mainItems->clear();

  std::vector<Document> documents = service.getAll();

  for (auto x : documents) {
    mainItems->addItem(QString::fromStdString(x.toString()));
  }
}

void GUI::onChangeText() {
  if (inputField->text().size() == 0) {
    populateList();
    return;
  }
  std::string keyword_or_name = inputField->text().toStdString();
  std::vector<Document> documents = service.search(keyword_or_name);

  mainItems->clear();
  for (auto x : documents) {
    mainItems->addItem(QString::fromStdString(x.toString()));
  }
}
void GUI::showBestMatchingFunc() {
  QWidget *window = new QWidget();
  QVBoxLayout *layout = new QVBoxLayout();
  std::string keyword_or_name = inputField->text().toStdString();
  auto ans = service.getBestMatchingDoc(keyword_or_name);

  QString matchingDocument =
      QString::fromStdString("Matching document: " + ans.toString());

  QLabel *label =
      new QLabel(keyword_or_name.size() == 0 ? QString("No matching document")
                                             : matchingDocument);

  layout->addWidget(label);
  window->setLayout(layout);

  window->show();
}