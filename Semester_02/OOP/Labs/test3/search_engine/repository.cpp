#include "repository.h"

#include <fstream>
#include <sstream>

void Repository::loadData() {
  char delimiter = '|', comma = ',';
  std::string name, content;
  std::vector<std::string> keywords;

  std::ifstream fin("../documents.txt");

  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);

    std::string keywords_str;
    std::getline(iss, name, delimiter);
    std::getline(iss, keywords_str, delimiter);
    std::getline(iss, content, delimiter);

    std::istringstream isss(keywords_str);
    std::string keyword;
    while (std::getline(isss, keyword, comma)) {
      keywords.push_back(keyword);
    }

    documents.push_back(Document(name, keywords, content));
    keywords.clear();
  }
}

std::vector<Document> Repository::getDocuments() const { return documents; }