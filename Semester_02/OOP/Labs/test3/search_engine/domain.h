#pragma once
#include <string>
#include <vector>

class Document {
 private:
  std::string name, content;
  std::vector<std::string> keywords;

 public:
  Document(std::string name, std::vector<std::string> keywords,
           std::string content)
      : name(name), content(content), keywords(keywords) {}
  ~Document(){};

  std::string getName() const { return name; }
  std::string getContent() const { return content; }
  std::vector<std::string> getKeywords() const { return keywords; }

  std::string toString() const {
    std::string result = name + " | ";
    for (auto keyword : keywords) {
      result += keyword + ",";
    }
    result.pop_back();
    result += " | " + content;
    return result;
  }
};
