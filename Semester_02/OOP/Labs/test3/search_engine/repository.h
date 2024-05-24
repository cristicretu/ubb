#pragma once
#include "domain.h"

class Repository {
 private:
  std::vector<Document> documents;

 public:
  Repository() { loadData(); }
  ~Repository(){};

  void loadData();
  std::vector<Document> getDocuments() const;
};