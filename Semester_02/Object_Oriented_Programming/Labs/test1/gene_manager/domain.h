#pragma once
#include <string>

class Gene {
 private:
  std::string org_name, gene_name, gene_seq;

 public:
  Gene(std::string org_name, std::string gene_name, std::string gene_seq);

  ~Gene();

  bool operator==(const Gene& other) const {
    return org_name == other.org_name && gene_name == other.gene_name &&
           gene_seq == other.gene_seq;
  }

  bool operator>(const Gene& other) const {
    if (org_name > other.org_name) {
      return true;
    }
    if (gene_name > other.gene_name) {
      return true;
    }
    if (gene_seq > other.gene_seq) {
      return true;
    }
    return false;
  }

  std::string get_org_name() const;
  std::string get_gene_name() const;
  std::string get_gene_seq() const;
  void set_org_name(std::string org_name);
  void set_gene_name(std::string gene_name);
  void set_gene_seq(std::string gene_seq);
};