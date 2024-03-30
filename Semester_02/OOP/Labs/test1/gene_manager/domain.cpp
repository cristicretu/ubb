#include "domain.h"

Gene::Gene(std::string org_name, std::string gene_name, std::string gene_seq)
    : org_name(org_name), gene_name(gene_name), gene_seq(gene_seq) {}

Gene::~Gene() {}

std::string Gene::get_org_name() const { return org_name; }
std::string Gene::get_gene_name() const { return gene_name; }
std::string Gene::get_gene_seq() const { return gene_seq; }
void Gene::set_org_name(std::string org_name) { this->org_name = org_name; }
void Gene::set_gene_name(std::string gene_name) { this->gene_name = gene_name; }
void Gene::set_gene_seq(std::string gene_seq) { this->gene_seq = gene_seq; }
