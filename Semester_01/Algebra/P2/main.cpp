#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

int n;
std::vector<std::vector<std::vector<int>>> tables;

bool isAssociative(const std::vector<std::vector<int>> &table) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      for (int k = 0; k < n; k++) {
        if (table[table[i][j]][k] != table[i][table[j][k]])
          return false;
      }
    }
  }
  return true;
}

void generateTables(int i, int j, std::vector<std::vector<int>> &table) {
  if (i == n) {
    if (isAssociative(table))
      tables.push_back(table);
    return;
  }
  for (int x = 0; x < n; x++) {
    table[i][j] = x;
    if (j == n - 1)
      generateTables(i + 1, 0, table);
    else
      generateTables(i, j + 1, table);
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " input_file.txt" << std::endl;
    return 1;
  }

  std::string input_file = argv[1];
  std::ifstream inFile(input_file);

  if (!inFile) {
    std::cout << "Input file '" << input_file << "' not found." << std::endl;
    return 1;
  }

  inFile >> n;
  if (inFile.fail()) {
    std::cout << "Invalid input in '" << input_file
              << "'. Please provide a single integer on the first line."
              << std::endl;
    return 1;
  }

  std::string baseName = input_file.substr(0, input_file.find('_'));
  std::string output_file = baseName + "_output.txt";
  std::ofstream outFile(output_file);

  std::vector<std::vector<int>> table(n, std::vector<int>(n, 0));
  generateTables(0, 0, table);

  outFile << "1. the number of associative operations on a set A of size " << n
          << " is: " << tables.size() << std::endl;

  if (n <= 4) {
    outFile << "2. the operation tables of each associative operation on a set "
               "A of size "
            << n << " are: " << std::endl;

    for (size_t t = 0; t < tables.size(); t++) {
      outFile << "Table " << t + 1 << ":" << std::endl;
      for (int i = 0; i < n; i++) {
        outFile << "(";
        outFile << "a" << tables[t][i][0] + 1;
        for (int j = 1; j < n; j++) {
          outFile << ", a" << tables[t][i][j] + 1;
        }
        outFile << ")" << std::endl;
      }
    }
  }
  return 0;
}
