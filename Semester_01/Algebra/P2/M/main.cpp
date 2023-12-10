#include <fstream>
#include <iostream>
#include <vector>

int n; // Size of the set A, this should be initialized correctly before use.

bool is_associative(const std::vector<std::vector<int>> &table) {
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      for (int k = 0; k < n; ++k) {
        if (table[table[i][j]][k] != table[i][table[j][k]])
          return false;
      }
    }
  }
  return true;
}

std::vector<std::vector<std::vector<int>>> generate_tables(int n) {
  std::vector<std::vector<std::vector<int>>> associative_tables;
  std::vector<std::vector<int>> table(n, std::vector<int>(n, 0));
  int i = 0, j = 0;

  while (i < n) {
    // When j reaches n, increment i and reset j
    if (j == n) {
      ++i;
      j = 0;
    }
    if (i == n) {
      // We have filled the table, check if it is associative
      if (is_associative(table)) {
        associative_tables.push_back(table);
      }
      // Backtrack to find the next table to fill
      --i;
      while (i >= 0 && ++table[i][n - 1] == n) {
        table[i][n - 1] = 0; // Reset the current cell
        --i;                 // Move to the previous row
      }
      if (i < 0)
        break;   // If we have reset the first row, we are done
      j = n - 1; // Set j to the last cell of the new current row
    } else {
      table[i][j] = 0; // Start with the first value
      ++j;             // Move to the next cell
    }
  }

  return associative_tables;
}

int main() {
  n = 2; // Example with n=3 to avoid long computation times
  auto associative_tables = generate_tables(n);
  int nr_associative_operations = associative_tables.size();

  std::ofstream output("output.txt");
  output << "1. the number of associative operations on set A = " << n << " is "
         << nr_associative_operations << "\n";
  if (n <= 4) {
    output << "2. on A, these are the following associative operations:\n";
    for (const auto &table : associative_tables) {
      for (const auto &row : table) {
        for (auto elem : row) {
          output << "a" << elem << " ";
        }
        output << "\n";
      }
      output << "\n";
    }
  }
  output.close();
  return 0;
}
