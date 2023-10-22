/*
P2. Count the number of associative operations on a set A of size n. For n <= 4,
print the operation tables of each associative operation on a set A of size n.

@Author: Cretu Cristian, 913

Usage: ./main 01_input.txt
*/

#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

// Global variables
int n;                                             // Size of the set A
std::vector<std::vector<std::vector<int>>> tables; // Vector of all the
                                                   // associative tables
std::mutex mtx; // Mutex to ensure thread safety when modifying the global
// tables vector

/*
 Check if a binary operation for a set A is associative
 Associativity is defined by the following property:
    (a * b) * c = a * (b * c)
 */
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

// Recursive backtracking function to generate all possible operation tables and
// check if they are associative
void generateTables(int i, int j, std::vector<std::vector<int>> &table) {
  if (i == n) {
    if (isAssociative(table)) {
      mtx.lock(); // Lock to ensure thread safety
      tables.push_back(table);
      mtx.unlock(); // Release the lock
    }
    return;
  }
  for (int x = 0; x < n; x++) {
    table[i][j] = x;
    if (j == n - 1) // If end of row, move to next row
      generateTables(i + 1, 0, table);
    else
      generateTables(i, j + 1, table);
  }
}

// Threaded function to distribute the task of generating tables
// Speeds up the execution time by a factor of ~4x on my machine
void threadedGenerateTables(int startValue) {
  std::vector<std::vector<int>> table(n, std::vector<int>(n, 0));
  table[0][0] = startValue;
  generateTables(0, 1, table);
}

int main(int argc, char *argv[]) {
  // Check for correct number of command line arguments
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

  inFile >> n; // Read the size of the set A
  if (inFile.fail()) {
    std::cout << "Invalid input in '" << input_file
              << "'. Please provide a single integer on the first line."
              << std::endl;
    return 1;
  }

  // Derive output file name from the input file name
  std::string baseName = input_file.substr(0, input_file.find('_'));
  std::string output_file = baseName + "_output.txt";
  std::ofstream outFile(output_file);

  int numThreads =
      std::thread::hardware_concurrency(); // Get number of supported threads
  std::vector<std::thread> threads;

  // Launch threads for each possible start value
  for (int i = 0; i < numThreads && i < n; ++i) {
    threads.push_back(std::thread(threadedGenerateTables, i));
  }

  // Wait for all threads to finish
  for (auto &th : threads) {
    th.join();
  }

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
