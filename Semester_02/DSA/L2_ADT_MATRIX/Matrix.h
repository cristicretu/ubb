#pragma once

// DO NOT CHANGE THIS PART
typedef int TElem;
#define NULL_TELEM 0

/*
ADT Matrix - represented as a sparse matrix, using a Double linked list with
<line, column, value> triples ordered lexicographifcally considering the line
and column of every element
*/

typedef struct Node {
  int line;
  int column;
  TElem value;
  Node* next;
  Node* prev;
};

class Matrix {
 private:
  // TODO - Representation
  Node* head;
  Node* tail;
  int lines;
  int cols;
  int length;

 public:
  // constructor
  Matrix(int nrLines, int nrCols);

  ~Matrix();

  // returns the number of lines
  int nrLines() const;

  // returns the number of columns
  int nrColumns() const;

  // returns the element from line i and column j (indexing starts from 0)
  // throws exception if (i,j) is not a valid position in the Matrix
  TElem element(int i, int j) const;

  // modifies the value from line i and column j
  // returns the previous value from the position
  // throws exception if (i,j) is not a valid position in the Matrix
  TElem modify(int i, int j, TElem e);

  bool isInBounds(int i, int j) const;

  void setElemsOnLine(int line, TElem elem);
};
