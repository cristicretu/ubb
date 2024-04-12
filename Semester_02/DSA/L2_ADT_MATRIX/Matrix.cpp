#include "Matrix.h"

#include <exception>
using namespace std;

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
Matrix::Matrix(int nrLines, int nrCols)
    : lines(nrLines), cols(nrCols), length(0) {
  head = nullptr;
  tail = nullptr;
}

/*
BC: Θ(1) - when the list is empty
WC: Θ(n) - when the list is not empty
TC: O(n)
*/
Matrix::~Matrix() {
  Node* current = head;
  while (current != nullptr) {
    Node* next = current->next;
    delete current;
    current = next;
  }
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Matrix::nrLines() const { return lines; }

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Matrix::nrColumns() const { return cols; }

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool Matrix::isInBounds(int i, int j) const {
  return i >= 0 && i < lines && j >= 0 && j < cols;
}

/*
BC: Θ(1) - when the list is empty, or the element is the first one
WC: Θ(n) - when the element is the last one or it is not in the list
TC: O(n)
*/
TElem Matrix::element(int i, int j) const {
  if (!isInBounds(i, j)) throw exception();  // Out of bounds

  Node* current = head;
  while (current != nullptr) {
    if (current->line == i && current->column == j) return current->value;
    current = current->next;
  }
  return NULL_TELEM;
}

/*
BC: Θ(1) - when we want to modify the first element
WC: Θ(n) - when we want to modify the last element or the element is not in the
list
TC: O(n)
*/
TElem Matrix::modify(int i, int j, TElem e) {
  if (!isInBounds(i, j)) throw exception();

  /// We try to find the good position to insert the element, in order to keep
  /// the list lexicographically ordered
  Node* current = head;
  Node* prev = nullptr;
  while (current != nullptr &&
         (current->line < i || (current->line == i && current->column < j))) {
    prev = current;
    current = current->next;
  }

  /// If the element is found
  if (current != nullptr && current->line == i && current->column == j) {
    TElem old = current->value;
    if (e == NULL_TELEM) {  /// If the new value is zero, remove the node
      if (prev) prev->next = current->next;
      if (current->next) current->next->prev = prev;

      if (current == head) head = current->next;
      if (current == tail) tail = prev;
      delete current;
      length--;
    } else {  /// If the new value is not zero, just update the value
      current->value = e;
    }
    return old;  /// Return the old value

  } else if (e != NULL_TELEM) {  /// If the element is not found and e is not
                                 /// zero, insert a new node
    Node* newNode = new Node{i, j, e, current, prev};
    if (prev) {
      prev->next = newNode;  /// Insert regulalry
    } else {
      head = newNode;  /// This is the first element then
    }
    if (current) {
      current->prev = newNode;  /// Insert regulalry
    } else {
      tail = newNode;  /// This is the last element then
    }
    length++;
  }
  return NULL_TELEM;
}

void Matrix::setElemsOnLine(int line, TElem elem) {
  if (line < 0 || line >= this->nrLines()) throw line;

  Node* current = head;
  Node* prev = nullptr;

  while (current != nullptr && current->line < line) {
    prev = current;
    current = current->next;
  }

  while (current != nullptr && current->line == line) {
    if (elem == NULL_TELEM) {
      Node* toDelete = current;
      if (prev) {
        prev->next = current->next;
      } else {
        head = current->next;
      }
      if (current->next) {
        current->next->prev = prev;
      } else {
        tail = prev;
      }
      current = current->next;
      delete toDelete;
      length--;
    } else if (current->value != elem) {
      current->value = elem;
      current = current->next;
    } else {
      current = current->next;
      prev = prev->next;
    }
  }

  if (elem != NULL_TELEM && (prev == nullptr || prev->line < line)) {
    for (int col = 0; col < this->nrColumns(); col++) {
      Node* newNode = new Node{line, col, elem, current, prev};
      if (prev) {
        prev->next = newNode;
      } else {
        head = newNode;
      }
      if (current) {
        current->prev = newNode;
      } else {
        tail = newNode;
      }
      prev = newNode;
      length++;
    }
  }
}