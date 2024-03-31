#include "Matrix.h"

#include <exception>
using namespace std;

Matrix::Matrix(int nrLines, int nrCols)
    : lines(nrLines), cols(nrCols), length(0) {
  head = nullptr;
  tail = nullptr;
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

// TElem Matrix::element(int i, int j) const {
//   if (!isInBounds(i, j)) {
//     throw exception();
//   }

//   Node* current = this->head->next;
//   while (current != this->tail) {
//     if (current->line == i && current->column == j) {
//       return current->value;
//     }
//     current = current->next;
//   }

//   return NULL_TELEM;
// }
TElem Matrix::element(int i, int j) const {
  if (!isInBounds(i, j)) throw exception();  // Out of bounds

  Node* current = head;
  while (current != nullptr) {
    if (current->line == i && current->column == j) return current->value;
    current = current->next;
  }
  return NULL_TELEM;
}

TElem Matrix::modify(int i, int j, TElem e) {
  if (!isInBounds(i, j)) throw exception();

  Node* current = head;
  Node* prev = nullptr;
  while (current != nullptr &&
         (current->line < i || (current->line == i && current->column < j))) {
    prev = current;
    current = current->next;
  }

  // If the element is found
  if (current != nullptr && current->line == i && current->column == j) {
    TElem old = current->value;
    if (e == NULL_TELEM) {  // If the new value is zero, remove the node
      if (prev) prev->next = current->next;
      if (current->next) current->next->prev = prev;
      if (current == head) head = current->next;
      if (current == tail) tail = prev;
      delete current;
      length--;
    } else {  // If the new value is not zero, just update the value
      current->value = e;
    }
    return old;
  } else if (e != NULL_TELEM) {  // If the element is not found and e is not
                                 // zero, insert a new node
    Node* newNode = new Node{i, j, e, current, prev};
    if (prev)
      prev->next = newNode;
    else
      head = newNode;
    if (current)
      current->prev = newNode;
    else
      tail = newNode;
    length++;
  }
  return NULL_TELEM;
}