package utils;

public interface IStack<T> {
  void push(T item);

  T pop();

  T top();

  boolean isEmpty();

  int size();
}
