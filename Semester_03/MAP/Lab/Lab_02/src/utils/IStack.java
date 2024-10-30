package utils;

import java.util.List;

import exceptions.StackException;

public interface IStack<T> {
  void push(T item);

  T pop() throws StackException;

  T top() throws StackException;

  boolean isEmpty();

  int size();

  List<T> getList();
}
