package utils;

import java.util.Stack;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;

public class MyStack<T> implements IStack<T> {
  private Stack<T> stack;

  public MyStack() {
    stack = new Stack<>();
  }

  @Override
  public void push(T item) {
    stack.push(item);
  }

  @Override
  public T pop() {
    return stack.pop();
  }

  @Override
  public T top() {
    return stack.peek();
  }

  @Override
  public boolean isEmpty() {
    return stack.isEmpty();
  }

  @Override
  public int size() {
    return stack.size();
  }

  @Override
  public String toString() {
    return stack.toString();
  }

  @Override
  public List<T> getList() {
    List<T> list = new ArrayList<>(stack);
    Collections.reverse(list);
    return list;
  }
}
