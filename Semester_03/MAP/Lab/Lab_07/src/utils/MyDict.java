package utils;

import java.util.Map;

import exceptions.DictionaryException;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public class MyDict<K, V> implements IDict<K, V> {
  private Map<K, V> dict;

  public MyDict() {
    dict = new HashMap<K, V>();
  }

  @Override
  public void put(K key, V value) {
    dict.put(key, value);
  }

  @Override
  public V get(K key) throws DictionaryException {
    if (!isDefined(key)) {
      throw new DictionaryException("Key not found in dictionary: " + key);
    }
    return dict.get(key);
  }

  @Override
  public boolean isDefined(K key) {
    return dict.containsKey(key);
  }

  @Override
  public String toString() {
    return dict.toString();
  }

  @Override
  public void update(K key, V value) throws DictionaryException {
    if (!isDefined(key)) {
      throw new DictionaryException("Key not found in dictionary: " + key);
    }
    dict.put(key, value);
  }

  @Override
  public List<V> getValues() {
    return new LinkedList<V>(dict.values());
  }

  @Override
  public IDict<K, V> deepCopy() {
    MyDict<K, V> copy = new MyDict<>();
    copy.dict = new HashMap<>(dict);
    return copy;
  }
}
