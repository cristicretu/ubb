package utils;

import exceptions.DictionaryException;

public interface IDict<K, V> {
  void put(K key, V value);

  V get(K key) throws DictionaryException;

  boolean isDefined(K key);

  void update(K key, V value) throws DictionaryException;

}
