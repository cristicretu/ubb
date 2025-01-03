package utils;

import java.util.List;
import java.util.Map;
import java.util.Set;

import exceptions.HeapException;
import model.value.IValue;

public interface IHeap<K, V> {
  V get(K key) throws HeapException;

  void put(K key, V value);

  void remove(K key);

  boolean isDefined(K key);

  String toString();

  Map<K, V> getHeap();

  List<V> getValues();

  void setHeap(Map<K, V> dictionary);

  Map<Integer, IValue> safeGarbageCollector(Set<Integer> unionSetOfUsedAddr, Map<Integer, IValue> heap);

  Integer allocate();
}