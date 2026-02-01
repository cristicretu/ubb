import AsyncStorage from '@react-native-async-storage/async-storage';
import { Item } from '../types/item';
import { log } from './logger';

const KEYS = {
  ITEMS: '@master:items',
  PENDING: '@master:pending',
  OWNER: '@master:owner',
};

export const saveItems = async (items: Item[]) => {
  try {
    await AsyncStorage.setItem(KEYS.ITEMS, JSON.stringify(items));
    log('Saved items to storage', 'success');
  } catch (error) {
    log('Failed to save items', 'error');
  }
};

export const getLocalItems = async (): Promise<Item[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.ITEMS);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    log('Failed to get local items', 'error');
    return [];
  }
};

export const savePendingItem = async (item: Item) => {
  try {
    const pending = await getPendingItems();
    pending.push(item);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log('Saved pending item', 'success');
  } catch (error) {
    log('Failed to save pending item', 'error');
  }
};

export const getPendingItems = async (): Promise<Item[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.PENDING);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    return [];
  }
};

export const clearPendingItems = async () => {
  try {
    await AsyncStorage.removeItem(KEYS.PENDING);
  } catch (error) {
    log('Failed to clear pending items', 'error');
  }
};

export const saveOwnerName = async (name: string) => {
  try {
    await AsyncStorage.setItem(KEYS.OWNER, name);
    log(`Saved owner: ${name}`, 'success');
  } catch (error) {
    log('Failed to save owner', 'error');
  }
};

export const getOwnerName = async (): Promise<string | null> => {
  try {
    return await AsyncStorage.getItem(KEYS.OWNER);
  } catch (error) {
    return null;
  }
};
