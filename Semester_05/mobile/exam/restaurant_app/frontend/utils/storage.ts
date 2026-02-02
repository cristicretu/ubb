import AsyncStorage from '@react-native-async-storage/async-storage';
import { Order } from '../types/order';
import { log } from './logger';

const KEYS = {
  ORDERS: '@restaurant_app:orders',
  PENDING: '@restaurant_app:pending',
};

export const saveOrders = async (orders: Order[]) => {
  try {
    await AsyncStorage.setItem(KEYS.ORDERS, JSON.stringify(orders));
    log('Saved orders to storage', 'success');
  } catch (error) {
    log('Failed to save orders', 'error');
  }
};

export const getLocalOrders = async (): Promise<Order[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.ORDERS);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    log('Failed to get local orders', 'error');
    return [];
  }
};

export const savePendingOrder = async (order: Order) => {
  try {
    const pending = await getPendingOrders();
    pending.push(order);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log('Saved pending order', 'success');
  } catch (error) {
    log('Failed to save pending order', 'error');
  }
};

export const getPendingOrders = async (): Promise<Order[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.PENDING);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    return [];
  }
};

export const clearPendingOrders = async () => {
  try {
    await AsyncStorage.removeItem(KEYS.PENDING);
    log('Cleared pending orders', 'success');
  } catch (error) {
    log('Failed to clear pending orders', 'error');
  }
};
