import AsyncStorage from '@react-native-async-storage/async-storage';
import { Payment } from '../types/payment';
import { log } from './logger';

const KEYS = {
  PAYMENTS: '@salary_app:payments',
  PAYMENT_DETAILS: '@salary_app:payment_details',
};

export const savePayments = async (payments: Payment[]) => {
  try {
    await AsyncStorage.setItem(KEYS.PAYMENTS, JSON.stringify(payments));
    log('Saved payments to storage', 'success');
  } catch (error) {
    log('Failed to save payments', 'error');
  }
};

export const getLocalPayments = async (): Promise<Payment[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.PAYMENTS);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    log('Failed to get local payments', 'error');
    return [];
  }
};

export const savePaymentDetail = async (id: number, payment: Payment) => {
  try {
    const key = `${KEYS.PAYMENT_DETAILS}:${id}`;
    await AsyncStorage.setItem(key, JSON.stringify(payment));
    log('Saved payment detail', 'success');
  } catch (error) {
    log('Failed to save payment detail', 'error');
  }
};

export const getPaymentDetail = async (id: number): Promise<Payment | null> => {
  try {
    const key = `${KEYS.PAYMENT_DETAILS}:${id}`;
    const data = await AsyncStorage.getItem(key);
    return data ? JSON.parse(data) : null;
  } catch (error) {
    log('Failed to get payment detail', 'error');
    return null;
  }
};
