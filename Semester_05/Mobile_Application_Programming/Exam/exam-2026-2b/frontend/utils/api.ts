import { API_URL } from '../config';
import { Payment } from '../types/payment';
import { log } from './logger';

interface ApiResponse<T> {
  data?: T;
  error?: string;
}

async function request<T>(endpoint: string, options?: RequestInit): Promise<ApiResponse<T>> {
  try {
    log(`API ${options?.method || 'GET'} ${endpoint}`, 'info');
    const response = await fetch(`${API_URL}${endpoint}`, {
      headers: { 'Content-Type': 'application/json' },
      ...options,
    });
    if (!response.ok) {
      const err = await response.json();
      log(`API Error: ${err.error || response.statusText}`, 'error');
      return { error: err.error || response.statusText };
    }
    const data = await response.json();
    log(`API Success: ${endpoint}`, 'success');
    return { data };
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Network error';
    log(`API Error: ${message}`, 'error');
    return { error: message };
  }
}

export const getPayments = () => request<Payment[]>('/payments');

export const getPaymentById = (id: number) => request<Payment>(`/payment/${id}`);

export const createPayment = (payment: Omit<Payment, 'id'>) =>
  request<Payment>('/payment', { method: 'POST', body: JSON.stringify(payment) });

export const deletePayment = (id: number) =>
  request<Payment>(`/payment/${id}`, { method: 'DELETE' });

export const getAllPayments = () => request<Payment[]>('/allPayments');
