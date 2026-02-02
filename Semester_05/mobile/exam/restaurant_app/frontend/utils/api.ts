import { API_URL } from '../config';
import { Order } from '../types/order';
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

export const createOrder = (order: Omit<Order, 'id'>) =>
  request<Order>('/order', { method: 'POST', body: JSON.stringify(order) });

export const getReadyOrders = () => request<Order[]>('/orders');

export const getOrderById = (id: number) => request<Order>(`/order/${id}`);

export const getRecordedOrders = () => request<Order[]>('/recorded');

export const updateStatus = (orderId: number, status: string) =>
  request<Order>('/status', { method: 'POST', body: JSON.stringify({ orderId, status }) });

export const getOrderByTable = (table: string) => request<Order>(`/my/${encodeURIComponent(table)}`);
