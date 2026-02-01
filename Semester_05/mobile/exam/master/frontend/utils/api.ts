import { API_URL } from '../config';
import { Item } from '../types/item';
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

export const createItem = (item: Omit<Item, 'id' | 'value2'>) =>
  request<Item>('/item', { method: 'POST', body: JSON.stringify(item) });

export const getAllItems = () => request<Item[]>('/all');

export const getItemsByOwner = (owner: string) => request<Item[]>(`/items/${encodeURIComponent(owner)}`);

export const getAvailableItems = () => request<Item[]>('/available');

export const deleteItem = (id: number) => request<{ success: boolean }>(`/item/${id}`, { method: 'DELETE' });

export const performAction = (itemId: number, status: string, owner: string) =>
  request<Item>('/action', { method: 'POST', body: JSON.stringify({ itemId, status, owner }) });
