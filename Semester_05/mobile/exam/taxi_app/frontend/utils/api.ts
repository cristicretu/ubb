import { Cab } from '../types/cab';
import { log } from './logger';
import { API_URL } from '../config';

interface ApiResponse<T> {
  data: T | null;
  error: string | null;
}

async function fetchWithErrorHandling<T>(
  url: string,
  options?: RequestInit
): Promise<ApiResponse<T>> {
  try {
    log(`${options?.method || 'GET'} ${url}`, 'info');
    
    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...options?.headers,
      },
    });

    if (!response.ok) {
      const errorText = await response.text();
      const errorMessage = `HTTP ${response.status}: ${errorText || response.statusText}`;
      log(errorMessage, 'error');
      return { data: null, error: errorMessage };
    }

    const data = await response.json();
    log(`Response OK from ${url}`, 'success');
    return { data, error: null };
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(errorMessage, 'error');
    return { data: null, error: errorMessage };
  }
}

export async function createCab(cab: Cab): Promise<ApiResponse<Cab>> {
  return fetchWithErrorHandling<Cab>(`${API_URL}/cab`, {
    method: 'POST',
    body: JSON.stringify(cab),
  });
}

export async function getAllCabs(): Promise<ApiResponse<Cab[]>> {
  return fetchWithErrorHandling<Cab[]>(`${API_URL}/all`);
}

export async function getColors(): Promise<ApiResponse<string[]>> {
  return fetchWithErrorHandling<string[]>(`${API_URL}/colors`);
}

export async function getCabsByColor(color: string): Promise<ApiResponse<Cab[]>> {
  return fetchWithErrorHandling<Cab[]>(`${API_URL}/cabs/${encodeURIComponent(color)}`);
}

export async function deleteCab(id: number): Promise<ApiResponse<void>> {
  return fetchWithErrorHandling<void>(`${API_URL}/cab/${id}`, {
    method: 'DELETE',
  });
}

export async function getMyCabs(driver: string): Promise<ApiResponse<Cab[]>> {
  return fetchWithErrorHandling<Cab[]>(`${API_URL}/my/${encodeURIComponent(driver)}`);
}
