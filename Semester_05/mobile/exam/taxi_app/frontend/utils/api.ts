import { FileItem } from '../types/file';
import { log } from './logger';
import { API_URL } from '../config';

interface ApiResponse<T> {
  data: T | null;
  error: string | null;
}

interface CreateFileInput {
  name: string;
  status: string;
  size: number;
  location: string;
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

export async function createFile(file: CreateFileInput): Promise<ApiResponse<FileItem>> {
  return fetchWithErrorHandling<FileItem>(`${API_URL}/file`, {
    method: 'POST',
    body: JSON.stringify(file),
  });
}

export async function getAllFiles(): Promise<ApiResponse<FileItem[]>> {
  return fetchWithErrorHandling<FileItem[]>(`${API_URL}/all`);
}

export async function getLocations(): Promise<ApiResponse<string[]>> {
  return fetchWithErrorHandling<string[]>(`${API_URL}/locations`);
}

export async function getFilesByLocation(location: string): Promise<ApiResponse<FileItem[]>> {
  return fetchWithErrorHandling<FileItem[]>(`${API_URL}/files/${encodeURIComponent(location)}`);
}

export async function deleteFile(id: number): Promise<ApiResponse<void>> {
  return fetchWithErrorHandling<void>(`${API_URL}/file/${id}`, {
    method: 'DELETE',
  });
}
