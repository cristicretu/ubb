import { Document } from '../types/document';
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

export async function createDocument(doc: Omit<Document, 'id' | 'usage'>): Promise<ApiResponse<Document>> {
  return fetchWithErrorHandling<Document>(`${API_URL}/document`, {
    method: 'POST',
    body: JSON.stringify(doc),
  });
}

export async function getAllDocuments(): Promise<ApiResponse<Document[]>> {
  return fetchWithErrorHandling<Document[]>(`${API_URL}/all`);
}

export async function getDocumentsByOwner(owner: string): Promise<ApiResponse<Document[]>> {
  return fetchWithErrorHandling<Document[]>(`${API_URL}/documents/${encodeURIComponent(owner)}`);
}

export async function deleteDocument(id: number): Promise<ApiResponse<void>> {
  return fetchWithErrorHandling<void>(`${API_URL}/document/${id}`, {
    method: 'DELETE',
  });
}
