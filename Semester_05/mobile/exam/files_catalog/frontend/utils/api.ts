import { FileItem } from '../types/file';
import { log } from './logger';

// API URL - configurable, defaults to Android emulator address
export const API_URL = process.env.EXPO_PUBLIC_API_URL || 'http://10.0.2.2:3000';

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

/**
 * Helper function for fetch with error handling
 */
async function fetchWithErrorHandling<T>(
  url: string,
  options?: RequestInit
): Promise<ApiResponse<T>> {
  try {
    log(`Making ${options?.method || 'GET'} request to ${url}`, 'info');
    
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
      log(`API Error: ${errorMessage}`, 'error');
      return { data: null, error: errorMessage };
    }

    const data = await response.json();
    log(`Successfully received response from ${url}`, 'success');
    return { data, error: null };
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error occurred';
    log(`Network error: ${errorMessage}`, 'error');
    return { data: null, error: errorMessage };
  }
}

/**
 * Create a new file
 * POST /file
 */
export async function createFile(file: CreateFileInput): Promise<ApiResponse<FileItem>> {
  log(`Creating file: ${file.name}`, 'info');
  return fetchWithErrorHandling<FileItem>(`${API_URL}/file`, {
    method: 'POST',
    body: JSON.stringify(file),
  });
}

/**
 * Get all files
 * GET /all
 */
export async function getAllFiles(): Promise<ApiResponse<FileItem[]>> {
  log('Fetching all files', 'info');
  return fetchWithErrorHandling<FileItem[]>(`${API_URL}/all`);
}

/**
 * Get all locations
 * GET /locations
 */
export async function getLocations(): Promise<ApiResponse<string[]>> {
  log('Fetching locations', 'info');
  return fetchWithErrorHandling<string[]>(`${API_URL}/locations`);
}

/**
 * Get files by location
 * GET /files/:location
 */
export async function getFilesByLocation(location: string): Promise<ApiResponse<FileItem[]>> {
  log(`Fetching files for location: ${location}`, 'info');
  return fetchWithErrorHandling<FileItem[]>(`${API_URL}/files/${encodeURIComponent(location)}`);
}

/**
 * Delete a file by ID
 * DELETE /file/:id
 */
export async function deleteFile(id: number): Promise<ApiResponse<void>> {
  log(`Deleting file with ID: ${id}`, 'info');
  return fetchWithErrorHandling<void>(`${API_URL}/file/${id}`, {
    method: 'DELETE',
  });
}
