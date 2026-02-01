import { Game } from '../types/game';
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

export async function createGame(game: Omit<Game, 'id' | 'popularityScore'>): Promise<ApiResponse<Game>> {
  return fetchWithErrorHandling<Game>(`${API_URL}/game`, {
    method: 'POST',
    body: JSON.stringify(game),
  });
}

export async function getAllGames(): Promise<ApiResponse<Game[]>> {
  return fetchWithErrorHandling<Game[]>(`${API_URL}/allGames`);
}

export async function getGamesByUser(user: string): Promise<ApiResponse<Game[]>> {
  return fetchWithErrorHandling<Game[]>(`${API_URL}/games/${encodeURIComponent(user)}`);
}

export async function getAvailableGames(): Promise<ApiResponse<Game[]>> {
  return fetchWithErrorHandling<Game[]>(`${API_URL}/ready`);
}

export async function bookGame(gameId: number, user: string): Promise<ApiResponse<Game>> {
  return fetchWithErrorHandling<Game>(`${API_URL}/book`, {
    method: 'POST',
    body: JSON.stringify({ gameId, user }),
  });
}
