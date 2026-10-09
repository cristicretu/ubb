import { API_URL } from '../config';
import { Recipe } from '../types/recipe';
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

export const getTypes = () => request<string[]>('/types');

export const getRecipesByType = (type: string) => request<Recipe[]>(`/recipes/${encodeURIComponent(type)}`);

export const createRecipe = (recipe: Omit<Recipe, 'id'>) =>
  request<Recipe>('/recipe', { method: 'POST', body: JSON.stringify(recipe) });

export const deleteRecipe = (id: number) =>
  request<{ success: boolean }>(`/recipe/${id}`, { method: 'DELETE' });

export const getLowRated = () => request<Recipe[]>('/low');

export const incrementRating = (recipeId: number) =>
  request<Recipe>('/increment', { method: 'POST', body: JSON.stringify({ recipeId }) });
