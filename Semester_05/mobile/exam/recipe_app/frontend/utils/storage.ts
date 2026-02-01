import AsyncStorage from '@react-native-async-storage/async-storage';
import { Recipe } from '../types/recipe';
import { log } from './logger';

const KEYS = {
  TYPES: '@recipe_app:types',
  RECIPES: '@recipe_app:recipes',
};

export const saveTypes = async (types: string[]) => {
  try {
    await AsyncStorage.setItem(KEYS.TYPES, JSON.stringify(types));
    log('Saved types to storage', 'success');
  } catch (error) {
    log('Failed to save types', 'error');
  }
};

export const getLocalTypes = async (): Promise<string[]> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.TYPES);
    return data ? JSON.parse(data) : [];
  } catch (error) {
    log('Failed to get local types', 'error');
    return [];
  }
};

export const saveRecipes = async (type: string, recipes: Recipe[]) => {
  try {
    const all = await getAllLocalRecipes();
    all[type] = recipes;
    await AsyncStorage.setItem(KEYS.RECIPES, JSON.stringify(all));
    log(`Saved ${recipes.length} recipes for type: ${type}`, 'success');
  } catch (error) {
    log('Failed to save recipes', 'error');
  }
};

export const getLocalRecipes = async (type: string): Promise<Recipe[]> => {
  try {
    const all = await getAllLocalRecipes();
    return all[type] || [];
  } catch (error) {
    log('Failed to get local recipes', 'error');
    return [];
  }
};

export const getAllLocalRecipes = async (): Promise<Record<string, Recipe[]>> => {
  try {
    const data = await AsyncStorage.getItem(KEYS.RECIPES);
    return data ? JSON.parse(data) : {};
  } catch (error) {
    return {};
  }
};
