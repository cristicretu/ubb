import AsyncStorage from '@react-native-async-storage/async-storage';
import { Game } from '../types/game';
import { log } from './logger';

const KEYS = {
  GAMES: '@games_app:games',
  PENDING: '@games_app:pending',
  USER: '@games_app:user',
};

export async function saveGames(games: Game[]): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.GAMES, JSON.stringify(games));
    log(`Saved ${games.length} games`, 'success');
  } catch (error) {
    log(`Error saving games: ${error}`, 'error');
    throw error;
  }
}

export async function getLocalGames(): Promise<Game[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.GAMES);
    if (json) {
      const games = JSON.parse(json) as Game[];
      log(`Loaded ${games.length} cached games`, 'success');
      return games;
    }
    log('No cached games', 'info');
    return [];
  } catch (error) {
    log(`Error loading games: ${error}`, 'error');
    return [];
  }
}

export async function savePendingGame(game: Game): Promise<void> {
  try {
    const pending = await getPendingGames();
    pending.push(game);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log(`Saved pending: ${game.name}`, 'success');
  } catch (error) {
    log(`Error saving pending: ${error}`, 'error');
    throw error;
  }
}

export async function getPendingGames(): Promise<Game[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.PENDING);
    return json ? JSON.parse(json) : [];
  } catch (error) {
    log(`Error loading pending: ${error}`, 'error');
    return [];
  }
}

export async function saveUserName(user: string): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.USER, user);
    log(`Saved user: ${user}`, 'success');
  } catch (error) {
    log(`Error saving user: ${error}`, 'error');
    throw error;
  }
}

export async function getUserName(): Promise<string | null> {
  try {
    const user = await AsyncStorage.getItem(KEYS.USER);
    return user;
  } catch (error) {
    log(`Error loading user: ${error}`, 'error');
    return null;
  }
}
