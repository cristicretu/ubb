import AsyncStorage from '@react-native-async-storage/async-storage';
import { Cab } from '../types/cab';
import { log } from './logger';

const KEYS = {
  CABS: '@taxi_app:cabs',
  PENDING: '@taxi_app:pending',
  DRIVER_NAME: '@taxi_app:driver_name',
};

export async function saveCabs(cabs: Cab[]): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.CABS, JSON.stringify(cabs));
    log(`Saved ${cabs.length} cabs`, 'success');
  } catch (error) {
    log(`Error saving cabs: ${error}`, 'error');
    throw error;
  }
}

export async function getLocalCabs(): Promise<Cab[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.CABS);
    if (json) {
      const cabs = JSON.parse(json) as Cab[];
      log(`Loaded ${cabs.length} cached cabs`, 'success');
      return cabs;
    }
    log('No cached cabs', 'info');
    return [];
  } catch (error) {
    log(`Error loading cabs: ${error}`, 'error');
    return [];
  }
}

export async function savePendingCab(cab: Cab): Promise<void> {
  try {
    const pending = await getPendingCabs();
    pending.push(cab);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log(`Saved pending: ${cab.name}`, 'success');
  } catch (error) {
    log(`Error saving pending: ${error}`, 'error');
    throw error;
  }
}

export async function getPendingCabs(): Promise<Cab[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.PENDING);
    return json ? JSON.parse(json) : [];
  } catch (error) {
    log(`Error loading pending: ${error}`, 'error');
    return [];
  }
}

export async function clearPendingCabs(): Promise<void> {
  try {
    await AsyncStorage.removeItem(KEYS.PENDING);
    log('Cleared pending cabs', 'success');
  } catch (error) {
    log(`Error clearing pending: ${error}`, 'error');
    throw error;
  }
}

export async function saveDriverName(name: string): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.DRIVER_NAME, name);
    log(`Saved driver name: ${name}`, 'success');
  } catch (error) {
    log(`Error saving driver name: ${error}`, 'error');
    throw error;
  }
}

export async function getDriverName(): Promise<string | null> {
  try {
    return await AsyncStorage.getItem(KEYS.DRIVER_NAME);
  } catch (error) {
    log(`Error loading driver name: ${error}`, 'error');
    return null;
  }
}
