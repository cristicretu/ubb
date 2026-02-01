import AsyncStorage from '@react-native-async-storage/async-storage';
import { FileItem } from '../types/file';
import { log } from './logger';

const KEYS = {
  FILES: '@files_catalog:files',
  PENDING: '@files_catalog:pending',
};

export async function saveFiles(files: FileItem[]): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.FILES, JSON.stringify(files));
    log(`Saved ${files.length} files`, 'success');
  } catch (error) {
    log(`Error saving files: ${error}`, 'error');
    throw error;
  }
}

export async function getLocalFiles(): Promise<FileItem[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.FILES);
    if (json) {
      const files = JSON.parse(json) as FileItem[];
      log(`Loaded ${files.length} cached files`, 'success');
      return files;
    }
    log('No cached files', 'info');
    return [];
  } catch (error) {
    log(`Error loading files: ${error}`, 'error');
    return [];
  }
}

export async function savePendingFile(file: FileItem): Promise<void> {
  try {
    const pending = await getPendingFiles();
    pending.push(file);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log(`Saved pending: ${file.name}`, 'success');
  } catch (error) {
    log(`Error saving pending: ${error}`, 'error');
    throw error;
  }
}

export async function getPendingFiles(): Promise<FileItem[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.PENDING);
    return json ? JSON.parse(json) : [];
  } catch (error) {
    log(`Error loading pending: ${error}`, 'error');
    return [];
  }
}

export async function clearPendingFiles(): Promise<void> {
  try {
    await AsyncStorage.removeItem(KEYS.PENDING);
    log('Cleared pending files', 'success');
  } catch (error) {
    log(`Error clearing pending: ${error}`, 'error');
    throw error;
  }
}
