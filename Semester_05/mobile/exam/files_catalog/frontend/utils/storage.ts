import AsyncStorage from '@react-native-async-storage/async-storage';
import { FileItem } from '../types/file';
import { log } from './logger';

const STORAGE_KEYS = {
  FILES: '@files_catalog:files',
  PENDING_FILES: '@files_catalog:pending_files',
};

/**
 * Save files locally for offline access
 */
export async function saveFiles(files: FileItem[]): Promise<void> {
  try {
    const jsonValue = JSON.stringify(files);
    await AsyncStorage.setItem(STORAGE_KEYS.FILES, jsonValue);
    log(`Saved ${files.length} files to local storage`, 'success');
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(`Error saving files to storage: ${errorMessage}`, 'error');
    throw error;
  }
}

/**
 * Get cached files from local storage
 */
export async function getLocalFiles(): Promise<FileItem[]> {
  try {
    const jsonValue = await AsyncStorage.getItem(STORAGE_KEYS.FILES);
    if (jsonValue != null) {
      const files = JSON.parse(jsonValue) as FileItem[];
      log(`Retrieved ${files.length} files from local storage`, 'success');
      return files;
    }
    log('No cached files found in local storage', 'info');
    return [];
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(`Error reading files from storage: ${errorMessage}`, 'error');
    return [];
  }
}

/**
 * Save a file to be synced when online
 */
export async function savePendingFile(file: FileItem): Promise<void> {
  try {
    const pendingFiles = await getPendingFiles();
    pendingFiles.push(file);
    const jsonValue = JSON.stringify(pendingFiles);
    await AsyncStorage.setItem(STORAGE_KEYS.PENDING_FILES, jsonValue);
    log(`Saved pending file: ${file.name}`, 'success');
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(`Error saving pending file: ${errorMessage}`, 'error');
    throw error;
  }
}

/**
 * Get all pending files that need to be synced
 */
export async function getPendingFiles(): Promise<FileItem[]> {
  try {
    const jsonValue = await AsyncStorage.getItem(STORAGE_KEYS.PENDING_FILES);
    if (jsonValue != null) {
      const files = JSON.parse(jsonValue) as FileItem[];
      log(`Retrieved ${files.length} pending files`, 'info');
      return files;
    }
    return [];
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(`Error reading pending files: ${errorMessage}`, 'error');
    return [];
  }
}

/**
 * Clear pending files after successful sync
 */
export async function clearPendingFiles(): Promise<void> {
  try {
    await AsyncStorage.removeItem(STORAGE_KEYS.PENDING_FILES);
    log('Cleared pending files', 'success');
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    log(`Error clearing pending files: ${errorMessage}`, 'error');
    throw error;
  }
}
