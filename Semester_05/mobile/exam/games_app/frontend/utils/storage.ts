import AsyncStorage from '@react-native-async-storage/async-storage';
import { Document } from '../types/document';
import { log } from './logger';

const KEYS = {
  DOCS: '@documents_app:docs',
  PENDING: '@documents_app:pending',
  OWNER: '@documents_app:owner',
};

export async function saveDocs(docs: Document[]): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.DOCS, JSON.stringify(docs));
    log(`Saved ${docs.length} docs`, 'success');
  } catch (error) {
    log(`Error saving docs: ${error}`, 'error');
    throw error;
  }
}

export async function getLocalDocs(): Promise<Document[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.DOCS);
    if (json) {
      const docs = JSON.parse(json) as Document[];
      log(`Loaded ${docs.length} cached docs`, 'success');
      return docs;
    }
    log('No cached docs', 'info');
    return [];
  } catch (error) {
    log(`Error loading docs: ${error}`, 'error');
    return [];
  }
}

export async function savePendingDoc(doc: Document): Promise<void> {
  try {
    const pending = await getPendingDocs();
    pending.push(doc);
    await AsyncStorage.setItem(KEYS.PENDING, JSON.stringify(pending));
    log(`Saved pending: ${doc.name}`, 'success');
  } catch (error) {
    log(`Error saving pending: ${error}`, 'error');
    throw error;
  }
}

export async function getPendingDocs(): Promise<Document[]> {
  try {
    const json = await AsyncStorage.getItem(KEYS.PENDING);
    return json ? JSON.parse(json) : [];
  } catch (error) {
    log(`Error loading pending: ${error}`, 'error');
    return [];
  }
}

export async function saveOwnerName(owner: string): Promise<void> {
  try {
    await AsyncStorage.setItem(KEYS.OWNER, owner);
    log(`Saved owner: ${owner}`, 'success');
  } catch (error) {
    log(`Error saving owner: ${error}`, 'error');
    throw error;
  }
}

export async function getOwnerName(): Promise<string | null> {
  try {
    const owner = await AsyncStorage.getItem(KEYS.OWNER);
    return owner;
  } catch (error) {
    log(`Error loading owner: ${error}`, 'error');
    return null;
  }
}
