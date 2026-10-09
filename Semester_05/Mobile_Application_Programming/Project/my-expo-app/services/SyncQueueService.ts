import { db } from '../db/client';
import { syncQueue as syncQueueTable } from '../db/schema';
import { eq } from 'drizzle-orm';

export type SyncOperation = 'CREATE' | 'UPDATE' | 'DELETE';
export type SyncStatus = 'pending' | 'syncing' | 'synced' | 'failed';

export interface SyncQueueItem {
  id?: number;
  operation: SyncOperation;
  carId: string;
  carData?: string; // JSON
  timestamp: string;
  status: SyncStatus;
}

export class SyncQueueService {
  async addToQueue(
    operation: SyncOperation,
    carId: string,
    carData?: any
  ): Promise<void> {
    try {
      await db.insert(syncQueueTable).values({
        operation,
        carId,
        carData: carData ? JSON.stringify(carData) : null,
        timestamp: new Date().toISOString(),
        status: 'pending',
      });
      console.log(`[SyncQueue] Added ${operation} for car ${carId}`);
    } catch (error) {
      console.error('[SyncQueue] Error adding to queue:', error);
      throw error;
    }
  }

  async getPendingOperations(): Promise<SyncQueueItem[]> {
    try {
      const items = await db
        .select()
        .from(syncQueueTable);

      const retryableItems = items.filter(
        item => item.status === 'pending' || item.status === 'failed'
      );

      return retryableItems.map(item => ({
        id: item.id,
        operation: item.operation as SyncOperation,
        carId: item.carId,
        carData: item.carData ?? undefined,
        timestamp: item.timestamp,
        status: item.status as SyncStatus,
      }));
    } catch (error) {
      console.error('[SyncQueue] Error getting pending operations:', error);
      return [];
    }
  }

  async updateStatus(id: number, status: SyncStatus): Promise<void> {
    try {
      await db
        .update(syncQueueTable)
        .set({ status })
        .where(eq(syncQueueTable.id, id));
      console.log(`[SyncQueue] Updated item ${id} status to ${status}`);
    } catch (error) {
      console.error('[SyncQueue] Error updating status:', error);
    }
  }

  async clearSynced(): Promise<void> {
    try {
      await db
        .delete(syncQueueTable)
        .where(eq(syncQueueTable.status, 'synced'));
      console.log('[SyncQueue] Cleared synced items');
    } catch (error) {
      console.error('[SyncQueue] Error clearing synced:', error);
    }
  }

  async getQueueSize(): Promise<number> {
    try {
      const items = await db.select().from(syncQueueTable);
      return items.length;
    } catch (error) {
      console.error('[SyncQueue] Error getting queue size:', error);
      return 0;
    }
  }

  async clearAll(): Promise<void> {
    try {
      await db.delete(syncQueueTable);
      console.log('[SyncQueue] Cleared all items');
    } catch (error) {
      console.error('[SyncQueue] Error clearing all:', error);
    }
  }
}

export const syncQueueService = new SyncQueueService();


