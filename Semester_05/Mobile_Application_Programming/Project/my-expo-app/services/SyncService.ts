import { syncQueueService, SyncQueueItem } from './SyncQueueService';
import { apiClient, ApiCar, CreateCarData } from './ApiClient';
import { db } from '../db/client';
import { cars as carsTable } from '../db/schema';
import { eq } from 'drizzle-orm';
import { carsRepository, isLocalId } from '../repositories/CarsRepository';
import { Car } from '../types/Car';

type IdMappingCallback = (localId: string, serverId: string) => void;
type CarsUpdatedCallback = (cars: Car[]) => void;

export class SyncService {
  private isSyncing: boolean = false;
  private syncListeners: Array<(status: 'syncing' | 'synced' | 'failed') => void> = [];
  private idMappingCallbacks: IdMappingCallback[] = [];
  private carsUpdatedCallbacks: CarsUpdatedCallback[] = [];

  onIdMapping(callback: IdMappingCallback) {
    this.idMappingCallbacks.push(callback);
  }

  removeIdMappingCallback(callback: IdMappingCallback) {
    this.idMappingCallbacks = this.idMappingCallbacks.filter(cb => cb !== callback);
  }

  onCarsUpdated(callback: CarsUpdatedCallback) {
    this.carsUpdatedCallbacks.push(callback);
  }

  removeCarsUpdatedCallback(callback: CarsUpdatedCallback) {
    this.carsUpdatedCallbacks = this.carsUpdatedCallbacks.filter(cb => cb !== callback);
  }

  private notifyIdMapping(localId: string, serverId: string) {
    console.log('[Sync] Notifying ID mapping:', localId, '->', serverId);
    this.idMappingCallbacks.forEach(cb => cb(localId, serverId));
  }

  private notifyCarsUpdated(cars: Car[]) {
    console.log('[Sync] Notifying cars updated:', cars.length, 'cars');
    this.carsUpdatedCallbacks.forEach(cb => cb(cars));
  }

  async syncPendingOperations(): Promise<{ success: boolean; message: string }> {
    if (this.isSyncing) {
      console.log('[Sync] Already syncing, skipping');
      return { success: false, message: 'Sync already in progress' };
    }

    try {
      this.isSyncing = true;
      this.notifyListeners('syncing');
      console.log('[Sync] Starting sync process');

      const pendingOps = await syncQueueService.getPendingOperations();

      if (pendingOps.length === 0) {
        console.log('[Sync] No pending operations');
        this.notifyListeners('synced');
        return { success: true, message: 'No pending operations' };
      }

      console.log(`[Sync] Processing ${pendingOps.length} operations`);

      let successCount = 0;
      let conflictCount = 0;
      let failCount = 0;

      for (const op of pendingOps) {
        try {
          await syncQueueService.updateStatus(op.id!, 'syncing');

          switch (op.operation) {
            case 'CREATE':
              await this.syncCreate(op);
              break;
            case 'UPDATE':
              await this.syncUpdate(op);
              break;
            case 'DELETE':
              await this.syncDelete(op);
              break;
          }

          await syncQueueService.updateStatus(op.id!, 'synced');
          successCount++;
        } catch (error) {
          const errorMsg = error instanceof Error ? error.message : 'Unknown error';

          if (errorMsg.includes('CONFLICT') || errorMsg.includes('Conflict')) {
            console.log(`[Sync] Conflict resolved for operation ${op.id} - server version kept`);
            await syncQueueService.updateStatus(op.id!, 'synced');
            conflictCount++;
          } else {
            console.log(`[Sync] Error syncing operation ${op.id}:`, errorMsg);
            await syncQueueService.updateStatus(op.id!, 'failed');
            failCount++;
          }
        }
      }

      await syncQueueService.clearSynced();

      let message = `Synced ${successCount} operations`;
      if (conflictCount > 0) {
        message += ` (${conflictCount} conflicts resolved - server version kept)`;
      }
      if (failCount > 0) {
        message += `, ${failCount} failed`;
      }
      console.log('[Sync]', message);

      this.notifyListeners(failCount > 0 ? 'failed' : 'synced');
      return { success: failCount === 0, message };

    } catch (error) {
      console.error('[Sync] Sync process error:', error);
      this.notifyListeners('failed');
      return {
        success: false,
        message: `Sync failed: ${error instanceof Error ? error.message : 'Unknown error'}`
      };
    } finally {
      this.isSyncing = false;
    }
  }

  async fetchAndMergeFromServer(): Promise<Car[]> {
    try {
      console.log('[Sync] Fetching cars from server...');
      const serverCars = await apiClient.fetchCars();
      console.log('[Sync] Fetched', serverCars.length, 'cars from server');

      const localCars = await carsRepository.loadCars();
      const localCarIds = new Set(localCars.map(c => c.id));
      const localPendingIds = new Set(localCars.filter(c => isLocalId(c.id)).map(c => c.id));

      const newCarsFromServer: Car[] = [];

      for (const serverCar of serverCars) {
        if (!localCarIds.has(serverCar.id)) {
          const car: Car & { updatedAt: string } = {
            id: serverCar.id,
            make: serverCar.make,
            model: serverCar.model,
            year: serverCar.year,
            licensePlate: serverCar.licensePlate,
            currentMileage: serverCar.currentMileage,
            lastITPDate: new Date(serverCar.lastITPDate),
            nextITPDate: new Date(serverCar.nextITPDate),
            rovinietaExpiryDate: new Date(serverCar.rovinietaExpiryDate),
            carDescription: serverCar.carDescription,
            updatedAt: serverCar.updatedAt,
          };

          await carsRepository.insertCarFromServer(car);
          newCarsFromServer.push(car);
          console.log('[Sync] Inserted new car from server:', serverCar.id);
        }
      }

      if (newCarsFromServer.length > 0) {
        console.log('[Sync] Inserted', newCarsFromServer.length, 'new cars from server');
        const allCars = await carsRepository.loadCars();
        this.notifyCarsUpdated(allCars);
        return allCars;
      }

      return localCars;
    } catch (error) {
      console.error('[Sync] Error fetching from server:', error);
      throw error;
    }
  }

  private async syncCreate(op: SyncQueueItem): Promise<void> {
    const carData = JSON.parse(op.carData || '{}');
    const localId = op.carId;

    const createData: CreateCarData = {
      localId,
      make: carData.make,
      model: carData.model,
      year: carData.year,
      licensePlate: carData.licensePlate,
      currentMileage: carData.currentMileage,
      lastITPDate: carData.lastITPDate,
      nextITPDate: carData.nextITPDate,
      rovinietaExpiryDate: carData.rovinietaExpiryDate,
      carDescription: carData.carDescription,
    };

    console.log('[Sync] Creating car on server, localId:', localId);
    const result = await apiClient.createCar(createData);

    const serverId = result.car.id;
    console.log('[Sync] Server assigned ID:', serverId, 'for localId:', localId);

    await carsRepository.replaceLocalIdWithServerId(localId, serverId, result.car.updatedAt);
    this.notifyIdMapping(localId, serverId);
  }

  private async syncUpdate(op: SyncQueueItem): Promise<void> {
    const carData = JSON.parse(op.carData || '{}');

    const apiCar: ApiCar = {
      id: op.carId,
      make: carData.make,
      model: carData.model,
      year: carData.year,
      licensePlate: carData.licensePlate,
      currentMileage: carData.currentMileage,
      lastITPDate: carData.lastITPDate,
      nextITPDate: carData.nextITPDate,
      rovinietaExpiryDate: carData.rovinietaExpiryDate,
      carDescription: carData.carDescription,
      updatedAt: carData.updatedAt,
    };

    try {
      const result = await apiClient.updateCar(apiCar);

      await db.update(carsTable)
        .set({
          updatedAt: result.updatedAt,
          syncStatus: 'synced'
        })
        .where(eq(carsTable.id, op.carId));
    } catch (error) {
      if (error instanceof Error && error.message.includes('CONFLICT')) {
        console.log('[Sync] Conflict detected, fetching server version for:', op.carId);
        await this.fetchAndApplyServerVersion(op.carId);
        throw new Error('CONFLICT: Server has newer version');
      }
      throw error;
    }
  }

  private async fetchAndApplyServerVersion(carId: string): Promise<void> {
    try {
      const serverCar = await apiClient.fetchCar(carId);

      if (!serverCar) {
        console.log('[Sync] Car not found on server, may have been deleted:', carId);
        return;
      }

      const car: Car & { updatedAt: string } = {
        id: serverCar.id,
        make: serverCar.make,
        model: serverCar.model,
        year: serverCar.year,
        licensePlate: serverCar.licensePlate,
        currentMileage: serverCar.currentMileage,
        lastITPDate: new Date(serverCar.lastITPDate),
        nextITPDate: new Date(serverCar.nextITPDate),
        rovinietaExpiryDate: new Date(serverCar.rovinietaExpiryDate),
        carDescription: serverCar.carDescription,
        updatedAt: serverCar.updatedAt,
      };

      await carsRepository.updateCarFromServer(car);
      console.log('[Sync] Applied server version for:', carId);

      const allCars = await carsRepository.loadCars();
      this.notifyCarsUpdated(allCars);
    } catch (error) {
      console.error('[Sync] Error fetching server version:', error);
    }
  }

  private async syncDelete(op: SyncQueueItem): Promise<void> {
    await apiClient.deleteCar(op.carId);
  }

  addSyncListener(listener: (status: 'syncing' | 'synced' | 'failed') => void) {
    this.syncListeners.push(listener);
  }

  removeSyncListener(listener: (status: 'syncing' | 'synced' | 'failed') => void) {
    this.syncListeners = this.syncListeners.filter(l => l !== listener);
  }

  private notifyListeners(status: 'syncing' | 'synced' | 'failed') {
    this.syncListeners.forEach(listener => listener(status));
  }

  getIsSyncing(): boolean {
    return this.isSyncing;
  }
}

export const syncService = new SyncService();
