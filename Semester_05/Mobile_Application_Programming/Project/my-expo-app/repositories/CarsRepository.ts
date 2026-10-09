import { randomUUID } from 'expo-crypto';
import { eq } from 'drizzle-orm';
import { db } from '../db/client';
import { cars as carsTable } from '../db/schema';
import { Car } from '../types/Car';
import { syncQueueService } from '../services/SyncQueueService';

const LOCAL_ID_PREFIX = 'local_';

export const isLocalId = (id: string): boolean => id.startsWith(LOCAL_ID_PREFIX);

export class CarsRepository {
  async loadCars(): Promise<Car[]> {
    try {
      const dbCars = await db.select().from(carsTable);
      const cars = dbCars.map((car) => ({
        id: car.id,
        make: car.make,
        model: car.model,
        year: car.year,
        licensePlate: car.licensePlate,
        currentMileage: car.currentMileage,
        lastITPDate: new Date(car.lastITPDate),
        nextITPDate: new Date(car.nextITPDate),
        rovinietaExpiryDate: new Date(car.rovinietaExpiryDate),
        carDescription: car.carDescription ?? undefined,
      }));

      return cars;
    } catch (error) {
      throw new Error(`Failed to load cars from database: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async createCar(carData: Omit<Car, 'id'>): Promise<Car> {
    try {
      const localId = `${LOCAL_ID_PREFIX}${randomUUID()}`;
      const updatedAt = new Date().toISOString();

      const newCar: Car = {
        id: localId,
        ...carData,
      };

      await db.insert(carsTable).values({
        id: localId,
        make: carData.make,
        model: carData.model,
        year: carData.year,
        licensePlate: carData.licensePlate,
        currentMileage: carData.currentMileage,
        lastITPDate: carData.lastITPDate.toISOString(),
        nextITPDate: carData.nextITPDate.toISOString(),
        rovinietaExpiryDate: carData.rovinietaExpiryDate.toISOString(),
        carDescription: carData.carDescription,
        updatedAt,
        syncStatus: 'pending',
      });

      await syncQueueService.addToQueue('CREATE', localId, {
        localId,
        make: carData.make,
        model: carData.model,
        year: carData.year,
        licensePlate: carData.licensePlate,
        currentMileage: carData.currentMileage,
        lastITPDate: carData.lastITPDate.toISOString(),
        nextITPDate: carData.nextITPDate.toISOString(),
        rovinietaExpiryDate: carData.rovinietaExpiryDate.toISOString(),
        carDescription: carData.carDescription,
        updatedAt,
      });

      console.log('[Repository] Car created with local ID:', localId, '(awaiting server ID)');
      return newCar;
    } catch (error) {
      throw new Error(`Failed to create car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async replaceLocalIdWithServerId(localId: string, serverId: string, updatedAt: string): Promise<void> {
    try {
      console.log('[Repository] Replacing local ID', localId, 'with server ID', serverId);

      const [serverCar] = await db.select().from(carsTable).where(eq(carsTable.id, serverId));

      if (serverCar) {
        console.log('[Repository] Server ID already exists, deleting local version only');
        await db.delete(carsTable).where(eq(carsTable.id, localId));
        console.log('[Repository] Successfully cleaned up local ID after WebSocket sync');
        return;
      }

      const [existingCar] = await db.select().from(carsTable).where(eq(carsTable.id, localId));

      if (!existingCar) {
        console.warn('[Repository] Car with local ID not found:', localId);
        return;
      }

      await db.delete(carsTable).where(eq(carsTable.id, localId));

      await db.insert(carsTable).values({
        ...existingCar,
        id: serverId,
        updatedAt,
        syncStatus: 'synced',
      });

      console.log('[Repository] Successfully replaced local ID with server ID');
    } catch (error) {
      console.error('[Repository] Error replacing local ID with server ID:', error);
      throw error;
    }
  }

  async updateCar(updatedCar: Car): Promise<void> {
    try {
      const updatedAt = new Date().toISOString();

      const result = await db
        .update(carsTable)
        .set({
          make: updatedCar.make,
          model: updatedCar.model,
          year: updatedCar.year,
          licensePlate: updatedCar.licensePlate,
          currentMileage: updatedCar.currentMileage,
          lastITPDate: updatedCar.lastITPDate.toISOString(),
          nextITPDate: updatedCar.nextITPDate.toISOString(),
          rovinietaExpiryDate: updatedCar.rovinietaExpiryDate.toISOString(),
          carDescription: updatedCar.carDescription,
          updatedAt,
          syncStatus: 'pending',
        })
        .where(eq(carsTable.id, updatedCar.id));

      if (result.changes === 0) {
        throw new Error('Car not found');
      }

      await syncQueueService.addToQueue('UPDATE', updatedCar.id, {
        id: updatedCar.id,
        make: updatedCar.make,
        model: updatedCar.model,
        year: updatedCar.year,
        licensePlate: updatedCar.licensePlate,
        currentMileage: updatedCar.currentMileage,
        lastITPDate: updatedCar.lastITPDate.toISOString(),
        nextITPDate: updatedCar.nextITPDate.toISOString(),
        rovinietaExpiryDate: updatedCar.rovinietaExpiryDate.toISOString(),
        carDescription: updatedCar.carDescription,
        updatedAt,
      });

      console.log('[Repository] Car updated locally and queued for sync:', updatedCar.id);
    } catch (error) {
      throw new Error(`Failed to update car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async deleteCar(id: string): Promise<void> {
    try {
      const result = await db.delete(carsTable).where(eq(carsTable.id, id));

      if (result.changes === 0) {
        throw new Error('Car not found');
      }

      await syncQueueService.addToQueue('DELETE', id);

      console.log('[Repository] Car deleted locally and queued for sync:', id);
    } catch (error) {
      throw new Error(`Failed to delete car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async updateCarFromServer(car: Car & { updatedAt: string }): Promise<void> {
    try {
      const result = await db
        .update(carsTable)
        .set({
          make: car.make,
          model: car.model,
          year: car.year,
          licensePlate: car.licensePlate,
          currentMileage: car.currentMileage,
          lastITPDate: car.lastITPDate.toISOString(),
          nextITPDate: car.nextITPDate.toISOString(),
          rovinietaExpiryDate: car.rovinietaExpiryDate.toISOString(),
          carDescription: car.carDescription,
          updatedAt: car.updatedAt,
          syncStatus: 'synced',
        })
        .where(eq(carsTable.id, car.id));

      if (result.changes === 0) {
        console.log('[Repository] Car not found for update, inserting instead:', car.id);
        await this.insertCarFromServer(car);
        return;
      }

      console.log('[Repository] Car updated from server (no sync queue):', car.id);
    } catch (error) {
      console.error('[Repository] Error updating car from server:', error);
    }
  }

  async insertCarFromServer(car: Car & { updatedAt: string }): Promise<void> {
    try {
      const [existingCar] = await db.select().from(carsTable).where(eq(carsTable.id, car.id));

      if (existingCar) {
        console.log('[Repository] Car already exists, updating instead:', car.id);
        await db
          .update(carsTable)
          .set({
            make: car.make,
            model: car.model,
            year: car.year,
            licensePlate: car.licensePlate,
            currentMileage: car.currentMileage,
            lastITPDate: car.lastITPDate.toISOString(),
            nextITPDate: car.nextITPDate.toISOString(),
            rovinietaExpiryDate: car.rovinietaExpiryDate.toISOString(),
            carDescription: car.carDescription,
            updatedAt: car.updatedAt,
            syncStatus: 'synced',
          })
          .where(eq(carsTable.id, car.id));
        return;
      }

      await db.insert(carsTable).values({
        id: car.id,
        make: car.make,
        model: car.model,
        year: car.year,
        licensePlate: car.licensePlate,
        currentMileage: car.currentMileage,
        lastITPDate: car.lastITPDate.toISOString(),
        nextITPDate: car.nextITPDate.toISOString(),
        rovinietaExpiryDate: car.rovinietaExpiryDate.toISOString(),
        carDescription: car.carDescription,
        updatedAt: car.updatedAt,
        syncStatus: 'synced',
      });

      console.log('[Repository] Car inserted from server:', car.id);
    } catch (error) {
      console.error('[Repository] Error inserting car from server:', error);
    }
  }

  async deleteCarFromServer(id: string): Promise<void> {
    try {
      const result = await db.delete(carsTable).where(eq(carsTable.id, id));

      if (result.changes === 0) {
        console.log('[Repository] Car not found in local DB for server delete:', id);
        return; // Not an error - car might not have been synced to this device yet
      }

      console.log('[Repository] Car deleted from server (no sync queue):', id);
    } catch (error) {
      console.error('[Repository] Error deleting car from server:', error);
    }
  }

  async clearAll(): Promise<void> {
    try {
      await db.delete(carsTable);
    } catch (error) {
      throw new Error(`Failed to clear database: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
}

export const carsRepository = new CarsRepository();
