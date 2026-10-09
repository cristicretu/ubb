import { Router } from 'express';
import { randomUUID } from 'crypto';
import { eq } from 'drizzle-orm';
import { db } from '../db/client';
import { cars as carsTable } from '../db/schema';

export const carsRouter = Router();

carsRouter.get('/', async (req, res) => {
  try {
    console.log('[API] GET /api/cars');
    const cars = await db.select().from(carsTable);
    res.json({ cars });
  } catch (error) {
    console.error('[API] Error fetching cars:', error);
    res.status(500).json({
      error: 'Failed to fetch cars',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});

carsRouter.post('/', async (req, res) => {
  try {
    const carData = req.body;
    const localId = carData.localId;
    console.log('[API] POST /api/cars - localId:', localId);

    if (!carData.make || !carData.model) {
      return res.status(400).json({
        error: 'Missing required fields',
        message: 'Car must have make and model'
      });
    }

    const id = randomUUID();
    const updatedAt = new Date().toISOString();

    const newCar = {
      id,
      make: carData.make,
      model: carData.model,
      year: carData.year,
      licensePlate: carData.licensePlate,
      currentMileage: carData.currentMileage,
      lastITPDate: carData.lastITPDate,
      nextITPDate: carData.nextITPDate,
      rovinietaExpiryDate: carData.rovinietaExpiryDate,
      carDescription: carData.carDescription,
      updatedAt,
    };

    await db.insert(carsTable).values(newCar);
    console.log('[API] Car created with server-generated ID:', id);

    const io = req.app.get('io');
    io.emit('car:created', newCar);

    res.status(201).json({
      car: newCar,
      localId,
      message: 'Car created successfully'
    });
  } catch (error) {
    console.error('[API] Error creating car:', error);
    res.status(500).json({
      error: 'Failed to create car',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});

carsRouter.put('/:id', async (req, res) => {
  try {
    const { id } = req.params;
    const carData = req.body;
    console.log('[API] PUT /api/cars/:id', id);

    const existingCars = await db.select().from(carsTable).where(eq(carsTable.id, id));

    if (existingCars.length === 0) {
      return res.status(404).json({
        error: 'Car not found',
        message: `No car found with id: ${id}`
      });
    }

    const existingCar = existingCars[0];
    const clientUpdatedAt = new Date(carData.updatedAt || 0);
    const serverUpdatedAt = new Date(existingCar.updatedAt);

    if (clientUpdatedAt < serverUpdatedAt) {
      console.log('[API] Conflict detected - server version is newer');
      return res.status(409).json({
        error: 'Conflict',
        message: 'Server has a newer version of this car',
        serverCar: existingCar
      });
    }

    const updatedAt = new Date().toISOString();

    await db.update(carsTable)
      .set({
        ...carData,
        updatedAt,
      })
      .where(eq(carsTable.id, id));

    const io = req.app.get('io');
    io.emit('car:updated', { ...carData, updatedAt });

    res.json({
      car: { ...carData, updatedAt },
      message: 'Car updated successfully'
    });
  } catch (error) {
    console.error('[API] Error updating car:', error);
    res.status(500).json({
      error: 'Failed to update car',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});

carsRouter.delete('/:id', async (req, res) => {
  try {
    const { id } = req.params;
    console.log('[API] DELETE /api/cars/:id', id);

    const existingCars = await db.select().from(carsTable).where(eq(carsTable.id, id));

    if (existingCars.length === 0) {
      return res.status(404).json({
        error: 'Car not found',
        message: `No car found with id: ${id}`
      });
    }

    await db.delete(carsTable).where(eq(carsTable.id, id));

    const io = req.app.get('io');
    io.emit('car:deleted', { id });

    res.json({
      message: 'Car deleted successfully',
      id
    });
  } catch (error) {
    console.error('[API] Error deleting car:', error);
    res.status(500).json({
      error: 'Failed to delete car',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
});


