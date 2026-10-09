import React, { createContext, useContext, useState, useEffect, useRef, ReactNode } from 'react';
import { Alert } from 'react-native';
import { Car } from '../types/Car';
import { carsRepository, isLocalId } from '../repositories/CarsRepository';
import { initDatabase } from '../db/client';
import { networkService } from '../services/NetworkService';
import { syncService } from '../services/SyncService';
import { webSocketService } from '../services/WebSocketService';
import { serverConnectionService } from '../services/ServerConnectionService';

interface CarsContextType {
  cars: Car[];
  isLoading: boolean;
  isSyncing: boolean;
  isOnline: boolean;
  error: string | null;
  addCar: (carData: Omit<Car, 'id'>) => Promise<void>;
  updateCar: (car: Car) => Promise<void>;
  deleteCar: (id: string) => Promise<void>;
  syncNow: () => Promise<void>;
  clearError: () => void;
}

const CarsContext = createContext<CarsContextType | undefined>(undefined);

export const CarsProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [cars, setCars] = useState<Car[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isSyncing, setIsSyncing] = useState(false);
  const [isOnline, setIsOnline] = useState(false);
  const [isServerReachable, setIsServerReachable] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const wasServerReachableRef = useRef(false);

  useEffect(() => {
    const init = async () => {
      try {
        setIsLoading(true);
        await initDatabase();

        const loadedCars = await carsRepository.loadCars();
        setCars(loadedCars);
        setError(null);

        const initialOnline = await networkService.fetch();
        setIsOnline(initialOnline);

        serverConnectionService.startMonitoring();

        if (initialOnline) {
          webSocketService.connect();
        }
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Failed to initialize app';
        setError(errorMessage);
        Alert.alert('Initialization Error', errorMessage, [{ text: 'OK' }]);
        console.error('Error initializing app:', err);
      } finally {
        setIsLoading(false);
      }
    };
    init();

    const networkListener = (online: boolean) => {
      setIsOnline(online);

      if (online) {
        console.log('[Context] Connected to network, triggering sync');
        webSocketService.connect();
        syncNowInternal();
      } else {
        console.log('[Context] Disconnected from network');
        webSocketService.disconnect();
      }
    };

    networkService.addListener(networkListener);

    const serverListener = async (isReachable: boolean) => {
      const wasReachable = wasServerReachableRef.current;
      wasServerReachableRef.current = isReachable;
      setIsServerReachable(isReachable);

      if (!isReachable) {
        console.log('[Context] Server unreachable - cannot sync');
      }

      // When server becomes reachable, do a full sync (push pending + pull from server)
      if (isReachable && !wasReachable) {
        console.log('[Context] Server became reachable - doing full sync');
        try {
          await syncService.syncPendingOperations();
          await syncService.fetchAndMergeFromServer();
        } catch (error) {
          console.error('[Context] Error during full sync:', error);
        }
      }
    };

    serverConnectionService.addListener(serverListener);

    const syncListener = (status: 'syncing' | 'synced' | 'failed') => {
      setIsSyncing(status === 'syncing');
      if (status === 'failed') {
        Alert.alert('Sync Issue', 'Some changes could not be synced. Will retry later.', [{ text: 'OK' }]);
      }
    };

    syncService.addSyncListener(syncListener);

    const idMappingCallback = (localId: string, serverId: string) => {
      console.log('[Context] ID mapping received:', localId, '->', serverId);
      setCars((prevCars) =>
        prevCars.map((car) =>
          car.id === localId ? { ...car, id: serverId } : car
        )
      );
    };
    syncService.onIdMapping(idMappingCallback);

    const carsUpdatedCallback = (updatedCars: Car[]) => {
      console.log('[Context] Cars updated from sync:', updatedCars.length, 'cars');
      setCars(updatedCars);
    };
    syncService.onCarsUpdated(carsUpdatedCallback);

    webSocketService.onCarCreated(async (car) => {
      console.log('[Context] WebSocket: car created from server:', car.id);

      try {
        await carsRepository.insertCarFromServer(car);
        console.log('[Context] WebSocket: car persisted to local DB:', car.id);
      } catch (error) {
        console.error('[Context] Error persisting WebSocket car to DB:', error);
      }

      setCars((prevCars) => {
        if (prevCars.find((c) => c.id === car.id)) {
          console.log('[Context] Skipping WebSocket car state update - already have it:', car.id);
          return prevCars;
        }
        const hasLocalVersion = prevCars.some((c) => isLocalId(c.id));
        if (hasLocalVersion) {
          console.log('[Context] Skipping WebSocket car state update - have local version pending sync');
          return prevCars;
        }
        return [...prevCars, car];
      });
    });

    webSocketService.onCarUpdated(async (car) => {
      console.log('[Context] WebSocket: car updated from server:', car.id);

      try {
        await carsRepository.updateCarFromServer(car);
        console.log('[Context] WebSocket: car update persisted to local DB:', car.id);
      } catch (error) {
        console.error('[Context] Error persisting WebSocket car update to DB:', error);
      }

      setCars((prevCars) =>
        prevCars.map((c) => (c.id === car.id ? car : c))
      );
    });

    webSocketService.onCarDeleted(async (id) => {
      console.log('[Context] WebSocket: car deleted from server:', id);

      try {
        await carsRepository.deleteCarFromServer(id);
        console.log('[Context] WebSocket: car delete persisted to local DB:', id);
      } catch (error) {
        console.error('[Context] Error deleting car from local DB:', error);
      }

      setCars((prevCars) => prevCars.filter((c) => c.id !== id));
    });

    return () => {
      networkService.removeListener(networkListener);
      serverConnectionService.removeListener(serverListener);
      serverConnectionService.stopMonitoring();
      syncService.removeSyncListener(syncListener);
      syncService.removeIdMappingCallback(idMappingCallback);
      syncService.removeCarsUpdatedCallback(carsUpdatedCallback);
      webSocketService.clearListeners();
      webSocketService.disconnect();
    };
  }, []);

  const syncNowInternal = async () => {
    if (!isOnline || !isServerReachable) {
      console.log('[Context] Cannot sync - server not reachable');
      return;
    }

    try {
      const result = await syncService.syncPendingOperations();
      if (!result.success) {
        console.error('[Context] Sync failed:', result.message);
      }
    } catch (error) {
      console.error('[Context] Error during sync:', error);
    }
  };

  const syncNow = async () => {
    if (!isOnline) {
      Alert.alert('Offline', 'Device has no internet connection', [{ text: 'OK' }]);
      return;
    }
    if (!isServerReachable) {
      Alert.alert('Server Unreachable', 'Cannot connect to server. Make sure server is running.', [{ text: 'OK' }]);
      return;
    }
    await syncNowInternal();
  };

  const addCar = async (carData: Omit<Car, 'id'>) => {
    try {
      setIsLoading(true);
      const newCar = await carsRepository.createCar(carData);
      setCars((prevCars) => [...prevCars, newCar]);
      setError(null);

      if (isOnline && isServerReachable) {
        syncNowInternal();
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to add car';
      setError(errorMessage);
      Alert.alert('Error Adding Car', errorMessage, [{ text: 'OK' }]);
      console.error('Error adding car:', err);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const updateCar = async (car: Car) => {
    try {
      setIsLoading(true);
      await carsRepository.updateCar(car);
      setCars((prevCars) =>
        prevCars.map((c) => (c.id === car.id ? car : c))
      );
      setError(null);

      if (isOnline && isServerReachable) {
        syncNowInternal();
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to update car';
      setError(errorMessage);
      Alert.alert('Error Updating Car', errorMessage, [{ text: 'OK' }]);
      console.error('Error updating car:', err);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const deleteCar = async (id: string) => {
    try {
      setIsLoading(true);
      await carsRepository.deleteCar(id);
      setCars((prevCars) => prevCars.filter((c) => c.id !== id));
      setError(null);

      if (isOnline && isServerReachable) {
        syncNowInternal();
      }
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to delete car';
      setError(errorMessage);
      Alert.alert('Error Deleting Car', errorMessage, [{ text: 'OK' }]);
      console.error('Error deleting car:', err);
      throw err;
    } finally {
      setIsLoading(false);
    }
  };

  const clearError = () => {
    setError(null);
  };

  return (
    <CarsContext.Provider
      value={{
        cars,
        isLoading,
        isSyncing,
        isOnline: isOnline && isServerReachable,
        error,
        addCar,
        updateCar,
        deleteCar,
        syncNow,
        clearError
      }}
    >
      {children}
    </CarsContext.Provider>
  );
};

export const useCars = () => {
  const context = useContext(CarsContext);
  if (context === undefined) {
    throw new Error('useCars must be used within a CarsProvider');
  }
  return context;
};
