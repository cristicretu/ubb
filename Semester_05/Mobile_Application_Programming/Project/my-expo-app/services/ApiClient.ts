const API_BASE_URL = 'http://localhost:3000/api';

export interface ApiCar {
  id: string;
  make: string;
  model: string;
  year: number;
  licensePlate: string;
  currentMileage: number;
  lastITPDate: string;
  nextITPDate: string;
  rovinietaExpiryDate: string;
  carDescription?: string;
  updatedAt: string;
}

export interface CreateCarData {
  localId: string;
  make: string;
  model: string;
  year: number;
  licensePlate: string;
  currentMileage: number;
  lastITPDate: string;
  nextITPDate: string;
  rovinietaExpiryDate: string;
  carDescription?: string;
}

export interface CreateCarResponse {
  car: ApiCar;
  localId: string;
  message: string;
}

export class ApiClient {
  async fetchCars(): Promise<ApiCar[]> {
    try {
      console.log('[API] Fetching all cars');
      const response = await fetch(`${API_BASE_URL}/cars`);

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data.cars || [];
    } catch (error) {
      console.error('[API] Error fetching cars:', error);
      throw new Error(`Failed to fetch cars: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async createCar(carData: CreateCarData): Promise<CreateCarResponse> {
    try {
      console.log('[API] Creating car with localId:', carData.localId);
      const response = await fetch(`${API_BASE_URL}/cars`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(carData),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || `HTTP ${response.status}`);
      }

      const data = await response.json();
      console.log('[API] Car created - Server ID:', data.car.id, 'LocalId:', data.localId);
      return data;
    } catch (error) {
      console.error('[API] Error creating car:', error);
      throw new Error(`Failed to create car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async updateCar(car: ApiCar): Promise<ApiCar> {
    try {
      console.log('[API] Updating car:', car.id);
      const response = await fetch(`${API_BASE_URL}/cars/${car.id}`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(car),
      });

      if (response.status === 409) {
        const error = await response.json();
        throw new Error(`CONFLICT: ${error.message}`);
      }

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || `HTTP ${response.status}`);
      }

      const data = await response.json();
      return data.car;
    } catch (error) {
      console.error('[API] Error updating car:', error);
      throw error;
    }
  }

  async fetchCar(id: string): Promise<ApiCar | null> {
    try {
      console.log('[API] Fetching car:', id);
      const response = await fetch(`${API_BASE_URL}/cars/${id}`);

      if (response.status === 404) {
        return null;
      }

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data.car;
    } catch (error) {
      console.error('[API] Error fetching car:', error);
      throw new Error(`Failed to fetch car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async deleteCar(id: string): Promise<void> {
    try {
      console.log('[API] Deleting car:', id);
      const response = await fetch(`${API_BASE_URL}/cars/${id}`, {
        method: 'DELETE',
      });

      if (response.status === 404) {
        console.log('[API] Car already deleted on server');
        return;
      }

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || `HTTP ${response.status}`);
      }
    } catch (error) {
      console.error('[API] Error deleting car:', error);
      throw new Error(`Failed to delete car: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }
}

export const apiClient = new ApiClient();

