export interface Car {
  id: string;
  make: string;
  model: string;
  year: number;
  licensePlate: string;
  currentMileage: number;
  lastITPDate: Date;
  nextITPDate: Date;
  rovinietaExpiryDate: Date;
  carDescription?: string;
}

