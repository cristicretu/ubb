import { drizzle } from 'drizzle-orm/expo-sqlite';
import { openDatabaseSync } from 'expo-sqlite';
import * as schema from './schema';

const expoDb = openDatabaseSync('cars.db');

export const db = drizzle(expoDb, { schema });

export async function initDatabase() {
  try {
    await expoDb.execAsync(`
      CREATE TABLE IF NOT EXISTS cars (
        id TEXT PRIMARY KEY NOT NULL,
        make TEXT NOT NULL,
        model TEXT NOT NULL,
        year INTEGER NOT NULL,
        license_plate TEXT NOT NULL,
        current_mileage INTEGER NOT NULL,
        last_itp_date TEXT NOT NULL,
        next_itp_date TEXT NOT NULL,
        rovinieta_expiry_date TEXT NOT NULL,
        car_description TEXT,
        updated_at TEXT NOT NULL,
        sync_status TEXT NOT NULL DEFAULT 'pending'
      );
      
      CREATE TABLE IF NOT EXISTS sync_queue (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        operation TEXT NOT NULL,
        car_id TEXT NOT NULL,
        car_data TEXT,
        timestamp TEXT NOT NULL,
        status TEXT NOT NULL DEFAULT 'pending'
      );
    `);
    console.log('Database initialized successfully');
  } catch (error) {
    console.error('Error initializing database:', error);
    throw new Error('Failed to initialize database');
  }
}
