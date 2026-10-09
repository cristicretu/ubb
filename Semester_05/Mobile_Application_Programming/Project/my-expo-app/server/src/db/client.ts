import { createClient } from '@libsql/client';
import { drizzle } from 'drizzle-orm/libsql';
import * as schema from './schema';

const client = createClient({
  url: 'file:cars-server.db',
});

export const db = drizzle(client, { schema });

export async function initDatabase() {
  try {
    await client.execute(`
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
        updated_at TEXT NOT NULL
      );
    `);
    console.log('[Database] Server database initialized successfully');
  } catch (error) {
    console.error('[Database] Error initializing database:', error);
    throw new Error('Failed to initialize server database');
  }
}

