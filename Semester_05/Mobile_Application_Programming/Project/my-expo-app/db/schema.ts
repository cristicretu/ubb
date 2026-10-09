import { sqliteTable, text, integer } from 'drizzle-orm/sqlite-core';

export const cars = sqliteTable('cars', {
  id: text('id').primaryKey(),
  make: text('make').notNull(),
  model: text('model').notNull(),
  year: integer('year').notNull(),
  licensePlate: text('license_plate').notNull(),
  currentMileage: integer('current_mileage').notNull(),
  lastITPDate: text('last_itp_date').notNull(),
  nextITPDate: text('next_itp_date').notNull(),
  rovinietaExpiryDate: text('rovinieta_expiry_date').notNull(),
  carDescription: text('car_description'),
  updatedAt: text('updated_at').notNull(),
  syncStatus: text('sync_status').notNull().default('pending'),
});

export const syncQueue = sqliteTable('sync_queue', {
  id: integer('id').primaryKey({ autoIncrement: true }),
  operation: text('operation').notNull(),
  carId: text('car_id').notNull(),
  carData: text('car_data'),
  timestamp: text('timestamp').notNull(),
  status: text('status').notNull().default('pending'),
});

export type DbCar = typeof cars.$inferSelect;
export type DbCarInsert = typeof cars.$inferInsert;
export type DbSyncQueue = typeof syncQueue.$inferSelect;
export type DbSyncQueueInsert = typeof syncQueue.$inferInsert;
