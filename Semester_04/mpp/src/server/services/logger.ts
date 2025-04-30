import { db } from "~/server/db";
import { type User } from "@prisma/client";

// Action types
export enum LogActionType {
  CREATE = "CREATE",
  READ = "READ",
  UPDATE = "UPDATE",
  DELETE = "DELETE",
}

// Entity types
export enum LogEntityType {
  USER = "USER",
  EXERCISE = "EXERCISE",
  POST = "POST",
}

interface LogActivityOptions {
  user: User | { id: string };
  action: LogActionType;
  entity: LogEntityType;
  entityId: string;
  details?: string;
}

/**
 * Log a user activity in the database
 */
export async function logActivity({
  user,
  action,
  entity,
  entityId,
  details,
}: LogActivityOptions) {
  try {
    return await db.activityLog.create({
      data: {
        userId: user.id,
        action,
        entity,
        entityId,
        details,
      },
    });
  } catch (error) {
    console.error("Failed to log activity:", error);
    // We don't want to throw errors from logging - it should be non-blocking
    return null;
  }
}

/**
 * Log a user create operation
 */
export function logCreate(
  user: User | { id: string },
  entity: LogEntityType,
  entityId: string,
  details?: string,
) {
  return logActivity({
    user,
    action: LogActionType.CREATE,
    entity,
    entityId,
    details,
  });
}

/**
 * Log a user read operation
 */
export function logRead(
  user: User | { id: string },
  entity: LogEntityType,
  entityId: string,
  details?: string,
) {
  return logActivity({
    user,
    action: LogActionType.READ,
    entity,
    entityId,
    details,
  });
}

/**
 * Log a user update operation
 */
export function logUpdate(
  user: User | { id: string },
  entity: LogEntityType,
  entityId: string,
  details?: string,
) {
  return logActivity({
    user,
    action: LogActionType.UPDATE,
    entity,
    entityId,
    details,
  });
}

/**
 * Log a user delete operation
 */
export function logDelete(
  user: User | { id: string },
  entity: LogEntityType,
  entityId: string,
  details?: string,
) {
  return logActivity({
    user,
    action: LogActionType.DELETE,
    entity,
    entityId,
    details,
  });
}
