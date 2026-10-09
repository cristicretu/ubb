import { db } from "~/server/db";
import { LogActionType } from "./logger";

// Constants for monitoring
const SUSPICIOUS_THRESHOLD = 10; // Number of actions in time window to be suspicious (reduced from 50 to 10)
const TIME_WINDOW_SECONDS = 10; // Time window in seconds (changed from minutes to seconds)
const TIME_WINDOW_MINUTES = TIME_WINDOW_SECONDS / 60; // Convert seconds to minutes for existing code
const CHECK_INTERVAL_SECONDS = 5; // How often to check for suspicious activity (reduced to 5 seconds)
const CHECK_INTERVAL_MINUTES = CHECK_INTERVAL_SECONDS / 60; // Convert for existing code

let monitoringInterval: NodeJS.Timeout | null = null;

/**
 * Check for suspicious activity by users
 * This detects users with a high number of actions in a short time window
 */
export async function checkForSuspiciousActivity() {
  try {
    const timeWindowAgo = new Date(Date.now() - TIME_WINDOW_SECONDS * 1000);

    console.log(
      `[Monitor] Checking for suspicious activity since ${timeWindowAgo.toISOString()}`,
    );

    // Get counts of actions by user in the last TIME_WINDOW_SECONDS
    const userActionCounts = await db.activityLog.groupBy({
      by: ["userId"],
      where: {
        createdAt: {
          gte: timeWindowAgo,
        },
      },
      _count: {
        id: true,
      },
      orderBy: {
        _count: {
          id: "desc",
        },
      },
    });

    // Filter for users who have more actions than the threshold
    const suspiciousUsers = userActionCounts.filter(
      (user) => user._count.id >= SUSPICIOUS_THRESHOLD,
    );

    console.log(`[Monitor] Found ${suspiciousUsers.length} suspicious users`);

    // For each suspicious user, add them to the monitoredUser table if not already there
    for (const user of suspiciousUsers) {
      const userId = user.userId;

      if (!userId) continue;

      const userDetails = await db.user.findUnique({
        where: { id: userId },
        select: { email: true, name: true },
      });

      console.log(
        `[Monitor] User ${userDetails?.email || userId} performed ${user._count.id} actions in ${TIME_WINDOW_SECONDS} seconds`,
      );

      // Check if user is already being monitored
      const existing = await db.monitoredUser.findUnique({
        where: { userId },
      });

      if (!existing) {
        // Add to monitored users
        await db.monitoredUser.create({
          data: {
            userId,
            reason: `Performed ${user._count.id} actions in ${TIME_WINDOW_SECONDS} seconds`,
            activelyMonitored: true,
          },
        });
        console.log(
          `[Monitor] Added user ${userDetails?.email || userId} to monitored users`,
        );
      } else {
        // Update the existing entry to ensure it's active
        await db.monitoredUser.update({
          where: { userId },
          data: {
            reason: `Performed ${user._count.id} actions in ${TIME_WINDOW_SECONDS} seconds (updated)`,
            activelyMonitored: true,
          },
        });
        console.log(
          `[Monitor] Updated monitoring status for ${userDetails?.email || userId}`,
        );
      }
    }

    return suspiciousUsers;
  } catch (error) {
    console.error("[Monitor] Error checking for suspicious activity:", error);
    return [];
  }
}

/**
 * Initialize the monitoring system
 * Starts a background process that periodically checks for suspicious activity
 */
export function initializeMonitoring() {
  // Clear any existing interval
  if (monitoringInterval) {
    clearInterval(monitoringInterval);
  }

  console.log(
    `[Monitor] Initializing monitoring system. Checking every ${CHECK_INTERVAL_SECONDS} seconds.`,
  );

  // Set up an interval to check for suspicious activity
  monitoringInterval = setInterval(async () => {
    try {
      await checkForSuspiciousActivity();
    } catch (error) {
      console.error("[Monitor] Error in monitoring check:", error);
    }
  }, CHECK_INTERVAL_SECONDS * 1000);

  return monitoringInterval;
}

/**
 * Stop the monitoring system
 */
export function stopMonitoring() {
  if (monitoringInterval) {
    clearInterval(monitoringInterval);
    monitoringInterval = null;
    console.log("[Monitor] Monitoring system stopped");
    return true;
  }
  return false;
}

/**
 * Get all currently monitored users
 */
export async function getMonitoredUsers() {
  return db.monitoredUser.findMany({
    include: {
      user: {
        select: {
          id: true,
          name: true,
          email: true,
          role: true,
        },
      },
    },
    orderBy: {
      createdAt: "desc",
    },
  });
}
