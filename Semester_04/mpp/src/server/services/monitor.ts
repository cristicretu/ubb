import { db } from "~/server/db";
import { LogActionType } from "./logger";

// Constants for monitoring
const SUSPICIOUS_THRESHOLD = 50; // Number of actions in time window to be suspicious
const TIME_WINDOW_MINUTES = 5; // Time window in minutes
const CHECK_INTERVAL_MINUTES = 2; // How often to check for suspicious activity

/**
 * Check for suspicious activity by users
 * This detects users with a high number of actions in a short time window
 */
export async function checkForSuspiciousActivity() {
  try {
    const timeWindowStart = new Date(
      Date.now() - TIME_WINDOW_MINUTES * 60 * 1000,
    );

    // Get counts of actions by user in the time window
    const userActionCounts = await db.activityLog.groupBy({
      by: ["userId"],
      where: {
        createdAt: {
          gte: timeWindowStart,
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

    // Identify suspicious users (those exceeding the threshold)
    const suspiciousUsers = userActionCounts.filter(
      (userCount) => userCount._count.id >= SUSPICIOUS_THRESHOLD,
    );

    // Add suspicious users to the monitored list
    for (const suspiciousUser of suspiciousUsers) {
      const userId = suspiciousUser.userId;

      // Check if user is already being monitored
      const existingMonitoring = await db.monitoredUser.findUnique({
        where: { userId },
      });

      if (!existingMonitoring) {
        // Get more details about the suspicious activity
        const recentActions = await db.activityLog.findMany({
          where: {
            userId,
            createdAt: {
              gte: timeWindowStart,
            },
          },
          orderBy: {
            createdAt: "desc",
          },
          take: 10, // Get the 10 most recent actions
        });

        const actionSummary = recentActions
          .map((log) => `${log.action} ${log.entity}`)
          .join(", ");

        // Add user to monitored list
        await db.monitoredUser.create({
          data: {
            userId,
            reason: `${suspiciousUser._count.id} actions in ${TIME_WINDOW_MINUTES} minutes. Recent actions: ${actionSummary}`,
          },
        });

        console.log(
          `Added user ${userId} to monitored list for suspicious activity: ${suspiciousUser._count.id} actions in ${TIME_WINDOW_MINUTES} minutes`,
        );
      }
    }

    return suspiciousUsers;
  } catch (error) {
    console.error("Error checking for suspicious activity:", error);
    return [];
  }
}

/**
 * Initialize the monitoring system that runs in the background
 * Can be called once when the server starts
 */
export function initializeMonitoring() {
  // Run the check initially
  void checkForSuspiciousActivity();

  // Then set up interval
  const intervalMs = CHECK_INTERVAL_MINUTES * 60 * 1000;

  const intervalId = setInterval(() => {
    void checkForSuspiciousActivity();
  }, intervalMs);

  console.log(
    `Monitoring system initialized. Checking every ${CHECK_INTERVAL_MINUTES} minutes.`,
  );

  // Return the interval ID so it can be cleared if needed
  return intervalId;
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
