// This script checks if the monitoring system is properly initialized
// and forces it to run once to check for suspicious activity

import { PrismaClient } from "@prisma/client";
import {
  checkForSuspiciousActivity,
  initializeMonitoring,
} from "../src/server/services/monitor";

const prisma = new PrismaClient();

async function checkMonitoringSystem() {
  try {
    console.log("Checking monitoring system status...");

    // Run the suspicious activity check directly
    console.log("Running suspicious activity check...");
    const suspiciousUsers = await checkForSuspiciousActivity();

    console.log(`Found ${suspiciousUsers.length} suspicious users`);

    if (suspiciousUsers.length > 0) {
      console.log("Suspicious users found:");
      for (const user of suspiciousUsers) {
        console.log(`- User ${user.userId}: ${user._count.id} actions`);
      }
    }

    // Get current monitored users
    const monitoredUsers = await prisma.monitoredUser.findMany({
      include: {
        user: {
          select: {
            id: true,
            email: true,
          },
        },
      },
    });

    console.log(`\nCurrently monitoring ${monitoredUsers.length} users:`);

    if (monitoredUsers.length === 0) {
      console.log("No users are currently being monitored.");
    } else {
      for (const monitored of monitoredUsers) {
        console.log(
          `- ${monitored.user.email || monitored.userId}: ${monitored.reason}`,
        );
      }
    }

    // Check for recent activity logs
    const recentLogs = await prisma.activityLog.findMany({
      take: 10,
      orderBy: {
        createdAt: "desc",
      },
      include: {
        user: {
          select: {
            email: true,
          },
        },
      },
    });

    console.log(`\nRecent activity logs (${recentLogs.length}):`);

    if (recentLogs.length === 0) {
      console.log("No recent activity logs found. This might be the issue!");
      console.log("Make sure actions are being properly logged.");
    } else {
      for (const log of recentLogs) {
        console.log(
          `- ${new Date(log.createdAt).toLocaleTimeString()}: ${log.user?.email || log.userId} - ${log.action} ${log.entity}`,
        );
      }
    }

    // Initialize the monitoring system (restarts it if already running)
    console.log("\nInitializing monitoring system...");
    const intervalId = initializeMonitoring();

    console.log("✅ Monitoring system initialized!");
    console.log(
      "The background process is now checking for suspicious activity.",
    );
    console.log(
      "You can now run rapid API calls to test if users get flagged.",
    );

    // Keep the process running for a minute to allow monitoring to happen
    console.log("\nWaiting for 60 seconds to let monitoring system work...");

    setTimeout(async () => {
      // Check one more time after waiting
      const finalMonitoredUsers = await prisma.monitoredUser.findMany();
      console.log(
        `After waiting, there are ${finalMonitoredUsers.length} monitored users.`,
      );

      // Clean up and exit
      clearInterval(intervalId);
      await prisma.$disconnect();
      process.exit(0);
    }, 60000);
  } catch (error) {
    console.error("Error checking monitoring system:", error);
    await prisma.$disconnect();
    process.exit(1);
  }
}

checkMonitoringSystem();
