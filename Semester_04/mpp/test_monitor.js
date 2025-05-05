// Script to test the monitoring system by:
// 1. Creating activity logs directly
// 2. Running the suspicious activity check
// 3. Verifying if a user gets monitored

import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

// Configuration - match the values in monitor.ts
const SUSPICIOUS_THRESHOLD = 10;
const TIME_WINDOW_SECONDS = 10;

async function logActivity(userId, action = "CREATE", entity = "EXERCISE") {
  try {
    return await prisma.activityLog.create({
      data: {
        userId,
        action,
        entity,
        entityId: `test-${Date.now()}`,
        details: `Test activity log created at ${new Date().toISOString()}`,
      },
    });
  } catch (error) {
    console.error("Failed to log activity:", error);
    return null;
  }
}

async function checkForSuspiciousActivity() {
  try {
    // Create a date for the time window
    const timeWindowStart = new Date(Date.now() - TIME_WINDOW_SECONDS * 1000);

    console.log(
      `Checking for activity since: ${timeWindowStart.toISOString()}`,
    );

    // Get counts of actions by user in the time window
    const userActionCounts = await prisma.activityLog.groupBy({
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

    console.log("User action counts:", userActionCounts);

    // Identify suspicious users (those exceeding the threshold)
    const suspiciousUsers = userActionCounts.filter(
      (userCount) => userCount._count.id >= SUSPICIOUS_THRESHOLD,
    );

    console.log("Suspicious users:", suspiciousUsers);

    // Add suspicious users to the monitored list
    for (const suspiciousUser of suspiciousUsers) {
      const userId = suspiciousUser.userId;

      // Check if user is already being monitored
      const existingMonitoring = await prisma.monitoredUser.findUnique({
        where: { userId },
      });

      if (!existingMonitoring) {
        // Get more details about the suspicious activity
        const recentActions = await prisma.activityLog.findMany({
          where: {
            userId,
            createdAt: {
              gte: timeWindowStart,
            },
          },
          orderBy: {
            createdAt: "desc",
          },
          take: 10,
        });

        const actionSummary = recentActions
          .map((log) => `${log.action} ${log.entity}`)
          .join(", ");

        // Add user to monitored list
        await prisma.monitoredUser.create({
          data: {
            userId,
            reason: `${suspiciousUser._count.id} actions in ${TIME_WINDOW_SECONDS} seconds. Recent actions: ${actionSummary}`,
            activelyMonitored: true,
          },
        });

        console.log(
          `Added user ${userId} to monitored list for suspicious activity: ${suspiciousUser._count.id} actions in ${TIME_WINDOW_SECONDS} seconds`,
        );
      }
    }

    return suspiciousUsers;
  } catch (error) {
    console.error("Error checking for suspicious activity:", error);
    return [];
  }
}

async function runTest() {
  try {
    // Step 1: Clear monitored users
    console.log("Clearing monitored users table...");
    await prisma.monitoredUser.deleteMany({});
    console.log("✅ Cleared monitored users table");

    // Step 2: Find a test user
    const testUser = await prisma.user.findFirst({
      select: { id: true, email: true },
    });

    if (!testUser) {
      console.error("❌ No users found in the database!");
      return;
    }

    console.log(`Testing with user: ${testUser.email} (${testUser.id})`);

    // Step 3: Create 15 activity logs rapidly (above threshold of 10)
    console.log(`Creating ${SUSPICIOUS_THRESHOLD + 5} activity logs...`);

    for (let i = 1; i <= SUSPICIOUS_THRESHOLD + 5; i++) {
      await logActivity(testUser.id);
      console.log(`Created log #${i}`);
    }

    // Step 4: Run the check
    console.log("\nRunning suspicious activity check...");
    const suspiciousUsers = await checkForSuspiciousActivity();

    console.log(`Found ${suspiciousUsers.length} suspicious users`);

    // Step 5: Check if user is monitored
    const isMonitored = await prisma.monitoredUser.findUnique({
      where: { userId: testUser.id },
    });

    if (isMonitored) {
      console.log("✅ SUCCESS! User was flagged for monitoring");
      console.log(`Reason: ${isMonitored.reason}`);
      console.log(
        "\nCheck the admin panel at http://localhost:3000/admin/monitoring to confirm",
      );
    } else {
      console.log("❌ FAILED! User was not flagged for monitoring");
      console.log("Possible issues:");
      console.log("1. Activity logs are not being created properly");
      console.log(
        "2. The monitoring threshold or time window settings don't match",
      );
      console.log("3. There's a bug in the monitoring detection logic");
    }
  } catch (error) {
    console.error("Error during test:", error);
  } finally {
    await prisma.$disconnect();
  }
}

runTest();
