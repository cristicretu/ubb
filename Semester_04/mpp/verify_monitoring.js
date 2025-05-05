// This script directly tests the monitoring system by:
// 1. Creating activity logs
// 2. Running the suspicious activity check
// 3. Verifying if a user gets monitored

import { PrismaClient } from "@prisma/client";
import { checkForSuspiciousActivity } from "../src/server/services/monitor";
import { LogActionType, LogEntityType } from "../src/server/services/logger";

const prisma = new PrismaClient();

async function runMonitoringTest() {
  try {
    console.log("Starting monitoring system verification...");

    // Step 1: Clean up existing monitored users
    await prisma.monitoredUser.deleteMany({});
    console.log("✅ Cleared monitored users table");

    // Find a user to test with
    const testUser = await prisma.user.findFirst({
      select: { id: true, email: true },
    });

    if (!testUser) {
      console.error("❌ No users found in the database!");
      return;
    }

    console.log(`Testing with user: ${testUser.email} (${testUser.id})`);

    // Step 2: Create 15 activity logs for this user (above the 10 threshold)
    console.log("Creating activity logs...");

    // Delete any recent activity logs for this user to start fresh
    const fiveSecondsAgo = new Date(Date.now() - 5000);
    await prisma.activityLog.deleteMany({
      where: {
        userId: testUser.id,
        createdAt: { gte: fiveSecondsAgo },
      },
    });

    // Create new activity logs in rapid succession
    const logs = [];
    for (let i = 1; i <= 15; i++) {
      const log = await prisma.activityLog.create({
        data: {
          userId: testUser.id,
          action: LogActionType.CREATE,
          entity: LogEntityType.EXERCISE,
          entityId: `test-entity-${i}`,
          details: `Test activity ${i} for monitoring verification`,
        },
      });
      logs.push(log);
      console.log(`  Created log #${i}: ${log.action} ${log.entity}`);
    }

    console.log(
      `✅ Created ${logs.length} activity logs in the last few seconds`,
    );

    // Step 3: Run the monitoring check
    console.log("\nRunning suspicious activity check...");
    const suspiciousUsers = await checkForSuspiciousActivity();

    console.log(`Found ${suspiciousUsers.length} suspicious users`);

    // Step 4: Check if our user is being monitored
    const isMonitored = await prisma.monitoredUser.findUnique({
      where: { userId: testUser.id },
    });

    if (isMonitored) {
      console.log("✅ SUCCESS! User was flagged for monitoring");
      console.log(`Reason: ${isMonitored.reason}`);
    } else {
      console.log(
        "❌ FAILED! User was not flagged for monitoring despite exceeding threshold",
      );
      console.log(
        "Check the monitoring system configuration and logs for errors",
      );
    }
  } catch (error) {
    console.error("Error during monitoring test:", error);
  } finally {
    await prisma.$disconnect();
  }
}

runMonitoringTest();
