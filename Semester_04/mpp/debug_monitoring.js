// Script to debug why monitoring is not working for the intended user

import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

async function debugSessionUser() {
  try {
    // Get all users in the system
    const allUsers = await prisma.user.findMany({
      select: {
        id: true,
        email: true,
        role: true,
      },
    });

    console.log("\n=== All Users ===");
    allUsers.forEach((user) => {
      console.log(
        `User ID: ${user.id}, Email: ${user.email}, Role: ${user.role}`,
      );
    });

    // Check for recent activity logs
    const recentLogs = await prisma.activityLog.findMany({
      take: 50,
      orderBy: {
        createdAt: "desc",
      },
      include: {
        user: {
          select: {
            id: true,
            email: true,
          },
        },
      },
    });

    // Group activities by user
    const userActivities = {};

    recentLogs.forEach((log) => {
      const userId = log.userId;
      const userEmail = log.user?.email || "Unknown";

      if (!userActivities[userId]) {
        userActivities[userId] = {
          email: userEmail,
          count: 0,
          logs: [],
        };
      }

      userActivities[userId].count++;
      userActivities[userId].logs.push({
        action: log.action,
        entity: log.entity,
        time: log.createdAt,
      });
    });

    console.log("\n=== Activity By User ===");
    for (const userId in userActivities) {
      const userData = userActivities[userId];
      console.log(`\nUser: ${userData.email} (${userId})`);
      console.log(`Total Activities: ${userData.count}`);

      // Count activities in last 10 seconds (monitoring window)
      const tenSecondsAgo = new Date(Date.now() - 10 * 1000);
      const recentCount = userData.logs.filter(
        (log) => log.time >= tenSecondsAgo,
      ).length;

      console.log(`Activities in last 10 seconds: ${recentCount}`);

      if (recentCount >= 10) {
        console.log(
          "⚠️ This user SHOULD BE monitored (10+ actions in 10 seconds)",
        );
      }

      // Show the most recent 5 activities
      console.log("Recent activities:");
      userData.logs.slice(0, 5).forEach((log) => {
        console.log(
          `- ${new Date(log.time).toLocaleTimeString()}: ${log.action} ${log.entity}`,
        );
      });
    }

    // Check monitored users
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

    console.log("\n=== Currently Monitored Users ===");
    if (monitoredUsers.length === 0) {
      console.log("No users are currently being monitored.");
    } else {
      monitoredUsers.forEach((monitored) => {
        console.log(`User: ${monitored.user.email} (${monitored.userId})`);
        console.log(`Reason: ${monitored.reason}`);
        console.log(`Since: ${monitored.createdAt}`);
        console.log(`Active: ${monitored.activelyMonitored}`);
        console.log("---");
      });
    }

    // Show the HTTP request user information from a@a.com
    const targetUser = allUsers.find((user) => user.email === "a@a.com");

    if (targetUser) {
      console.log("\n=== Target User (a@a.com) Details ===");
      console.log(`User ID: ${targetUser.id}`);
      console.log(`Email: ${targetUser.email}`);
      console.log(`Role: ${targetUser.role}`);

      // Get their activity count
      const targetUserActivities = recentLogs.filter(
        (log) => log.userId === targetUser.id,
      );
      console.log(`Total Recent Activities: ${targetUserActivities.length}`);

      // Count activities in last 10 seconds (monitoring window)
      const tenSecondsAgo = new Date(Date.now() - 10 * 1000);
      const recentCount = targetUserActivities.filter(
        (log) => log.createdAt >= tenSecondsAgo,
      ).length;

      console.log(`Activities in last 10 seconds: ${recentCount}`);

      if (recentCount >= 10) {
        console.log(
          "⚠️ This user SHOULD BE monitored (10+ actions in 10 seconds)",
        );
      }
    } else {
      console.log("\n⚠️ Target user a@a.com not found in the database!");
    }

    console.log("\n=== Monitoring Conclusion ===");
    console.log("Possible issues:");
    console.log(
      "1. The session might not be correctly identifying the user making the requests",
    );
    console.log(
      "2. Activity logs might not be created for the correct user when exercises are created",
    );
    console.log(
      "3. The monitoring check might not be running or has incorrect threshold settings",
    );
  } catch (error) {
    console.error("Error debugging monitoring:", error);
  } finally {
    await prisma.$disconnect();
  }
}

// Run the debug function
debugSessionUser();
