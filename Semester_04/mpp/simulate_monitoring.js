// Script to simulate monitoring for a specific user (a@a.com)

import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

async function monitorSpecificUser() {
  try {
    console.log("Starting targeted monitoring for a@a.com user...");

    // Clear existing monitored users
    await prisma.monitoredUser.deleteMany({});
    console.log("✅ Cleared monitored users table");

    // Find the target user (a@a.com)
    const targetUser = await prisma.user.findFirst({
      where: { email: "a@a.com" },
    });

    if (!targetUser) {
      console.error("❌ Target user a@a.com not found in the database!");
      return;
    }

    console.log(`Found target user: ${targetUser.email} (${targetUser.id})`);

    // Create monitoring entry directly
    const monitoredUser = await prisma.monitoredUser.create({
      data: {
        userId: targetUser.id,
        reason: "Manual monitoring flag set for testing purposes",
        activelyMonitored: true,
      },
    });

    console.log("✅ Successfully created monitoring entry:");
    console.log(`User ID: ${monitoredUser.userId}`);
    console.log(`Reason: ${monitoredUser.reason}`);
    console.log(`Created At: ${monitoredUser.createdAt}`);
    console.log(`Active: ${monitoredUser.activelyMonitored}`);

    console.log(
      "\nNow check the admin panel at http://localhost:3000/admin/monitoring",
    );
    console.log("You should see a@a.com user being monitored.");
  } catch (error) {
    console.error("Error setting up monitoring:", error);
  } finally {
    await prisma.$disconnect();
  }
}

monitorSpecificUser();
