import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

async function clearMonitoredUsers() {
  try {
    const deleteResult = await prisma.monitoredUser.deleteMany({});

    console.log(
      `Cleared ${deleteResult.count} entries from the monitored users table.`,
    );
  } catch (error) {
    console.error("Error clearing monitored users table:", error);
  } finally {
    await prisma.$disconnect();
  }
}

clearMonitoredUsers();
