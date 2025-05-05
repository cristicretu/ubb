// Script to generate sample MonitoredUser entries
import { PrismaClient } from "@prisma/client";
const prisma = new PrismaClient();

async function main() {
  console.log("Seeding monitored users...");

  // Get some users from the database that are not already monitored
  const users = await prisma.user.findMany({
    where: {
      monitoredUser: null,
    },
    take: 3,
  });

  if (users.length === 0) {
    console.error("No eligible users found for monitoring.");
    return;
  }

  const reasons = [
    "Multiple failed login attempts",
    "Suspicious activity pattern",
    "High volume of data access in short time",
    "Accessing sensitive resources",
    "Multiple IP address changes",
  ];

  // Create a monitored record for each eligible user
  const monitoredResults = [];

  for (const user of users) {
    const randomReason = reasons[Math.floor(Math.random() * reasons.length)];

    try {
      const result = await prisma.monitoredUser.create({
        data: {
          userId: user.id,
          reason: randomReason,
          activelyMonitored: true,
        },
      });

      monitoredResults.push(result);
      console.log(`Monitoring user: ${user.email || user.id}`);
    } catch (error) {
      console.error(`Failed to create monitored user for ${user.id}:`, error);
    }
  }

  console.log(`Added ${monitoredResults.length} monitored users`);
}

main()
  .then(async () => {
    await prisma.$disconnect();
    process.exit(0);
  })
  .catch(async (e) => {
    console.error(e);
    await prisma.$disconnect();
    process.exit(1);
  });
