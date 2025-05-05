import { db } from "~/server/db";
import { LogActionType, LogEntityType } from "~/server/services/logger";

async function main() {
  console.log("Seeding activity logs...");

  // Get some users from the database
  const users = await db.user.findMany({ take: 3 });

  if (users.length === 0) {
    console.error(
      "No users found in the database. Please create some users first.",
    );
    return;
  }

  // Sample activities
  const activities = [
    {
      action: "CREATE",
      entity: "EXERCISE",
      details: "Created 'Push ups' exercise",
    },
    {
      action: "UPDATE",
      entity: "EXERCISE",
      details: "Updated exercise duration to 30 minutes",
    },
    {
      action: "DELETE",
      entity: "EXERCISE",
      details: "Deleted 'Running' exercise",
    },
    {
      action: "CREATE",
      entity: "POST",
      details: "Created new post 'My fitness journey'",
    },
    { action: "READ", entity: "USER", details: "Viewed user profile" },
    {
      action: "UPDATE",
      entity: "USER",
      details: "Updated profile information",
    },
  ];

  // Create 20 random logs
  const sampleLogs = [];
  for (let i = 0; i < 20; i++) {
    const randomUser = users[Math.floor(Math.random() * users.length)];
    const randomActivity =
      activities[Math.floor(Math.random() * activities.length)];

    sampleLogs.push({
      userId: randomUser.id,
      action: randomActivity.action,
      entity: randomActivity.entity,
      entityId: `sample-${i}`,
      details: randomActivity.details,
      createdAt: new Date(
        Date.now() - Math.floor(Math.random() * 7 * 24 * 60 * 60 * 1000),
      ), // Random date within the last week
    });
  }

  // Insert logs in batches
  const createdLogs = await db.activityLog.createMany({
    data: sampleLogs,
    skipDuplicates: true,
  });

  console.log(`Created ${createdLogs.count} sample activity logs`);
}

main()
  .then(() => process.exit(0))
  .catch((e) => {
    console.error(e);
    process.exit(1);
  });
