import { PrismaClient } from "@prisma/client";
import { faker } from "@faker-js/faker";
import { performance } from "perf_hooks";

const prisma = new PrismaClient();

const USERS_COUNT = 1000;
const EXERCISES_PER_USER = 100;
const ACTIVITY_LOGS_PER_USER = 100;
const MONITORED_USERS_PERCENTAGE = 5;
const BATCH_SIZE = 500;

async function main() {
  console.log("Starting database stress test data generation...");
  const startTime = performance.now();

  try {
    const userCount = await prisma.user.count();
    const exerciseCount = await prisma.exercise.count();
    const activityLogCount = await prisma.activityLog.count();

    if (userCount > 900 && exerciseCount > 90000 && activityLogCount > 90000) {
      console.log("Database already contains large volume of test data.");
      console.log(
        `Users: ${userCount}, Exercises: ${exerciseCount}, Activity Logs: ${activityLogCount}`,
      );
      return;
    }

    console.log(`Generating ${USERS_COUNT} users...`);
    const userIds = [];

    for (let i = 0; i < USERS_COUNT; i += BATCH_SIZE) {
      const batchSize = Math.min(BATCH_SIZE, USERS_COUNT - i);
      const userBatch = [];

      for (let j = 0; j < batchSize; j++) {
        const firstName = faker.person.firstName();
        const lastName = faker.person.lastName();

        userBatch.push({
          name: `${firstName} ${lastName}`,
          email: faker.internet.email({
            firstName,
            lastName,
            provider: "example.com",
          }),
          password: faker.internet.password(),
          role: faker.helpers.arrayElement([
            "USER",
            "USER",
            "USER",
            "USER",
            "ADMIN",
          ]),
          image: faker.image.avatar(),
        });
      }

      const createdUsers = await prisma.user.createMany({
        data: userBatch,
        skipDuplicates: true,
      });

      console.log(
        `Created batch of ${createdUsers.count} users (${i + createdUsers.count}/${USERS_COUNT})`,
      );

      const batchUserIds = await prisma.user.findMany({
        where: {
          email: {
            in: userBatch.map((user) => user.email),
          },
        },
        select: { id: true },
      });

      userIds.push(...batchUserIds.map((user) => user.id));
    }

    console.log(`Created ${userIds.length} users`);

    console.log(
      `Generating ${EXERCISES_PER_USER} exercises per user (${USERS_COUNT * EXERCISES_PER_USER} total)...`,
    );

    const exerciseNames = [
      "Push-ups",
      "Pull-ups",
      "Squats",
      "Lunges",
      "Deadlifts",
      "Bench Press",
      "Shoulder Press",
      "Bicep Curls",
      "Tricep Extensions",
      "Plank",
      "Crunches",
      "Leg Raises",
      "Russian Twists",
      "Mountain Climbers",
      "Burpees",
      "Jumping Jacks",
      "High Knees",
      "Jumping Rope",
      "Box Jumps",
      "Kettlebell Swings",
    ];

    const videoUrlPatterns = [
      "https://fitnessvids.com/exercise-",
      "https://workoutlibrary.net/videos/",
      "https://exercisedb.org/video/",
      "https://trainingmedia.com/exercises/",
    ];

    let totalExercises = 0;

    for (const userId of userIds) {
      for (let i = 0; i < EXERCISES_PER_USER; i += BATCH_SIZE) {
        const batchSize = Math.min(BATCH_SIZE, EXERCISES_PER_USER - i);
        const exerciseBatch = [];

        for (let j = 0; j < batchSize; j++) {
          const name = faker.helpers.arrayElement(exerciseNames);
          const videoUrlBase = faker.helpers.arrayElement(videoUrlPatterns);

          const pastDate = new Date();
          pastDate.setDate(
            pastDate.getDate() - faker.number.int({ min: 1, max: 365 }),
          );

          const duration = faker.number.int({ min: 30, max: 600 });

          exerciseBatch.push({
            name: `${name} ${faker.number.int({ min: 1, max: 100 })}`,
            videoUrl: `${videoUrlBase}${name.toLowerCase().replace(/\s+/g, "-")}-${faker.number.int({ min: 1, max: 1000 })}.mp4`,
            form: faker.helpers.arrayElement(["bad", "medium", "good"]),
            date: pastDate,
            duration,
            userId,
          });
        }

        const createdExercises = await prisma.exercise.createMany({
          data: exerciseBatch,
        });

        totalExercises += createdExercises.count;
        if (
          totalExercises % 10000 === 0 ||
          totalExercises === USERS_COUNT * EXERCISES_PER_USER
        ) {
          console.log(
            `Created ${totalExercises}/${USERS_COUNT * EXERCISES_PER_USER} exercises`,
          );
        }
      }
    }

    console.log(
      `Generating ${ACTIVITY_LOGS_PER_USER} activity logs per user (${USERS_COUNT * ACTIVITY_LOGS_PER_USER} total)...`,
    );

    const actions = ["CREATE", "READ", "UPDATE", "DELETE"];
    const entities = ["Exercise", "User", "Post"];

    let totalLogs = 0;

    for (const userId of userIds) {
      for (let i = 0; i < ACTIVITY_LOGS_PER_USER; i += BATCH_SIZE) {
        const batchSize = Math.min(BATCH_SIZE, ACTIVITY_LOGS_PER_USER - i);
        const logBatch = [];

        for (let j = 0; j < batchSize; j++) {
          const action = faker.helpers.arrayElement(actions);
          const entity = faker.helpers.arrayElement(entities);

          const pastDate = new Date();
          pastDate.setDate(
            pastDate.getDate() - faker.number.int({ min: 1, max: 90 }),
          );

          logBatch.push({
            action,
            entity,
            entityId: faker.string.uuid(),
            details: faker.lorem.sentence(),
            userId,
            createdAt: pastDate,
          });
        }

        const createdLogs = await prisma.activityLog.createMany({
          data: logBatch,
        });

        totalLogs += createdLogs.count;
        if (
          totalLogs % 10000 === 0 ||
          totalLogs === USERS_COUNT * ACTIVITY_LOGS_PER_USER
        ) {
          console.log(
            `Created ${totalLogs}/${USERS_COUNT * ACTIVITY_LOGS_PER_USER} activity logs`,
          );
        }
      }
    }

    const monitoredUsersCount = Math.floor(
      USERS_COUNT * (MONITORED_USERS_PERCENTAGE / 100),
    );
    console.log(`Generating ${monitoredUsersCount} monitored users...`);

    const randomUserIds = faker.helpers.arrayElements(
      userIds,
      monitoredUsersCount,
    );

    for (const userId of randomUserIds) {
      await prisma.monitoredUser.create({
        data: {
          reason: faker.lorem.sentence(),
          activelyMonitored: Math.random() > 0.5,
          userId,
        },
      });
    }

    console.log(`Created ${monitoredUsersCount} monitored users`);

    const endTime = performance.now();
    console.log(
      `✅ Database populated successfully in ${((endTime - startTime) / 1000 / 60).toFixed(2)} minutes!`,
    );
    console.log(
      `Created ${userIds.length} users, ${totalExercises} exercises, ${totalLogs} activity logs, and ${monitoredUsersCount} monitored users.`,
    );
  } catch (error) {
    console.error("Error generating test data:", error);
  } finally {
    await prisma.$disconnect();
  }
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
