import { PrismaClient } from "@prisma/client";
import bcrypt from "bcryptjs";

const prisma = new PrismaClient();

const USER_ROLE = "USER";
const ADMIN_ROLE = "ADMIN";

const exercises = [
  {
    name: "Battle Ropes 89",
    videoUrl: "https://fitnessvids.com/exercise-battle-ropes-986.mp4",
    form: "bad",
    date: new Date("2025-03-25T09:11:08.551Z"),
    duration: 465,
  },
  {
    name: "Russian Twists 14",
    videoUrl: "https://workoutlibrary.net/videos/russian-twists-415.mp4",
    form: "good",
    date: new Date("2025-02-11T09:11:08.555Z"),
    duration: 512,
  },
  {
    name: "Jumping Jacks 63",
    videoUrl: "https://exercisedb.org/video/jumping-jacks-269.mp4",
    form: "good",
    date: new Date("2025-03-13T09:11:08.555Z"),
    duration: 477,
  },
  {
    name: "Flutter Kicks 57",
    videoUrl: "https://workoutlibrary.net/videos/flutter-kicks-986.mp4",
    form: "medium",
    date: new Date("2025-03-19T09:11:08.555Z"),
    duration: 178,
  },
  {
    name: "Flutter Kicks 2",
    videoUrl: "https://fitnessvids.com/exercise-flutter-kicks-582.mp4",
    form: "medium",
    date: new Date("2025-02-22T09:11:08.555Z"),
    duration: 589,
  },
  {
    name: "Flutter Kicks 34",
    videoUrl: "https://exercisedb.org/video/flutter-kicks-303.mp4",
    form: "medium",
    date: new Date("2025-02-23T09:11:08.555Z"),
    duration: 562,
  },
  {
    name: "Pull-ups 89",
    videoUrl: "https://trainingmedia.com/exercises/pull-ups-160.mp4",
    form: "good",
    date: new Date("2025-03-20T09:11:08.555Z"),
    duration: 554,
  },
  {
    name: "Hip Thrusts 86",
    videoUrl: "https://fitnessvids.com/exercise-hip-thrusts-562.mp4",
    form: "medium",
    date: new Date("2025-02-12T09:11:08.555Z"),
    duration: 202,
  },
  {
    name: "Hip Thrusts 76",
    videoUrl: "https://workoutlibrary.net/videos/hip-thrusts-809.mp4",
    form: "bad",
    date: new Date("2025-03-24T09:11:08.555Z"),
    duration: 167,
  },
  {
    name: "Crunches 60",
    videoUrl: "https://trainingmedia.com/exercises/crunches-66.mp4",
    form: "bad",
    date: new Date("2025-04-04T08:11:08.555Z"),
    duration: 465,
  },
];

async function main() {
  console.log("Starting seed...");

  // Create a test regular user
  const hashedPassword = await bcrypt.hash("password123", 10);

  // Check if test user already exists
  const existingUser = await prisma.user.findUnique({
    where: { email: "test@example.com" },
  });

  let userId;

  if (existingUser) {
    userId = existingUser.id;
    console.log("Test user already exists, using existing user");

    if (!existingUser.role) {
      await prisma.user.update({
        where: { id: existingUser.id },
        data: { role: USER_ROLE },
      });
    }
  } else {
    const newUser = await prisma.user.create({
      data: {
        name: "Test User",
        email: "test@example.com",
        password: hashedPassword,
        role: USER_ROLE,
      },
    });
    userId = newUser.id;
    console.log("Created test user:", newUser.email);
  }

  const existingAdmin = await prisma.user.findUnique({
    where: { email: "admin@example.com" },
  });

  if (existingAdmin) {
    console.log("Admin user already exists");

    // Update the admin with the ADMIN role if not set
    if (!existingAdmin.role || existingAdmin.role !== ADMIN_ROLE) {
      await prisma.user.update({
        where: { id: existingAdmin.id },
        data: { role: ADMIN_ROLE },
      });
      console.log("Updated admin user with ADMIN role");
    }
  } else {
    const adminUser = await prisma.user.create({
      data: {
        name: "Admin User",
        email: "admin@example.com",
        password: hashedPassword,
        role: ADMIN_ROLE,
      },
    });
    console.log("Created admin user:", adminUser.email);
  }

  // Delete existing exercises
  await prisma.exercise.deleteMany();
  console.log("Cleared existing exercises");

  // Insert seed data and associate with the test user
  for (const exercise of exercises) {
    await prisma.exercise.create({
      data: {
        ...exercise,
        userId: userId,
      },
    });
  }

  // Also create some exercises without a user (for users who aren't logged in)
  for (let i = 0; i < 5; i++) {
    await prisma.exercise.create({
      data: {
        name: `Public Exercise ${i + 1}`,
        videoUrl: `https://example.com/public-exercise-${i + 1}.mp4`,
        form: ["bad", "medium", "good"][Math.floor(Math.random() * 3)] as
          | "bad"
          | "medium"
          | "good",
        date: new Date(
          Date.now() - Math.floor(Math.random() * 30 * 24 * 60 * 60 * 1000),
        ),
        duration: Math.floor(Math.random() * 600) + 60,
      },
    });
  }

  console.log(
    `Seeded ${exercises.length} user exercises and 5 public exercises`,
  );
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  })
  .finally(async () => {
    await prisma.$disconnect();
  });
