import { PrismaClient } from "@prisma/client";

const prisma = new PrismaClient();

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

  // Delete existing data
  await prisma.exercise.deleteMany();
  console.log("Cleared existing exercises");

  // Insert seed data
  for (const exercise of exercises) {
    await prisma.exercise.create({
      data: exercise,
    });
  }

  console.log(`Seeded ${exercises.length} exercises`);
}

main()
  .catch((e) => {
    console.error(e);
    process.exit(1);
  })
  .finally(async () => {
    await prisma.$disconnect();
  });
