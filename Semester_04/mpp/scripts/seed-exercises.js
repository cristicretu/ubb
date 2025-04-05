const fs = require("fs");
const path = require("path");
const { v4: uuidv4 } = require("uuid");

// Get the absolute path to the store file
const storePath = path.resolve(__dirname, "../src/app/api/_lib/store.ts");

// Read the current store content
let storeContent = fs.readFileSync(storePath, "utf8");

// Check if we already have exercises in the store
const hasInitialExercises = storeContent.includes("this.exercises = [");

// Exercise names that could be used for fitness tracking
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
  "Medicine Ball Throws",
  "Battle Ropes",
  "TRX Rows",
  "Lat Pulldowns",
  "Leg Press",
  "Calf Raises",
  "Dips",
  "Wall Sits",
  "Glute Bridges",
  "Hip Thrusts",
  "Side Planks",
  "Flutter Kicks",
  "Superman",
  "Good Mornings",
  "Step-ups",
  "Face Pulls",
  "Shrugs",
  "Lateral Raises",
  "Front Raises",
  "Hammer Curls",
  "Preacher Curls",
  "Skull Crushers",
  "Cable Pushdowns",
  "Reverse Flyes",
  "Chest Flyes",
];

// Video URL patterns (these are placeholders - not real videos)
const videoUrlPatterns = [
  "https://fitnessvids.com/exercise-",
  "https://workoutlibrary.net/videos/",
  "https://exercisedb.org/video/",
  "https://trainingmedia.com/exercises/",
];

// Generate 200+ random exercises
const generateRandomExercises = (count) => {
  const exercises = [];

  for (let i = 0; i < count; i++) {
    const form = ["bad", "medium", "good"][Math.floor(Math.random() * 3)];
    const name =
      exerciseNames[Math.floor(Math.random() * exerciseNames.length)] ||
      "Exercise";
    const videoUrlBase =
      videoUrlPatterns[Math.floor(Math.random() * videoUrlPatterns.length)];

    // Generate a random date within the last 60 days
    const date = new Date();
    date.setDate(date.getDate() - Math.floor(Math.random() * 60));

    // Duration between 30 seconds and 10 minutes (in seconds)
    const duration = Math.floor(Math.random() * (600 - 30 + 1)) + 30;

    exercises.push({
      id: uuidv4(),
      name: `${name} ${Math.floor(Math.random() * 100)}`,
      videoUrl: `${videoUrlBase}${name.toLowerCase().replace(/\s+/g, "-")}-${Math.floor(Math.random() * 1000)}.mp4`,
      form,
      date: date.toISOString(),
      duration,
    });
  }

  return exercises;
};

// Generate 210 random exercises
const newExercises = generateRandomExercises(210);

// Update the store initialization
if (hasInitialExercises) {
  // Find where the exercises array is initialized
  const exercisesInitPattern = /private exercises: Exercise\[] = \[.*?\];/s;

  // Replace the initialization with our new array
  storeContent = storeContent.replace(
    exercisesInitPattern,
    `private exercises: Exercise[] = ${JSON.stringify(newExercises, null, 2)};`,
  );
} else {
  // Add initialization if it doesn't exist
  storeContent = storeContent.replace(
    "private exercises: Exercise[] = [];",
    `private exercises: Exercise[] = ${JSON.stringify(newExercises, null, 2)};`,
  );
}

// Write the updated content back to the file
fs.writeFileSync(storePath, storeContent, "utf8");

console.log(
  `✅ Successfully added ${newExercises.length} random exercises to the store!`,
);
