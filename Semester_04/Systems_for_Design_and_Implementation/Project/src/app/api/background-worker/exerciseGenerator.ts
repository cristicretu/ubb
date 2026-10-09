// Types
interface Exercise {
  id: string;
  name: string;
  videoUrl: string;
  form: "bad" | "medium" | "good";
  date: string;
  duration: number;
}

// Exercise names to randomly select from
const exerciseNames = [
  "Push-ups",
  "Squats",
  "Deadlifts",
  "Pull-ups",
  "Lunges",
  "Bench Press",
  "Bicep Curls",
  "Tricep Extensions",
  "Shoulder Press",
  "Plank",
];

// Exercise forms
const exerciseForms: ("bad" | "medium" | "good")[] = ["bad", "medium", "good"];

// Placeholder video URLs
const videoUrls = [
  "https://example.com/videos/exercise1.mp4",
  "https://example.com/videos/exercise2.mp4",
  "https://example.com/videos/exercise3.mp4",
  "https://example.com/videos/exercise4.mp4",
  "https://example.com/videos/exercise5.mp4",
];

// Host URL for server-side API calls - use process.env.VERCEL_URL or localhost in development
const getHostUrl = () => {
  if (process.env.VERCEL_URL) {
    return `https://${process.env.VERCEL_URL}`;
  }
  return "http://localhost:3000";
};

// Random data generator functions
const getRandomElement = <T>(array: T[]): T => {
  if (array.length === 0) {
    throw new Error("Cannot get random element from empty array");
  }
  const randomIndex = Math.floor(Math.random() * array.length);
  return array[randomIndex]!;
};

const getRandomDuration = (): number => {
  return Math.floor(Math.random() * 120) + 30; // 30 to 150 seconds
};

const getRandomDate = (): string => {
  const now = new Date();
  const daysAgo = Math.floor(Math.random() * 30); // Up to 30 days ago
  now.setDate(now.getDate() - daysAgo);
  return now.toISOString();
};

// Generate a random exercise
export const generateRandomExercise = (): Exercise => {
  return {
    id: `gen_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`,
    name: getRandomElement(exerciseNames),
    videoUrl: getRandomElement(videoUrls),
    form: getRandomElement(exerciseForms),
    date: getRandomDate(),
    duration: getRandomDuration(),
  };
};

// Function to emit Socket.io events
const emitSocketEvent = async (event: string, data: any) => {
  try {
    // Call the API route to emit events through the Socket.io server
    const response = await fetch(`${getHostUrl()}/api/socket-emit`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ event, data }),
    });

    return response.ok;
  } catch (error) {
    console.error("Error emitting socket event:", error);
    return false;
  }
};

// Function to simulate creating a new exercise
export const simulateNewExercise = async () => {
  const newExercise = generateRandomExercise();

  // Emit socket event for real-time update
  await fetch(`${getHostUrl()}/api/socket-emit`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      event: "exercise:created",
      data: newExercise,
    }),
  }).catch((err) => console.error("Failed to emit event:", err));

  // Return the generated exercise
  return newExercise;
};

// Function to simulate updating an existing exercise
export const simulateExerciseUpdate = async (exerciseId: string) => {
  const updates = {
    form: getRandomElement(exerciseForms),
    duration: getRandomDuration(),
  };

  // Emit socket event for real-time update
  await fetch(`${getHostUrl()}/api/socket-emit`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      event: "exercise:updated",
      data: { id: exerciseId, ...updates },
    }),
  }).catch((err) => console.error("Failed to emit event:", err));

  return { id: exerciseId, ...updates };
};

// Start a timer to periodically generate new exercises
export const startExerciseGenerator = (intervalMs = 30000) => {
  console.log(`Starting exercise generator with interval of ${intervalMs}ms`);

  // Store the interval ID to allow stopping later
  const intervalId = setInterval(async () => {
    try {
      console.log("Generating random exercise...");
      const exercise = await simulateNewExercise();
      console.log("Generated exercise:", exercise);
    } catch (error) {
      console.error("Error generating exercise:", error);
    }
  }, intervalMs);

  // Return a function to stop the generator
  return () => {
    console.log("Stopping exercise generator");
    clearInterval(intervalId);
  };
};
