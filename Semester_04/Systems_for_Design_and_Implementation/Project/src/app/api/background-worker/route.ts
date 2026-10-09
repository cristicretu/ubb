import { NextRequest, NextResponse } from "next/server";
import prisma from "../_lib/prisma";
import { LogEntityType, logCreate } from "~/server/services/logger";

let generatorInterval: NodeJS.Timeout | null = null;
let isRunning = false;

// Function to generate a random exercise
async function generateRandomExercise() {
  try {
    const exerciseNames = [
      "Squats",
      "Push-ups",
      "Pull-ups",
      "Deadlifts",
      "Lunges",
      "Bench Press",
      "Planks",
      "Bicep Curls",
      "Shoulder Press",
      "Russian Twists",
      "Crunches",
      "Leg Raises",
      "Mountain Climbers",
      "Jumping Jacks",
      "Burpees",
      "Box Jumps",
      "Kettlebell Swings",
    ];

    const forms = ["bad", "medium", "good"];

    const randomExercise = {
      name: `${exerciseNames[Math.floor(Math.random() * exerciseNames.length)]} ${Math.floor(Math.random() * 100)}`,
      videoUrl: `https://example.com/videos/exercise-${Math.floor(Math.random() * 1000)}.mp4`,
      form: forms[Math.floor(Math.random() * forms.length)] as
        | "bad"
        | "medium"
        | "good",
      date: new Date(
        Date.now() - Math.floor(Math.random() * 30 * 24 * 60 * 60 * 1000),
      ), // Random date in the last 30 days
      duration: Math.floor(Math.random() * 600) + 60, // 60-660 seconds
    };

    const createdExercise = await prisma.exercise.create({
      data: randomExercise,
    });

    // We don't have a real user for background generated exercises,
    // but we should still log the activity using a system user ID
    void logCreate(
      { id: "system" }, // Using a special "system" user ID
      LogEntityType.EXERCISE,
      createdExercise.id,
      `Auto-generated exercise: ${createdExercise.name}`,
    );

    console.log("Generated random exercise:", createdExercise);

    // Emit WebSocket event if needed
    // You could add WebSocket emit code here

    return createdExercise;
  } catch (error) {
    console.error("Error generating random exercise:", error);
    throw error;
  }
}

// GET handler to check status
export async function GET(request: NextRequest) {
  return NextResponse.json({ isRunning });
}

// POST handler to start/stop generator
export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { action, interval = 30000 } = body;

    if (action === "start") {
      if (isRunning) {
        return NextResponse.json({
          success: false,
          message: "Generator already running",
          isRunning,
        });
      }

      // Generate one exercise immediately
      await generateRandomExercise();

      // Set up interval for regular generation
      generatorInterval = setInterval(generateRandomExercise, interval);
      isRunning = true;

      return NextResponse.json({
        success: true,
        message: `Started exercise generator with interval ${interval}ms`,
        isRunning,
      });
    } else if (action === "stop") {
      if (!isRunning || !generatorInterval) {
        return NextResponse.json({
          success: false,
          message: "Generator not running",
          isRunning: false,
        });
      }

      clearInterval(generatorInterval);
      generatorInterval = null;
      isRunning = false;

      return NextResponse.json({
        success: true,
        message: "Stopped exercise generator",
        isRunning,
      });
    } else {
      return NextResponse.json(
        {
          success: false,
          message: "Invalid action. Use 'start' or 'stop'",
          isRunning,
        },
        { status: 400 },
      );
    }
  } catch (error) {
    console.error("Error handling background worker request:", error);
    return NextResponse.json(
      {
        success: false,
        message: `Error: ${error instanceof Error ? error.message : String(error)}`,
        isRunning,
      },
      { status: 500 },
    );
  }
}
