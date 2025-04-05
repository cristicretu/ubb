import { NextResponse } from "next/server";
import { startExerciseGenerator } from "./exerciseGenerator";

let stopGenerator: (() => void) | null = null;

const getHostUrl = () => {
  return "http://localhost:3000";
};

export async function GET(req: Request) {
  const isRunning = stopGenerator !== null;

  return NextResponse.json({
    isRunning,
    message: isRunning ? "Generator is running" : "Generator is not running",
  });
}

export async function POST(req: Request) {
  try {
    try {
      await fetch(`${getHostUrl()}/api/socket`);
    } catch (error) {
      console.error("Failed to initialize Socket.io server:", error);
    }

    const body = await req.json();
    const { action, interval } = body;

    if (action === "start") {
      if (stopGenerator !== null) {
        return NextResponse.json({
          success: false,
          message: "Generator is already running",
        });
      }

      const intervalMs = interval ? Number(interval) : 30000;
      stopGenerator = startExerciseGenerator(intervalMs);

      return NextResponse.json({
        success: true,
        message: `Generator started with interval of ${intervalMs}ms`,
      });
    } else if (action === "stop") {
      if (stopGenerator === null) {
        return NextResponse.json({
          success: false,
          message: "Generator is not running",
        });
      }

      stopGenerator();
      stopGenerator = null;

      return NextResponse.json({
        success: true,
        message: "Generator stopped",
      });
    } else {
      return NextResponse.json(
        {
          success: false,
          message: 'Invalid action. Use "start" or "stop"',
        },
        { status: 400 },
      );
    }
  } catch (error) {
    console.error("Error handling background worker request:", error);
    return NextResponse.json(
      {
        success: false,
        message: "Internal server error",
      },
      { status: 500 },
    );
  }
}
