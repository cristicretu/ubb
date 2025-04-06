import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import { exerciseStore, Exercise } from "../_lib/store";

const createExerciseSchema = z.object({
  name: z.string().min(1, "Name is required"),
  videoUrl: z
    .string()
    .refine(
      (url) => {
        console.log("Validating URL:", url);
        return url && typeof url === "string";
      },
      {
        message: "Video URL must be a non-empty string",
      },
    )
    .refine(
      (url) => {
        try {
          // Try to create a URL object from the string
          // This will validate if it's a well-formed URL
          new URL(url);
          return true;
        } catch (e) {
          console.error("URL validation error:", e);
          return false;
        }
      },
      {
        message: "Valid video URL is required (must be a well-formed URL)",
      },
    ),
  form: z.enum(["bad", "medium", "good"]),
  date: z.string().datetime({ message: "Valid date is required" }),
  duration: z.number().positive("Duration must be positive"),
});

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url);

  const form = searchParams.get("form");
  const search = searchParams.get("search");
  const sortBy = searchParams.get("sortBy");
  const page = parseInt(searchParams.get("page") || "1");
  const limit = parseInt(searchParams.get("limit") || "10");

  let exercises = exerciseStore.getAll();

  if (form) {
    exercises = exercises.filter((ex) => ex.form === form);
  }

  if (search && search.trim() !== "") {
    const searchLower = search.toLowerCase();
    exercises = exercises.filter((ex) =>
      ex.name.toLowerCase().includes(searchLower),
    );
  }

  const total = exercises.length;

  if (sortBy) {
    exercises = [...exercises].sort((a, b) => {
      switch (sortBy) {
        case "date-newest":
          return new Date(b.date).getTime() - new Date(a.date).getTime();
        case "date-oldest":
          return new Date(a.date).getTime() - new Date(b.date).getTime();
        case "name-asc":
          return a.name.localeCompare(b.name);
        case "name-desc":
          return b.name.localeCompare(a.name);
        case "duration-highest":
          return b.duration - a.duration;
        case "duration-lowest":
          return a.duration - b.duration;
        default:
          return 0;
      }
    });
  }

  const startIndex = (page - 1) * limit;
  const endIndex = startIndex + limit;
  const paginatedExercises = exercises.slice(startIndex, endIndex);

  return NextResponse.json({
    exercises: paginatedExercises,
    total,
    page,
    limit,
    totalPages: Math.ceil(total / limit),
  });
}

export async function POST(request: NextRequest) {
  try {
    let body;
    try {
      body = await request.json();
    } catch (error) {
      console.error("Failed to parse request body:", error);
      return NextResponse.json(
        { error: "Invalid request body - not valid JSON" },
        { status: 400 },
      );
    }

    console.log("Received exercise data:", JSON.stringify(body, null, 2));

    // Validate the basic structure before schema validation
    if (!body || typeof body !== "object") {
      return NextResponse.json(
        { error: "Invalid request body - expected an object" },
        { status: 400 },
      );
    }

    // Check required fields
    const requiredFields = ["name", "videoUrl", "form", "date", "duration"];
    const missingFields = requiredFields.filter((field) => !(field in body));

    if (missingFields.length > 0) {
      return NextResponse.json(
        {
          error: "Missing required fields",
          details: { missingFields },
          received: body,
        },
        { status: 400 },
      );
    }

    // Validate specific fields before schema validation
    if (body.videoUrl && typeof body.videoUrl === "string") {
      console.log("Processing videoUrl:", body.videoUrl);

      // Try to convert to a valid URL if needed
      if (!body.videoUrl.startsWith("http")) {
        const originalUrl = body.videoUrl;
        try {
          // If it's a relative URL, try to make it absolute with a dummy base
          const url = new URL(originalUrl, "http://localhost:3000");
          body.videoUrl = url.toString();
          console.log(
            `Converted relative URL "${originalUrl}" to "${body.videoUrl}"`,
          );
        } catch (error) {
          console.error(`Failed to convert URL "${originalUrl}":`, error);
          // Keep the original URL and let schema validation handle it
        }
      }
    }

    const result = createExerciseSchema.safeParse(body);

    if (!result.success) {
      console.error(
        "Validation failed:",
        JSON.stringify(result.error.format(), null, 2),
      );
      return NextResponse.json(
        {
          error: "Validation failed",
          details: result.error.flatten(),
          received: body,
        },
        { status: 400 },
      );
    }

    const newExercise = exerciseStore.add(result.data);
    console.log("Created new exercise:", JSON.stringify(newExercise, null, 2));

    return NextResponse.json(newExercise, { status: 201 });
  } catch (error) {
    console.error("Error creating exercise:", error);
    return NextResponse.json(
      { error: "Failed to create exercise", details: String(error) },
      { status: 500 },
    );
  }
}
