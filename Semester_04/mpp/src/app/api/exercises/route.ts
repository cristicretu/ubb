import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import prisma from "../_lib/prisma";
import { auth } from "~/server/auth";
import { logRead, logCreate, LogEntityType } from "~/server/services/logger";

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
  const sortBy = searchParams.get("sortBy") || "date-newest";
  const page = parseInt(searchParams.get("page") || "1");
  const limit = parseInt(searchParams.get("limit") || "10");

  // Get the authenticated user (if any)
  const session = await auth();
  const userId = session?.user?.id;

  // Build query conditions
  const where: any = {};

  if (form && form !== "all") {
    where.form = form;
  }

  if (search && search.trim() !== "") {
    where.name = {
      contains: search,
      mode: "insensitive", // Case-insensitive search
    };
  }

  // If authenticated, show only user's exercises
  // If not authenticated, show public exercises
  if (userId) {
    where.userId = userId;
  }

  // Determine sort order
  let orderBy: any = {};
  switch (sortBy) {
    case "date-newest":
    case "date-desc":
      orderBy = { date: "desc" };
      break;
    case "date-oldest":
    case "date-asc":
      orderBy = { date: "asc" };
      break;
    case "name-asc":
      orderBy = { name: "asc" };
      break;
    case "name-desc":
      orderBy = { name: "desc" };
      break;
    case "duration-highest":
      orderBy = { duration: "desc" };
      break;
    case "duration-lowest":
      orderBy = { duration: "asc" };
      break;
    default:
      orderBy = { date: "desc" };
  }

  try {
    // Count total matching exercises
    const total = await prisma.exercise.count({ where });

    // Get paginated exercises
    const exercises = await prisma.exercise.findMany({
      where,
      orderBy,
      skip: (page - 1) * limit,
      take: limit,
      include: {
        user: {
          select: {
            id: true,
            name: true,
          },
        },
      },
    });

    // Log this read action if user is authenticated
    if (session?.user) {
      void logRead(
        session.user,
        LogEntityType.EXERCISE,
        "all",
        "Get all exercises via REST API",
      );
    }

    return NextResponse.json({
      exercises,
      total,
      page,
      limit,
      totalPages: Math.ceil(total / limit),
    });
  } catch (error) {
    console.error("Error fetching exercises:", error);
    return NextResponse.json(
      { error: "Failed to fetch exercises" },
      { status: 500 },
    );
  }
}

export async function POST(request: NextRequest) {
  try {
    // Get the authenticated user
    const session = await auth();
    const userId = session?.user?.id;

    // Check if user is authenticated
    if (!userId) {
      return NextResponse.json(
        { error: "You must be signed in to create exercises" },
        { status: 401 },
      );
    }

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

    // Create new exercise with Prisma and associate with user
    const newExercise = await prisma.exercise.create({
      data: {
        name: result.data.name,
        videoUrl: result.data.videoUrl,
        form: result.data.form,
        date: new Date(result.data.date),
        duration: result.data.duration,
        userId: userId,
      },
    });

    // Log this creation
    void logCreate(
      session.user,
      LogEntityType.EXERCISE,
      newExercise.id,
      `Created exercise: ${newExercise.name} via REST API`,
    );

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
