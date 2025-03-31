import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import { exerciseStore, Exercise } from "../_lib/store";

const createExerciseSchema = z.object({
  name: z.string().min(1, "Name is required"),
  videoUrl: z.string().url("Valid video URL is required"),
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
    const body = await request.json();

    const result = createExerciseSchema.safeParse(body);

    if (!result.success) {
      return NextResponse.json(
        { error: "Validation failed", details: result.error.flatten() },
        { status: 400 },
      );
    }

    const newExercise = exerciseStore.add(result.data);

    return NextResponse.json(newExercise, { status: 201 });
  } catch (error) {
    return NextResponse.json(
      { error: "Failed to create exercise" },
      { status: 500 },
    );
  }
}
