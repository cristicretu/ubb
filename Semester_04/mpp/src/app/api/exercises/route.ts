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

  const sort = searchParams.get("sort");
  const order = searchParams.get("order") === "desc" ? -1 : 1;

  let exercises = exerciseStore.getAll();

  if (form) {
    exercises = exerciseStore.filter({
      form: form as "bad" | "medium" | "good",
    });
  }

  if (sort) {
    exercises.sort((a, b) => {
      // @ts-ignore
      const valueA = a[sort];
      // @ts-ignore
      const valueB = b[sort];

      if (typeof valueA === "string" && typeof valueB === "string") {
        return valueA.localeCompare(valueB) * order;
      }

      return ((valueA as number) - (valueB as number)) * order;
    });
  }

  return NextResponse.json(exercises);
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
