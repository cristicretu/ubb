import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import { exerciseStore } from "../../_lib/store";

const updateExerciseSchema = z.object({
  name: z.string().min(1, "Name is required").optional(),
  videoUrl: z.string().url("Valid video URL is required").optional(),
  form: z.enum(["bad", "medium", "good"]).optional(),
  date: z.string().datetime({ message: "Valid date is required" }).optional(),
  duration: z.number().positive("Duration must be positive").optional(),
});

export async function GET(
  request: NextRequest,
  { params }: { params: { id: string } },
) {
  const exercise = exerciseStore.getById(params.id);

  if (!exercise) {
    return NextResponse.json({ error: "Exercise not found" }, { status: 404 });
  }

  return NextResponse.json(exercise);
}

export async function PUT(
  request: NextRequest,
  { params }: { params: { id: string } },
) {
  try {
    const body = await request.json();

    const result = updateExerciseSchema.safeParse(body);

    if (!result.success) {
      return NextResponse.json(
        { error: "Validation failed", details: result.error.flatten() },
        { status: 400 },
      );
    }

    const updatedExercise = exerciseStore.update(params.id, result.data);

    if (!updatedExercise) {
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    return NextResponse.json(updatedExercise);
  } catch (error) {
    return NextResponse.json(
      { error: "Failed to update exercise" },
      { status: 500 },
    );
  }
}

export async function DELETE(
  request: NextRequest,
  { params }: { params: { id: string } },
) {
  const deletedExercise = exerciseStore.delete(params.id);

  if (!deletedExercise) {
    return NextResponse.json({ error: "Exercise not found" }, { status: 404 });
  }

  return NextResponse.json(deletedExercise);
}
