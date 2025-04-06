import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import { exerciseStore } from "../../_lib/store";

const updateExerciseSchema = z.object({
  name: z.string().min(1, "Name is required").optional(),
  videoUrl: z
    .string()
    .refine(
      (url) => {
        console.log("Validating update URL:", url);
        return url && typeof url === "string";
      },
      {
        message: "Video URL must be a non-empty string",
      },
    )
    .refine(
      (url) => {
        try {
          new URL(url);
          return true;
        } catch (e) {
          console.error("URL validation error in update:", e);
          return false;
        }
      },
      {
        message: "Valid video URL is required (must be a well-formed URL)",
      },
    )
    .optional(),
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

    console.log("Updating exercise:", params.id, JSON.stringify(body, null, 2));

    // Process videoUrl if provided
    if (body.videoUrl && typeof body.videoUrl === "string") {
      console.log("Processing videoUrl for update:", body.videoUrl);

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

    const result = updateExerciseSchema.safeParse(body);

    if (!result.success) {
      console.error(
        "Validation failed for update:",
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

    const existingExercise = exerciseStore.getById(params.id);
    if (!existingExercise) {
      console.error(`Exercise with ID ${params.id} not found for update`);
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    const updatedExercise = exerciseStore.update(params.id, result.data);

    if (!updatedExercise) {
      console.error(`Failed to update exercise with ID ${params.id}`);
      return NextResponse.json(
        { error: "Failed to update exercise" },
        { status: 500 },
      );
    }

    console.log(
      "Successfully updated exercise:",
      JSON.stringify(updatedExercise, null, 2),
    );
    return NextResponse.json(updatedExercise);
  } catch (error) {
    console.error("Error updating exercise:", error);
    return NextResponse.json(
      { error: "Failed to update exercise", details: String(error) },
      { status: 500 },
    );
  }
}

export async function DELETE(
  request: NextRequest,
  { params }: { params: { id: string } },
) {
  try {
    console.log(`Deleting exercise with ID: ${params.id}`);

    const existingExercise = exerciseStore.getById(params.id);
    if (!existingExercise) {
      console.error(`Exercise with ID ${params.id} not found for deletion`);
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    const deletedExercise = exerciseStore.delete(params.id);

    if (!deletedExercise) {
      console.error(`Failed to delete exercise with ID ${params.id}`);
      return NextResponse.json(
        { error: "Failed to delete exercise" },
        { status: 500 },
      );
    }

    console.log(
      "Successfully deleted exercise:",
      JSON.stringify(deletedExercise, null, 2),
    );
    return NextResponse.json(deletedExercise);
  } catch (error) {
    console.error("Error deleting exercise:", error);
    return NextResponse.json(
      { error: "Failed to delete exercise", details: String(error) },
      { status: 500 },
    );
  }
}
