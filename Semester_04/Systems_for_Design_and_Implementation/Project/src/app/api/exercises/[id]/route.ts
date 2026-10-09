import { NextRequest, NextResponse } from "next/server";
import { z } from "zod";
import prisma from "../../_lib/prisma";
import { auth } from "~/server/auth";
import {
  logRead,
  logUpdate,
  logDelete,
  LogEntityType,
} from "~/server/services/logger";

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
  try {
    const id = params.id;

    // Get authenticated user
    const session = await auth();
    const userId = session?.user?.id;

    const exercise = await prisma.exercise.findUnique({
      where: { id },
      include: {
        user: {
          select: {
            id: true,
            name: true,
          },
        },
      },
    });

    if (!exercise) {
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    // If the exercise belongs to a user, verify ownership
    if (exercise.userId && exercise.userId !== userId) {
      return NextResponse.json(
        { error: "You don't have permission to access this exercise" },
        { status: 403 },
      );
    }

    // Log this read action if user is authenticated
    if (session?.user) {
      void logRead(
        session.user,
        LogEntityType.EXERCISE,
        id,
        `Get exercise by ID: ${exercise.name} via REST API`,
      );
    }

    return NextResponse.json(exercise);
  } catch (error) {
    console.error(`Error fetching exercise ${params.id}:`, error);
    return NextResponse.json(
      { error: "Failed to fetch exercise" },
      { status: 500 },
    );
  }
}

export async function PUT(
  request: NextRequest,
  { params }: { params: { id: string } },
) {
  try {
    const id = params.id;

    // Get authenticated user
    const session = await auth();
    const userId = session?.user?.id;

    if (!userId) {
      return NextResponse.json(
        { error: "You must be signed in to update exercises" },
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

    console.log("Updating exercise:", id, JSON.stringify(body, null, 2));

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

    // Check if exercise exists and if the user owns it
    const existingExercise = await prisma.exercise.findUnique({
      where: { id },
    });

    if (!existingExercise) {
      console.error(`Exercise with ID ${id} not found for update`);
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    // Verify ownership
    if (existingExercise.userId && existingExercise.userId !== userId) {
      return NextResponse.json(
        { error: "You don't have permission to update this exercise" },
        { status: 403 },
      );
    }

    // Prepare data for update
    const updateData: any = { ...result.data };

    // Convert date string to Date object if provided
    if (updateData.date) {
      updateData.date = new Date(updateData.date);
    }

    // Update the exercise in the database
    const updatedExercise = await prisma.exercise.update({
      where: { id },
      data: updateData,
    });

    // Log this update if user is authenticated
    if (session?.user) {
      void logUpdate(
        session.user,
        LogEntityType.EXERCISE,
        id,
        `Updated exercise: ${updatedExercise.name} via REST API`,
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
    const id = params.id;

    // Get authenticated user
    const session = await auth();
    const userId = session?.user?.id;

    if (!userId) {
      return NextResponse.json(
        { error: "You must be signed in to delete exercises" },
        { status: 401 },
      );
    }

    console.log(`Deleting exercise with ID: ${id}`);

    // Check if exercise exists and if user owns it
    const existingExercise = await prisma.exercise.findUnique({
      where: { id },
    });

    if (!existingExercise) {
      console.error(`Exercise with ID ${id} not found for deletion`);
      return NextResponse.json(
        { error: "Exercise not found" },
        { status: 404 },
      );
    }

    // Verify ownership
    if (existingExercise.userId && existingExercise.userId !== userId) {
      return NextResponse.json(
        { error: "You don't have permission to delete this exercise" },
        { status: 403 },
      );
    }

    // Delete the exercise
    const deletedExercise = await prisma.exercise.delete({
      where: { id },
    });

    // Log this deletion
    void logDelete(
      session.user,
      LogEntityType.EXERCISE,
      id,
      `Deleted exercise: ${existingExercise.name} via REST API`,
    );

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
