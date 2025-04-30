import { z } from "zod";
import {
  createTRPCRouter,
  protectedProcedure,
  publicProcedure,
} from "~/server/api/trpc";
import {
  logCreate,
  logDelete,
  logRead,
  logUpdate,
  LogEntityType,
} from "~/server/services/logger";

export const exerciseRouter = createTRPCRouter({
  getAll: publicProcedure.query(async ({ ctx }) => {
    const exercises = await ctx.db.exercise.findMany({
      orderBy: { date: "desc" },
      include: {
        user: {
          select: {
            id: true,
            name: true,
          },
        },
      },
    });

    // Log this action if user is authenticated
    if (ctx.session?.user) {
      void logRead(
        ctx.session.user,
        LogEntityType.EXERCISE,
        "all",
        "Get all exercises",
      );
    }

    return exercises;
  }),

  getById: publicProcedure
    .input(z.object({ id: z.string() }))
    .query(async ({ ctx, input }) => {
      const exercise = await ctx.db.exercise.findUnique({
        where: { id: input.id },
        include: {
          user: {
            select: {
              id: true,
              name: true,
            },
          },
        },
      });

      // Log this action if user is authenticated
      if (ctx.session?.user) {
        void logRead(
          ctx.session.user,
          LogEntityType.EXERCISE,
          input.id,
          "Get exercise by ID",
        );
      }

      return exercise;
    }),

  getUserExercises: protectedProcedure.query(async ({ ctx }) => {
    const exercises = await ctx.db.exercise.findMany({
      where: { userId: ctx.session.user.id },
      orderBy: { date: "desc" },
    });

    // Log this action - we can directly log it since this is a protected procedure
    void logRead(
      ctx.session.user,
      LogEntityType.EXERCISE,
      "user-exercises",
      "Get user exercises",
    );

    return exercises;
  }),

  create: protectedProcedure
    .input(
      z.object({
        name: z.string(),
        videoUrl: z.string(),
        form: z.string(),
        date: z.date(),
        duration: z.number(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const exercise = await ctx.db.exercise.create({
        data: {
          ...input,
          userId: ctx.session.user.id,
        },
      });

      // Log this creation
      void logCreate(
        ctx.session.user,
        LogEntityType.EXERCISE,
        exercise.id,
        `Created exercise: ${exercise.name}`,
      );

      return exercise;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string(),
        name: z.string().optional(),
        videoUrl: z.string().optional(),
        form: z.string().optional(),
        date: z.date().optional(),
        duration: z.number().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input;

      // First check if the exercise belongs to the user
      const exercise = await ctx.db.exercise.findUnique({
        where: { id },
      });

      if (!exercise || exercise.userId !== ctx.session.user.id) {
        throw new Error("Not authorized to update this exercise");
      }

      const updatedExercise = await ctx.db.exercise.update({
        where: { id },
        data,
      });

      // Log this update
      void logUpdate(
        ctx.session.user,
        LogEntityType.EXERCISE,
        id,
        `Updated exercise: ${updatedExercise.name}`,
      );

      return updatedExercise;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string() }))
    .mutation(async ({ ctx, input }) => {
      // First check if the exercise belongs to the user
      const exercise = await ctx.db.exercise.findUnique({
        where: { id: input.id },
      });

      if (!exercise || exercise.userId !== ctx.session.user.id) {
        throw new Error("Not authorized to delete this exercise");
      }

      await ctx.db.exercise.delete({
        where: { id: input.id },
      });

      // Log this deletion
      void logDelete(
        ctx.session.user,
        LogEntityType.EXERCISE,
        input.id,
        `Deleted exercise: ${exercise.name}`,
      );

      return { success: true };
    }),
});
