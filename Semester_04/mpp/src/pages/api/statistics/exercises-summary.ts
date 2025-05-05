import { NextApiRequest, NextApiResponse } from "next";
import { PrismaClient, Prisma } from "@prisma/client";
import { performance } from "perf_hooks";

// Initialize Prisma client
const prisma = new PrismaClient();

type FormType = "good" | "medium" | "bad";

const customJSONStringify = (data: any) => {
  return JSON.stringify(data, (_, value) =>
    typeof value === "bigint" ? Number(value) : value,
  );
};

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  if (req.method !== "GET") {
    return res.status(405).json({ error: "Method not allowed" });
  }

  const startTime = performance.now();

  try {
    const {
      timeframe = "30",
      formFilter = "all",
      limit = "10",
      userId,
    } = req.query;

    const timeframeDays = parseInt(timeframe as string);
    const resultLimit = parseInt(limit as string);

    const startDate = new Date();
    startDate.setDate(startDate.getDate() - timeframeDays);

    const formCondition =
      formFilter !== "all" &&
      ["good", "medium", "bad"].includes(formFilter as string)
        ? { form: formFilter as FormType }
        : {};

    const userCondition = userId ? { userId: userId as string } : {};

    const [
      totalExercises,
      exercisesByForm,
      exercisesByName,
      exercisesByDay,
      averageDuration,
      mostActiveUsers,
    ] = await Promise.all([
      prisma.exercise.count({
        where: {
          date: {
            gte: startDate,
          },
          ...formCondition,
          ...userCondition,
        },
      }),

      prisma.exercise.groupBy({
        by: ["form"],
        where: {
          date: {
            gte: startDate,
          },
          ...userCondition,
        },
        _count: true,
      }),

      prisma.exercise.groupBy({
        by: ["name"],
        where: {
          date: {
            gte: startDate,
          },
          ...formCondition,
          ...userCondition,
        },
        _count: true,
        orderBy: {
          _count: {
            name: "desc",
          },
        },
        take: resultLimit,
      }),

      // Exercises by day for time series chart
      prisma.$queryRaw`
        SELECT 
          DATE_TRUNC('day', "date") as day,
          COUNT(*)::integer as count
        FROM "Exercise"
        WHERE "date" >= ${startDate}
        ${
          formFilter !== "all" &&
          ["good", "medium", "bad"].includes(formFilter as string)
            ? Prisma.sql`AND "form" = ${formFilter as string}`
            : Prisma.empty
        }
        ${userId ? Prisma.sql`AND "userId" = ${userId}` : Prisma.empty}
        GROUP BY DATE_TRUNC('day', "date")
        ORDER BY day ASC
      `,

      // Average duration by exercise type
      prisma.exercise.groupBy({
        by: ["name"],
        where: {
          date: {
            gte: startDate,
          },
          ...formCondition,
          ...userCondition,
        },
        _avg: {
          duration: true,
        },
        orderBy: {
          _avg: {
            duration: "desc",
          },
        },
        take: resultLimit,
      }),

      // Most active users (if no specific user is requested)
      !userId
        ? prisma.exercise.groupBy({
            by: ["userId"],
            where: {
              date: {
                gte: startDate,
              },
              ...formCondition,
              userId: {
                not: null,
              },
            },
            _count: true,
            orderBy: {
              _count: {
                id: "desc",
              },
            },
            take: resultLimit,
          })
        : Promise.resolve([]),
    ]);

    // Calculate percentage distribution of forms
    const formDistribution = exercisesByForm.reduce(
      (acc, curr) => {
        const countValue = curr._count || 0;
        const percentage =
          totalExercises > 0
            ? Number(((countValue / totalExercises) * 100).toFixed(2))
            : 0;

        acc[curr.form] = {
          count: countValue,
          percentage: percentage.toString(),
        };
        return acc;
      },
      {} as Record<string, { count: number; percentage: string }>,
    );

    // Format the results
    const results = {
      totalExercises,
      formDistribution,
      topExercises: exercisesByName.map((ex) => ({
        name: ex.name,
        count: ex._count || 0,
      })),
      exercisesByDay: exercisesByDay.map((day: any) => ({
        day: day.day,
        count: typeof day.count === "bigint" ? Number(day.count) : day.count,
      })),
      averageDurationByExercise: averageDuration.map((ex) => ({
        name: ex.name,
        averageDuration: ex._avg?.duration || 0,
      })),
      mostActiveUsers: !userId
        ? await Promise.all(
            mostActiveUsers.map(async (user) => {
              // Get user name for each userId
              const userInfo = await prisma.user.findUnique({
                where: { id: user.userId! },
                select: { name: true },
              });

              return {
                userId: user.userId,
                name: userInfo?.name || "Unknown User",
                exerciseCount: user._count || 0,
              };
            }),
          )
        : [],
    };

    const endTime = performance.now();
    const executionTimeMs = endTime - startTime;

    // Return the results with performance metrics
    res.setHeader("Content-Type", "application/json");
    res.status(200).end(
      customJSONStringify({
        data: results,
        meta: {
          timeframe: timeframeDays,
          formFilter,
          executionTimeMs,
        },
      }),
    );
  } catch (error) {
    console.error("Error fetching exercise statistics:", error);
    return res.status(500).json({
      error: "Failed to fetch exercise statistics",
      details: error instanceof Error ? error.message : String(error),
    });
  }
}
