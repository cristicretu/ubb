import { z } from "zod";
import { adminProcedure, createTRPCRouter } from "~/server/api/trpc";
import { Prisma } from "@prisma/client";

const userSelect = {
  id: true,
  name: true,
  email: true,
  role: true,
  emailVerified: true,
  image: true,
  _count: {
    select: {
      exercises: true,
      posts: true,
    },
  },
} satisfies Prisma.UserSelect;

export const adminRouter = createTRPCRouter({
  getAllUsers: adminProcedure.query(async ({ ctx }) => {
    const users = await ctx.db.user.findMany({
      select: userSelect,
      orderBy: {
        email: "asc",
      },
    });

    return users;
  }),
});
