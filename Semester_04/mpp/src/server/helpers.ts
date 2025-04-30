import { createServerSideHelpers as createServerSideHelpersBase } from "@trpc/react-query/server";
import { appRouter } from "~/server/api/root";
import { createTRPCContext } from "~/server/api/trpc";
import superjson from "superjson";
import { headers } from "next/headers";

export async function createServerSideHelpers() {
  const headersData = headers();

  return createServerSideHelpersBase({
    router: appRouter,
    ctx: await createTRPCContext({
      headers: {
        get: (key) => headersData.get(key) ?? undefined,
        has: (key) => headersData.has(key),
        entries: () => headersData.entries(),
        keys: () => headersData.keys(),
        values: () => headersData.values(),
        forEach: (callback) => {
          headersData.forEach(callback);
          return undefined;
        },
      },
    }),
    transformer: superjson,
  });
}
