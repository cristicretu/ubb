import { redirect } from "next/navigation";
import { auth } from "~/server/auth";
import { ADMIN_ROLE } from "~/server/api/trpc";
import { db } from "~/server/db";
import Link from "next/link";

export default async function ActivityLogsPage() {
  const session = await auth();

  // Check if the user is logged in and is an admin
  if (!session || session.user.role !== ADMIN_ROLE) {
    redirect("/");
  }

  // Fetch activity logs with user information, limit to 100 most recent
  let activityLogs = [];

  try {
    activityLogs = await db.activityLog.findMany({
      include: {
        user: {
          select: {
            id: true,
            name: true,
            email: true,
            role: true,
          },
        },
      },
      orderBy: {
        createdAt: "desc",
      },
      take: 100,
    });
  } catch (error) {
    console.error("Error fetching activity logs:", error);
    // Continue with empty logs
  }

  return (
    <div className="container mx-auto py-8">
      <div className="mb-6 flex items-center justify-between">
        <h1 className="text-3xl font-bold">Activity Logs</h1>
        <Link
          href="/admin"
          className="rounded-md bg-gray-800 px-4 py-2 text-sm font-medium text-white hover:bg-gray-700"
        >
          Back to Dashboard
        </Link>
      </div>

      <div className="overflow-hidden rounded-lg bg-white shadow-md dark:bg-gray-800">
        <div className="border-b border-gray-200 p-4 dark:border-gray-700">
          <h2 className="text-xl font-semibold">Recent User Activities</h2>
          <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
            Showing the 100 most recent activities across all users.
          </p>
        </div>

        <div className="overflow-x-auto">
          {activityLogs.length === 0 ? (
            <div className="p-6 text-center text-gray-500 dark:text-gray-400">
              No activity logs found.
            </div>
          ) : (
            <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
              <thead className="bg-gray-50 dark:bg-gray-700">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Time
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    User
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Action
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Entity
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Details
                  </th>
                </tr>
              </thead>
              <tbody className="divide-y divide-gray-200 bg-white dark:divide-gray-700 dark:bg-gray-800">
                {activityLogs.map((log) => (
                  <tr key={log.id}>
                    <td className="whitespace-nowrap px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      {new Date(log.createdAt).toLocaleString()}
                    </td>
                    <td className="whitespace-nowrap px-6 py-4 text-sm font-medium text-gray-900 dark:text-white">
                      {log.user?.name || log.user?.email || "Unknown"}
                      <div className="text-xs text-gray-500">
                        {log.user?.email}
                      </div>
                    </td>
                    <td className="whitespace-nowrap px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      <span
                        className={`inline-flex items-center rounded-full px-2.5 py-0.5 text-xs font-medium ${
                          log.action === "CREATE"
                            ? "bg-green-100 text-green-800 dark:bg-green-900 dark:text-green-200"
                            : log.action === "DELETE"
                              ? "bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200"
                              : log.action === "UPDATE"
                                ? "bg-yellow-100 text-yellow-800 dark:bg-yellow-900 dark:text-yellow-200"
                                : "bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200"
                        }`}
                      >
                        {log.action}
                      </span>
                    </td>
                    <td className="whitespace-nowrap px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      {log.entity}
                    </td>
                    <td className="px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      {log.details || (
                        <span className="italic text-gray-400">No details</span>
                      )}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
      </div>
    </div>
  );
}
