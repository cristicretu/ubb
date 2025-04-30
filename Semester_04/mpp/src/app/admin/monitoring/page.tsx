import { redirect } from "next/navigation";
import { auth } from "~/server/auth";
import { ADMIN_ROLE } from "~/server/api/trpc";
import { getMonitoredUsers } from "~/server/services/monitor";
import Link from "next/link";

export default async function MonitoringDashboard() {
  const session = await auth();

  // Check if the user is logged in and is an admin
  if (!session || session.user.role !== ADMIN_ROLE) {
    redirect("/");
  }

  // Fetch monitored users
  const monitoredUsers = await getMonitoredUsers();

  return (
    <div className="container mx-auto py-8">
      <div className="mb-6 flex items-center justify-between">
        <h1 className="text-3xl font-bold">Monitored Users</h1>
        <Link
          href="/admin"
          className="rounded-md bg-gray-800 px-4 py-2 text-sm font-medium text-white hover:bg-gray-700"
        >
          Back to Dashboard
        </Link>
      </div>

      <div className="overflow-hidden rounded-lg bg-white shadow-md dark:bg-gray-800">
        <div className="border-b border-gray-200 p-4 dark:border-gray-700">
          <h2 className="text-xl font-semibold">
            Users Flagged for Suspicious Activity
          </h2>
          <p className="mt-1 text-sm text-gray-500 dark:text-gray-400">
            These users have been automatically flagged for unusually high
            activity rates.
          </p>
        </div>

        <div className="overflow-x-auto">
          {monitoredUsers.length === 0 ? (
            <div className="p-6 text-center text-gray-500 dark:text-gray-400">
              No users are currently being monitored.
            </div>
          ) : (
            <table className="min-w-full divide-y divide-gray-200 dark:divide-gray-700">
              <thead className="bg-gray-50 dark:bg-gray-700">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    User
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Reason
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Monitored Since
                  </th>
                  <th className="px-6 py-3 text-left text-xs font-medium uppercase tracking-wider text-gray-500 dark:text-gray-300">
                    Status
                  </th>
                </tr>
              </thead>
              <tbody className="divide-y divide-gray-200 bg-white dark:divide-gray-700 dark:bg-gray-800">
                {monitoredUsers.map((monitoredUser) => (
                  <tr key={monitoredUser.id}>
                    <td className="whitespace-nowrap px-6 py-4 text-sm font-medium text-gray-900 dark:text-white">
                      {monitoredUser.user.name ||
                        monitoredUser.user.email ||
                        "Unknown user"}
                      <div className="text-xs text-gray-500">
                        {monitoredUser.user.email}
                      </div>
                    </td>
                    <td className="px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      {monitoredUser.reason}
                    </td>
                    <td className="whitespace-nowrap px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      {new Date(monitoredUser.createdAt).toLocaleString()}
                    </td>
                    <td className="whitespace-nowrap px-6 py-4 text-sm text-gray-500 dark:text-gray-300">
                      <span
                        className={`inline-flex items-center rounded-full px-2.5 py-0.5 text-xs font-medium ${
                          monitoredUser.activelyMonitored
                            ? "bg-red-100 text-red-800 dark:bg-red-900 dark:text-red-200"
                            : "bg-gray-100 text-gray-800 dark:bg-gray-700 dark:text-gray-300"
                        }`}
                      >
                        {monitoredUser.activelyMonitored
                          ? "Active Monitoring"
                          : "Inactive"}
                      </span>
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
