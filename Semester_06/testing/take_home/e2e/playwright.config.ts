import { defineConfig, devices } from "@playwright/test";

const FRONTEND_URL = "http://localhost:2069";
const BACKEND_URL = "http://localhost:2070";

export default defineConfig({
  testDir: "./tests",
  fullyParallel: false,
  workers: 1,
  retries: process.env.CI ? 1 : 0,
  reporter: [["list"], ["html", { open: "never" }]],
  timeout: 30_000,
  expect: { timeout: 5_000 },
  use: {
    baseURL: FRONTEND_URL,
    trace: "on-first-retry",
    screenshot: "only-on-failure",
    video: "off",
    actionTimeout: 10_000,
    navigationTimeout: 15_000,
  },
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
  webServer: [
    {
      command: "npm --prefix ../app/backend run start",
      url: `${BACKEND_URL}/api/health`,
      reuseExistingServer: true,
      timeout: 60_000,
      env: { PORT: "2070" },
      stdout: "ignore",
      stderr: "pipe",
    },
    {
      command: "npm --prefix ../app/frontend run dev",
      url: FRONTEND_URL,
      reuseExistingServer: true,
      timeout: 60_000,
      stdout: "ignore",
      stderr: "pipe",
    },
  ],
});
