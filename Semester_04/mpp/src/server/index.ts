import { initializeMonitoring, stopMonitoring } from "./services/monitor";

// Store the interval ID in case we need to clean it up
let monitoringIntervalId: NodeJS.Timeout | null = null;

/**
 * Initialize background services when the server starts
 */
export function initializeServices() {
  console.log("Initializing background services...");

  // Start the monitoring service
  initializeMonitoring();

  console.log("All background services initialized");

  return {
    cleanupServices: () => {
      // Clean up the monitoring service
      stopMonitoring();
      console.log("All background services stopped");
    },
  };
}

// For development environments we need to check if we're in a hot-reload situation
// to avoid initializing services multiple times
if (process.env.NODE_ENV === "development") {
  // Only initialize if this is the initial load, not a hot-reload
  if (!global.__servicesInitialized) {
    global.__servicesInitialized = true;
    initializeServices();
    console.log("Services initialized in development mode");
  }
} else {
  // In production, we can initialize right away
  initializeServices();
}

// Add TypeScript types for the global object
declare global {
  // eslint-disable-next-line no-var
  var __servicesInitialized: boolean;
}
