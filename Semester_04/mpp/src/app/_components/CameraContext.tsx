"use client";

import {
  createContext,
  useContext,
  useState,
  ReactNode,
  useCallback,
  useEffect,
  useRef,
} from "react";
import { useNetwork } from "./NetworkContext";
import { toast } from "sonner";
import * as offlineStorage from "../_services/offlineStorage";
import { useWebSocket } from "./WebSocketProvider";

export interface Exercise {
  id: string;
  name: string;
  videoUrl: string;
  form: "bad" | "medium" | "good";
  date: string;
  duration: number;
}

interface CameraSettings {
  videoQuality: string;
  cameraFacing: string;
  recordAudio: boolean;
}

export interface ExerciseFilters {
  form?: "bad" | "medium" | "good" | "all";
  search?: string;
  sortBy?: string;
  page?: number;
  limit?: number;
}

interface CameraContextType {
  settings: CameraSettings;
  updateSettings: (newSettings: Partial<CameraSettings>) => void;
  recordedVideos: string[];
  addRecordedVideo: (videoUrl: string) => void;
  exercises: Exercise[];
  fetchFilteredExercises: (filters: ExerciseFilters) => Promise<{
    exercises: Exercise[];
    total: number;
  }>;
  addExercise: (exercise: Omit<Exercise, "id">) => void;
  updateExercise: (id: string, updates: Partial<Omit<Exercise, "id">>) => void;
  deleteExercise: (id: string) => void;
  getExerciseById: (id: string) => Exercise | undefined;
  addEventListener: (event: "exercisesChange", callback: () => void) => void;
  removeEventListener: (event: "exercisesChange", callback: () => void) => void;
  hasMoreExercises: boolean;
  isLoadingMore: boolean;
  loadMoreExercises: () => Promise<boolean>;
  pendingOperationsCount: number;
  syncPendingOperations: () => Promise<void>;
  isSyncing: boolean;
  startExerciseGenerator: (intervalMs?: number) => Promise<boolean>;
  stopExerciseGenerator: () => Promise<boolean>;
  isGeneratorRunning: boolean;
}

const defaultSettings: CameraSettings = {
  videoQuality: "high",
  cameraFacing: "user",
  recordAudio: true,
};

const DEFAULT_PAGE_SIZE = 10;

const CameraContext = createContext<CameraContextType | undefined>(undefined);

export function CameraProvider({ children }: { children: ReactNode }) {
  const { isOnline, isServerAvailable } = useNetwork();
  const { lastEvent, socket, isConnected } = useWebSocket();
  const [settings, setSettings] = useState<CameraSettings>(defaultSettings);
  const [recordedVideos, setRecordedVideos] = useState<string[]>([]);
  const [exercises, setExercises] = useState<Exercise[]>([]);
  const [totalExercises, setTotalExercises] = useState<number>(0);
  const [currentPage, setCurrentPage] = useState<number>(1);
  const [hasMore, setHasMore] = useState<boolean>(true);
  const [isLoadingMore, setIsLoadingMore] = useState<boolean>(false);
  const [pendingOperationsCount, setPendingOperationsCount] =
    useState<number>(0);
  const [isGeneratorRunning, setIsGeneratorRunning] = useState<boolean>(false);

  const [isSyncing, setIsSyncing] = useState<boolean>(false);

  const currentFiltersRef = useRef<ExerciseFilters>({
    form: "all",
    sortBy: "date-desc",
    page: 1,
    limit: DEFAULT_PAGE_SIZE,
  });

  const [exerciseChangeListeners] = useState<(() => void)[]>([]);

  useEffect(() => {
    if (!lastEvent) return;

    console.log("Handling WebSocket event:", lastEvent.name, lastEvent.data);

    if (lastEvent.name === "exercise:created") {
      const newExercise = lastEvent.data as Exercise;

      setExercises((prev) => {
        if (prev.some((ex) => ex.id === newExercise.id)) {
          return prev;
        }

        return [newExercise, ...prev];
      });

      const storedExercises = offlineStorage.loadExercisesFromStorage();
      if (!storedExercises.some((ex) => ex.id === newExercise.id)) {
        offlineStorage.saveExercisesToStorage([
          newExercise,
          ...storedExercises,
        ]);
      }

      setTimeout(() => notifyExerciseChange(), 50);
    } else if (lastEvent.name === "exercise:updated") {
      const updatedExercise = lastEvent.data as Partial<Exercise> & {
        id: string;
      };

      setExercises((prev) =>
        prev.map((ex) =>
          ex.id === updatedExercise.id ? { ...ex, ...updatedExercise } : ex,
        ),
      );

      const storedExercises = offlineStorage.loadExercisesFromStorage();
      const updatedStoredExercises = storedExercises.map((ex) =>
        ex.id === updatedExercise.id ? { ...ex, ...updatedExercise } : ex,
      );
      offlineStorage.saveExercisesToStorage(updatedStoredExercises);

      setTimeout(() => notifyExerciseChange(), 50);
    } else if (lastEvent.name === "exercise:deleted") {
      const { id } = lastEvent.data as { id: string };

      setExercises((prev) => prev.filter((ex) => ex.id !== id));

      const storedExercises = offlineStorage.loadExercisesFromStorage();
      const updatedStoredExercises = storedExercises.filter(
        (ex) => ex.id !== id,
      );
      offlineStorage.saveExercisesToStorage(updatedStoredExercises);

      setTimeout(() => notifyExerciseChange(), 50);
    }
  }, [lastEvent]);

  useEffect(() => {
    const pendingOps = offlineStorage.getPendingOperations();
    setPendingOperationsCount(pendingOps.length);
  }, []);

  useEffect(() => {
    const storedExercises = offlineStorage.loadExercisesFromStorage();

    if (storedExercises.length > 0) {
      const exercisesWithPendingChanges =
        offlineStorage.applyPendingOperations(storedExercises);
      setExercises(exercisesWithPendingChanges);
    }

    if (isOnline && isServerAvailable && !isSyncing) {
      fetchExercisesFromServer();

      checkGeneratorStatus();
    }
  }, [isOnline, isServerAvailable, isSyncing]);

  const checkGeneratorStatus = async () => {
    try {
      const response = await fetch("/api/background-worker");
      if (response.ok) {
        const data = await response.json();
        setIsGeneratorRunning(data.isRunning);
      }
    } catch (error) {
      console.error("Failed to check generator status:", error);
    }
  };

  const startExerciseGenerator = async (intervalMs = 30000) => {
    if (!isOnline || !isServerAvailable) {
      toast.error("Cannot start generator while offline");
      return false;
    }

    try {
      const response = await fetch("/api/background-worker", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ action: "start", interval: intervalMs }),
      });

      const data = await response.json();

      if (data.success) {
        setIsGeneratorRunning(true);
        toast.success("Started generating exercises in the background");
        return true;
      } else {
        toast.error(data.message || "Failed to start generator");
        return false;
      }
    } catch (error) {
      console.error("Error starting generator:", error);
      toast.error("Failed to start generator");
      return false;
    }
  };

  const stopExerciseGenerator = async () => {
    if (!isOnline || !isServerAvailable) {
      toast.error("Cannot stop generator while offline");
      return false;
    }

    try {
      const response = await fetch("/api/background-worker", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ action: "stop" }),
      });

      const data = await response.json();

      if (data.success) {
        setIsGeneratorRunning(false);
        toast.success("Stopped generating exercises");
        return true;
      } else {
        toast.error(data.message || "Failed to stop generator");
        return false;
      }
    } catch (error) {
      console.error("Error stopping generator:", error);
      toast.error("Failed to stop generator");
      return false;
    }
  };

  useEffect(() => {
    if (isOnline && isServerAvailable && pendingOperationsCount > 0) {
      setIsSyncing(true);
      syncPendingOperations();
    }
  }, [isOnline, isServerAvailable, pendingOperationsCount]);

  useEffect(() => {
    let isMounted = true;

    if (isOnline && isServerAvailable && !isSyncing) {
      if (pendingOperationsCount === 0) {
        fetchExercisesFromServer();
      }
    }

    return () => {
      isMounted = false;
    };
  }, [isOnline, isServerAvailable, pendingOperationsCount, isSyncing]);

  const fetchExercisesFromServer = async () => {
    if (isSyncing) return;

    try {
      const filters = currentFiltersRef.current;
      const result = await fetchFilteredExercises(filters);

      const storedExercises = offlineStorage.loadExercisesFromStorage();

      const localCreatedExercises = storedExercises.filter(
        (ex) =>
          ex.id.startsWith("local_") &&
          !result.exercises.some((serverEx: Exercise) => serverEx.id === ex.id),
      );

      const mergedExercises = [...result.exercises, ...localCreatedExercises];

      offlineStorage.saveExercisesToStorage(mergedExercises);

      const exercisesWithPendingChanges =
        offlineStorage.applyPendingOperations(mergedExercises);

      setExercises(exercisesWithPendingChanges);
      setTotalExercises(result.total + localCreatedExercises.length);
      setHasMore(
        filters.page ? result.exercises.length >= filters.limit! : false,
      );
    } catch (error) {
      console.error("Failed to fetch exercises from server:", error);
    }
  };

  const syncPendingOperations = async () => {
    if (!isOnline || !isServerAvailable) {
      toast.error(
        "Cannot sync while offline. Will try again when connection is restored.",
      );
      setIsSyncing(false);
      return;
    }

    try {
      toast.loading("Syncing changes to server...", { id: "sync-toast" });
      const result = await offlineStorage.syncWithServer("/api");

      if (result.success) {
        toast.success(`Successfully synced ${result.successCount} changes`, {
          id: "sync-toast",
        });
        setPendingOperationsCount((prev) => prev - result.successCount);

        await fetchExercisesFromServer();
      } else {
        toast.error(
          `Synced ${result.successCount} changes, but ${result.failureCount} failed`,
          { id: "sync-toast" },
        );
        setPendingOperationsCount((prev) => prev - result.successCount);

        await fetchExercisesFromServer();
      }
    } catch (error) {
      toast.error("Failed to sync changes. Will try again later.", {
        id: "sync-toast",
      });
      console.error("Sync error:", error);

      await fetchExercisesFromServer();
    } finally {
      setIsSyncing(false);
    }
  };

  const fetchFilteredExercises = useCallback(
    async (filters: ExerciseFilters) => {
      currentFiltersRef.current = { ...currentFiltersRef.current, ...filters };

      if (!isOnline || !isServerAvailable) {
        const storedExercises = offlineStorage.loadExercisesFromStorage();

        const exercisesWithPendingChanges =
          offlineStorage.applyPendingOperations(storedExercises);

        return offlineStorage.filterExercisesLocally(
          exercisesWithPendingChanges,
          filters,
        );
      }

      try {
        const params = new URLSearchParams();
        if (filters.form && filters.form !== "all")
          params.append("form", filters.form);
        if (filters.search) params.append("search", filters.search);
        if (filters.sortBy) params.append("sortBy", filters.sortBy);
        if (filters.page) params.append("page", filters.page.toString());
        if (filters.limit) params.append("limit", filters.limit.toString());

        const url = `/api/exercises?${params.toString()}`;
        const response = await fetch(url);

        if (!response.ok) {
          throw new Error("Failed to fetch filtered exercises");
        }

        const data = await response.json();

        if (!filters.page || filters.page === 1) {
          offlineStorage.saveExercisesToStorage(data.exercises);
        } else if (filters.page && filters.page > 1) {
          const storedExercises = offlineStorage.loadExercisesFromStorage();
          const updatedExercises = [...storedExercises];

          data.exercises.forEach((newEx: Exercise) => {
            if (!updatedExercises.some((ex) => ex.id === newEx.id)) {
              updatedExercises.push(newEx);
            }
          });

          offlineStorage.saveExercisesToStorage(updatedExercises);
        }

        return data;
      } catch (error) {
        console.error("Failed to fetch filtered exercises:", error);

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const exercisesWithPendingChanges =
          offlineStorage.applyPendingOperations(storedExercises);
        return offlineStorage.filterExercisesLocally(
          exercisesWithPendingChanges,
          filters,
        );
      }
    },
    [isOnline, isServerAvailable],
  );

  const loadMoreExercises = async () => {
    if (!hasMore || isLoadingMore) return false;

    setIsLoadingMore(true);

    try {
      const nextPage = currentPage + 1;
      const filters = {
        ...currentFiltersRef.current,
        page: nextPage,
      };

      const result = await fetchFilteredExercises(filters);

      if (result.exercises.length > 0) {
        setExercises((prev) => {
          const newExercises = result.exercises.filter(
            (newEx: Exercise) =>
              !prev.some((existingEx) => existingEx.id === newEx.id),
          );
          return [...prev, ...newExercises];
        });
        setCurrentPage(nextPage);
        setHasMore(result.exercises.length >= DEFAULT_PAGE_SIZE);
        return true;
      } else {
        setHasMore(false);
        return false;
      }
    } catch (error) {
      console.error("Failed to load more exercises:", error);
      return false;
    } finally {
      setIsLoadingMore(false);
    }
  };

  const notifyExerciseChange = useCallback(() => {
    exerciseChangeListeners.forEach((listener) => listener());
  }, [exerciseChangeListeners]);

  const updateSettings = useCallback((newSettings: Partial<CameraSettings>) => {
    setSettings((prev) => ({ ...prev, ...newSettings }));
  }, []);

  const addRecordedVideo = useCallback((videoUrl: string) => {
    setRecordedVideos((prev) => {
      const prevArray = Array.isArray(prev) ? prev : [];
      return [videoUrl, ...prevArray];
    });
  }, []);

  const addExercise = useCallback(
    async (exercise: Omit<Exercise, "id">) => {
      let validExercise = { ...exercise };

      if (!validExercise.videoUrl.startsWith("http")) {
        validExercise.videoUrl = new URL(
          validExercise.videoUrl,
          window.location.origin,
        ).toString();
      }

      if (!isOnline || !isServerAvailable) {
        offlineStorage.addPendingOperation({
          type: "create",
          entity: "exercise",
          data: validExercise,
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = offlineStorage.applyCreateOperation(
          storedExercises,
          validExercise,
        );
        offlineStorage.saveExercisesToStorage(updatedExercises);

        setExercises((prev) =>
          offlineStorage.applyCreateOperation(prev, validExercise),
        );
        setPendingOperationsCount((prev) => prev + 1);

        toast.success("Exercise saved locally. Will sync when online.");
        setTimeout(() => notifyExerciseChange(), 50);
        return;
      }

      try {
        console.log(
          "Sending exercise to API:",
          JSON.stringify(validExercise, null, 2),
        );
        const response = await fetch("/api/exercises", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(validExercise),
        });

        let responseData;
        try {
          responseData = await response.json();
        } catch (jsonError) {
          console.error("Failed to parse API response:", jsonError);
          responseData = { error: "Invalid response from server" };
        }

        console.log("API response:", response.status, responseData);

        if (!response.ok) {
          // Extract validation error details
          if (responseData?.details) {
            console.error("Validation errors:", responseData.details);

            // Check for specific validation errors
            if (responseData.details.fieldErrors) {
              const fieldErrors = responseData.details.fieldErrors;

              // Log each field error
              Object.entries(fieldErrors).forEach(([field, errors]) => {
                console.error(`Field '${field}' errors:`, errors);
              });

              // Create a more helpful error message
              const errorMessages = Object.entries(fieldErrors)
                .map(([field, errors]) => {
                  const errorText = Array.isArray(errors)
                    ? errors.join(", ")
                    : typeof errors === "string"
                      ? errors
                      : "Invalid value";
                  return `${field}: ${errorText}`;
                })
                .join("; ");

              throw new Error(`Validation failed: ${errorMessages}`);
            }
          }

          throw new Error(responseData?.error || "Failed to create exercise");
        }

        const newExercise = responseData;
        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          return [newExercise, ...prevArray];
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = [newExercise, ...storedExercises];
        offlineStorage.saveExercisesToStorage(updatedExercises);

        if (socket && isConnected) {
          socket.emit("exercise:create", newExercise);
        } else {
          try {
            await fetch("/api/socket-emit", {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                event: "exercise:created",
                data: newExercise,
              }),
            });
          } catch (wsError) {
            console.error("Failed to emit WebSocket event:", wsError);
          }
        }

        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to add exercise:", error);
        toast.error(
          error instanceof Error
            ? error.message
            : "Failed to add exercise. Saving locally for later sync.",
        );

        offlineStorage.addPendingOperation({
          type: "create",
          entity: "exercise",
          data: validExercise,
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = offlineStorage.applyCreateOperation(
          storedExercises,
          validExercise,
        );
        offlineStorage.saveExercisesToStorage(updatedExercises);

        setExercises((prev) =>
          offlineStorage.applyCreateOperation(prev, validExercise),
        );
        setPendingOperationsCount((prev) => prev + 1);

        setTimeout(() => notifyExerciseChange(), 50);
      }
    },
    [isOnline, isServerAvailable, notifyExerciseChange, socket, isConnected],
  );

  const getExerciseById = useCallback(
    (id: string) => {
      const exercisesArray = Array.isArray(exercises) ? exercises : [];

      // First try direct lookup
      let exercise = exercisesArray.find((ex) => ex.id === id);

      // If that fails, try using normalized ID comparison
      if (
        !exercise &&
        typeof offlineStorage.findExerciseByNormalizedId === "function"
      ) {
        exercise = offlineStorage.findExerciseByNormalizedId(
          exercisesArray,
          id,
        );
      }

      return exercise;
    },
    [exercises],
  );

  const updateExercise = useCallback(
    async (id: string, updates: Partial<Omit<Exercise, "id">>) => {
      if (!isOnline || !isServerAvailable) {
        offlineStorage.addPendingOperation({
          type: "update",
          entity: "exercise",
          entityId: id,
          data: updates,
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = offlineStorage.applyUpdateOperation(
          storedExercises,
          id,
          updates,
        );
        offlineStorage.saveExercisesToStorage(updatedExercises);

        setExercises((prev) =>
          offlineStorage.applyUpdateOperation(prev, id, updates),
        );
        setPendingOperationsCount((prev) => prev + 1);

        toast.success("Exercise updated locally. Will sync when online.");
        setTimeout(() => notifyExerciseChange(), 50);
        return;
      }

      // Move variable declarations outside the try block so they're available in the catch block
      let serverIdToUse = id;
      let updatedExercise = null;

      try {
        console.log("Updating exercise:", id, updates);
        console.log(
          `[MEZI] updateExercise called with ID: ${id}, Type: ${typeof id}, URL will be: /api/exercises/${id}`,
        );

        // Get the exercise from local state to help with matching
        const exerciseToUpdate = getExerciseById(id);
        if (!exerciseToUpdate) {
          console.log(
            `[MEZI] Warning: Cannot find exercise with ID ${id} in local state`,
          );
        }

        // Try to find the correct server ID if there's a mismatch
        let serverExerciseLookupFailed = false;

        // Only do a server lookup if we have a name to match (safer than just ID)
        if (exerciseToUpdate?.name) {
          try {
            console.log(
              `[MEZI] Looking up exercise on server by name: "${exerciseToUpdate.name}"`,
            );
            // Fetch all exercises from the server
            const serverResponse = await fetch("/api/exercises");
            if (serverResponse.ok) {
              const data = await serverResponse.json();
              const serverExercises = data.exercises || [];

              // Try to find a matching exercise by name (more reliable than ID)
              const matchingServerExercise = serverExercises.find(
                (ex: Exercise) => ex.name === exerciseToUpdate.name,
              );

              if (matchingServerExercise) {
                console.log(
                  `[MEZI] Found matching exercise on server with ID: ${matchingServerExercise.id}`,
                );
                if (matchingServerExercise.id !== id) {
                  console.log(
                    `[MEZI] ID mismatch - Local: ${id}, Server: ${matchingServerExercise.id}`,
                  );
                  // Use the server's ID for the API call
                  serverIdToUse = matchingServerExercise.id;
                }
              } else {
                console.log(
                  `[MEZI] No matching exercise found on server by name: "${exerciseToUpdate.name}"`,
                );
              }
            } else {
              console.log(
                `[MEZI] Server lookup failed with status: ${serverResponse.status}`,
              );
              serverExerciseLookupFailed = true;
            }
          } catch (lookupError) {
            console.error(
              `[MEZI] Error looking up exercise on server:`,
              lookupError,
            );
            serverExerciseLookupFailed = true;
          }
        }

        // If our server lookup failed but we found a local ID format that differs
        if (
          serverExerciseLookupFailed &&
          exerciseToUpdate &&
          exerciseToUpdate.id !== id
        ) {
          serverIdToUse = exerciseToUpdate.id;
          console.log(
            `[MEZI] Falling back to local ID format: ${serverIdToUse}`,
          );
        }

        console.log(`[MEZI] Making API call with ID: ${serverIdToUse}`);

        // Log the exact URL being used
        const apiUrl = `/api/exercises/${serverIdToUse}`;
        console.log(`[MEZI] API URL: "${apiUrl}"`);

        // Try to extract any hidden characters or encoding issues
        console.log(
          `[MEZI] URL character codes:`,
          [...apiUrl].map((c) => `${c}:${c.charCodeAt(0)}`),
        );

        const response = await fetch(apiUrl, {
          method: "PUT",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(updates),
        });

        const responseData = await response.json().catch(() => ({
          error: "Failed to parse response",
        }));
        console.log("[MEZI] Update response:", response.status, responseData);
        console.log("Update response:", response.status, responseData);

        if (!response.ok) {
          const errorMessage =
            responseData?.error || "Failed to update exercise";
          throw new Error(`${errorMessage} (Status: ${response.status})`);
        }

        // Make sure we have a complete exercise object with all properties
        updatedExercise = {
          ...getExerciseById(serverIdToUse),
          ...responseData,
          id: serverIdToUse, // Ensure ID is preserved
        };

        console.log("Updated exercise:", updatedExercise);

        // Update local state
        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          const normalizedId = String(serverIdToUse).trim();
          return prevArray.map((ex) => {
            const normalizedExId = String(ex.id).trim();
            return normalizedExId === normalizedId ? updatedExercise : ex;
          });
        });

        // Update offline storage
        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const normalizedId = String(serverIdToUse).trim();
        const updatedExercises = storedExercises.map((ex) => {
          const normalizedExId = String(ex.id).trim();
          return normalizedExId === normalizedId
            ? { ...ex, ...updatedExercise }
            : ex;
        });
        offlineStorage.saveExercisesToStorage(updatedExercises);

        // Emit WebSocket event
        if (socket && isConnected) {
          console.log("Emitting socket update event:", updatedExercise);
          socket.emit("exercise:update", updatedExercise);
        } else {
          try {
            console.log("Emitting HTTP update event:", updatedExercise);
            const wsResponse = await fetch("/api/socket-emit", {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                event: "exercise:updated",
                data: updatedExercise,
              }),
            });

            if (!wsResponse.ok) {
              console.error(
                "WebSocket emit HTTP response error:",
                await wsResponse.text(),
              );
            }
          } catch (wsError) {
            console.error("Failed to emit WebSocket event:", wsError);
          }
        }

        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to update exercise:", error);
        toast.error(
          error instanceof Error
            ? error.message
            : "Failed to update exercise. Saving locally for later sync.",
        );

        // Add to pending operations with the correct ID
        offlineStorage.addPendingOperation({
          type: "update",
          entity: "exercise",
          entityId: serverIdToUse, // Now using the scoped variable
          data: updates,
        });

        // If the update failed but we got far enough to create the updated exercise object
        if (updatedExercise) {
          // Use the updated exercise object for storage
          const storedExercises = offlineStorage.loadExercisesFromStorage();
          const normalizedId = String(serverIdToUse).trim();
          const updatedExercises = storedExercises.map((ex) => {
            const normalizedExId = String(ex.id).trim();
            return normalizedExId === normalizedId
              ? { ...ex, ...updatedExercise }
              : ex;
          });
          offlineStorage.saveExercisesToStorage(updatedExercises);

          setExercises((prev) => {
            const prevArray = Array.isArray(prev) ? prev : [];
            return prevArray.map((ex) => {
              const normalizedExId = String(ex.id).trim();
              return normalizedExId === normalizedId
                ? { ...ex, ...updatedExercise }
                : ex;
            });
          });
        } else {
          // Fallback to the standard update operation
          const storedExercises = offlineStorage.loadExercisesFromStorage();
          // This will use our updated normalized ID comparison in applyUpdateOperation
          const updatedExercises = offlineStorage.applyUpdateOperation(
            storedExercises,
            serverIdToUse,
            updates,
          );
          offlineStorage.saveExercisesToStorage(updatedExercises);

          // Apply the same update to the state
          setExercises((prev) =>
            offlineStorage.applyUpdateOperation(prev, serverIdToUse, updates),
          );
        }

        setPendingOperationsCount((prev) => prev + 1);
        setTimeout(() => notifyExerciseChange(), 50);
      }
    },
    [
      isOnline,
      isServerAvailable,
      notifyExerciseChange,
      socket,
      isConnected,
      getExerciseById,
    ],
  );

  const deleteExercise = useCallback(
    async (id: string) => {
      if (!isOnline || !isServerAvailable) {
        offlineStorage.addPendingOperation({
          type: "delete",
          entity: "exercise",
          entityId: id,
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = offlineStorage.applyDeleteOperation(
          storedExercises,
          id,
        );
        offlineStorage.saveExercisesToStorage(updatedExercises);

        setExercises((prev) => offlineStorage.applyDeleteOperation(prev, id));
        setPendingOperationsCount((prev) => prev + 1);

        toast.success("Exercise deleted locally. Will sync when online.");
        setTimeout(() => notifyExerciseChange(), 50);
        return;
      }

      try {
        const exerciseToDelete = getExerciseById(id);
        console.log("Deleting exercise:", id, exerciseToDelete?.name);

        const response = await fetch(`/api/exercises/${id}`, {
          method: "DELETE",
        });

        const responseData = await response.json().catch(() => ({
          error: "Failed to parse response",
        }));
        console.log("Delete response:", response.status, responseData);

        if (!response.ok) {
          const errorMessage =
            responseData?.error || "Failed to delete exercise";
          throw new Error(`${errorMessage} (Status: ${response.status})`);
        }

        // Update local state
        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          return prevArray.filter((ex) => ex.id !== id);
        });

        // Update offline storage
        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = storedExercises.filter((ex) => ex.id !== id);
        offlineStorage.saveExercisesToStorage(updatedExercises);

        // Emit WebSocket event
        if (socket && isConnected) {
          console.log("Emitting socket delete event:", {
            id,
            name: exerciseToDelete?.name,
          });
          socket.emit("exercise:delete", { id, name: exerciseToDelete?.name });
        } else {
          try {
            console.log("Emitting HTTP delete event:", {
              id,
              name: exerciseToDelete?.name,
            });
            const wsResponse = await fetch("/api/socket-emit", {
              method: "POST",
              headers: {
                "Content-Type": "application/json",
              },
              body: JSON.stringify({
                event: "exercise:deleted",
                data: { id, name: exerciseToDelete?.name },
              }),
            });

            if (!wsResponse.ok) {
              console.error(
                "WebSocket emit HTTP response error:",
                await wsResponse.text(),
              );
            }
          } catch (wsError) {
            console.error("Failed to emit WebSocket event:", wsError);
          }
        }

        setTimeout(() => notifyExerciseChange(), 50);
        toast.success("Exercise deleted successfully");
      } catch (error) {
        console.error("Failed to delete exercise:", error);
        toast.error(
          error instanceof Error
            ? error.message
            : "Failed to delete exercise. Will try again when online.",
        );

        // Add to pending operations
        offlineStorage.addPendingOperation({
          type: "delete",
          entity: "exercise",
          entityId: id,
        });

        const storedExercises = offlineStorage.loadExercisesFromStorage();
        const updatedExercises = offlineStorage.applyDeleteOperation(
          storedExercises,
          id,
        );
        offlineStorage.saveExercisesToStorage(updatedExercises);

        setExercises((prev) => offlineStorage.applyDeleteOperation(prev, id));
        setPendingOperationsCount((prev) => prev + 1);

        setTimeout(() => notifyExerciseChange(), 50);
      }
    },
    [
      isOnline,
      isServerAvailable,
      getExerciseById,
      notifyExerciseChange,
      socket,
      isConnected,
    ],
  );

  const addEventListener = useCallback(
    (event: "exercisesChange", callback: () => void) => {
      if (event === "exercisesChange") {
        exerciseChangeListeners.push(callback);
      }
    },
    [exerciseChangeListeners],
  );

  const removeEventListener = useCallback(
    (event: "exercisesChange", callback: () => void) => {
      if (event === "exercisesChange") {
        const index = exerciseChangeListeners.indexOf(callback);
        if (index !== -1) {
          exerciseChangeListeners.splice(index, 1);
        }
      }
    },
    [exerciseChangeListeners],
  );

  const contextValue = {
    settings,
    updateSettings,
    recordedVideos,
    addRecordedVideo,
    exercises: Array.isArray(exercises) ? exercises : [],
    fetchFilteredExercises,
    addExercise,
    updateExercise,
    deleteExercise,
    getExerciseById,
    addEventListener,
    removeEventListener,
    hasMoreExercises: hasMore,
    isLoadingMore,
    loadMoreExercises,
    pendingOperationsCount,
    syncPendingOperations,
    isSyncing,
    startExerciseGenerator,
    stopExerciseGenerator,
    isGeneratorRunning,
  };

  return (
    <CameraContext.Provider value={contextValue}>
      {children}
    </CameraContext.Provider>
  );
}

export function useCameraContext() {
  const context = useContext(CameraContext);
  if (context === undefined) {
    throw new Error("useCameraContext must be used within a CameraProvider");
  }
  return context;
}
