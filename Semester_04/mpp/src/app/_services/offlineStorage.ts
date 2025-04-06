import { Exercise, ExerciseFilters } from "../_components/CameraContext";

type PendingOperation = {
  id: string;
  type: "create" | "update" | "delete";
  timestamp: number;
  entity: "exercise";
  data?: any;
  entityId?: string;
};

const STORAGE_PREFIX = "offline_exercise_app_";

const KEYS = {
  EXERCISES: `${STORAGE_PREFIX}exercises`,
  PENDING_OPERATIONS: `${STORAGE_PREFIX}pending_operations`,
};

export const saveExercisesToStorage = (exercises: Exercise[]): void => {
  try {
    localStorage.setItem(KEYS.EXERCISES, JSON.stringify(exercises));
  } catch (error) {
    console.error("Failed to save exercises to local storage:", error);
  }
};

export const loadExercisesFromStorage = (): Exercise[] => {
  try {
    const stored = localStorage.getItem(KEYS.EXERCISES);
    return stored ? JSON.parse(stored) : [];
  } catch (error) {
    console.error("Failed to load exercises from local storage:", error);
    return [];
  }
};

export const addPendingOperation = (
  operation: Omit<PendingOperation, "id" | "timestamp">,
): string => {
  try {
    const pendingOps = getPendingOperations();
    const newOp: PendingOperation = {
      ...operation,
      id: `${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      timestamp: Date.now(),
    };

    pendingOps.push(newOp);
    localStorage.setItem(KEYS.PENDING_OPERATIONS, JSON.stringify(pendingOps));
    return newOp.id;
  } catch (error) {
    console.error("Failed to add pending operation:", error);
    return "";
  }
};

export const getPendingOperations = (): PendingOperation[] => {
  try {
    const stored = localStorage.getItem(KEYS.PENDING_OPERATIONS);
    return stored ? JSON.parse(stored) : [];
  } catch (error) {
    console.error("Failed to get pending operations:", error);
    return [];
  }
};

export const removePendingOperation = (operationId: string): void => {
  try {
    const pendingOps = getPendingOperations();
    const updatedOps = pendingOps.filter((op) => op.id !== operationId);
    localStorage.setItem(KEYS.PENDING_OPERATIONS, JSON.stringify(updatedOps));
  } catch (error) {
    console.error("Failed to remove pending operation:", error);
  }
};

export const filterExercisesLocally = (
  exercises: Exercise[],
  filters: ExerciseFilters,
): { exercises: Exercise[]; total: number } => {
  let filteredExercises = [...exercises];

  if (filters.form && filters.form !== "all") {
    filteredExercises = filteredExercises.filter(
      (ex) => ex.form === filters.form,
    );
  }

  if (filters.search) {
    const searchLower = filters.search.toLowerCase();
    filteredExercises = filteredExercises.filter((ex) =>
      ex.name.toLowerCase().includes(searchLower),
    );
  }

  if (filters.sortBy) {
    filteredExercises.sort((a, b) => {
      switch (filters.sortBy) {
        case "date-desc":
          return new Date(b.date).getTime() - new Date(a.date).getTime();
        case "date-asc":
          return new Date(a.date).getTime() - new Date(b.date).getTime();
        case "name-asc":
          return a.name.localeCompare(b.name);
        case "name-desc":
          return b.name.localeCompare(a.name);
        default:
          return 0;
      }
    });
  }

  const total = filteredExercises.length;

  if (filters.page !== undefined && filters.limit !== undefined) {
    const startIndex = (filters.page - 1) * filters.limit;
    filteredExercises = filteredExercises.slice(
      startIndex,
      startIndex + filters.limit,
    );
  }

  return { exercises: filteredExercises, total };
};

export const applyCreateOperation = (
  exercises: Exercise[],
  data: Omit<Exercise, "id">,
): Exercise[] => {
  const newExercise: Exercise = {
    ...data,
    id: `local_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
  };
  return [newExercise, ...exercises];
};

export const applyUpdateOperation = (
  exercises: Exercise[],
  id: string,
  updates: Partial<Exercise>,
): Exercise[] => {
  const normalizedInputId = normalizeExerciseId(id);
  return exercises.map((ex) =>
    normalizeExerciseId(ex.id) === normalizedInputId
      ? { ...ex, ...updates }
      : ex,
  );
};

export const applyDeleteOperation = (
  exercises: Exercise[],
  id: string,
): Exercise[] => {
  const normalizedInputId = normalizeExerciseId(id);
  return exercises.filter(
    (ex) => normalizeExerciseId(ex.id) !== normalizedInputId,
  );
};

export const applyPendingOperations = (exercises: Exercise[]): Exercise[] => {
  const pendingOps = getPendingOperations();

  if (pendingOps.length === 0) return exercises;

  const sortedOps = [...pendingOps].sort((a, b) => a.timestamp - b.timestamp);

  return sortedOps.reduce((currentExercises, op) => {
    if (op.entity !== "exercise") return currentExercises;

    switch (op.type) {
      case "create":
        return applyCreateOperation(currentExercises, op.data);
      case "update":
        return op.entityId
          ? applyUpdateOperation(currentExercises, op.entityId, op.data)
          : currentExercises;
      case "delete":
        return op.entityId
          ? applyDeleteOperation(currentExercises, op.entityId)
          : currentExercises;
      default:
        return currentExercises;
    }
  }, exercises);
};

/**
 * Normalizes exercise IDs to ensure consistency between client and server
 * This helps prevent issues when exercises have different ID formats
 */
export function normalizeExerciseId(id: string | number): string {
  // First convert to string if it's not already
  const idStr = String(id);

  // Log for debugging
  console.log(`Normalizing exercise ID: ${idStr}`);
  console.log(
    `[] Normalizing exercise ID: ${idStr}, Type: ${typeof id}, Empty check: ${!idStr || idStr === "undefined"}`,
  );

  // If it's a numeric string, ensure it's formatted consistently
  if (/^\d+$/.test(idStr)) {
    // For timestamp-like numeric IDs, make sure there's no formatting differences
    console.log(`[] Detected numeric ID: ${idStr}`);
    return idStr.trim();
  }

  return idStr;
}

/**
 * Find an exercise in an array using a normalized ID comparison
 * This helps with ID format mismatches (numeric vs uuid)
 */
export function findExerciseByNormalizedId(
  exercises: Exercise[],
  id: string,
): Exercise | undefined {
  const normalizedId = normalizeExerciseId(id);
  console.log(`[] Looking for exercise with normalized ID: ${normalizedId}`);

  // First try exact match
  let exercise = exercises.find((ex) => ex.id === id);

  // If that fails, try normalized match
  if (!exercise) {
    console.log(`[] Exact match failed, trying normalized comparison`);
    exercise = exercises.find(
      (ex) => normalizeExerciseId(ex.id) === normalizedId,
    );

    if (exercise) {
      console.log(
        `[] Found exercise via normalized ID. Original ID: ${exercise.id}, Search ID: ${id}`,
      );
    }
  }

  return exercise;
}

/**
 * Checks if the local storage version of the exercise and the server version
 * have ID format mismatches, and attempts to normalize them
 */
export function fixExerciseIdMismatches(
  localExercises: Exercise[],
  serverExercises: Exercise[],
): Exercise[] {
  console.log("Checking for ID mismatches between local and server exercises");
  console.log(
    `[] Checking ID mismatches - Local exercises: ${localExercises.map((e) => e.id).join(", ")}`,
  );
  console.log(
    `[] Checking ID mismatches - Server exercises: ${serverExercises.map((e) => e.id).join(", ")}`,
  );

  // Create a map of server exercise IDs with normalized IDs as keys
  const serverMap = new Map<string, Exercise>();
  serverExercises.forEach((exercise) => {
    const normalizedId = normalizeExerciseId(exercise.id);
    serverMap.set(normalizedId, exercise);
    console.log(
      `[] Server exercise mapped: Original ID=${exercise.id}, Normalized ID=${normalizedId}`,
    );
  });

  // Update local exercises if we find matches with normalized IDs
  return localExercises.map((localExercise) => {
    const normalizedId = normalizeExerciseId(localExercise.id);
    const serverMatch = serverMap.get(normalizedId);

    if (serverMatch && serverMatch.id !== localExercise.id) {
      console.log(
        `Found ID mismatch: Local=${localExercise.id}, Server=${serverMatch.id}. Using server ID.`,
      );
      return { ...localExercise, id: serverMatch.id };
    }

    return localExercise;
  });
}

// Modify the syncWithServer function to handle ID mismatches
export const syncWithServer = async (apiBasePath: string) => {
  const pendingOps = getPendingOperations();

  // Try to fetch server exercises to check for ID mismatches
  let serverExercises: Exercise[] = [];
  try {
    console.log("Fetching server exercises to check for ID mismatches");
    const response = await fetch(`${apiBasePath}/exercises`);
    if (response.ok) {
      const data = await response.json();
      serverExercises = data.exercises || [];
      console.log(`Fetched ${serverExercises.length} exercises from server`);

      // Fix any ID mismatches before attempting to sync
      if (serverExercises.length > 0) {
        const localExercises = loadExercisesFromStorage();
        const fixedLocalExercises = fixExerciseIdMismatches(
          localExercises,
          serverExercises,
        );

        // See if we fixed any mismatches
        const mismatchCount = fixedLocalExercises.filter(
          (ex, i) => ex.id !== localExercises[i]?.id,
        ).length;

        if (mismatchCount > 0) {
          console.log(
            `Fixed ${mismatchCount} ID mismatches between client and server`,
          );
          saveExercisesToStorage(fixedLocalExercises);
        }
      }
    } else {
      console.error(
        "Failed to fetch server exercises for ID checking:",
        await response.text(),
      );
    }
  } catch (error) {
    console.error("Error fetching server exercises for ID checking:", error);
  }

  if (pendingOps.length === 0) {
    return { success: true, successCount: 0, failureCount: 0, errors: [] };
  }

  const sortedOps = [...pendingOps].sort((a, b) => a.timestamp - b.timestamp);

  const results = {
    success: true,
    successCount: 0,
    failureCount: 0,
    errors: [] as any[],
  };

  const idMappings: Record<string, string> = {};

  for (const op of sortedOps) {
    if (op.entity !== "exercise") continue;

    try {
      let response;
      let entityId = op.entityId;

      // Normalize the entity ID if it exists
      if (entityId) {
        const normalizedId = normalizeExerciseId(entityId);
        console.log(
          `[] Sync operation with entityId: ${entityId}, Normalized: ${normalizedId}`,
        );

        // Check if a server exercise exists with a matching normalized ID
        const serverMatch = serverExercises.find(
          (ex) => normalizeExerciseId(ex.id) === normalizedId,
        );

        if (serverMatch && serverMatch.id !== entityId) {
          console.log(
            `Using server ID ${serverMatch.id} instead of ${entityId} for operation`,
          );
          console.log(
            `[] ID mismatch in operation - Using server ID ${serverMatch.id} instead of local ID ${entityId}`,
          );
          entityId = serverMatch.id;
        }
      }

      // Use ID mappings for local IDs that were created in this sync session
      if (
        (op.type === "update" || op.type === "delete") &&
        entityId &&
        entityId.startsWith("local_") &&
        idMappings[entityId]
      ) {
        console.log(
          `Mapping local ID ${entityId} to server ID ${idMappings[entityId]}`,
        );
        console.log(
          `[] Mapping temporary local ID ${entityId} to permanent server ID ${idMappings[entityId]}`,
        );
        entityId = idMappings[entityId];
      }

      switch (op.type) {
        case "create":
          response = await fetch(`${apiBasePath}/exercises`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(op.data),
          });

          if (response.ok) {
            const newExercise = await response.json();
            if (op.data.id && op.data.id.startsWith("local_")) {
              idMappings[op.data.id] = newExercise.id;

              const storedExercises = loadExercisesFromStorage();
              const updatedExercises = storedExercises.map((ex) =>
                ex.id === op.data.id ? { ...ex, id: newExercise.id } : ex,
              );
              saveExercisesToStorage(updatedExercises);
            }
          }
          break;

        case "update":
          response = await fetch(`${apiBasePath}/exercises/${entityId}`, {
            method: "PUT",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(op.data),
          });
          break;

        case "delete":
          response = await fetch(`${apiBasePath}/exercises/${entityId}`, {
            method: "DELETE",
          });
          break;
      }

      if (response && response.ok) {
        removePendingOperation(op.id);
        results.successCount++;
      } else {
        results.failureCount++;
        results.errors.push({
          operation: op,
          error: response ? await response.text() : "No response",
        });
        results.success = false;
      }
    } catch (error) {
      results.failureCount++;
      results.errors.push({ operation: op, error });
      results.success = false;
    }
  }

  return results;
};
