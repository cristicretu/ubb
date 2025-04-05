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
  return exercises.map((ex) => (ex.id === id ? { ...ex, ...updates } : ex));
};

export const applyDeleteOperation = (
  exercises: Exercise[],
  id: string,
): Exercise[] => {
  return exercises.filter((ex) => ex.id !== id);
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

export const syncWithServer = async (
  apiPath: string,
): Promise<{
  success: boolean;
  successCount: number;
  failureCount: number;
  errors: any[];
}> => {
  const pendingOps = getPendingOperations();
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

      if (
        (op.type === "update" || op.type === "delete") &&
        entityId &&
        entityId.startsWith("local_") &&
        idMappings[entityId]
      ) {
        entityId = idMappings[entityId];
      }

      switch (op.type) {
        case "create":
          response = await fetch(`${apiPath}/exercises`, {
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
          response = await fetch(`${apiPath}/exercises/${entityId}`, {
            method: "PUT",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(op.data),
          });
          break;

        case "delete":
          response = await fetch(`${apiPath}/exercises/${entityId}`, {
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
