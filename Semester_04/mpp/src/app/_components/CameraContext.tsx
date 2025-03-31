"use client";

import {
  createContext,
  useContext,
  useState,
  ReactNode,
  useCallback,
  useEffect,
} from "react";

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
}

const defaultSettings: CameraSettings = {
  videoQuality: "high",
  cameraFacing: "user",
  recordAudio: true,
};

const CameraContext = createContext<CameraContextType | undefined>(undefined);

export function CameraProvider({ children }: { children: ReactNode }) {
  const [settings, setSettings] = useState<CameraSettings>(defaultSettings);
  const [recordedVideos, setRecordedVideos] = useState<string[]>([]);
  const [exercises, setExercises] = useState<Exercise[]>([]);

  const [exerciseChangeListeners] = useState<(() => void)[]>([]);

  useEffect(() => {
    const fetchExercises = async () => {
      try {
        const response = await fetch("/api/exercises");
        if (!response.ok) {
          throw new Error("Failed to fetch exercises");
        }
        const data = await response.json();
        console.log("API response data:", data);

        // Ensure data is properly formatted
        if (Array.isArray(data)) {
          setExercises(data);
        } else if (
          data &&
          typeof data === "object" &&
          Array.isArray(data.exercises)
        ) {
          // Handle case where API returns { exercises: [...] }
          setExercises(data.exercises);
        } else {
          console.error("Unexpected API response format:", data);
          setExercises([]);
        }
      } catch (error) {
        console.error("Failed to fetch exercises:", error);
        setExercises([]);
      }
    };

    fetchExercises();
  }, []);

  const fetchFilteredExercises = useCallback(
    async (filters: ExerciseFilters) => {
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
        return data;
      } catch (error) {
        console.error("Failed to fetch filtered exercises:", error);
        return { exercises: [], total: 0 };
      }
    },
    [],
  );

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
      try {
        const response = await fetch("/api/exercises", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(exercise),
        });

        if (!response.ok) {
          throw new Error("Failed to create exercise");
        }

        const newExercise = await response.json();
        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          return [newExercise, ...prevArray];
        });
        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to add exercise:", error);
      }
    },
    [notifyExerciseChange],
  );

  const updateExercise = useCallback(
    async (id: string, updates: Partial<Omit<Exercise, "id">>) => {
      try {
        const response = await fetch(`/api/exercises/${id}`, {
          method: "PUT",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify(updates),
        });

        if (!response.ok) {
          const errorData = await response.json().catch(() => null);
          const errorMessage = errorData?.error || "Failed to update exercise";
          throw new Error(`${errorMessage} (Status: ${response.status})`);
        }

        const updatedExercise = await response.json();
        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          return prevArray.map((ex) => (ex.id === id ? updatedExercise : ex));
        });
        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to update exercise:", error);
        throw error; // Re-throw to allow calling code to handle the error
      }
    },
    [notifyExerciseChange],
  );

  const deleteExercise = useCallback(
    async (id: string) => {
      try {
        const response = await fetch(`/api/exercises/${id}`, {
          method: "DELETE",
        });

        if (!response.ok) {
          throw new Error("Failed to delete exercise");
        }

        setExercises((prev) => {
          const prevArray = Array.isArray(prev) ? prev : [];
          return prevArray.filter((ex) => ex.id !== id);
        });
        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to delete exercise:", error);
      }
    },
    [notifyExerciseChange],
  );

  const getExerciseById = useCallback(
    (id: string) => {
      const exercisesArray = Array.isArray(exercises) ? exercises : [];
      return exercisesArray.find((ex) => ex.id === id);
    },
    [exercises],
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
