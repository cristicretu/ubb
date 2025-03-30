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

interface CameraContextType {
  settings: CameraSettings;
  updateSettings: (newSettings: Partial<CameraSettings>) => void;
  recordedVideos: string[];
  addRecordedVideo: (videoUrl: string) => void;
  exercises: Exercise[];
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
        setExercises(data);
      } catch (error) {
        console.error("Failed to fetch exercises:", error);
      }
    };

    fetchExercises();
  }, []);

  const notifyExerciseChange = useCallback(() => {
    exerciseChangeListeners.forEach((listener) => listener());
  }, [exerciseChangeListeners]);

  const updateSettings = useCallback((newSettings: Partial<CameraSettings>) => {
    setSettings((prev) => ({ ...prev, ...newSettings }));
  }, []);

  const addRecordedVideo = useCallback((videoUrl: string) => {
    setRecordedVideos((prev) => [videoUrl, ...prev]);
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
        setExercises((prev) => [newExercise, ...prev]);
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
          throw new Error("Failed to update exercise");
        }

        const updatedExercise = await response.json();
        setExercises((prev) =>
          prev.map((ex) => (ex.id === id ? updatedExercise : ex)),
        );
        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to update exercise:", error);
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

        setExercises((prev) => prev.filter((ex) => ex.id !== id));
        setTimeout(() => notifyExerciseChange(), 50);
      } catch (error) {
        console.error("Failed to delete exercise:", error);
      }
    },
    [notifyExerciseChange],
  );

  const getExerciseById = useCallback(
    (id: string) => {
      return exercises.find((ex) => ex.id === id);
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
    exercises,
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
