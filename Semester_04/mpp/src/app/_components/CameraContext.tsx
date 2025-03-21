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
    const savedExercises = localStorage.getItem("exercises");
    if (savedExercises) {
      try {
        setExercises(JSON.parse(savedExercises));
      } catch (error) {
        console.error("Failed to parse saved exercises:", error);
      }
    }
  }, []);

  useEffect(() => {
    if (exercises.length > 0) {
      localStorage.setItem("exercises", JSON.stringify(exercises));
    }
  }, [exercises]);

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
    (exercise: Omit<Exercise, "id">) => {
      const newExercise = {
        ...exercise,
        id: Date.now().toString(),
      };
      setExercises((prev) => {
        const newState = [newExercise, ...prev];
        return newState;
      });

      setTimeout(() => notifyExerciseChange(), 50);
    },
    [notifyExerciseChange],
  );

  const updateExercise = useCallback(
    (id: string, updates: Partial<Omit<Exercise, "id">>) => {
      setExercises((prev) =>
        prev.map((ex) => (ex.id === id ? { ...ex, ...updates } : ex)),
      );

      setTimeout(() => notifyExerciseChange(), 50);
    },
    [notifyExerciseChange],
  );

  const deleteExercise = useCallback(
    (id: string) => {
      setExercises((prev) => prev.filter((ex) => ex.id !== id));

      setTimeout(() => notifyExerciseChange(), 50);
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
