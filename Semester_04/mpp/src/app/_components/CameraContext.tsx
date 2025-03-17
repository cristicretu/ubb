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

  const updateSettings = useCallback((newSettings: Partial<CameraSettings>) => {
    setSettings((prev) => ({ ...prev, ...newSettings }));
  }, []);

  const addRecordedVideo = useCallback((videoUrl: string) => {
    setRecordedVideos((prev) => [videoUrl, ...prev]);
  }, []);

  const addExercise = useCallback((exercise: Omit<Exercise, "id">) => {
    const newExercise = {
      ...exercise,
      id: Date.now().toString(),
    };
    setExercises((prev) => {
      const newState = [newExercise, ...prev];
      return newState;
    });
  }, []);

  const updateExercise = useCallback(
    (id: string, updates: Partial<Omit<Exercise, "id">>) => {
      setExercises((prev) =>
        prev.map((ex) => (ex.id === id ? { ...ex, ...updates } : ex)),
      );
    },
    [],
  );

  const deleteExercise = useCallback((id: string) => {
    setExercises((prev) => prev.filter((ex) => ex.id !== id));
  }, []);

  const getExerciseById = useCallback(
    (id: string) => {
      return exercises.find((ex) => ex.id === id);
    },
    [exercises],
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
