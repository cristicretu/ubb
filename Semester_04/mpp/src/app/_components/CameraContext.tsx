"use client";

import { createContext, useContext, useState, ReactNode } from "react";

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

  const updateSettings = (newSettings: Partial<CameraSettings>) => {
    setSettings((prev) => ({ ...prev, ...newSettings }));
  };

  const addRecordedVideo = (videoUrl: string) => {
    setRecordedVideos((prev) => [videoUrl, ...prev]);
  };

  return (
    <CameraContext.Provider
      value={{
        settings,
        updateSettings,
        recordedVideos,
        addRecordedVideo,
      }}
    >
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
