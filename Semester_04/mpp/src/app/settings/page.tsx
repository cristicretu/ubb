"use client";

import { useState, useEffect } from "react";
import PageLayout from "../_components/PageLayout";
import { useCameraContext } from "../_components/CameraContext";

export default function Settings() {
  const { settings, updateSettings } = useCameraContext();
  const [localSettings, setLocalSettings] = useState(settings);

  useEffect(() => {
    setLocalSettings(settings);
  }, [settings]);

  const handleSave = () => {
    updateSettings(localSettings);
  };

  return (
    <PageLayout>
      <div className="container mx-auto px-4 py-8">
        <h1 className="mb-6 text-3xl font-bold">Settings</h1>

        <div className="mx-auto max-w-md rounded-lg bg-zinc-900 p-6">
          <div className="mb-6">
            <label className="mb-2 block text-sm font-medium">
              Video Quality
            </label>
            <select
              value={localSettings.videoQuality}
              onChange={(e) =>
                setLocalSettings({
                  ...localSettings,
                  videoQuality: e.target.value,
                })
              }
              className="w-full rounded-md border border-zinc-700 bg-zinc-800 px-3 py-2 text-white"
            >
              <option value="low">Low (480p)</option>
              <option value="medium">Medium (720p)</option>
              <option value="high">High (1080p)</option>
              <option value="ultra">Ultra HD (4K)</option>
            </select>
          </div>

          <div className="mb-6">
            <label className="mb-2 block text-sm font-medium">
              Default Camera
            </label>
            <div className="flex gap-4">
              <label className="flex items-center">
                <input
                  type="radio"
                  name="cameraFacing"
                  value="user"
                  checked={localSettings.cameraFacing === "user"}
                  onChange={() =>
                    setLocalSettings({ ...localSettings, cameraFacing: "user" })
                  }
                  className="mr-2"
                />
                Front
              </label>
              <label className="flex items-center">
                <input
                  type="radio"
                  name="cameraFacing"
                  value="environment"
                  checked={localSettings.cameraFacing === "environment"}
                  onChange={() =>
                    setLocalSettings({
                      ...localSettings,
                      cameraFacing: "environment",
                    })
                  }
                  className="mr-2"
                />
                Back
              </label>
            </div>
          </div>

          <div className="mb-6">
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={localSettings.recordAudio}
                onChange={(e) =>
                  setLocalSettings({
                    ...localSettings,
                    recordAudio: e.target.checked,
                  })
                }
                className="mr-2"
              />
              Record Audio
            </label>
          </div>

          <button
            onClick={handleSave}
            className="w-full rounded-md bg-blue-600 px-4 py-2 text-white transition-colors hover:bg-blue-700"
          >
            Save Settings
          </button>
        </div>
      </div>
    </PageLayout>
  );
}
