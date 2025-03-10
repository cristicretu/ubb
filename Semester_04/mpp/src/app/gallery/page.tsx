"use client";

import PageLayout from "../_components/PageLayout";
import VideoPlayer from "../_components/VideoPlayer";
import { useCameraContext } from "../_components/CameraContext";

export default function Gallery() {
  const { recordedVideos } = useCameraContext();

  return (
    <PageLayout>
      <div className="container mx-auto px-4 py-8">
        <h1 className="mb-6 text-3xl font-bold">Gallery</h1>

        {recordedVideos.length > 0 ? (
          <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
            {recordedVideos.map((videoSrc, index) => (
              <div
                key={index}
                className="overflow-hidden rounded-lg bg-zinc-900"
              >
                <VideoPlayer src={videoSrc} />
                <div className="p-3">
                  <p className="text-sm text-zinc-400">Recording {index + 1}</p>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <div className="flex flex-col items-center justify-center py-20">
            <div className="max-w-md rounded-lg bg-zinc-900 p-8 text-center">
              <h2 className="mb-2 text-xl font-semibold">No videos yet</h2>
              <p className="mb-4 text-zinc-400">
                Your recorded videos will appear here. Go to the Camera tab to
                start recording.
              </p>
            </div>
          </div>
        )}
      </div>
    </PageLayout>
  );
}
