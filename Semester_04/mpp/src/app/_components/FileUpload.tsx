"use client";

import { useState, useRef, ChangeEvent } from "react";
import { toast } from "sonner";
import { useNetwork } from "./NetworkContext";

interface UploadProps {
  onUploadComplete?: (fileUrl: string) => void;
  maxFileSize?: number;
  accept?: string;
  className?: string;
}

const CHUNK_SIZE = 2 * 1024 * 1024;

export default function FileUpload({
  onUploadComplete,
  maxFileSize = 512 * 1024 * 1024,
  accept = "video/*,image/*",
  className = "",
}: UploadProps) {
  const [uploading, setUploading] = useState(false);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const { isOnline } = useNetwork();

  const isLargeFile = (file: File) => {
    return file.size > 10 * 1024 * 1024;
  };

  const handleFileChange = async (e: ChangeEvent<HTMLInputElement>) => {
    const files = e.target.files;
    if (!files || files.length === 0) return;

    const file = files[0];
    if (!file) return;

    // Validate file size
    if (file.size > maxFileSize) {
      setError(
        `File is too large. Maximum size is ${maxFileSize / (1024 * 1024)}MB`,
      );
      toast.error(
        `File is too large. Maximum size is ${maxFileSize / (1024 * 1024)}MB`,
      );
      return;
    }

    // Reset state
    setError(null);
    setUploading(true);
    setProgress(0);

    try {
      // Check if we should use chunked upload
      if (isLargeFile(file)) {
        await uploadLargeFile(file);
      } else {
        await uploadFile(file);
      }
    } catch (error) {
      console.error("Upload error:", error);
      setError("Failed to upload file. Please try again.");
      toast.error("Failed to upload file. Please try again.");
    } finally {
      setUploading(false);
      // Reset the file input
      if (fileInputRef.current) {
        fileInputRef.current.value = "";
      }
    }
  };

  // Regular file upload for smaller files
  const uploadFile = async (file: File) => {
    const formData = new FormData();
    formData.append("file", file);

    try {
      const response = await fetch("/api/upload", {
        method: "POST",
        body: formData,
      });

      if (!response.ok) {
        throw new Error(`Upload failed with status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success && data.fileUrl) {
        toast.success("File uploaded successfully");
        if (onUploadComplete) {
          onUploadComplete(data.fileUrl);
        }
      } else {
        throw new Error(data.error || "Upload failed");
      }
    } catch (error) {
      console.error("Error uploading file:", error);
      throw error;
    }
  };

  // Chunked upload for large files
  const uploadLargeFile = async (file: File) => {
    const fileId = Date.now().toString();
    const totalChunks = Math.ceil(file.size / CHUNK_SIZE);
    let uploadedChunks = 0;

    // Upload each chunk
    for (let chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
      const start = chunkIndex * CHUNK_SIZE;
      const end = Math.min(start + CHUNK_SIZE, file.size);
      const chunk = file.slice(start, end);

      const formData = new FormData();
      formData.append("chunk", chunk);
      formData.append("fileId", fileId);
      formData.append("chunkIndex", chunkIndex.toString());
      formData.append("totalChunks", totalChunks.toString());
      formData.append("filename", file.name);

      try {
        const response = await fetch("/api/upload/chunked", {
          method: "POST",
          body: formData,
        });

        if (!response.ok) {
          throw new Error(
            `Chunk upload failed with status: ${response.status}`,
          );
        }

        const data = await response.json();

        if (data.success) {
          uploadedChunks++;

          // Update progress
          const newProgress = Math.round((uploadedChunks / totalChunks) * 100);
          setProgress(newProgress);

          // Check if all chunks uploaded and file assembled
          if (data.assembled && data.fileUrl) {
            toast.success("File uploaded and assembled successfully");
            if (onUploadComplete) {
              onUploadComplete(data.fileUrl);
            }
          }
        } else {
          throw new Error(data.error || "Chunk upload failed");
        }
      } catch (error) {
        console.error(`Error uploading chunk ${chunkIndex}:`, error);
        throw error;
      }
    }
  };

  return (
    <div className={`w-full ${className}`}>
      <div className="flex w-full flex-col items-center justify-center rounded-lg border-2 border-dashed border-gray-600 p-4">
        <input
          ref={fileInputRef}
          type="file"
          accept={accept}
          onChange={handleFileChange}
          disabled={uploading || !isOnline}
          className="hidden"
          id="file-upload-input"
        />

        {uploading ? (
          <div className="w-full space-y-2">
            <p className="text-center text-sm text-gray-400">
              Uploading... {progress}%
            </p>
            <div className="h-2.5 w-full rounded-full bg-gray-700">
              <div
                className="h-2.5 rounded-full bg-blue-600"
                style={{ width: `${progress}%` }}
              ></div>
            </div>
          </div>
        ) : (
          <label
            htmlFor="file-upload-input"
            className={`flex h-32 w-full cursor-pointer flex-col items-center justify-center ${
              !isOnline ? "cursor-not-allowed opacity-50" : ""
            }`}
          >
            <svg
              className="mb-3 h-8 w-8 text-gray-400"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M7 16a4 4 0 01-.88-7.903A5 5 0 1115.9 6L16 6a5 5 0 011 9.9M15 13l-3-3m0 0l-3 3m3-3v12"
              />
            </svg>
            {!isOnline ? (
              <p className="text-sm text-gray-500">
                You must be online to upload files
              </p>
            ) : (
              <>
                <p className="mb-2 text-sm text-gray-500">
                  <span className="font-semibold">Click to upload</span> or drag
                  and drop
                </p>
                <p className="text-xs text-gray-500">
                  Videos, images (max {maxFileSize / (1024 * 1024)}MB)
                </p>
              </>
            )}
          </label>
        )}

        {error && <p className="mt-2 text-sm text-red-500">{error}</p>}
      </div>
    </div>
  );
}
