import { NextRequest, NextResponse } from "next/server";
import { writeFile, mkdir, appendFile, readdir, unlink } from "fs/promises";
import { join } from "path";
import { existsSync } from "fs";

const CHUNKS_DIR = join(process.cwd(), "temp", "chunks");
const UPLOAD_DIR = join(process.cwd(), "public", "uploads");

const ensureDirectories = async () => {
  if (!existsSync(CHUNKS_DIR)) {
    await mkdir(CHUNKS_DIR, { recursive: true });
  }
  if (!existsSync(UPLOAD_DIR)) {
    await mkdir(UPLOAD_DIR, { recursive: true });
  }
};

export async function POST(request: NextRequest) {
  try {
    await ensureDirectories();

    const formData = await request.formData();
    const file = formData.get("chunk") as File | null;
    const fileId = formData.get("fileId") as string;
    const chunkIndex = formData.get("chunkIndex") as string;
    const totalChunks = formData.get("totalChunks") as string;
    const filename = formData.get("filename") as string;

    if (!file || !fileId || !chunkIndex || !totalChunks || !filename) {
      return NextResponse.json(
        { error: "Missing required parameters" },
        { status: 400 },
      );
    }

    const chunkFilename = `${fileId}_${chunkIndex}`;
    const chunkPath = join(CHUNKS_DIR, chunkFilename);

    const bytes = await file.arrayBuffer();
    const buffer = Buffer.from(bytes);
    await writeFile(chunkPath, buffer);

    if (parseInt(chunkIndex) + 1 === parseInt(totalChunks)) {
      const extension = filename.split(".").pop() || "";
      const timestamp = Date.now();
      const uniqueId = Math.random().toString(36).substring(2, 10);
      const finalFilename = `${timestamp}_${uniqueId}.${extension}`;
      const finalPath = join(UPLOAD_DIR, finalFilename);

      await writeFile(finalPath, Buffer.from([]));

      for (let i = 0; i < parseInt(totalChunks); i++) {
        const currentChunkPath = join(CHUNKS_DIR, `${fileId}_${i}`);
        const chunkBuffer = await readChunkFile(currentChunkPath);
        await appendFile(finalPath, chunkBuffer);

        await unlink(currentChunkPath);
      }

      const fileUrl = `/uploads/${finalFilename}`;

      return NextResponse.json({
        success: true,
        fileUrl,
        filename: finalFilename,
        assembled: true,
      });
    }

    return NextResponse.json({
      success: true,
      chunkIndex,
      totalChunks,
      assembled: false,
    });
  } catch (error) {
    console.error("Error handling chunked upload:", error);
    return NextResponse.json(
      { error: "Failed to process chunked upload" },
      { status: 500 },
    );
  }
}

async function readChunkFile(path: string): Promise<Buffer> {
  try {
    const { readFile } = await import("fs/promises");
    return await readFile(path);
  } catch (error) {
    console.error(`Error reading chunk file ${path}:`, error);
    return Buffer.from([]);
  }
}
