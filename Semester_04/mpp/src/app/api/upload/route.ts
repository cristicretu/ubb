import { NextRequest, NextResponse } from "next/server";
import { writeFile, mkdir } from "fs/promises";
import { join } from "path";
import { existsSync } from "fs";

const UPLOAD_DIR = join(process.cwd(), "public", "uploads");

const ensureUploadDir = async () => {
  if (!existsSync(UPLOAD_DIR)) {
    await mkdir(UPLOAD_DIR, { recursive: true });
  }
};

export async function POST(request: NextRequest) {
  try {
    const formData = await request.formData();
    const file = formData.get("file") as File | null;

    if (!file) {
      return NextResponse.json({ error: "No file provided" }, { status: 400 });
    }

    const bytes = await file.arrayBuffer();
    const buffer = Buffer.from(bytes);

    const originalName = file.name;
    const extension = originalName.split(".").pop() || "";

    const timestamp = Date.now();
    const uniqueId = Math.random().toString(36).substring(2, 10);
    const filename = `${timestamp}_${uniqueId}.${extension}`;

    await ensureUploadDir();

    const filePath = join(UPLOAD_DIR, filename);
    await writeFile(filePath, buffer);

    // Get the host from the request headers
    const host = request.headers.get("host") || "localhost:3000";
    const protocol = host.includes("localhost") ? "http" : "https";

    // Generate a relative URL (for local storage)
    const relativeUrl = `/uploads/${filename}`;

    // Generate an absolute URL (for API validation)
    const absoluteUrl = `${protocol}://${host}${relativeUrl}`;

    console.log("File uploaded successfully:", {
      filename,
      relativeUrl,
      absoluteUrl,
    });

    return NextResponse.json({
      success: true,
      fileUrl: absoluteUrl, // Use absolute URL
      relativeUrl, // Also provide relative URL
      filename,
      size: file.size,
      type: file.type,
    });
  } catch (error) {
    console.error("Error handling file upload:", error);
    return NextResponse.json(
      { error: "Failed to upload file" },
      { status: 500 },
    );
  }
}

export async function GET() {
  return NextResponse.json({
    success: true,
    message: "Upload API is running",
    maxFileSize: "512MB",
  });
}
