import type { NextApiRequest, NextApiResponse } from "next";

export default function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== "POST") {
    return res.status(405).json({ error: "Method not allowed" });
  }

  try {
    const { event, data } = req.body;

    console.log(
      `Socket emit request for event: ${event}`,
      JSON.stringify(data, null, 2),
    );

    if (!event) {
      console.error("Socket emit failed: Event name is required");
      return res.status(400).json({ error: "Event name is required" });
    }

    if (!res.socket.server.io) {
      console.error("Socket emit failed: Socket.io server not initialized");
      return res.status(500).json({
        error: "Socket.io server not initialized",
        message: "Try refreshing the page to initialize the Socket.io server",
      });
    }

    // Validate exercise events
    if (event === "exercise:updated" || event === "exercise:deleted") {
      if (!data || !data.id) {
        console.error(`Invalid data for ${event} event: missing id`, data);
        return res.status(400).json({
          error: "Invalid data",
          message: "Exercise events require an id",
        });
      }
    }

    // Log before emitting event
    console.log(`Emitting socket event: ${event}`);
    res.socket.server.io.emit(event, data);
    console.log(`Successfully emitted event: ${event}`);

    return res.status(200).json({
      success: true,
      event,
      message: `Event ${event} successfully emitted`,
    });
  } catch (error) {
    console.error("Error emitting socket event:", error);
    return res.status(500).json({
      error: "Failed to emit event",
      message: error instanceof Error ? error.message : "Unknown error",
    });
  }
}
