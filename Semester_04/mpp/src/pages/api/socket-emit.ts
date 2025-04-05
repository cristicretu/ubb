import type { NextApiRequest, NextApiResponse } from "next";

export default function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== "POST") {
    return res.status(405).json({ error: "Method not allowed" });
  }

  try {
    const { event, data } = req.body;

    if (!event) {
      return res.status(400).json({ error: "Event name is required" });
    }

    if (!res.socket.server.io) {
      return res.status(500).json({
        error: "Socket.io server not initialized",
        message: "Try refreshing the page to initialize the Socket.io server",
      });
    }

    res.socket.server.io.emit(event, data);

    return res.status(200).json({ success: true });
  } catch (error) {
    console.error("Error emitting socket event:", error);
    return res.status(500).json({ error: "Failed to emit event" });
  }
}
