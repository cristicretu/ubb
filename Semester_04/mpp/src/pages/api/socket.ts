import { Server as ServerIO } from "socket.io";
import type { NextApiRequest, NextApiResponse } from "next";

export default function SocketHandler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  if (res.socket.server.io) {
    console.log("Socket is already running");
    res.end();
    return;
  }

  console.log("Setting up socket");
  const io = new ServerIO(res.socket.server, {
    path: "/api/socket",
    addTrailingSlash: false,
    cors: {
      origin: "*",
      methods: ["GET", "POST"],
      credentials: true,
    },

    transports: ["polling", "websocket"],
    pingTimeout: 30000,
    pingInterval: 25000,
  });

  res.socket.server.io = io;

  io.on("connection", (socket) => {
    console.log(`Client connected: ${socket.id}`);

    socket.on("disconnect", () => {
      console.log(`Client disconnected: ${socket.id}`);
    });

    socket.on("exercise:create", (data) => {
      console.log("Exercise created:", data);
      io.emit("exercise:created", data);
    });

    socket.on("exercise:update", (data) => {
      console.log("Exercise updated:", data);
      io.emit("exercise:updated", data);
    });

    socket.on("exercise:delete", (data) => {
      console.log("Exercise deleted:", data);
      io.emit("exercise:deleted", data);
    });

    socket.on("data:synced", (data) => {
      console.log("Data synced:", data);
      io.emit("data:synced", data);
    });
  });

  console.log("Socket server started");
  res.end();
}
