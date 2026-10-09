import { NextRequest } from "next/server";
import { addWebSocketConnection, removeWebSocketConnection } from "../route";

// Handle real-time updates using SSE (acting like WebSocket)
export async function GET(request: NextRequest) {
  // Create a readable stream for real-time updates
  const stream = new ReadableStream({
    start(controller) {
      // Mock WebSocket-like behavior using SSE
      const mockWs = {
        readyState: 1, // OPEN
        send: (message: string) => {
          try {
            controller.enqueue(`data: ${message}\n\n`);
          } catch (error) {
            console.error("Error sending SSE message:", error);
          }
        },
        close: () => {
          try {
            controller.close();
          } catch (error) {
            // Connection already closed
          }
        },
      };

      // Add to WebSocket connections
      addWebSocketConnection(mockWs);

      // Send initial connection message
      mockWs.send(
        JSON.stringify({
          type: "connected",
          message: "Real-time updates connected",
        }),
      );

      // Clean up when connection closes
      request.signal.addEventListener("abort", () => {
        removeWebSocketConnection(mockWs);
        mockWs.close();
      });
    },
  });

  // Return the stream with appropriate headers for SSE
  return new Response(stream, {
    headers: {
      "Content-Type": "text/event-stream",
      "Cache-Control": "no-cache",
      Connection: "keep-alive",
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "GET",
      "Access-Control-Allow-Headers": "Cache-Control",
    },
  });
}
